// SPDX-License-Identifier: GPL-2.0
/*
 * LCD backlight driver for the MediaTek MT6323 PMIC (Innioasis Y2).
 *
 * The panel backlight is driven by the MT6323's four constant-current ISINK
 * channels wired in PARALLEL (the vendor "lcd-backlight" = PMIC LCD_ISINK,
 * ganging ISINK0..3). The mainline leds-mt6323 exposes the channels as four
 * independent LEDs, which is the wrong abstraction for one backlight - driving a
 * single channel only modulates a quarter of the current and can't turn the
 * panel off. This driver gangs all four channels and presents one
 * backlight-class device.
 *
 * Binds as a regmap child of the PMIC wrapper (pwrap), like mt6323-charger; the
 * ISINK register map + enable sequence match leds-mt6323.
 */
#include <linux/backlight.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define MT6323_ISINK_CKCON1	0x126		/* clock select */
#define MT6323_ISINK_CKPDN2	0x10e		/* clock power-down */
#define MT6323_ISINK_EN_CTRL	0x356		/* per-channel enable */
#define MT6323_ISINK_CON0	0x330		/* DIM duty (brightness) */
#define MT6323_ISINK_CON1	0x332		/* DIM frequency select */
#define MT6323_ISINK_CON2	0x334		/* current step */
#define ISINK_CON(base, ch)	((base) + 8 * (ch))

#define ISINK_NCHAN		4
#define ISINK_CK_SEL(ch)	(BIT(10) << (ch))
#define ISINK_CK_PDN(ch)	BIT(ch)
#define ISINK_CH_EN(ch)		BIT(ch)
#define ISINK_DIM_DUTY_MASK	GENMASK(12, 8)
#define ISINK_DIM_DUTY(v)	(((v) << 8) & ISINK_DIM_DUTY_MASK)
#define ISINK_DIM_FSEL_MASK	GENMASK(15, 0)
#define ISINK_STEP_MASK		GENMASK(14, 12)
#define ISINK_STEP(v)		(((v) << 12) & ISINK_STEP_MASK)
#define ISINK_DUTY_MAX		31
/*
 * The rest of the vendor "lcd-backlight" ISINK setup (leds.c, the
 * MT65XX_LED_PMIC_LCD_ISINK init): each channel on the 1 MHz clock (CK_SEL
 * = 1), PWM mode, soft-start off, double current, phase delay and chop
 * enabled, PWM divider 2 (20 kHz). After a cold power-off the PMIC comes up
 * with these at their defaults (slow clock, divider left over), which is a
 * visibly flashing backlight; warm reboots had kept the stock settings.
 */
#define MT6323_TOP_CKPDN1	0x108
#define  DRV_2M_CK_PDN		BIT(6)
#define MT6323_ISINK_ANA0	0x350
#define  ISINK_DOUBLE_EN(ch)	BIT(11 - (ch))
#define MT6323_ISINK_PHASE_DLY	0x354
#define  ISINK_PHASE_DLY_EN(ch)	BIT(ch)
#define  ISINK_PHASE_DLY_TC	GENMASK(5, 4)
#define  ISINK_CHOP_EN(ch)	BIT(4 + (ch))	/* in EN_CTRL */
#define  ISINK_MODE_MASK	GENMASK(3, 2)	/* in CON0; 0 = PWM */
#define  ISINK_SFSTR_EN		BIT(0)		/* in CON2 */
#define ISINK_DIM_FSEL_20K	2

/*
 * Brightness is mapped across the current step (0..MAX_STEP) and the 5-bit DIM
 * duty (1..31). Step 0 alone (what the vendor kernel uses) is only ~half of LK's
 * boot brightness, so we let it reach step 1 (which LK itself uses = a safe
 * ceiling). Max brightness = (MAX_STEP+1)*31, mapped step = (b-1)/31,
 * duty = ((b-1)%31)+1.
 */
#define ISINK_MAX_STEP		5	/* 24 mA, the vendor backlight ceiling */
#define BL_MAX_BRIGHTNESS	((ISINK_MAX_STEP + 1) * ISINK_DUTY_MAX)

struct mt6323_backlight {
	struct regmap *regmap;
};

static void mt6323_isink_on(struct regmap *r, int ch, unsigned int step,
			    unsigned int duty)
{
	regmap_update_bits(r, MT6323_TOP_CKPDN1, DRV_2M_CK_PDN, 0);
	regmap_update_bits(r, MT6323_ISINK_CKPDN2, ISINK_CK_PDN(ch), 0);
	regmap_update_bits(r, MT6323_ISINK_CKCON1, ISINK_CK_SEL(ch),
			   ISINK_CK_SEL(ch));			/* 1 MHz */
	regmap_update_bits(r, ISINK_CON(MT6323_ISINK_CON0, ch),
			   ISINK_MODE_MASK, 0);			/* PWM mode */
	regmap_update_bits(r, ISINK_CON(MT6323_ISINK_CON2, ch),
			   ISINK_STEP_MASK | ISINK_SFSTR_EN, ISINK_STEP(step));
	regmap_update_bits(r, MT6323_ISINK_ANA0, ISINK_DOUBLE_EN(ch),
			   ISINK_DOUBLE_EN(ch));
	regmap_update_bits(r, MT6323_ISINK_PHASE_DLY,
			   ISINK_PHASE_DLY_EN(ch) | ISINK_PHASE_DLY_TC,
			   ISINK_PHASE_DLY_EN(ch));
	regmap_update_bits(r, MT6323_ISINK_EN_CTRL, ISINK_CHOP_EN(ch),
			   ISINK_CHOP_EN(ch));
	regmap_update_bits(r, ISINK_CON(MT6323_ISINK_CON0, ch),
			   ISINK_DIM_DUTY_MASK, ISINK_DIM_DUTY(duty));
	regmap_update_bits(r, ISINK_CON(MT6323_ISINK_CON1, ch),
			   ISINK_DIM_FSEL_MASK, ISINK_DIM_FSEL_20K);
	usleep_range(100, 300);
	regmap_update_bits(r, MT6323_ISINK_EN_CTRL, ISINK_CH_EN(ch),
			   ISINK_CH_EN(ch));
}

static void mt6323_isink_off(struct regmap *r, int ch)
{
	regmap_update_bits(r, MT6323_ISINK_EN_CTRL, ISINK_CH_EN(ch), 0);
	usleep_range(100, 300);
	regmap_update_bits(r, MT6323_ISINK_CKPDN2, ISINK_CK_PDN(ch),
			   ISINK_CK_PDN(ch));
}

static int mt6323_backlight_update(struct backlight_device *bl)
{
	struct mt6323_backlight *b = bl_get_data(bl);
	int level = backlight_get_brightness(bl);	/* 0 when blanked/off */
	unsigned int step = 0, duty = 0;
	int ch;

	if (level) {
		step = (level - 1) / ISINK_DUTY_MAX;
		duty = (level - 1) % ISINK_DUTY_MAX + 1;
	}

	for (ch = 0; ch < ISINK_NCHAN; ch++) {
		if (level)
			mt6323_isink_on(b->regmap, ch, step, duty);
		else
			mt6323_isink_off(b->regmap, ch);
	}
	return 0;
}

static const struct backlight_ops mt6323_backlight_ops = {
	.update_status = mt6323_backlight_update,
};

static int mt6323_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = BL_MAX_BRIGHTNESS,
		.brightness = BL_MAX_BRIGHTNESS,
	};
	struct mt6323_backlight *b;
	struct backlight_device *bl;

	b = devm_kzalloc(dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->regmap = dev_get_regmap(dev->parent, NULL);
	if (!b->regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");

	bl = devm_backlight_device_register(dev, "mt6323-backlight", dev, b,
					    &mt6323_backlight_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	backlight_update_status(bl);		/* apply initial (full on) */
	return 0;
}

static const struct of_device_id mt6323_backlight_of_match[] = {
	{ .compatible = "mediatek,mt6323-backlight" },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6323_backlight_of_match);

static struct platform_driver mt6323_backlight_driver = {
	.probe = mt6323_backlight_probe,
	.driver = {
		.name = "mt6323-backlight",
		.of_match_table = mt6323_backlight_of_match,
	},
};
module_platform_driver(mt6323_backlight_driver);

MODULE_DESCRIPTION("MediaTek MT6323 PMIC LCD backlight driver");
MODULE_LICENSE("GPL");
