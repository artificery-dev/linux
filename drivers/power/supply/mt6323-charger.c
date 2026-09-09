// SPDX-License-Identifier: GPL-2.0
/*
 * Battery charger driver for the MediaTek MT6323 PMIC (as used on the MT6582).
 *
 * The MT6323 has a hardware constant-voltage charger. This driver binds as a
 * child of the pwrap PMIC bus (grabbing its regmap directly, so it does NOT
 * need the mt6397 MFD or the PMIC EINT), programs safe limits, and enables the
 * hardware charge loop; the PMIC then does CC/CV autonomously.
 *
 * A periodic maintenance worker re-asserts the charge-enable path: the MT6323's
 * charge state machine can latch CSDAC_EN+CHR_EN back off (an under-low-current
 * / CV excursion transient) minutes into a charge, and with a one-shot enable
 * that leaves the device drawing only its ~30mA system load forever. The vendor
 * runs a charging daemon that re-enables periodically; this worker is the
 * mainline equivalent. It also exposes battery level + charge status via
 * power_supply.
 *
 * Register/field map + the BATSNS AUXADC channel derived + cross-validated from
 * the vendor sources in references/vendor-pmic/ (charging_hw_pmic.c,
 * pmic_mt6323.c, upmu_hw.h).
 */
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/module.h>
#include <linux/mfd/mt6323/registers.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include "mt6323-charge-policy.h"

/* CHR_CON0 */
#define RG_CSDAC_EN		BIT(3)
#define RG_CHR_EN		BIT(4)
/* CHR_CON1 */
#define RG_VCDT_HV_EN		BIT(0)
#define RG_VCDT_HV_VTH_MASK	GENMASK(7, 4)
#define RG_VCDT_HV_VTH_7V	(0xb << 4)	/* input OVP 7V */
/* CHR_CON2 */
#define RG_VBAT_CV_EN		BIT(1)
#define RG_CS_EN		BIT(3)
/* CHR_CON3 */
#define RG_VBAT_CV_VTH_MASK	GENMASK(4, 0)
#define RG_VBAT_CV_VTH_4_2V	0x0		/* constant-voltage target 4.2V */
/* CHR_CON4 */
#define RG_CS_VTH_MASK		GENMASK(3, 0)
#define RG_CS_VTH_450MA		0xc		/* charge current ~450mA */
/* CHR_CON6 */
#define RG_VBAT_OV_EN		BIT(0)
#define RG_VBAT_OV_VTH_MASK	GENMASK(3, 1)
#define RG_VBAT_OV_VTH_4_3V	(0x1 << 1)	/* battery OVP 4.3V */
/* CHR_CON13: charger watchdog */
#define RG_CHRWDT_TD_MASK	GENMASK(3, 0)	/* timeout duration; 0 = 4s */
#define RG_CHRWDT_EN		BIT(4)		/* arm the WDT */
#define RG_CHRWDT_WR		BIT(8)		/* commit/kick (petting) */
/* CHR_CON15: watchdog interrupt-enable + flag */
#define RG_CHRWDT_INT_EN	BIT(0)
#define RG_CHRWDT_FLAG_WR	BIT(1)
/* CHR_CON23 */
#define RG_CSDAC_MODE		BIT(2)
#define RG_HWCV_EN		BIT(6)
#define RG_ULC_DET_EN		BIT(7)
/* CHR_CON0 status: charger detect lives in the CON0/status region */
#define RGS_CHRDET		BIT(5)		/* CHR_CON0 charger-present status */
/* CHR_CON7: battery presence detection */
#define RG_BATON_EN		BIT(0)
#define RG_BATON_HT_EN		BIT(1)
/* CHR_CON16: USB download mode */
#define RG_USBDL_RST		BIT(2)
#define RG_USBDL_SET		BIT(3)
/* CHR_CON18: BC1.1 charger-port detection */
#define RG_BC11_BB_CTRL		BIT(0)
#define RG_BC11_RST		BIT(1)
/* CHR_CON20: current-source DAC soft-start step inc/dec */
#define RG_CSDAC_STP_INC_MASK	GENMASK(2, 0)
#define RG_CSDAC_STP_DEC_MASK	GENMASK(6, 4)
/* CHR_CON21: current-source DAC delay + step */
#define RG_CSDAC_DLY_MASK	GENMASK(2, 0)
#define RG_CSDAC_STP_MASK	GENMASK(6, 4)
/* CHR_CON22: low charge-current debounce */
#define RG_LOW_ICH_DB_MASK	GENMASK(5, 0)
/* CHR_CON23: VCDT mode (paired with CSDAC_MODE/HWCV/ULC already defined) */
#define RG_VCDT_MODE		BIT(1)

/*
 * PMIC AUXADC battery-voltage read (BATSNS). AP-immediate: pulse a 0->1 edge on
 * BATSNS's request bit (ch 7) in AUXADC_CON22 RQST_LIST, then poll its RESULT
 * register ADC0 for bit15=RDY and read bits[14:0]. Vendor pmic_mt6323.c uses
 * r_val 4: Volt(mV) = raw * 4 * VOLTAGE_FULL_RANGE(1800) / ADC_PRECISE(32768).
 */
#define AUXADC_RDY		BIT(15)
#define AUXADC_VAL_MASK		GENMASK(14, 0)
#define AUXADC_CH_BATSNS	7

/* Maintenance worker period. Must be < the 4s charge-watchdog timeout so the
 * worker's pet keeps it from firing and cutting CHR_EN. */
#define CHG_MAINT_MS		2000

/* Opt-in recovery policy, ported from hyptrace's tested charge gate. */
static bool recovery_1a;
module_param(recovery_1a, bool, 0444);
MODULE_PARM_DESC(recovery_1a, "Recovery: promote 450mA to 1A after healthy checks");

struct mt6323_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *batt;
	struct delayed_work maint;
	unsigned int healthy_checks;
	bool input_present;
};

/* One AUXADC conversion. Returns the 15-bit raw sample or a negative errno. */
static int mt6323_auxadc_raw(struct mt6323_charger *chg, unsigned int ch,
			     unsigned int adc_reg)
{
	struct regmap *r = chg->regmap;
	unsigned int val;
	int ret, timeout = 100;

	regmap_update_bits(r, MT6323_AUXADC_CON22, BIT(ch), 0);
	regmap_update_bits(r, MT6323_AUXADC_CON22, BIT(ch), BIT(ch));

	do {
		ret = regmap_read(r, adc_reg, &val);
		if (ret)
			return ret;
		if (val & AUXADC_RDY)
			return val & AUXADC_VAL_MASK;
		usleep_range(200, 400);
	} while (--timeout);

	return -ETIMEDOUT;
}

/* Battery voltage in microvolts (BATSNS, r_val 4: mV = raw*4*1800/32768). */
static int mt6323_vbat_uv(struct mt6323_charger *chg)
{
	int raw = mt6323_auxadc_raw(chg, AUXADC_CH_BATSNS, MT6323_AUXADC_ADC0);

	if (raw < 0)
		return raw;
	return raw * 225 / 1024 * 1000;
}

/* Rough open-circuit-voltage -> percent curve (elevated while charging). */
static int mt6323_capacity(struct mt6323_charger *chg)
{
	static const struct { int uv, pct; } ocv[] = {
		{ 3300000, 0 }, { 3600000, 15 }, { 3700000, 30 }, { 3750000, 45 },
		{ 3800000, 58 }, { 3850000, 68 }, { 3900000, 76 }, { 4000000, 88 },
		{ 4100000, 96 }, { 4200000, 100 },
	};
	int uv = mt6323_vbat_uv(chg), k;

	if (uv < 0)
		return uv;
	if (uv <= ocv[0].uv)
		return 0;
	for (k = 1; k < ARRAY_SIZE(ocv); k++) {
		if (uv < ocv[k].uv)
			return ocv[k - 1].pct +
			       (ocv[k].pct - ocv[k - 1].pct) * (uv - ocv[k - 1].uv) /
			       (ocv[k].uv - ocv[k - 1].uv);
	}
	return 100;
}

/*
 * Pet the charge watchdog (vendor charging_reset_watch_dog_timer). On this HW
 * clearing RG_CHRWDT_EN does NOT stop the 4s WDT - it keeps firing and the
 * hardware clears CHR_EN (the ~5s relatch, INT_STATUS0 bit4 latched). So instead
 * keep it armed and kick it faster than its timeout, exactly like the vendor.
 */
static void mt6323_charger_kick_wdt(struct regmap *r)
{
	regmap_update_bits(r, MT6323_CHR_CON13, RG_CHRWDT_TD_MASK, 0);		/* 4s */
	regmap_update_bits(r, MT6323_CHR_CON13, RG_CHRWDT_WR, RG_CHRWDT_WR);
	regmap_update_bits(r, MT6323_CHR_CON15, RG_CHRWDT_INT_EN, RG_CHRWDT_INT_EN);
	regmap_update_bits(r, MT6323_CHR_CON13, RG_CHRWDT_EN, RG_CHRWDT_EN);
	regmap_update_bits(r, MT6323_CHR_CON15, RG_CHRWDT_FLAG_WR, RG_CHRWDT_FLAG_WR);
}

/*
 * Re-assert the charge-enable path (idempotent). Split from the one-time
 * safety-limit setup so the maintenance worker can call it to recover from a
 * latched-off charge state.
 */
static void mt6323_charger_enable(struct mt6323_charger *chg)
{
	struct regmap *r = chg->regmap;

	/*
	 * Arm the current-source DAC soft-start BEFORE enabling the charger, in
	 * the vendor charging_enable order. Without this stepping config the CSDAC
	 * never ramps current, so the charge FSM immediately drops CHR_EN again -
	 * the ~1s relatch we saw. Values from vendor charging_enable_pmic().
	 */
	regmap_update_bits(r, MT6323_CHR_CON21, RG_CSDAC_DLY_MASK, 0x4 << 0);
	regmap_update_bits(r, MT6323_CHR_CON21, RG_CSDAC_STP_MASK, 0x1 << 4);
	regmap_update_bits(r, MT6323_CHR_CON20, RG_CSDAC_STP_INC_MASK, 0x1 << 0);
	regmap_update_bits(r, MT6323_CHR_CON20, RG_CSDAC_STP_DEC_MASK, 0x2 << 4);
	regmap_update_bits(r, MT6323_CHR_CON2, RG_CS_EN, RG_CS_EN);
	regmap_update_bits(r, MT6323_CHR_CON23, RG_HWCV_EN, RG_HWCV_EN);
	regmap_update_bits(r, MT6323_CHR_CON2, RG_VBAT_CV_EN, RG_VBAT_CV_EN);
	regmap_update_bits(r, MT6323_CHR_CON0, RG_CSDAC_EN, RG_CSDAC_EN);
	regmap_update_bits(r, MT6323_CHR_CON0, RG_CHR_EN, RG_CHR_EN);
}

static int mt6323_charger_hw_init(struct mt6323_charger *chg)
{
	struct regmap *r = chg->regmap;

	/*
	 * Charger watchdog: clearing RG_CHRWDT_EN does NOT stop it on this HW - it
	 * keeps timing out (~4s) and the hardware clears CHR_EN, which was the whole
	 * relatch. So arm it (vendor charging_hw_init) and have the maintenance
	 * worker pet it faster than the timeout instead.
	 */
	mt6323_charger_kick_wdt(r);

	/* Safety limits (set once): input OVP 7V, battery OVP 4.3V, CV 4.2V. */
	regmap_update_bits(r, MT6323_CHR_CON1, RG_VCDT_HV_VTH_MASK, RG_VCDT_HV_VTH_7V);
	regmap_update_bits(r, MT6323_CHR_CON1, RG_VCDT_HV_EN, RG_VCDT_HV_EN);
	regmap_update_bits(r, MT6323_CHR_CON6, RG_VBAT_OV_VTH_MASK, RG_VBAT_OV_VTH_4_3V);
	regmap_update_bits(r, MT6323_CHR_CON6, RG_VBAT_OV_EN, RG_VBAT_OV_EN);
	regmap_update_bits(r, MT6323_CHR_CON3, RG_VBAT_CV_VTH_MASK, RG_VBAT_CV_VTH_4_2V);
	regmap_update_bits(r, MT6323_CHR_CON4, RG_CS_VTH_MASK, RG_CS_VTH_450MA);

	/*
	 * Charger-port + battery detection (vendor charging_hw_init). Leaving these
	 * out kept the charge FSM from sustaining CHR_EN: it needs BC1.1 charger-port
	 * detection and BATON battery-presence detection, out of USB-download mode.
	 */
	regmap_update_bits(r, MT6323_CHR_CON16, RG_USBDL_SET, 0);
	regmap_update_bits(r, MT6323_CHR_CON16, RG_USBDL_RST, RG_USBDL_RST);
	regmap_update_bits(r, MT6323_CHR_CON18, RG_BC11_BB_CTRL, RG_BC11_BB_CTRL);
	regmap_update_bits(r, MT6323_CHR_CON18, RG_BC11_RST, RG_BC11_RST);
	regmap_update_bits(r, MT6323_CHR_CON7, RG_BATON_EN, RG_BATON_EN);
	regmap_update_bits(r, MT6323_CHR_CON7, RG_BATON_HT_EN, 0);
	regmap_update_bits(r, MT6323_CHR_CON22, RG_LOW_ICH_DB_MASK, 0x1);
	regmap_update_bits(r, MT6323_CHR_CON23, RG_VCDT_MODE, 0);

	/*
	 * CSDAC mode + ULC (under-low-current) detect, per vendor init. ULC must be
	 * ENABLED: empirically, with ULC enabled the charge path holds and the pack
	 * charges (a sustained multi-minute climb was observed); DISABLING it made
	 * the hardware clear CHR_EN within ~1s every time, so the maintenance worker
	 * re-enabled it every 5s forever ("charge path latched, re-enabling" flap,
	 * stuck at ~30mA). The vendor enables ULC too. The worker's vbat guard still
	 * prevents fighting a legitimately-full pack.
	 */
	regmap_update_bits(r, MT6323_CHR_CON23, RG_CSDAC_MODE, RG_CSDAC_MODE);
	regmap_update_bits(r, MT6323_CHR_CON23, RG_ULC_DET_EN, RG_ULC_DET_EN);

	/* Enable the current source + hardware CV loop, then the charger. */
	mt6323_charger_enable(chg);

	dev_info(chg->dev, "MT6323 charger enabled (CV 4.2V, ~450mA, OVP 7V/4.3V)\n");
	return 0;
}

static void mt6323_charger_maint_work(struct work_struct *work)
{
	struct mt6323_charger *chg =
		container_of(to_delayed_work(work), struct mt6323_charger, maint);
	unsigned int con0;
	int uv;

	mt6323_charger_kick_wdt(chg->regmap);
	uv = mt6323_vbat_uv(chg);
	if (regmap_read(chg->regmap, MT6323_CHR_CON0, &con0)) {
		chg->healthy_checks = 0;
		if (recovery_1a)
			regmap_update_bits(chg->regmap, MT6323_CHR_CON4,
					   RG_CS_VTH_MASK, RG_CS_VTH_450MA);
		goto reschedule;
	}
	if (recovery_1a) {
		/* A live cable replug needs the full USB-download-release sequence. */
		if ((con0 & RGS_CHRDET) && !chg->input_present) {
			mt6323_charger_hw_init(chg);
			chg->healthy_checks = 0;
		}
		chg->input_present = !!(con0 & RGS_CHRDET);
		regmap_update_bits(chg->regmap, MT6323_CHR_CON4, RG_CS_VTH_MASK,
			mt6323_recovery_current_code(&chg->healthy_checks, con0, uv));
	}
	/* Never interpret an ADC error as a depleted battery. */
	if ((con0 & RGS_CHRDET) && !(con0 & RG_CHR_EN) &&
	    uv >= 0 && uv < 4150000 &&
	    (!recovery_1a || ((con0 & BIT(6)) && !(con0 & BIT(7))))) {
		mt6323_charger_enable(chg);
		dev_info_ratelimited(chg->dev, "charge path had latched off; re-enabled\n");
		power_supply_changed(chg->psy);
	}
reschedule:
	schedule_delayed_work(&chg->maint, msecs_to_jiffies(CHG_MAINT_MS));
}

static int mt6323_charger_get_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6323_charger *chg = power_supply_get_drvdata(psy);
	unsigned int con0;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = regmap_read(chg->regmap, MT6323_CHR_CON4, &con0);
		if (ret)
			return ret;
		/* Report the programmed limit, not a measured charging current. */
		if ((con0 & RG_CS_VTH_MASK) == 0x6)
			val->intval = 1000000;
		else if ((con0 & RG_CS_VTH_MASK) == RG_CS_VTH_450MA)
			val->intval = 450000;
		else
			return -ENODATA;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, MT6323_CHR_CON0, &con0);
		if (ret)
			return ret;
		val->intval = !!(con0 & RGS_CHRDET);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property mt6323_charger_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc mt6323_charger_desc = {
	.name		= "mt6323-charger",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= mt6323_charger_props,
	.num_properties	= ARRAY_SIZE(mt6323_charger_props),
	.get_property	= mt6323_charger_get_prop,
};

static int mt6323_battery_get_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6323_charger *chg = power_supply_get_drvdata(psy);
	unsigned int con0;
	int v;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		v = mt6323_vbat_uv(chg);
		if (v < 0)
			return v;
		val->intval = v;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		v = mt6323_capacity(chg);
		if (v < 0)
			return v;
		val->intval = v;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (regmap_read(chg->regmap, MT6323_CHR_CON0, &con0)) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		if (!(con0 & RGS_CHRDET))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (mt6323_vbat_uv(chg) >= 4150000)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (con0 & RG_CHR_EN)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property mt6323_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc mt6323_battery_desc = {
	.name		= "mt6323-battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= mt6323_battery_props,
	.num_properties	= ARRAY_SIZE(mt6323_battery_props),
	.get_property	= mt6323_battery_get_prop,
};

static int mt6323_charger_probe(struct platform_device *pdev)
{
	struct mt6323_charger *chg;
	struct power_supply_config cfg = {};
	int ret;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "no PMIC regmap\n");

	cfg.drv_data = chg;
	cfg.of_node = pdev->dev.of_node;
	chg->psy = devm_power_supply_register(&pdev->dev, &mt6323_charger_desc, &cfg);
	if (IS_ERR(chg->psy))
		return PTR_ERR(chg->psy);

	chg->batt = devm_power_supply_register(&pdev->dev, &mt6323_battery_desc, &cfg);
	if (IS_ERR(chg->batt))
		return PTR_ERR(chg->batt);

	ret = mt6323_charger_hw_init(chg);
	if (ret)
		return ret;

	/* Keep the charge path alive against the intermittent HW latch-off. */
	ret = devm_delayed_work_autocancel(&pdev->dev, &chg->maint,
					   mt6323_charger_maint_work);
	if (ret)
		return ret;
	schedule_delayed_work(&chg->maint, msecs_to_jiffies(CHG_MAINT_MS));

	return 0;
}

static const struct of_device_id mt6323_charger_of_match[] = {
	{ .compatible = "mediatek,mt6323-charger" },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6323_charger_of_match);

static struct platform_driver mt6323_charger_driver = {
	.probe = mt6323_charger_probe,
	.driver = {
		.name = "mt6323-charger",
		.of_match_table = mt6323_charger_of_match,
	},
};
module_platform_driver(mt6323_charger_driver);

MODULE_DESCRIPTION("MediaTek MT6323 PMIC charger driver");
MODULE_LICENSE("GPL");
