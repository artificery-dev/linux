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
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/module.h>
#include <linux/mfd/mt6323/registers.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mt6582-u2.h>
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
#define RG_CS_VTH_1A		0x6		/* charge current ~1A */
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
/*
 * CHR_CON18/19: BC1.1 charger-port detection. The field layout was read off
 * the stock kernel's PMIC-wrapper traffic (tempo.old/build/hyptrace, the
 * DCD step sets CON19 0x180, 0x190, 0x191, 0x199 in the vendor's ipu, ipd,
 * vref, cmp order) and matches the vendor's upmu_hw.h for the MT6582+MT6323
 * (sprout), which also places VSRC_EN at CON18 [3:2] and the comparator's
 * output at CON18 bit 7.
 */
#define RG_BC11_BB_CTRL		BIT(0)
#define RG_BC11_RST		BIT(1)
#define RG_BC11_VSRC_EN_MASK	GENMASK(3, 2)	/* the 0.6 V source on D+ */
#define RGS_BC11_CMP_OUT	BIT(7)
#define RG_BC11_VREF_VTH_MASK	GENMASK(1, 0)
#define RG_BC11_CMP_EN_MASK	GENMASK(3, 2)
#define RG_BC11_IPD_EN_MASK	GENMASK(5, 4)
#define RG_BC11_IPU_EN_MASK	GENMASK(7, 6)
#define RG_BC11_FIELDS_MASK	GENMASK(7, 0)
#define RG_BC11_BIAS_EN		BIT(8)
/* The stock kernel's settle after arming the detector and after each step. */
#define BC11_ARM_MS		50
#define BC11_STEP_MS		80
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

/*
 * RG_CS_VTH code -> charge current limit in microamps (vendor CS_VTH[] table).
 * The register's reset value is 0xf, a 70mA trickle: less than the system
 * draws with the display on, so a pack "charging" at that code still drains.
 */
static const int mt6323_cs_vth_ua[16] = {
	1600000, 1500000, 1400000, 1300000, 1200000, 1100000, 1000000, 900000,
	800000, 700000, 650000, 550000, 450000, 300000, 200000, 70000,
};

/* The most this single-cell pack is ever asked to take, whatever userspace asks. */
#define CS_VTH_CODE_MAX_CURRENT	RG_CS_VTH_1A

struct mt6323_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *batt;
	struct delayed_work maint;
	unsigned int healthy_checks;
	unsigned int current_code;	/* RG_CS_VTH code the driver wants programmed */
	bool input_present;
	struct phy *phy;		/* the USB2 PHY whose D+/D- the detector borrows */
	enum power_supply_usb_type usb_type;
	bool detect_again;		/* debugfs asked for one more detection */
	struct dentry *debug;
};

/*
 * (Re)program the charge current limit. The PMIC does not keep CHR_CON4
 * across its own charge-path resets: after a latch-off the register was found
 * back at its 0xf reset value with CHR_EN happily set again, so the limit is
 * asserted on every enable and on every maintenance pass rather than once.
 */
static void mt6323_charger_assert_current(struct mt6323_charger *chg, bool report)
{
	unsigned int con4;

	if (regmap_read(chg->regmap, MT6323_CHR_CON4, &con4))
		return;
	if ((con4 & RG_CS_VTH_MASK) == chg->current_code)
		return;
	regmap_update_bits(chg->regmap, MT6323_CHR_CON4, RG_CS_VTH_MASK,
			   chg->current_code);
	if (report && !recovery_1a)
		dev_info_ratelimited(chg->dev,
			"charge current code was 0x%x, restored to 0x%x (%d mA)\n",
			con4 & RG_CS_VTH_MASK, chg->current_code,
			mt6323_cs_vth_ua[chg->current_code] / 1000);
}

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
	mt6323_charger_assert_current(chg, true);
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
	regmap_update_bits(r, MT6323_CHR_CON4, RG_CS_VTH_MASK, chg->current_code);

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

	dev_info(chg->dev, "MT6323 charger enabled (CV 4.2V, ~%dmA, OVP 7V/4.3V)\n",
		 mt6323_cs_vth_ua[chg->current_code] / 1000);
	return 0;
}

/*
 * One BC1.1 comparison: drive the lines as the step says, wait, read the
 * comparator, let go. The vendor's step bodies, in its own order of fields.
 */
static bool mt6323_bc11_step(struct mt6323_charger *chg, const char *name,
			     unsigned int vsrc, unsigned int ipu, unsigned int ipd,
			     unsigned int vref, unsigned int cmp)
{
	struct regmap *r = chg->regmap;
	unsigned int con18 = 0;

	regmap_update_bits(r, MT6323_CHR_CON18, RG_BC11_VSRC_EN_MASK,
			   FIELD_PREP(RG_BC11_VSRC_EN_MASK, vsrc));
	regmap_update_bits(r, MT6323_CHR_CON19, RG_BC11_FIELDS_MASK,
			   FIELD_PREP(RG_BC11_IPU_EN_MASK, ipu) |
			   FIELD_PREP(RG_BC11_IPD_EN_MASK, ipd) |
			   FIELD_PREP(RG_BC11_VREF_VTH_MASK, vref) |
			   FIELD_PREP(RG_BC11_CMP_EN_MASK, cmp));
	msleep(BC11_STEP_MS);
	regmap_read(r, MT6323_CHR_CON18, &con18);
	regmap_update_bits(r, MT6323_CHR_CON19, RG_BC11_FIELDS_MASK, 0);
	regmap_update_bits(r, MT6323_CHR_CON18, RG_BC11_VSRC_EN_MASK, 0);
	dev_dbg(chg->dev, "bc11 %s: CON18=0x%04x\n", name, con18);
	return con18 & RGS_BC11_CMP_OUT;
}

/*
 * Classify the port the way the stock kernel did on every plug: data-contact
 * detect first, then the primary comparison; a charging port gets the
 * secondary one to tell a dedicated charger from a charging host. Runs in the
 * maintenance worker, with the PHY's BC11 switch closed for the duration, so
 * it belongs before the gadget claims the lines - at boot and on a live plug.
 */
static enum power_supply_usb_type mt6323_bc11_detect(struct mt6323_charger *chg)
{
	struct regmap *r = chg->regmap;
	enum power_supply_usb_type type;

	if (!chg->phy)
		return POWER_SUPPLY_USB_TYPE_UNKNOWN;
	phy_set_mode_ext(chg->phy, PHY_MODE_USB_DEVICE, MT6582_U2PHY_BC11_SET);
	regmap_update_bits(r, MT6323_CHR_CON19, RG_BC11_BIAS_EN | RG_BC11_FIELDS_MASK,
			   RG_BC11_BIAS_EN);
	regmap_update_bits(r, MT6323_CHR_CON18, RG_BC11_BB_CTRL | RG_BC11_RST,
			   RG_BC11_BB_CTRL | RG_BC11_RST);
	msleep(BC11_ARM_MS);

	if (mt6323_bc11_step(chg, "dcd", 0, 2, 1, 1, 2)) {
		/* Contact never settled: a non-standard supply, or an Apple one. */
		type = mt6323_bc11_step(chg, "a1", 0, 0, 1, 0, 1)
			? POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
			: POWER_SUPPLY_USB_TYPE_UNKNOWN;
	} else if (!mt6323_bc11_step(chg, "a2", 2, 0, 1, 0, 1)) {
		/* Primary detection: 0.6 V on D+, sink on D-; a charger lifts D-. */
		type = POWER_SUPPLY_USB_TYPE_SDP;
	} else {
		type = mt6323_bc11_step(chg, "b2", 0, 2, 0, 1, 1)
			? POWER_SUPPLY_USB_TYPE_DCP
			: POWER_SUPPLY_USB_TYPE_CDP;
	}

	regmap_update_bits(r, MT6323_CHR_CON18, RG_BC11_BB_CTRL | RG_BC11_RST,
			   RG_BC11_BB_CTRL | RG_BC11_RST);
	regmap_update_bits(r, MT6323_CHR_CON19, RG_BC11_BIAS_EN | RG_BC11_FIELDS_MASK, 0);
	phy_set_mode_ext(chg->phy, PHY_MODE_USB_DEVICE, MT6582_U2PHY_BC11_CLR);
	return type;
}

static const char *const mt6323_usb_type_names[] = {
	[POWER_SUPPLY_USB_TYPE_UNKNOWN] = "non-standard",
	[POWER_SUPPLY_USB_TYPE_SDP] = "standard host (SDP)",
	[POWER_SUPPLY_USB_TYPE_DCP] = "dedicated charger (DCP)",
	[POWER_SUPPLY_USB_TYPE_CDP] = "charging host (CDP)",
	[POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID] = "Apple supply",
};

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
		if (recovery_1a) {
			chg->current_code = RG_CS_VTH_450MA;
			mt6323_charger_assert_current(chg, false);
		}
		goto reschedule;
	}
	if ((con0 & RGS_CHRDET) && (!chg->input_present || chg->detect_again)) {
		/* A live cable replug needs the full USB-download-release sequence. */
		if (recovery_1a && !chg->input_present) {
			mt6323_charger_hw_init(chg);
			chg->healthy_checks = 0;
		}
		chg->detect_again = false;
		chg->usb_type = mt6323_bc11_detect(chg);
		dev_info(chg->dev, "input is a %s\n",
			 mt6323_usb_type_names[chg->usb_type]);
		power_supply_changed(chg->psy);
	} else if (!(con0 & RGS_CHRDET) && chg->input_present) {
		chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		power_supply_changed(chg->psy);
	}
	chg->input_present = !!(con0 & RGS_CHRDET);
	if (recovery_1a)
		chg->current_code =
			mt6323_recovery_current_code(&chg->healthy_checks, con0, uv);
	mt6323_charger_assert_current(chg, true);
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
		val->intval = mt6323_cs_vth_ua[con0 & RG_CS_VTH_MASK];
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, MT6323_CHR_CON0, &con0);
		if (ret)
			return ret;
		val->intval = !!(con0 & RGS_CHRDET);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = chg->usb_type;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * Userspace picks the limit once it knows what the cable can give (a host port
 * that enumerated us, a wall supply that did not). The largest table entry not
 * above the request is programmed, capped at 1A; the recovery policy owns the
 * code while recovery_1a is set.
 */
static int mt6323_charger_set_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct mt6323_charger *chg = power_supply_get_drvdata(psy);
	unsigned int code;

	if (psp != POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX)
		return -EINVAL;
	if (recovery_1a)
		return -EBUSY;
	if (val->intval < mt6323_cs_vth_ua[ARRAY_SIZE(mt6323_cs_vth_ua) - 1])
		return -EINVAL;
	for (code = CS_VTH_CODE_MAX_CURRENT; code < ARRAY_SIZE(mt6323_cs_vth_ua); code++)
		if (mt6323_cs_vth_ua[code] <= val->intval)
			break;
	if (code == chg->current_code)
		return 0;
	chg->current_code = code;
	mt6323_charger_assert_current(chg, false);
	dev_info(chg->dev, "charge current limit set to %d mA\n",
		 mt6323_cs_vth_ua[code] / 1000);
	power_supply_changed(chg->psy);
	return 0;
}

static int mt6323_charger_prop_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX;
}

static enum power_supply_property mt6323_charger_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc mt6323_charger_desc = {
	.name		= "mt6323-charger",
	.type		= POWER_SUPPLY_TYPE_USB,
	.usb_types	= BIT(POWER_SUPPLY_USB_TYPE_SDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_CDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_DCP) |
			  BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID) |
			  BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.properties	= mt6323_charger_props,
	.num_properties	= ARRAY_SIZE(mt6323_charger_props),
	.get_property	= mt6323_charger_get_prop,
	.set_property	= mt6323_charger_set_prop,
	.property_is_writeable = mt6323_charger_prop_is_writeable,
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

/*
 * debugfs mt6323-charger/detect: write anything to run the port detection
 * again without a replug. Bring-up only: it borrows D+/D- for ~300 ms, so a
 * host that has us enumerated may see a glitch.
 */
static ssize_t mt6323_detect_write(struct file *file, const char __user *buf,
				   size_t len, loff_t *ppos)
{
	struct mt6323_charger *chg = file->private_data;

	chg->detect_again = true;
	mod_delayed_work(system_wq, &chg->maint, 0);
	return len;
}

static const struct file_operations mt6323_detect_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = mt6323_detect_write,
	.llseek = noop_llseek,
};

static void mt6323_charger_remove_debugfs(void *data)
{
	debugfs_remove_recursive(data);
}

static int mt6323_charger_probe(struct platform_device *pdev)
{
	struct mt6323_charger *chg;
	struct power_supply_config cfg = {};
	int ret;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->current_code = RG_CS_VTH_450MA;
	chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	chg->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "no PMIC regmap\n");
	chg->phy = devm_phy_optional_get(&pdev->dev, "usb");
	if (IS_ERR(chg->phy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->phy), "no USB PHY\n");
	if (!chg->phy)
		dev_info(&pdev->dev, "no USB PHY: charger ports go undetected\n");

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
	/* The first pass runs the port detection before the gadget comes up. */
	schedule_delayed_work(&chg->maint, 0);

	chg->debug = debugfs_create_dir("mt6323-charger", NULL);
	if (!IS_ERR_OR_NULL(chg->debug)) {
		debugfs_create_file("detect", 0200, chg->debug, chg,
				    &mt6323_detect_fops);
		devm_add_action_or_reset(&pdev->dev,
					 mt6323_charger_remove_debugfs, chg->debug);
	}

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
