// SPDX-License-Identifier: GPL-2.0
/*
 * MT6582 CONSYS hardware control, mainline port of the vendor
 * arch/arm/mach-mt6582/<board>/wmt/mtk_wcn_consys_hw.c (MediaTek, GPL-2.0).
 *
 * Same sequence as the vendor code, on mainline plumbing: the MT6323 rails
 * through the regulator framework plus the pwrap regmap for the LDO
 * "hardware control" bits the regulator driver does not model, the CONN
 * MTCMOS switched here with the vendor spm_mtcmos_ctrl_connsys() steps (the
 * clkmgr's conn_power_on/off), the CONNMCU infra gate through CCF, and the
 * CONSYS EMI window from a reserved-memory region instead of a stolen
 * memblock (no EMI MPU protection; nothing else touches that memory).
 */
#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG "[WMT-CONSYS-HW]"

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mfd/mt6323/registers.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include "mtk_wcn_consys_hw.h"
#include "consys_plat.h"
#include <mach/mt_clkmgr.h>

/* SPM (0x10006000) */
#define SPM_POWERON_CONFIG_EN	0x0000
#define  SPM_PROJECT_CODE	(0xb16 << 16)
#define  SPM_REGWR_EN		BIT(0)
#define SPM_CONN_PWR_CON	0x0280
#define SPM_PWR_STATUS		0x060c
#define SPM_PWR_STATUS_S	0x0610
#define  PWR_RST_B		BIT(0)
#define  PWR_ISO		BIT(1)
#define  PWR_ON			BIT(2)
#define  PWR_ON_2ND		BIT(3)
#define  PWR_CLK_DIS		BIT(4)
#define  CONN_SRAM_PDN		BIT(8)
#define  CONN_PWR_STA		BIT(1)
/* INFRACFG_AO (0x10001000) */
#define INFRA_TOPAXI_PROT_EN	0x0220
#define INFRA_TOPAXI_PROT_STA1	0x0228
#define  CONN_PROT_MASK		0x0104

/* MT6323 bits the vendor drove with upmu_set_*() */
#define VCN28_ON_CTRL		BIT(14)	/* ANALDO_CON19 (0x41c): 1 = HW mode */
#define VCN33_ON_CTRL_BT	BIT(5)	/* ANALDO_CON16 (0x416) */
#define VCN33_ON_CTRL_WIFI	BIT(14)	/* ANALDO_CON17 (0x418) */
#define VCN_1V8_LP_MODE_SET	BIT(1)	/* DIGLDO_CON11 (0x512) */

UINT8 __iomem *pEmibaseaddr;
phys_addr_t gConEmiPhyBase;

static bool vcn18_on, vcn28_on, vcn33_bt_on, vcn33_wifi_on;

/* bring-up knobs (/sys/module/mtk_wcn_consys_hw/parameters/): */
/* Keep VCN28 on even in co-clock mode. Not needed on this board: the stock
 * kernel's dct_pmic_VCN28_enable_bt is a vendor misnomer - the trace shows it
 * selecting VCN33 to 3300mV, which is the rail we already drive. */
static bool force_vcn28;
module_param(force_vcn28, bool, 0644);
static bool ap2conn_osc;	/* set AP2CONN_OSC_EN (0x10001f00 bit 10) at power-on */
module_param(ap2conn_osc, bool, 0644);
static bool pmic_hw_mode = true;	/* vendor PMIC init: TOP_CKCON 0x120 SRCLKEN/OSC in HW mode */
module_param(pmic_hw_mode, bool, 0644);
static unsigned int pmic_clock_settle_ms = 100;
module_param(pmic_clock_settle_ms, uint, 0644);
static bool paldo_sw;		/* keep VCN33_BT/WIFI under software control (no HW request line) */
module_param(paldo_sw, bool, 0644);
static bool force_vcn33_wifi;	/* keep the WiFi PA rail on while BT is on */
module_param(force_vcn33_wifi, bool, 0644);
#define MT6323_TOP_CKCON_120	0x120
#define  RG_SRCLKEN_HW_MODE	BIT(4)
#define  RG_OSC_HW_MODE		BIT(5)

static bool pmic_clock_prepared;
static unsigned long pmic_clock_ready_jiffies;

static int consys_pmic_clock_field(u16 reg, u16 mask, u16 val)
{
	int ret;

	ret = regmap_update_bits(consys_plat.pmic, reg, mask, val);
	/* Stock's pmic_config_interface() performs each field as an independent
	 * PWRAP transaction.  Keep that ordering visible to the PMIC instead of
	 * collapsing adjacent fields into one update. */
	udelay(5);
	return ret;
}

INT32 mtk_wcn_consys_hw_pmic_init(void)
{
	struct consys_plat *p = &consys_plat;
	u32 ckpdn, ckcon, rst, vtcxo;
	int ret;

	if (!pmic_hw_mode || pmic_clock_prepared)
		return 0;
	if (!p->pmic)
		return -ENODEV;

	/* Exact connectivity-clock subset and field order from MT6323
	 * PMIC_INIT_SETTING_V1().  Stock runs this from the PMIC fs_initcall,
	 * seconds before the first CONSYS power-on. */
	ret = consys_pmic_clock_field(0x102, BIT(6), BIT(6));
	if (!ret)
		ret = consys_pmic_clock_field(0x102, BIT(11), 0);
	if (!ret)
		ret = consys_pmic_clock_field(0x102, BIT(15), BIT(15));
	if (!ret)
		ret = consys_pmic_clock_field(MT6323_TOP_CKCON_120,
					       RG_SRCLKEN_HW_MODE,
					       RG_SRCLKEN_HW_MODE);
	if (!ret)
		ret = consys_pmic_clock_field(MT6323_TOP_CKCON_120,
					       RG_OSC_HW_MODE,
					       RG_OSC_HW_MODE);
	if (!ret)
		ret = consys_pmic_clock_field(0x148, BIT(1), BIT(1));
	if (!ret)
		ret = consys_pmic_clock_field(0x148, BIT(3), BIT(3));
	if (!ret)
		ret = consys_pmic_clock_field(0x402, BIT(0), BIT(0));
	if (!ret)
		ret = consys_pmic_clock_field(0x402, BIT(11), 0);
	if (ret) {
		WMT_PLAT_ERR_FUNC("early PMIC clock setup failed (%d)\n", ret);
		return ret;
	}

	pmic_clock_ready_jiffies = jiffies;
	pmic_clock_prepared = true;
	regmap_read(p->pmic, 0x102, &ckpdn);
	regmap_read(p->pmic, MT6323_TOP_CKCON_120, &ckcon);
	regmap_read(p->pmic, 0x148, &rst);
	regmap_read(p->pmic, 0x402, &vtcxo);
	WMT_PLAT_INFO_FUNC("early PMIC clock setup: 102=%04x 120=%04x 148=%04x 402=%04x, settle %u ms\n",
			   ckpdn, ckcon, rst, vtcxo, pmic_clock_settle_ms);
	return 0;
}

static void consys_pmic_clock_wait(void)
{
	unsigned long deadline, remaining;

	if (!pmic_clock_prepared || !pmic_clock_settle_ms)
		return;
	deadline = pmic_clock_ready_jiffies +
		msecs_to_jiffies(pmic_clock_settle_ms);
	if (!time_before(jiffies, deadline))
		return;
	remaining = deadline - jiffies;
	WMT_PLAT_INFO_FUNC("waiting %u ms for early PMIC clock setup\n",
			   jiffies_to_msecs(remaining));
	msleep(jiffies_to_msecs(remaining) + 1);
}

static int consys_rail(struct regulator *r, bool *state, bool on)
{
	int ret = 0;

	if (on && !*state) {
		ret = regulator_enable(r);
		if (!ret)
			*state = true;
	} else if (!on && *state) {
		ret = regulator_disable(r);
		if (!ret)
			*state = false;
	}
	return ret;
}

/* The vendor clkmgr's spm_mtcmos_ctrl_connsys(). */
static int consys_mtcmos_ctrl(bool on)
{
	void __iomem *spm = consys_io_spm, *infra = consys_io_infra;
	u32 v;
	int ret;

	writel(SPM_PROJECT_CODE | SPM_REGWR_EN, spm + SPM_POWERON_CONFIG_EN);
	WMT_PLAT_INFO_FUNC("MTCMOS %s: CONN_PWR_CON 0x%x PWR_STATUS 0x%x/0x%x PROT_EN 0x%x\n",
			   on ? "on" : "off", readl(spm + SPM_CONN_PWR_CON),
			   readl(spm + SPM_PWR_STATUS), readl(spm + SPM_PWR_STATUS_S),
			   readl(infra + INFRA_TOPAXI_PROT_EN));
	if (on) {
		writel(readl(spm + SPM_CONN_PWR_CON) | PWR_ON, spm + SPM_CONN_PWR_CON);
		writel(readl(spm + SPM_CONN_PWR_CON) | PWR_ON_2ND, spm + SPM_CONN_PWR_CON);
		ret = readl_poll_timeout(spm + SPM_PWR_STATUS, v, v & CONN_PWR_STA, 10, 10000);
		if (!ret)
			ret = readl_poll_timeout(spm + SPM_PWR_STATUS_S, v, v & CONN_PWR_STA, 10, 10000);
		if (ret) {
			WMT_PLAT_ERR_FUNC("CONN power ack timeout (0x%x/0x%x)\n",
					  readl(spm + SPM_PWR_STATUS), readl(spm + SPM_PWR_STATUS_S));
			return ret;
		}
		writel(readl(spm + SPM_CONN_PWR_CON) & ~PWR_CLK_DIS, spm + SPM_CONN_PWR_CON);
		writel(readl(spm + SPM_CONN_PWR_CON) & ~PWR_ISO, spm + SPM_CONN_PWR_CON);
		writel(readl(spm + SPM_CONN_PWR_CON) | PWR_RST_B, spm + SPM_CONN_PWR_CON);
		writel(readl(spm + SPM_CONN_PWR_CON) & ~CONN_SRAM_PDN, spm + SPM_CONN_PWR_CON);
		writel(readl(infra + INFRA_TOPAXI_PROT_EN) & ~CONN_PROT_MASK, infra + INFRA_TOPAXI_PROT_EN);
		ret = readl_poll_timeout(infra + INFRA_TOPAXI_PROT_STA1, v, !(v & CONN_PROT_MASK), 10, 10000);
		if (ret)
			WMT_PLAT_ERR_FUNC("CONN AXI protection release timeout (0x%x)\n",
					  readl(infra + INFRA_TOPAXI_PROT_STA1));
		return ret;
	}

	writel(readl(infra + INFRA_TOPAXI_PROT_EN) | CONN_PROT_MASK, infra + INFRA_TOPAXI_PROT_EN);
	if (readl_poll_timeout(infra + INFRA_TOPAXI_PROT_STA1, v,
			       (v & CONN_PROT_MASK) == CONN_PROT_MASK, 10, 10000))
		WMT_PLAT_WARN_FUNC("CONN AXI protection set timeout\n");
	writel(readl(spm + SPM_CONN_PWR_CON) | CONN_SRAM_PDN, spm + SPM_CONN_PWR_CON);
	writel(readl(spm + SPM_CONN_PWR_CON) | PWR_ISO, spm + SPM_CONN_PWR_CON);
	v = readl(spm + SPM_CONN_PWR_CON);
	writel((v & ~PWR_RST_B) | PWR_CLK_DIS, spm + SPM_CONN_PWR_CON);
	writel(readl(spm + SPM_CONN_PWR_CON) & ~(PWR_ON | PWR_ON_2ND), spm + SPM_CONN_PWR_CON);
	if (readl_poll_timeout(spm + SPM_PWR_STATUS, v, !(v & CONN_PWR_STA), 10, 10000) ||
	    readl_poll_timeout(spm + SPM_PWR_STATUS_S, v, !(v & CONN_PWR_STA), 10, 10000))
		WMT_PLAT_WARN_FUNC("CONN power-down ack timeout\n");
	WMT_PLAT_INFO_FUNC("MTCMOS off done: CONN_PWR_CON 0x%x PWR_STATUS 0x%x/0x%x\n",
			   readl(spm + SPM_CONN_PWR_CON), readl(spm + SPM_PWR_STATUS),
			   readl(spm + SPM_PWR_STATUS_S));
	return 0;
}

INT32 mtk_wcn_consys_hw_reg_ctrl(UINT32 on, UINT32 co_clock_en)
{
	struct consys_plat *p = &consys_plat;
	INT32 iRet;
	UINT32 retry = 10;
	UINT32 consysHwChipId = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW-REG-CTRL(0x%08x),start\n", on);
	if (!p->ready) {
		WMT_PLAT_ERR_FUNC("CONSYS platform device not probed\n");
		return -ENODEV;
	}
	if (on) {
		/* Probe normally performed this well before WMT can request power.
		 * Retain a fallback for unusual built-as-module/deferred-probe order. */
		if (pmic_hw_mode && !pmic_clock_prepared) {
			iRet = mtk_wcn_consys_hw_pmic_init();
			if (iRet)
				return iRet;
		}
		consys_pmic_clock_wait();
		/* 1. VCN_1V8 LDO on, out of low-power mode (0x512[1] = 0) */
		regmap_update_bits(p->pmic, MT6323_DIGLDO_CON11, VCN_1V8_LP_MODE_SET, 0);
		iRet = consys_rail(p->vcn18, &vcn18_on, true);
		if (iRet)
			WMT_PLAT_ERR_FUNC("vcn18 enable failed (%d)\n", iRet);
		if (co_clock_en && !force_vcn28) {
			/* 2. VCN28 in SW control mode (0x41c[14] = 0): the
			 * co-clock case, the 26 MHz comes from the AP. */
			regmap_update_bits(p->pmic, MT6323_ANALDO_CON19, VCN28_ON_CTRL, 0);
		} else {
			/* 2.1 VCN28 in HW control mode (0x41c[14] = 1), 2.2 on */
			regmap_update_bits(p->pmic, MT6323_ANALDO_CON19, VCN28_ON_CTRL, VCN28_ON_CTRL);
			iRet = consys_rail(p->vcn28, &vcn28_on, true);
			if (iRet)
				WMT_PLAT_ERR_FUNC("vcn28 enable failed (%d)\n", iRet);
		}
		/* (the CONSYS CPU SW reset and the AFE CR writes moved into the
		 * firmware patch in the vendor tree - ALPS00544691 - so not here) */
		if (force_vcn33_wifi)
			consys_rail(p->vcn33_wifi, &vcn33_wifi_on, true);
		iRet = consys_mtcmos_ctrl(true);
		if (iRet)
			return iRet;
		WMT_PLAT_INFO_FUNC("reg dump:CONSYS_PWR_CONN_ACK_REG(0x%x)\n", CONSYS_REG_READ(CONSYS_PWR_CONN_ACK_REG));
		WMT_PLAT_INFO_FUNC("reg dump:CONSYS_PWR_CONN_ACK_S_REG(0x%x)\n", CONSYS_REG_READ(CONSYS_PWR_CONN_ACK_S_REG));
		WMT_PLAT_INFO_FUNC("reg dump:CONSYS_TOP1_PWR_CTRL_REG(0x%x)\n", CONSYS_REG_READ(CONSYS_TOP1_PWR_CTRL_REG));
		if (ap2conn_osc) {
			CONSYS_REG_WRITE(CONSYS_AP2CONN_OSC_EN_REG, CONSYS_REG_READ(CONSYS_AP2CONN_OSC_EN_REG) | CONSYS_AP2CONN_OSC_EN_BIT);
			WMT_PLAT_INFO_FUNC("AP2CONN_OSC_EN set: 0x%x\n", CONSYS_REG_READ(CONSYS_AP2CONN_OSC_EN_REG));
		}
		/* 11. delay 10us, 26M is ready */
		udelay(10);
		enable_clock(MT_CG_INFRA_CONNMCU, "WMT_MOD");
		/* 12. poll CONSYS CHIP ID until 0x6582 */
		while (retry-- > 0) {
			consysHwChipId = CONSYS_REG_READ(CONSYS_CHIP_ID_REG);
			if (consysHwChipId == 0x6582 || consysHwChipId == 0x6572) {
				WMT_PLAT_INFO_FUNC("retry(%d)consys chipId(0x%08x)\n", retry, consysHwChipId);
				break;
			}
			msleep(20);
		}
		if (consysHwChipId != 0x6582 && consysHwChipId != 0x6572)
			WMT_PLAT_ERR_FUNC("consys chip id never answered (last 0x%08x)\n", consysHwChipId);
		msleep(5);
		iRet = 0;
	} else {
		disable_clock(MT_CG_INFRA_CONNMCU, "WMT_MOD");
		consys_mtcmos_ctrl(false);
		/* VCN28 back to SW control and off, VCN_1V8 off */
		regmap_update_bits(p->pmic, MT6323_ANALDO_CON19, VCN28_ON_CTRL, 0);
		consys_rail(p->vcn28, &vcn28_on, false);
		regmap_update_bits(p->pmic, MT6323_DIGLDO_CON11, VCN_1V8_LP_MODE_SET, 0);
		iRet = consys_rail(p->vcn18, &vcn18_on, false);
		if (iRet)
			WMT_PLAT_ERR_FUNC("vcn18 disable failed (%d)\n", iRet);
		iRet = 0;
	}
	WMT_PLAT_INFO_FUNC("CONSYS-HW-REG-CTRL(0x%08x),finish\n", on);
	return iRet;
}

INT32 mtk_wcn_consys_hw_gpio_ctrl(UINT32 on)
{
	INT32 iRet = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW-GPIO-CTRL(0x%08x), start\n", on);
	if (on) {
		/* GPS sync/LNA and the I2S group are no-ops on this board
		 * (wmt_plat_alps.c has no pins defined for them). */
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_SYNC, PIN_STA_INIT);
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_LNA, PIN_STA_INIT);
		iRet += wmt_plat_gpio_ctrl(PIN_I2S_GRP, PIN_STA_INIT);
		/* The BGF (CONN2AP wakeup) IRQ: registered, then disabled until
		 * the STP power-saving machine wants it. */
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_INIT);
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_EINT_DIS);
		WMT_PLAT_INFO_FUNC("CONSYS-HW, BGF IRQ registered and disabled\n");
	} else {
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_EINT_DIS);
		iRet += wmt_plat_eirq_ctrl(PIN_BGF_EINT, PIN_STA_DEINIT);
		WMT_PLAT_INFO_FUNC("CONSYS-HW, BGF IRQ unregistered and disabled\n");
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_SYNC, PIN_STA_DEINIT);
		iRet += wmt_plat_gpio_ctrl(PIN_I2S_GRP, PIN_STA_DEINIT);
		iRet += wmt_plat_gpio_ctrl(PIN_GPS_LNA, PIN_STA_DEINIT);
	}
	WMT_PLAT_INFO_FUNC("CONSYS-HW-GPIO-CTRL(0x%08x), finish\n", on);
	return iRet;
}

INT32 mtk_wcn_consys_hw_pwr_on(UINT32 co_clock_en)
{
	INT32 iRet = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-ON, start\n");
	iRet += mtk_wcn_consys_hw_reg_ctrl(1, co_clock_en);
	iRet += mtk_wcn_consys_hw_gpio_ctrl(1);
	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-ON, finish(%d)\n", iRet);
	return iRet;
}

INT32 mtk_wcn_consys_hw_pwr_off(VOID)
{
	INT32 iRet = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-OFF, start\n");
	iRet += mtk_wcn_consys_hw_reg_ctrl(0, 0);
	iRet += mtk_wcn_consys_hw_gpio_ctrl(0);
	WMT_PLAT_INFO_FUNC("CONSYS-HW-PWR-OFF, finish(%d)\n", iRet);
	return iRet;
}

INT32 mtk_wcn_consys_hw_rst(UINT32 co_clock_en)
{
	INT32 iRet = 0;

	WMT_PLAT_INFO_FUNC("CONSYS-HW, hw_rst start, eirq should be disabled before this step\n");
	iRet += mtk_wcn_consys_hw_reg_ctrl(0, co_clock_en);
	iRet += mtk_wcn_consys_hw_reg_ctrl(1, co_clock_en);
	WMT_PLAT_INFO_FUNC("CONSYS-HW, hw_rst finish, eirq should be enabled after this step\n");
	return iRet;
}

/* BT PA LDO: VCN33_BT on, then handed to hardware control (0x416[5]). */
INT32 mtk_wcn_consys_hw_bt_paldo_ctrl(UINT32 enable)
{
	struct consys_plat *p = &consys_plat;

	if (enable) {
		consys_rail(p->vcn33_bt, &vcn33_bt_on, true);
		if (!paldo_sw)
			regmap_update_bits(p->pmic, MT6323_ANALDO_CON16, VCN33_ON_CTRL_BT, VCN33_ON_CTRL_BT);
		WMT_PLAT_INFO_FUNC("WMT do BT PMIC on\n");
	} else {
		regmap_update_bits(p->pmic, MT6323_ANALDO_CON16, VCN33_ON_CTRL_BT, 0);
		consys_rail(p->vcn33_bt, &vcn33_bt_on, false);
		WMT_PLAT_INFO_FUNC("WMT do BT PMIC off\n");
	}
	return 0;
}

/* WiFi PA LDO: VCN33_WIFI on, then hardware control (0x418[14]). */
INT32 mtk_wcn_consys_hw_wifi_paldo_ctrl(UINT32 enable)
{
	struct consys_plat *p = &consys_plat;

	if (enable) {
		consys_rail(p->vcn33_wifi, &vcn33_wifi_on, true);
		if (!paldo_sw)
			regmap_update_bits(p->pmic, MT6323_ANALDO_CON17, VCN33_ON_CTRL_WIFI, VCN33_ON_CTRL_WIFI);
		WMT_PLAT_INFO_FUNC("WMT do WIFI PMIC on\n");
	} else {
		regmap_update_bits(p->pmic, MT6323_ANALDO_CON17, VCN33_ON_CTRL_WIFI, 0);
		if (!force_vcn33_wifi)
			consys_rail(p->vcn33_wifi, &vcn33_wifi_on, false);
		WMT_PLAT_INFO_FUNC("WMT do WIFI PMIC off\n");
	}
	return 0;
}

/* In co-clock mode VCN28 is only needed while FM/GPS run. */
INT32 mtk_wcn_consys_hw_vcn28_ctrl(UINT32 enable)
{
	consys_rail(consys_plat.vcn28, &vcn28_on, !!enable);
	WMT_PLAT_INFO_FUNC("turn %s vcn28 for fm/gps usage in co-clock mode\n", enable ? "on" : "off");
	return 0;
}

INT32 mtk_wcn_consys_hw_state_show(VOID)
{
	return 0;
}

#if CONSYS_WMT_REG_SUSPEND_CB_ENABLE
UINT32 mtk_wcn_consys_hw_osc_en_ctrl(UINT32 en)
{
	if (en)
		CONSYS_REG_WRITE(CONSYS_AP2CONN_OSC_EN_REG, CONSYS_REG_READ(CONSYS_AP2CONN_OSC_EN_REG) & ~CONSYS_AP2CONN_OSC_EN_BIT);
	else
		CONSYS_REG_WRITE(CONSYS_AP2CONN_OSC_EN_REG, CONSYS_REG_READ(CONSYS_AP2CONN_OSC_EN_REG) | CONSYS_AP2CONN_OSC_EN_BIT);
	return 0;
}
#endif

INT32 mtk_wcn_consys_hw_init(void)
{
	UINT32 addrPhy;

	if (!consys_plat.ready) {
		WMT_PLAT_ERR_FUNC("CONSYS platform device not probed\n");
		return -ENODEV;
	}
	gConEmiPhyBase = consys_plat.emi_phys;
	WMT_PLAT_INFO_FUNC("consys EMI window at %pa\n", &gConEmiPhyBase);
	/* consys-to-AP EMI remapping register (0x10002310): MiB index of the
	 * window, plus the enable bit 12 */
	addrPhy = (gConEmiPhyBase & 0xFFF00000) >> 20;
	addrPhy |= 0x1000;
	CONSYS_REG_WRITE(CONSYS_EMI_MAPPING, CONSYS_REG_READ(CONSYS_EMI_MAPPING) | addrPhy);
	WMT_PLAT_INFO_FUNC("CONSYS_EMI_MAPPING dump(0x%08x)\n", CONSYS_REG_READ(CONSYS_EMI_MAPPING));
	pEmibaseaddr = ioremap(gConEmiPhyBase + CONSYS_EMI_AP_PHY_OFFSET, CONSYS_EMI_MEM_SIZE);
	if (!pEmibaseaddr) {
		WMT_PLAT_ERR_FUNC("EMI mapping fail\n");
		return -1;
	}
	memset_io(pEmibaseaddr, 0, CONSYS_EMI_MEM_SIZE);
	WMT_PLAT_INFO_FUNC("EMI mapping OK(0x%p)\n", pEmibaseaddr);
	return 0;
}

INT32 mtk_wcn_consys_hw_deinit(void)
{
	if (pEmibaseaddr) {
		iounmap(pEmibaseaddr);
		pEmibaseaddr = NULL;
	}
	return 0;
}

UINT8 *mtk_wcn_consys_emi_virt_addr_get(UINT32 ctrl_state_offset)
{
	if (!pEmibaseaddr) {
		WMT_PLAT_ERR_FUNC("EMI base address is NULL\n");
		return NULL;
	}
	return pEmibaseaddr + ctrl_state_offset;
}

UINT32 mtk_wcn_consys_soc_chipid(void)
{
	return PLATFORM_SOC_CHIP;
}
