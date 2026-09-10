// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: jitao.shi <jitao.shi@mediatek.com>
 */

#include "phy-mtk-io.h"

#include <linux/io.h>
#include <linux/delay.h>
static inline void y2_fbmark(u16 color)
{
	void __iomem *fb = ioremap(0xbfb54600, 480 * 360 * 2);
	int i;
	if (fb) {
		for (i = 0; i < 480 * 360; i++)
			writew(color, fb + i * 2);
		iounmap(fb);
	}
	mdelay(400);
}
#include "phy-mtk-mipi-dsi.h"

#define MIPITX_DSI_CON		0x00
#define RG_DSI_LDOCORE_EN		BIT(0)
#define RG_DSI_CKG_LDOOUT_EN		BIT(1)
#define RG_DSI_BCLK_SEL			GENMASK(3, 2)
#define RG_DSI_LD_IDX_SEL		GENMASK(6, 4)
#define RG_DSI_PHYCLK_SEL		GENMASK(9, 8)
#define RG_DSI_DSICLK_FREQ_SEL		BIT(10)
#define RG_DSI_LPTX_CLMP_EN		BIT(11)

#define MIPITX_DSI_CLOCK_LANE	0x04
#define MIPITX_DSI_DATA_LANE0	0x08
#define MIPITX_DSI_DATA_LANE1	0x0c
#define MIPITX_DSI_DATA_LANE2	0x10
#define MIPITX_DSI_DATA_LANE3	0x14
#define RG_DSI_LNTx_LDOOUT_EN		BIT(0)
#define RG_DSI_LNTx_CKLANE_EN		BIT(1)
#define RG_DSI_LNTx_LPTX_IPLUS1		BIT(2)
#define RG_DSI_LNTx_LPTX_IPLUS2		BIT(3)
#define RG_DSI_LNTx_LPTX_IMINUS		BIT(4)
#define RG_DSI_LNTx_LPCD_IPLUS		BIT(5)
#define RG_DSI_LNTx_LPCD_IMINUS		BIT(6)
#define RG_DSI_LNTx_RT_CODE		GENMASK(11, 8)

#define MIPITX_DSI_TOP_CON	0x40
#define RG_DSI_LNT_INTR_EN		BIT(0)
#define RG_DSI_LNT_HS_BIAS_EN		BIT(1)
#define RG_DSI_LNT_IMP_CAL_EN		BIT(2)
#define RG_DSI_LNT_TESTMODE_EN		BIT(3)
#define RG_DSI_LNT_IMP_CAL_CODE		GENMASK(7, 4)
#define RG_DSI_LNT_AIO_SEL		GENMASK(10, 8)
#define RG_DSI_PAD_TIE_LOW_EN		BIT(11)
#define RG_DSI_DEBUG_INPUT_EN		BIT(12)
#define RG_DSI_PRESERVE			GENMASK(15, 13)

#define MIPITX_DSI_BG_CON	0x44
#define RG_DSI_BG_CORE_EN		BIT(0)
#define RG_DSI_BG_CKEN			BIT(1)
#define RG_DSI_BG_DIV			GENMASK(3, 2)
#define RG_DSI_BG_FAST_CHARGE		BIT(4)

#define RG_DSI_V12_SEL			GENMASK(7, 5)
#define RG_DSI_V10_SEL			GENMASK(10, 8)
#define RG_DSI_V072_SEL			GENMASK(13, 11)
#define RG_DSI_V04_SEL			GENMASK(16, 14)
#define RG_DSI_V032_SEL			GENMASK(19, 17)
#define RG_DSI_V02_SEL			GENMASK(22, 20)
#define RG_DSI_VOUT_MSK			\
		(RG_DSI_V12_SEL | RG_DSI_V10_SEL | RG_DSI_V072_SEL | \
		 RG_DSI_V04_SEL | RG_DSI_V032_SEL | RG_DSI_V02_SEL)
#define RG_DSI_BG_R1_TRIM		GENMASK(27, 24)
#define RG_DSI_BG_R2_TRIM		GENMASK(31, 28)

#define MIPITX_DSI_PLL_CON0	0x50
#define RG_DSI_MPPLL_PLL_EN		BIT(0)
#define RG_DSI_MPPLL_PREDIV		GENMASK(2, 1)
#define RG_DSI_MPPLL_TXDIV0		GENMASK(4, 3)
#define RG_DSI_MPPLL_TXDIV1		GENMASK(6, 5)
#define RG_DSI_MPPLL_POSDIV		GENMASK(9, 7)
#define RG_DSI_MPPLL_DIV_MSK		\
		(RG_DSI_MPPLL_PREDIV | RG_DSI_MPPLL_TXDIV0 | \
		 RG_DSI_MPPLL_TXDIV1 | RG_DSI_MPPLL_POSDIV)
#define RG_DSI_MPPLL_MONVC_EN		BIT(10)
#define RG_DSI_MPPLL_MONREF_EN		BIT(11)
#define RG_DSI_MPPLL_VOD_EN		BIT(12)

#define MIPITX_DSI_PLL_CON1	0x54
#define RG_DSI_MPPLL_SDM_FRA_EN		BIT(0)
#define RG_DSI_MPPLL_SDM_SSC_PH_INIT	BIT(1)
#define RG_DSI_MPPLL_SDM_SSC_EN		BIT(2)
#define RG_DSI_MPPLL_SDM_SSC_PRD	GENMASK(31, 16)

#define MIPITX_DSI_PLL_CON2	0x58

#define MIPITX_DSI_PLL_TOP	0x64
#define RG_DSI_MPPLL_PRESERVE		GENMASK(15, 8)

#define MIPITX_DSI_PLL_PWR	0x68
#define RG_DSI_MPPLL_SDM_PWR_ON		BIT(0)
#define RG_DSI_MPPLL_SDM_ISO_EN		BIT(1)
#define RG_DSI_MPPLL_SDM_PWR_ACK	BIT(8)

#define MIPITX_DSI_SW_CTRL	0x80
#define SW_CTRL_EN			BIT(0)

#define MIPITX_DSI_SW_CTRL_CON0	0x84
#define SW_LNTC_LPTX_PRE_OE		BIT(0)
#define SW_LNTC_LPTX_OE			BIT(1)
#define SW_LNTC_LPTX_P			BIT(2)
#define SW_LNTC_LPTX_N			BIT(3)
#define SW_LNTC_HSTX_PRE_OE		BIT(4)
#define SW_LNTC_HSTX_OE			BIT(5)
#define SW_LNTC_HSTX_ZEROCLK		BIT(6)
#define SW_LNT0_LPTX_PRE_OE		BIT(7)
#define SW_LNT0_LPTX_OE			BIT(8)
#define SW_LNT0_LPTX_P			BIT(9)
#define SW_LNT0_LPTX_N			BIT(10)
#define SW_LNT0_HSTX_PRE_OE		BIT(11)
#define SW_LNT0_HSTX_OE			BIT(12)
#define SW_LNT0_LPRX_EN			BIT(13)
#define SW_LNT1_LPTX_PRE_OE		BIT(14)
#define SW_LNT1_LPTX_OE			BIT(15)
#define SW_LNT1_LPTX_P			BIT(16)
#define SW_LNT1_LPTX_N			BIT(17)
#define SW_LNT1_HSTX_PRE_OE		BIT(18)
#define SW_LNT1_HSTX_OE			BIT(19)
#define SW_LNT2_LPTX_PRE_OE		BIT(20)
#define SW_LNT2_LPTX_OE			BIT(21)
#define SW_LNT2_LPTX_P			BIT(22)
#define SW_LNT2_LPTX_N			BIT(23)
#define SW_LNT2_HSTX_PRE_OE		BIT(24)
#define SW_LNT2_HSTX_OE			BIT(25)

static int mtk_mipi_tx_pll_prepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;
	u32 reg;

	/*
	 * MT6582 mipi-tx PLL bring-up, following the vendor DSI_PHY_clk_setting
	 * (references/vendor-src/mt6582-dsi/dsi_drv.c). Differences from mt8173
	 * that matter on this SoC:
	 *  - the clock/data lane LDOs are enabled BEFORE PLL_EN and the PLL_TOP
	 *    access; doing them afterwards (mt8173 order) leaves PLL_TOP dead on
	 *    the bus;
	 *  - the SDM is powered with isolation, then de-isolated;
	 *  - PRESERVE_L=3 gives the /4 post-divide;
	 *  - fixed TXDIV0=1/TXDIV1=0 and integer PCW_H=52 (fbk_div 13<<2), the
	 *    LCM PLL_CLOCK=0 path that LK uses for this panel.
	 */

	/* bias + core/clk LDO */
	mtk_phy_update_bits(base + MIPITX_DSI_TOP_CON,
			    RG_DSI_LNT_IMP_CAL_CODE | RG_DSI_LNT_HS_BIAS_EN,
			    FIELD_PREP(RG_DSI_LNT_IMP_CAL_CODE, 8) |
			    RG_DSI_LNT_HS_BIAS_EN);
	mtk_phy_update_bits(base + MIPITX_DSI_BG_CON,
			    RG_DSI_VOUT_MSK | RG_DSI_BG_CKEN | RG_DSI_BG_CORE_EN,
			    FIELD_PREP(RG_DSI_V02_SEL, 4) |
			    FIELD_PREP(RG_DSI_V032_SEL, 4) |
			    FIELD_PREP(RG_DSI_V04_SEL, 4) |
			    FIELD_PREP(RG_DSI_V072_SEL, 4) |
			    FIELD_PREP(RG_DSI_V10_SEL, 4) |
			    FIELD_PREP(RG_DSI_V12_SEL, 4) |
			    RG_DSI_BG_CKEN | RG_DSI_BG_CORE_EN);
	mdelay(10);
	mtk_phy_set_bits(base + MIPITX_DSI_CON,
			 RG_DSI_CKG_LDOOUT_EN | RG_DSI_LDOCORE_EN);

	/* SDM power: PWR_ON with ISO, settle, then de-isolate */
	mtk_phy_update_bits(base + MIPITX_DSI_PLL_PWR,
			    RG_DSI_MPPLL_SDM_PWR_ON | RG_DSI_MPPLL_SDM_ISO_EN,
			    RG_DSI_MPPLL_SDM_PWR_ON | RG_DSI_MPPLL_SDM_ISO_EN);
	mdelay(10);
	mtk_phy_clear_bits(base + MIPITX_DSI_PLL_PWR, RG_DSI_MPPLL_SDM_ISO_EN);

	/* PLL dividers: PREDIV=0, POSDIV=0, TXDIV0=1, TXDIV1=0 */
	mtk_phy_clear_bits(base + MIPITX_DSI_PLL_CON0, RG_DSI_MPPLL_PLL_EN);
	mtk_phy_update_bits(base + MIPITX_DSI_PLL_CON0,
			    RG_DSI_MPPLL_PREDIV | RG_DSI_MPPLL_POSDIV |
			    RG_DSI_MPPLL_TXDIV0 | RG_DSI_MPPLL_TXDIV1,
			    FIELD_PREP(RG_DSI_MPPLL_TXDIV0, 1));

	/* PCW: integer PCW_H (bits 24..30) = fbk_div(13) << 2 = 52, no fraction */
	writel(52u << 24, base + MIPITX_DSI_PLL_CON2);
	mtk_phy_set_bits(base + MIPITX_DSI_PLL_CON1, RG_DSI_MPPLL_SDM_FRA_EN);

	/* Enable clock + data lane LDOs BEFORE PLL_EN (vendor order). */
	for (reg = MIPITX_DSI_CLOCK_LANE; reg <= MIPITX_DSI_DATA_LANE3; reg += 4)
		mtk_phy_set_bits(base + reg, RG_DSI_LNTx_LDOOUT_EN);

	/* Enable the PLL */
	mtk_phy_set_bits(base + MIPITX_DSI_PLL_CON0, RG_DSI_MPPLL_PLL_EN);
	mdelay(1);
	mtk_phy_clear_bits(base + MIPITX_DSI_PLL_CON1, RG_DSI_MPPLL_SDM_SSC_EN);

	/* PRESERVE_L=3 => /4 post-divide (blind write; RMW-read of 0x64 stalls) */
	writel(FIELD_PREP(RG_DSI_MPPLL_PRESERVE, 3), base + MIPITX_DSI_PLL_TOP);

	/* release the DSI pad */
	mtk_phy_clear_bits(base + MIPITX_DSI_TOP_CON, RG_DSI_PAD_TIE_LOW_EN);

	return 0;
}

static void mtk_mipi_tx_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;

	dev_dbg(mipi_tx->dev, "unprepare\n");

	mtk_phy_clear_bits(base + MIPITX_DSI_PLL_CON0, RG_DSI_MPPLL_PLL_EN);

	mtk_phy_clear_bits(base + MIPITX_DSI_PLL_TOP, RG_DSI_MPPLL_PRESERVE);

	mtk_phy_update_bits(base + MIPITX_DSI_PLL_PWR,
			    RG_DSI_MPPLL_SDM_ISO_EN | RG_DSI_MPPLL_SDM_PWR_ON,
			    RG_DSI_MPPLL_SDM_ISO_EN);

	mtk_phy_clear_bits(base + MIPITX_DSI_TOP_CON, RG_DSI_LNT_HS_BIAS_EN);

	mtk_phy_clear_bits(base + MIPITX_DSI_CON,
			   RG_DSI_CKG_LDOOUT_EN | RG_DSI_LDOCORE_EN);

	mtk_phy_clear_bits(base + MIPITX_DSI_BG_CON,
			   RG_DSI_BG_CKEN | RG_DSI_BG_CORE_EN);

	mtk_phy_clear_bits(base + MIPITX_DSI_PLL_CON0, RG_DSI_MPPLL_DIV_MSK);
}

static long mtk_mipi_tx_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *prate)
{
	return clamp_val(rate, 50000000, 1250000000);
}

static const struct clk_ops mtk_mipi_tx_pll_ops = {
	.prepare = mtk_mipi_tx_pll_prepare,
	.unprepare = mtk_mipi_tx_pll_unprepare,
	.round_rate = mtk_mipi_tx_pll_round_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate,
};

static void mtk_mipi_tx_power_on_signal(struct phy *phy)
{
	struct mtk_mipi_tx *mipi_tx = phy_get_drvdata(phy);
	u32 reg;

	for (reg = MIPITX_DSI_CLOCK_LANE;
	     reg <= MIPITX_DSI_DATA_LANE3; reg += 4)
		mtk_phy_set_bits(mipi_tx->regs + reg, RG_DSI_LNTx_LDOOUT_EN);

	mtk_phy_clear_bits(mipi_tx->regs + MIPITX_DSI_TOP_CON,
			   RG_DSI_PAD_TIE_LOW_EN);
}

static void mtk_mipi_tx_power_off_signal(struct phy *phy)
{
	struct mtk_mipi_tx *mipi_tx = phy_get_drvdata(phy);
	u32 reg;

	mtk_phy_set_bits(mipi_tx->regs + MIPITX_DSI_TOP_CON,
			 RG_DSI_PAD_TIE_LOW_EN);

	for (reg = MIPITX_DSI_CLOCK_LANE;
	     reg <= MIPITX_DSI_DATA_LANE3; reg += 4)
		mtk_phy_clear_bits(mipi_tx->regs + reg, RG_DSI_LNTx_LDOOUT_EN);
}

const struct mtk_mipitx_data mt2701_mipitx_data = {
	.mppll_preserve = 3,
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
};

const struct mtk_mipitx_data mt8173_mipitx_data = {
	.mppll_preserve = 0,
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
};
