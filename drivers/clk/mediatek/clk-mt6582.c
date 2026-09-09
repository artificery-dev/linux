// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6582 topckgen + infracfg + pericfg clock driver.
 *
 * Register offsets, mux fields (CLK_CFG_x) and gate bits are from the vendor
 * GPL kernel (android-mediatek-sprout-3.4) mt_clkmgr.c, cross-referenced with
 * the mainline mt8135 driver (same register layout: infra CG 0x40/0x44/0x48,
 * peri CG 0x08/0x10/0x18). The top muxes are already configured by the
 * preloader/LK; Linux does not re-parent them, so the parent selector order
 * below is best-effort (clk26m at index 0) and only affects rate reporting
 * until each consumer is migrated off its fixed-clock placeholder. Verify
 * against /sys/kernel/debug/clk/clk_summary and the MT6582 register spec.
 */
#include <dt-bindings/clock/mt6582-clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

static DEFINE_SPINLOCK(mt6582_clk_lock);

/* ---- root / fixed clocks ---- */
static const struct mtk_fixed_clk fixed_clks[] = {
	/* clk26m is provided by the board DT oscillator; referenced by name. */
	FIXED_CLK(CLK_TOP_RTC32K, "rtc32k", NULL, 32768),
	/*
	 * VENCPLL lives in the DDRPHY block (0x1000f800), is owned by the
	 * preloader and always on; model it as a fixed root at its typical rate
	 * rather than a separate controller. mm_sel derives from it.
	 */
	FIXED_CLK(CLK_TOP_VENCPLL, "vencpll", NULL, 500000000),
};

/* ---- fixed factors (PLL dividers) ---- */
static const struct mtk_fixed_factor top_divs[] = {
	FACTOR(CLK_TOP_MAINPLL_D2, "mainpll_d2", "mainpll", 1, 2),
	FACTOR(CLK_TOP_MAINPLL_D3, "mainpll_d3", "mainpll", 1, 3),
	FACTOR(CLK_TOP_MAINPLL_D4, "mainpll_d4", "mainpll", 1, 4),
	FACTOR(CLK_TOP_MAINPLL_D6, "mainpll_d6", "mainpll", 1, 6),
	FACTOR(CLK_TOP_MAINPLL_D8, "mainpll_d8", "mainpll", 1, 8),
	FACTOR(CLK_TOP_MAINPLL_D2P5, "mainpll_d2p5", "mainpll", 2, 5),
	FACTOR(CLK_TOP_MAINPLL_D5, "mainpll_d5", "mainpll", 1, 5),
	FACTOR(CLK_TOP_UNIVPLL_D2, "univpll_d2", "univpll", 1, 2),
	FACTOR(CLK_TOP_UNIVPLL_D3, "univpll_d3", "univpll", 1, 3),
	FACTOR(CLK_TOP_UNIVPLL_D5, "univpll_d5", "univpll", 1, 5),
	FACTOR(CLK_TOP_UNIVPLL_D26, "univpll_d26", "univpll", 1, 26),
	FACTOR(CLK_TOP_UNIVPLL1_D2, "univpll1_d2", "univpll", 1, 4),
	FACTOR(CLK_TOP_UNIVPLL1_D4, "univpll1_d4", "univpll", 1, 8),
	FACTOR(CLK_TOP_UNIVPLL1_D8, "univpll1_d8", "univpll", 1, 16),
	FACTOR(CLK_TOP_UNIVPLL2_D2, "univpll2_d2", "univpll", 1, 6),
	FACTOR(CLK_TOP_UNIVPLL2_D4, "univpll2_d4", "univpll", 1, 12),
	FACTOR(CLK_TOP_UNIVPLL2_D8, "univpll2_d8", "univpll", 1, 24),
	FACTOR(CLK_TOP_MMPLL_D2, "mmpll_d2", "mmpll", 1, 2),
	FACTOR(CLK_TOP_MSDCPLL_D2, "msdcpll_d2", "msdcpll", 1, 2),
	FACTOR(CLK_TOP_MSDCPLL_D4, "msdcpll_d4", "msdcpll", 1, 4),
	FACTOR(CLK_TOP_VENCPLL_D3, "vencpll_d3", "vencpll", 1, 3),
};

/* ---- top muxes (parent selector order is best-effort; see file header) ---- */
static const char * const mm_parents[] = {
	"clk26m", "vencpll", "univpll1_d2", "mainpll_d2",
	"mainpll_d3", "univpll_d5", "msdcpll_d2", "univpll1_d4"
};
static const char * const camtg_parents[] = {
	"clk26m", "univpll_d26", "univpll2_d2", "univpll1_d4",
	"univpll2_d4", "univpll1_d8", "univpll2_d8"
};
static const char * const mfg_parents[] = {
	"clk26m", "mmpll_d2", "univpll_d3", "univpll1_d2",
	"mainpll_d2", "univpll_d5", "vencpll", "mainpll_d3"
};
static const char * const vdec_parents[] = {
	"clk26m", "vencpll", "univpll_d3", "mainpll_d2",
	"univpll1_d2", "mainpll_d3", "univpll_d5", "univpll1_d4", "msdcpll_d2"
};
static const char * const pwm_parents[] = {
	"clk26m", "univpll2_d4", "univpll_d26", "univpll1_d8"
};
static const char * const msdc30_parents[] = {
	"clk26m", "msdcpll_d2", "univpll1_d4", "mainpll_d4",
	"univpll1_d8", "msdcpll_d4"
};
static const char * const usb20_parents[] = {
	"clk26m", "univpll1_d8", "univpll2_d4"
};
static const char * const spi_parents[] = {
	"clk26m", "msdcpll_d2", "univpll1_d4", "mainpll_d4", "univpll2_d4"
};
static const char * const uart_parents[] = {
	"clk26m", "univpll2_d8"
};
static const char * const audintbus_parents[] = {
	"clk26m", "mainpll_d6", "univpll_d26", "mainpll_d4",
	"univpll1_d8", "univpll2_d8"
};
static const char * const audio_parents[] = {
	"clk26m", "mainpll_d8"
};

static const struct mtk_composite top_muxes[] = {
	/* CLK_CFG_0 (0x40) */
	MUX_GATE(CLK_TOP_MM_SEL, "mm_sel", mm_parents, 0x0040, 24, 3, 31),
	/* CLK_CFG_1 (0x50) */
	MUX_GATE(CLK_TOP_CAMTG_SEL, "camtg_sel", camtg_parents, 0x0050, 24, 3, 31),
	MUX_GATE(CLK_TOP_MFG_SEL, "mfg_sel", mfg_parents, 0x0050, 16, 3, 23),
	MUX_GATE(CLK_TOP_VDEC_SEL, "vdec_sel", vdec_parents, 0x0050, 8, 4, 15),
	MUX_GATE(CLK_TOP_PWM_SEL, "pwm_sel", pwm_parents, 0x0050, 0, 2, 7),
	/* CLK_CFG_2 (0x60) */
	MUX_GATE(CLK_TOP_MSDC30_0_SEL, "msdc30_0_sel", msdc30_parents, 0x0060, 24, 3, 31),
	MUX_GATE(CLK_TOP_USB20_SEL, "usb20_sel", usb20_parents, 0x0060, 16, 3, 23),
	MUX_GATE(CLK_TOP_SPI_SEL, "spi_sel", spi_parents, 0x0060, 8, 3, 15),
	MUX_GATE(CLK_TOP_UART_SEL, "uart_sel", uart_parents, 0x0060, 0, 1, 7),
	/* CLK_CFG_3 (0x70) */
	MUX_GATE(CLK_TOP_AUDINTBUS_SEL, "audintbus_sel", audintbus_parents, 0x0070, 24, 3, 31),
	MUX_GATE(CLK_TOP_AUDIO_SEL, "audio_sel", audio_parents, 0x0070, 16, 1, 23),
	MUX_GATE(CLK_TOP_MSDC30_2_SEL, "msdc30_2_sel", msdc30_parents, 0x0070, 8, 3, 15),
	MUX_GATE(CLK_TOP_MSDC30_1_SEL, "msdc30_1_sel", msdc30_parents, 0x0070, 0, 3, 7),
};

/* ---- infracfg gates (STA 0x48 / CLR 0x44 / SET 0x40) ---- */
static const struct mtk_gate_regs infra_cg_regs = {
	.set_ofs = 0x0040,
	.clr_ofs = 0x0044,
	.sta_ofs = 0x0048,
};

#define GATE_ICG(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &infra_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_ICG_AO(_id, _name, _parent, _shift)	\
	GATE_MTK_FLAGS(_id, _name, _parent, &infra_cg_regs, _shift,	\
		       &mtk_clk_gate_ops_setclr, CLK_IS_CRITICAL)

static const struct mtk_gate infra_clks[] = {
	GATE_ICG(CLK_INFRA_DBGCLK, "infra_dbgclk", "clk26m", 0),
	GATE_ICG(CLK_INFRA_SMI, "infra_smi", "mm_sel", 1),
	GATE_ICG(CLK_INFRA_AUDIO, "infra_audio", "audintbus_sel", 5),
	GATE_ICG(CLK_INFRA_EFUSE, "infra_efuse", "clk26m", 6),
	GATE_ICG_AO(CLK_INFRA_L2C_SRAM, "infra_l2c_sram", "clk26m", 7),
	GATE_ICG_AO(CLK_INFRA_M4U, "infra_m4u", "mm_sel", 8),
	GATE_ICG(CLK_INFRA_CONNMCU, "infra_connmcu", "clk26m", 12),
	GATE_ICG(CLK_INFRA_TRNG, "infra_trng", "clk26m", 13),
	GATE_ICG(CLK_INFRA_CPUM, "infra_cpum", "clk26m", 15),
	GATE_ICG(CLK_INFRA_KP, "infra_kp", "clk26m", 16),
	GATE_ICG(CLK_INFRA_CCIF0, "infra_ccif0", "clk26m", 20),
	GATE_ICG_AO(CLK_INFRA_PMIC_WRAP, "infra_pmic_wrap", "clk26m", 23),
};

/* ---- pericfg gates (PERI_PDN0: STA 0x18 / CLR 0x10 / SET 0x08) ---- */
static const struct mtk_gate_regs peri_cg_regs = {
	.set_ofs = 0x0008,
	.clr_ofs = 0x0010,
	.sta_ofs = 0x0018,
};

#define GATE_PERI(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &peri_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate peri_gates[] = {
	GATE_PERI(CLK_PERI_NFI, "peri_nfi", "clk26m", 0),
	GATE_PERI(CLK_PERI_THERM, "peri_therm", "clk26m", 1),
	GATE_PERI(CLK_PERI_PWM1, "peri_pwm1", "pwm_sel", 2),
	GATE_PERI(CLK_PERI_PWM2, "peri_pwm2", "pwm_sel", 3),
	GATE_PERI(CLK_PERI_PWM3, "peri_pwm3", "pwm_sel", 4),
	GATE_PERI(CLK_PERI_PWM4, "peri_pwm4", "pwm_sel", 5),
	GATE_PERI(CLK_PERI_PWM5, "peri_pwm5", "pwm_sel", 6),
	GATE_PERI(CLK_PERI_PWM6, "peri_pwm6", "pwm_sel", 7),
	GATE_PERI(CLK_PERI_PWM7, "peri_pwm7", "pwm_sel", 8),
	GATE_PERI(CLK_PERI_PWM, "peri_pwm", "pwm_sel", 9),
	GATE_PERI(CLK_PERI_USB0, "peri_usb0", "usb20_sel", 10),
	GATE_PERI(CLK_PERI_AP_DMA, "peri_ap_dma", "clk26m", 11),
	GATE_PERI(CLK_PERI_MSDC30_0, "peri_msdc30_0", "msdc30_0_sel", 12),
	GATE_PERI(CLK_PERI_MSDC30_1, "peri_msdc30_1", "msdc30_1_sel", 13),
	GATE_PERI(CLK_PERI_MSDC30_2, "peri_msdc30_2", "msdc30_2_sel", 14),
	GATE_PERI(CLK_PERI_NLI, "peri_nli", "clk26m", 15),
	GATE_PERI(CLK_PERI_UART0, "peri_uart0", "uart_sel", 16),
	GATE_PERI(CLK_PERI_UART1, "peri_uart1", "uart_sel", 17),
	GATE_PERI(CLK_PERI_UART2, "peri_uart2", "uart_sel", 18),
	GATE_PERI(CLK_PERI_UART3, "peri_uart3", "uart_sel", 19),
	GATE_PERI(CLK_PERI_BTIF, "peri_btif", "clk26m", 20),
	GATE_PERI(CLK_PERI_I2C0, "peri_i2c0", "clk26m", 21),
	GATE_PERI(CLK_PERI_I2C1, "peri_i2c1", "clk26m", 22),
	GATE_PERI(CLK_PERI_I2C2, "peri_i2c2", "clk26m", 23),
	GATE_PERI(CLK_PERI_AUXADC, "peri_auxadc", "clk26m", 24),
	GATE_PERI(CLK_PERI_SPI0, "peri_spi0", "spi_sel", 25),
};

static const struct mtk_clk_desc topck_desc = {
	.fixed_clks = fixed_clks,
	.num_fixed_clks = ARRAY_SIZE(fixed_clks),
	.factor_clks = top_divs,
	.num_factor_clks = ARRAY_SIZE(top_divs),
	.composite_clks = top_muxes,
	.num_composite_clks = ARRAY_SIZE(top_muxes),
	.clk_lock = &mt6582_clk_lock,
};

static const struct mtk_clk_desc infra_desc = {
	.clks = infra_clks,
	.num_clks = ARRAY_SIZE(infra_clks),
};

static const struct mtk_clk_desc peri_desc = {
	.clks = peri_gates,
	.num_clks = ARRAY_SIZE(peri_gates),
	.clk_lock = &mt6582_clk_lock,
};

static const struct of_device_id of_match_clk_mt6582[] = {
	{ .compatible = "mediatek,mt6582-topckgen", .data = &topck_desc },
	{ .compatible = "mediatek,mt6582-infracfg", .data = &infra_desc },
	{ .compatible = "mediatek,mt6582-pericfg", .data = &peri_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6582);

static struct platform_driver clk_mt6582_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6582",
		.of_match_table = of_match_clk_mt6582,
	},
};
module_platform_driver(clk_mt6582_drv);

MODULE_DESCRIPTION("MediaTek MT6582 topckgen/infracfg/pericfg clocks driver");
MODULE_LICENSE("GPL");
