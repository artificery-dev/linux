/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MT6582 clock IDs.
 *
 * Derived from the vendor GPL kernel (android-mediatek-sprout-3.4) mt_clkmgr,
 * cross-referenced against the mainline mt8135 (topckgen/apmixed/infra/peri)
 * and mt2701-mm (display) drivers, which share MT6582's register layout.
 */
#ifndef _DT_BINDINGS_CLK_MT6582_H
#define _DT_BINDINGS_CLK_MT6582_H

/* APMIXEDSYS (0x10209000) */
#define CLK_APMIXED_ARMPLL		0
#define CLK_APMIXED_MAINPLL		1
#define CLK_APMIXED_UNIVPLL		2
#define CLK_APMIXED_MMPLL		3
#define CLK_APMIXED_MSDCPLL		4
#define CLK_APMIXED_NR_CLK		5

/*
 * TOPCKGEN (0x10000000). NOTE: mtk_clk_simple_probe sizes the clock-provider
 * array by the NUMBER of table entries (fixed + factor + composite), and each
 * clock is stored at hws[id]. So the IDs here MUST be contiguous with no holes
 * (max id == entry_count - 1). clk26m is provided by the board DT oscillator,
 * not this controller, so it is intentionally absent from the ID space.
 */
/* roots / fixed */
#define CLK_TOP_RTC32K			0
#define CLK_TOP_VENCPLL			1
/* fixed factors (PLL dividers) */
#define CLK_TOP_MAINPLL_D2		2
#define CLK_TOP_MAINPLL_D3		3
#define CLK_TOP_MAINPLL_D4		4
#define CLK_TOP_MAINPLL_D6		5
#define CLK_TOP_MAINPLL_D8		6
#define CLK_TOP_MAINPLL_D2P5		7
#define CLK_TOP_MAINPLL_D5		8
#define CLK_TOP_UNIVPLL_D2		9
#define CLK_TOP_UNIVPLL_D3		10
#define CLK_TOP_UNIVPLL_D5		11
#define CLK_TOP_UNIVPLL_D26		12
#define CLK_TOP_UNIVPLL1_D2		13
#define CLK_TOP_UNIVPLL1_D4		14
#define CLK_TOP_UNIVPLL1_D8		15
#define CLK_TOP_UNIVPLL2_D2		16
#define CLK_TOP_UNIVPLL2_D4		17
#define CLK_TOP_UNIVPLL2_D8		18
#define CLK_TOP_MMPLL_D2		19
#define CLK_TOP_MSDCPLL_D2		20
#define CLK_TOP_MSDCPLL_D4		21
#define CLK_TOP_VENCPLL_D3		22
/* muxes (CLK_CFG_x) */
#define CLK_TOP_MM_SEL			23
#define CLK_TOP_CAMTG_SEL		24
#define CLK_TOP_MFG_SEL			25
#define CLK_TOP_VDEC_SEL		26
#define CLK_TOP_PWM_SEL			27
#define CLK_TOP_MSDC30_0_SEL		28
#define CLK_TOP_USB20_SEL		29
#define CLK_TOP_SPI_SEL			30
#define CLK_TOP_UART_SEL		31
#define CLK_TOP_AUDINTBUS_SEL		32
#define CLK_TOP_AUDIO_SEL		33
#define CLK_TOP_MSDC30_2_SEL		34
#define CLK_TOP_MSDC30_1_SEL		35
#define CLK_TOP_NR_CLK			36

/* INFRACFG_AO (0x10001000) */
#define CLK_INFRA_DBGCLK		0
#define CLK_INFRA_SMI			1
#define CLK_INFRA_AUDIO			2
#define CLK_INFRA_EFUSE			3
#define CLK_INFRA_L2C_SRAM		4
#define CLK_INFRA_M4U			5
#define CLK_INFRA_CONNMCU		6
#define CLK_INFRA_TRNG			7
#define CLK_INFRA_CPUM			8
#define CLK_INFRA_KP			9
#define CLK_INFRA_CCIF0			10
#define CLK_INFRA_PMIC_WRAP		11
#define CLK_INFRA_NR_CLK		12

/* PERICFG (0x10003000) */
#define CLK_PERI_NFI			0
#define CLK_PERI_THERM			1
#define CLK_PERI_PWM1			2
#define CLK_PERI_PWM2			3
#define CLK_PERI_PWM3			4
#define CLK_PERI_PWM4			5
#define CLK_PERI_PWM5			6
#define CLK_PERI_PWM6			7
#define CLK_PERI_PWM7			8
#define CLK_PERI_PWM			9
#define CLK_PERI_USB0			10
#define CLK_PERI_AP_DMA			11
#define CLK_PERI_MSDC30_0		12
#define CLK_PERI_MSDC30_1		13
#define CLK_PERI_MSDC30_2		14
#define CLK_PERI_NLI			15
#define CLK_PERI_UART0			16
#define CLK_PERI_UART1			17
#define CLK_PERI_UART2			18
#define CLK_PERI_UART3			19
#define CLK_PERI_BTIF			20
#define CLK_PERI_I2C0			21
#define CLK_PERI_I2C1			22
#define CLK_PERI_I2C2			23
#define CLK_PERI_AUXADC			24
#define CLK_PERI_SPI0			25
#define CLK_PERI_NR_CLK			26

/* MMSYS (0x14000000) — DISP0 + DISP1 gate groups */
#define CLK_MM_SMI_COMMON		0
#define CLK_MM_SMI_LARB0		1
#define CLK_MM_CMDQ			2
#define CLK_MM_MUTEX			3
#define CLK_MM_DISP_COLOR		4
#define CLK_MM_DISP_BLS			5
#define CLK_MM_DISP_WDMA		6
#define CLK_MM_DISP_RDMA		7
#define CLK_MM_DISP_OVL			8
#define CLK_MM_MDP_TDSHP		9
#define CLK_MM_MDP_WROT			10
#define CLK_MM_MDP_WDMA			11
#define CLK_MM_MDP_RSZ1			12
#define CLK_MM_MDP_RSZ0			13
#define CLK_MM_MDP_RDMA			14
#define CLK_MM_MDP_BLS_26M		15
#define CLK_MM_CAM_MDP			16
#define CLK_MM_FAKE_ENG			17
#define CLK_MM_MUTEX_32K		18
#define CLK_MM_DSI_ENGINE		19
#define CLK_MM_DSI_DIGITAL		20
#define CLK_MM_DPI_DIGITAL_LANE		21
#define CLK_MM_DPI_ENGINE		22
#define CLK_MM_NR_CLK			23

/* MFGCFG (0x13000000) */
#define CLK_MFG_G3D			0
#define CLK_MFG_NR_CLK			1

#endif /* _DT_BINDINGS_CLK_MT6582_H */
