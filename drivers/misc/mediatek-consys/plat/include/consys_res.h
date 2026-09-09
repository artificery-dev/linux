/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Runtime resources for the MediaTek MT6582 CONSYS port: the vendor stack
 * addressed its blocks with fixed virtual constants (mt_reg_base.h) and IRQ
 * numbers (mt_irq.h); here the DT platform driver fills these in at probe
 * and the mach/ shim headers map the old names onto them.
 */
#ifndef _CONSYS_RES_H_
#define _CONSYS_RES_H_
#include <linux/types.h>

extern void __iomem *consys_io_btif;	/* BTIF, 0x1100c000 */
extern void __iomem *consys_io_apdma;	/* AP_DMA, 0x11000000 (BTIF VFIFOs at +0x780/+0x800) */
extern void __iomem *consys_io_spm;	/* SPM, 0x10006000 */
extern void __iomem *consys_io_infra;	/* INFRACFG_AO, 0x10001000 */
extern void __iomem *consys_io_rgu;	/* AP_RGU (watchdog), 0x10007000 */
extern void __iomem *consys_io_conn_mcu;	/* CONN_MCU_CONFIG, 0x18070000 */
extern void __iomem *consys_io_conn_top;	/* CONN_TOP_CR, 0x180b0000 */

extern int consys_irq_btif;		/* GIC SPI 50 */
extern int consys_irq_btif_tx_dma;	/* GIC SPI 71 */
extern int consys_irq_btif_rx_dma;	/* GIC SPI 72 */
extern int consys_irq_bgf;		/* CONN2AP BTIF wakeup, GIC SPI 185 */

enum consys_clk {
	CONSYS_CLK_BTIF,
	CONSYS_CLK_APDMA,
	CONSYS_CLK_CONNMCU,
	CONSYS_CLK_NR,
};
struct clk;
extern struct clk *consys_clks[CONSYS_CLK_NR];	/* set by whoever owns the DT node with the clock */
int consys_clk_enable(enum consys_clk id);
int consys_clk_disable(enum consys_clk id);
bool consys_clk_is_on(enum consys_clk id);

#endif
