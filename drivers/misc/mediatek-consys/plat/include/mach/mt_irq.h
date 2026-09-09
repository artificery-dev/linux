/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MACH_MT_IRQ_SHIM_H_
#define _MACH_MT_IRQ_SHIM_H_
#include "../consys_res.h"
#define MT_BTIF_IRQ_ID			consys_irq_btif
#define MT_DMA_BTIF_TX_IRQ_ID		consys_irq_btif_tx_dma
#define MT_DMA_BTIF_RX_IRQ_ID		consys_irq_btif_rx_dma
#define MT_CONN2AP_BTIF_WAKEUP_IRQ_ID	consys_irq_bgf
#endif
