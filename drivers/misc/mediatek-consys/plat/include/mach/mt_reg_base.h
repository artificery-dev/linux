/* SPDX-License-Identifier: GPL-2.0 */
/* Shim: the vendor's fixed virtual bases become the DT-mapped ones. */
#ifndef _MACH_MT_REG_BASE_SHIM_H_
#define _MACH_MT_REG_BASE_SHIM_H_
#include "../consys_res.h"
#define BTIF_BASE		((unsigned long)consys_io_btif)
#define AP_DMA_BASE		((unsigned long)consys_io_apdma)
#define SPM_BASE		((unsigned long)consys_io_spm)
#define INFRA_BASE		((unsigned long)consys_io_infra)
#define AP_RGU_BASE		((unsigned long)consys_io_rgu)
#define CONN_MCU_CONFIG_BASE	((unsigned long)consys_io_conn_mcu)
#define CONN_TOP_CR_BASE	((unsigned long)consys_io_conn_top)
#endif
