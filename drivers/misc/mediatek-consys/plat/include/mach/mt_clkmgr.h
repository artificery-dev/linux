/* SPDX-License-Identifier: GPL-2.0 */
/* Shim: vendor clkmgr gate ids -> CCF clocks held by the CONSYS platform driver. */
#ifndef _MACH_MT_CLKMGR_SHIM_H_
#define _MACH_MT_CLKMGR_SHIM_H_
#include "../consys_res.h"
#define MT_CG_PERI_BTIF		CONSYS_CLK_BTIF
#define MT_CG_PERI_AP_DMA	CONSYS_CLK_APDMA
#define MT_CG_INFRA_CONNMCU	CONSYS_CLK_CONNMCU
#define enable_clock(id, name)	consys_clk_enable(id)
#define disable_clock(id, name)	consys_clk_disable(id)
#define clock_is_on(id)		consys_clk_is_on(id)
#endif
