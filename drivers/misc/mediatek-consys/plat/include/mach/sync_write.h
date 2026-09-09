/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MACH_SYNC_WRITE_SHIM_H_
#define _MACH_SYNC_WRITE_SHIM_H_
#include <linux/io.h>
#define mt65xx_reg_sync_writel(v, a)	writel((v), (void __iomem *)(unsigned long)(a))
#define mt65xx_reg_sync_writew(v, a)	writew((v), (void __iomem *)(unsigned long)(a))
#define mt65xx_reg_sync_writeb(v, a)	writeb((v), (void __iomem *)(unsigned long)(a))
#define mt_reg_sync_writel(v, a)	mt65xx_reg_sync_writel(v, a)
#define mt_reg_sync_writew(v, a)	mt65xx_reg_sync_writew(v, a)
#define mt_reg_sync_writeb(v, a)	mt65xx_reg_sync_writeb(v, a)
#endif
