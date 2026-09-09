/* SPDX-License-Identifier: GPL-2.0 */
/* What the MT6582 CONSYS platform driver collected from the device tree. */
#ifndef _CONSYS_PLAT_H_
#define _CONSYS_PLAT_H_
#include <linux/types.h>

struct device;
struct regmap;
struct regulator;

struct consys_plat {
	struct device *dev;
	struct regmap *pmic;		/* the MT6323 through pwrap */
	struct regulator *vcn18, *vcn28, *vcn33_bt, *vcn33_wifi;
	phys_addr_t emi_phys;
	resource_size_t emi_size;
	bool co_clock;
	bool ready;
};
extern struct consys_plat consys_plat;

struct device *wmt_plat_get_dev(void);

/* consys_wmt.c: the launcher's job, in the kernel */
int consys_wmt_autoconf(void);
int consys_wmt_patch_search(void);

#endif
