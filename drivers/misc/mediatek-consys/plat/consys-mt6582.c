// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6582 CONSYS platform driver (mainline port).
 *
 * The vendor stack (drivers/misc/mediatek/conn_soc + arch/arm/mach-mt6582/
 * <board>/wmt/) reached the connectivity subsystem's blocks through fixed
 * virtual addresses, the clkmgr, upmu_* PMIC helpers and a stolen memblock
 * for the CONSYS EMI window. This driver owns the device-tree node for all of
 * that, publishes the resources the shim headers in include/mach map the old
 * names onto, and hands the rails/regmap to mtk_wcn_consys_hw.c which keeps
 * the vendor power sequence.
 *
 *   consys: connectivity@18070000 {
 *     compatible = "mediatek,mt6582-consys";
 *     reg = spm, infracfg (0x2000: EMI remap at +0x1310, AP2CONN_OSC_EN at
 *           +0x1f00), rgu, conn-mcu, conn-top;
 *     interrupts = the CONN2AP BTIF wakeup line ("bgf");
 *     clocks = infracfg CONNMCU gate ("connmcu");
 *     vcn18/vcn28/vcn33-bt/vcn33-wifi supplies (MT6323 LDOs);
 *     mediatek,pwrap = the PMIC wrapper (its regmap, for the LDO HW-control
 *           bits the regulator driver does not model);
 *     memory-region = a 1 MiB, 1 MiB-aligned CONSYS EMI window;
 *     mediatek,co-clock = the 26 MHz co-clock mode (WMT_SOC.cfg co_clock_flag).
 *   };
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include "consys_res.h"
#include "consys_plat.h"

struct consys_plat consys_plat;

static void __iomem *consys_map(struct platform_device *pdev, const char *name)
{
	struct resource *res;

	/* Plain ioremap: SPM and INFRACFG are shared with the MFG power domain
	 * and the clock controller, which already hold those regions. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return ERR_PTR(-ENODEV);
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int consys_get_regulator(struct device *dev, const char *name,
				struct regulator **out)
{
	struct regulator *r = devm_regulator_get(dev, name);

	if (IS_ERR(r))
		return dev_err_probe(dev, PTR_ERR(r), "%s supply\n", name);
	*out = r;
	return 0;
}

static int consys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct consys_plat *p = &consys_plat;
	struct device_node *np;
	struct platform_device *pwrap;
	struct reserved_mem *rmem;
	struct clk *clk;
	int ret;

	p->dev = dev;

	consys_io_spm = consys_map(pdev, "spm");
	consys_io_infra = consys_map(pdev, "infracfg");
	consys_io_rgu = consys_map(pdev, "rgu");
	consys_io_conn_mcu = consys_map(pdev, "conn-mcu");
	consys_io_conn_top = consys_map(pdev, "conn-top");
	if (IS_ERR(consys_io_spm) || IS_ERR(consys_io_infra) ||
	    IS_ERR(consys_io_rgu) || IS_ERR(consys_io_conn_mcu) ||
	    IS_ERR(consys_io_conn_top))
		return dev_err_probe(dev, -ENODEV, "register windows\n");

	consys_irq_bgf = platform_get_irq_byname(pdev, "bgf");
	if (consys_irq_bgf < 0)
		return consys_irq_bgf;

	clk = devm_clk_get(dev, "connmcu");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "connmcu clock\n");
	consys_clks[CONSYS_CLK_CONNMCU] = clk;

	ret = consys_get_regulator(dev, "vcn18", &p->vcn18);
	if (!ret)
		ret = consys_get_regulator(dev, "vcn28", &p->vcn28);
	if (!ret)
		ret = consys_get_regulator(dev, "vcn33-bt", &p->vcn33_bt);
	if (!ret)
		ret = consys_get_regulator(dev, "vcn33-wifi", &p->vcn33_wifi);
	if (ret)
		return ret;

	/* The PMIC wrapper's regmap, for the LDO hardware-control bits. */
	np = of_parse_phandle(dev->of_node, "mediatek,pwrap", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "mediatek,pwrap missing\n");
	pwrap = of_find_device_by_node(np);
	of_node_put(np);
	if (!pwrap)
		return -EPROBE_DEFER;
	p->pmic = dev_get_regmap(&pwrap->dev, NULL);
	put_device(&pwrap->dev);
	if (!p->pmic)
		return -EPROBE_DEFER;

	/* The CONSYS EMI window: 1 MiB, 1 MiB aligned (the remap register
	 * takes the address >> 20). The vendor stole it from memblock. */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "memory-region missing\n");
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return dev_err_probe(dev, -EINVAL, "memory-region unusable\n");
	if (rmem->size < SZ_1M || (rmem->base & (SZ_1M - 1)))
		return dev_err_probe(dev, -EINVAL,
				     "memory-region must be 1 MiB, 1 MiB aligned\n");
	p->emi_phys = rmem->base;
	p->emi_size = rmem->size;

	p->co_clock = of_property_read_bool(dev->of_node, "mediatek,co-clock");

	dev_info(dev, "MT6582 CONSYS: EMI window %pa (%pa), bgf irq %d, %s\n",
		 &p->emi_phys, &p->emi_size, consys_irq_bgf,
		 p->co_clock ? "co-clock" : "own crystal");
	p->ready = true;
	return 0;
}

static const struct of_device_id consys_of_match[] = {
	{ .compatible = "mediatek,mt6582-consys" },
	{ }
};
MODULE_DEVICE_TABLE(of, consys_of_match);

static struct platform_driver consys_driver = {
	.probe = consys_probe,
	.driver = {
		.name = "mt6582-consys",
		.of_match_table = consys_of_match,
		.suppress_bind_attrs = true,
	},
};

/* Before the WMT/STP/BTIF module_inits (device_initcall level). */
static int __init consys_init(void)
{
	return platform_driver_register(&consys_driver);
}
subsys_initcall_sync(consys_init);

struct device *wmt_plat_get_dev(void)
{
	return consys_plat.dev;
}
EXPORT_SYMBOL(wmt_plat_get_dev);

MODULE_DESCRIPTION("MediaTek MT6582 CONSYS platform glue");
MODULE_LICENSE("GPL");
