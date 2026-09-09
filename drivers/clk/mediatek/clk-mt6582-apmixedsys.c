// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6582 apmixedsys (main PLL) clock driver.
 *
 * Register offsets and the enable/pcw layout are from the vendor GPL kernel
 * (android-mediatek-sprout-3.4) mt_clkmgr.c `plls[]`. The PLLs are already
 * running as the preloader/LK configured them; Linux mostly needs to expose
 * them (and their derived rates) to the rest of the clock tree. They are marked
 * critical in DT-less consumers via the topckgen factors, so we never gate them.
 *
 * MT6582 SDM PLL n_info layout (CON1): bits[20:14] integer, bits[13:0] frac,
 * i.e. a 21-bit PCW with 7 integer + 14 fractional bits — exactly the mainline
 * mtk_pll model (INTEGER_BITS=7, pcwbits=21). The post-divider model here is an
 * approximation of the vendor's vcodivsel(CON0[19]) + prediv(CON0[5:4]) scheme
 * (ARMPLL additionally has a 3-bit post-divider in CON1[26:24]); reported rates
 * should be calibrated against /sys/kernel/debug/clk/clk_summary on-device.
 */
#include <dt-bindings/clock/mt6582-clk.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-pll.h"

#define MT6582_PLL_FMAX		(2000 * MHZ)
#define CON0_MT6582_RST_BAR	BIT(24)

#define PLL(_id, _name, _reg, _pwr_reg, _en_mask, _flags, _pd_reg, _pd_shift) { \
		.id = _id,						\
		.name = _name,						\
		.reg = _reg,						\
		.pwr_reg = _pwr_reg,					\
		.en_mask = _en_mask,					\
		.flags = _flags,					\
		.rst_bar_mask = CON0_MT6582_RST_BAR,			\
		.fmax = MT6582_PLL_FMAX,				\
		.pcwbits = 21,						\
		.pd_reg = _pd_reg,					\
		.pd_shift = _pd_shift,					\
		.tuner_reg = 0,						\
		.pcw_reg = (_reg) + 4,					\
		.pcw_shift = 0,						\
	}

/*
 * PLL_AO marks the always-on PLLs (CLK_IS_CRITICAL): armpll feeds the CPU,
 * mainpll the bus/AXI (and the CPU DVFS intermediate), univpll is fixed-freq
 * and feeds USB/peri, mmpll feeds the GPU/mm path. The vendor clkmgr presets
 * these with a refcount and never releases them; without CLK_IS_CRITICAL the
 * core clk_disable_unused sweep would gate them off (armpll -> CPU hang) since
 * their consumers are not all migrated to the CCF yet. MSDCPLL is intentionally
 * NOT always-on: mtk-sd now claims it on demand, so it idle-gates for real.
 */
static const struct mtk_pll_data plls[] = {
	/* ARMPLL: post-divider lives in CON1[26:24] */
	PLL(CLK_APMIXED_ARMPLL, "armpll", 0x200, 0x20c, 0x00000001, PLL_AO, 0x204, 24),
	PLL(CLK_APMIXED_MAINPLL, "mainpll", 0x210, 0x21c, 0x78000001, HAVE_RST_BAR | PLL_AO, 0x210, 4),
	PLL(CLK_APMIXED_UNIVPLL, "univpll", 0x220, 0x22c, 0xfc000001, HAVE_RST_BAR | PLL_AO, 0x220, 4),
	PLL(CLK_APMIXED_MMPLL, "mmpll", 0x230, 0x23c, 0x00000001, PLL_AO, 0x230, 4),
	PLL(CLK_APMIXED_MSDCPLL, "msdcpll", 0x240, 0x24c, 0x00000001, 0, 0x240, 4),
};

static int clk_mt6582_apmixed_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	clk_data = mtk_alloc_clk_data(CLK_APMIXED_NR_CLK);
	if (!clk_data)
		return -ENOMEM;

	ret = mtk_clk_register_plls(node, plls, ARRAY_SIZE(plls), clk_data);
	if (ret)
		goto free_clk_data;

	ret = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, clk_data);
	if (ret)
		goto unregister_plls;

	return 0;

unregister_plls:
	mtk_clk_unregister_plls(plls, ARRAY_SIZE(plls), clk_data);
free_clk_data:
	mtk_free_clk_data(clk_data);

	return ret;
}

static void clk_mt6582_apmixed_remove(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_hw_onecell_data *clk_data = platform_get_drvdata(pdev);

	of_clk_del_provider(node);
	mtk_clk_unregister_plls(plls, ARRAY_SIZE(plls), clk_data);
	mtk_free_clk_data(clk_data);
}

static const struct of_device_id of_match_clk_mt6582_apmixed[] = {
	{ .compatible = "mediatek,mt6582-apmixedsys" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6582_apmixed);

static struct platform_driver clk_mt6582_apmixed_drv = {
	.probe = clk_mt6582_apmixed_probe,
	.remove = clk_mt6582_apmixed_remove,
	.driver = {
		.name = "clk-mt6582-apmixed",
		.of_match_table = of_match_clk_mt6582_apmixed,
	},
};
module_platform_driver(clk_mt6582_apmixed_drv);

MODULE_DESCRIPTION("MediaTek MT6582 apmixedsys clocks driver");
MODULE_LICENSE("GPL");
