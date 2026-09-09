// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO + EINT (external interrupt) controller driver for the MediaTek MT6582.
 *
 * GPIO block (phys 0x10005000): classic MediaTek "v1" layout - each functional
 * register group covers 16 GPIOs at a 0x10 stride, and the writable blocks
 * provide atomic SET (+0x4) / CLR (+0x8) aliases beside the value reg (+0x0):
 *     DIR 0x000 (1=out), DOUT 0x400, DIN 0x500 (ro), MODE 0x600.
 *
 * EINT block (phys 0x1000B000): register-identical to the mainline
 * mtk_generic_eint_regs, so we reuse drivers/pinctrl/mediatek/mtk-eint.c for the
 * whole irqchip/irqdomain/wakeup machinery. On the MT6582 the EINT number equals
 * the GPIO number (no translation table), there are 169 EINTs across 6 ports,
 * and hardware debounce exists only for EINT <= 15.
 *
 * All register offsets + the EINT==GPIO mapping were confirmed by disassembling
 * the vendor kernel (mt_get_gpio_in / mt_eint_* accessors). The bootloader
 * already muxes the pins this board uses; this driver does GPIO in/out + EINT.
 */
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "../pinctrl/mediatek/mtk-eint.h"

#define MT6582_GPIO_DIR		0x000
#define MT6582_GPIO_DOUT	0x400
#define MT6582_GPIO_DIN		0x500
#define MT6582_REG_SET		0x4
#define MT6582_REG_CLR		0x8
#define MT6582_GPIO_STRIDE	0x10	/* bytes per 16-GPIO group */

struct mt6582_gpio {
	struct gpio_chip chip;
	void __iomem *base;
	struct mtk_eint eint;
};

static void __iomem *mt6582_reg(struct mt6582_gpio *g, u32 block,
				unsigned int off, unsigned int variant)
{
	return g->base + block + (off / 16) * MT6582_GPIO_STRIDE + variant;
}

static int mt6582_gpio_get(struct gpio_chip *chip, unsigned int off)
{
	struct mt6582_gpio *g = gpiochip_get_data(chip);

	return !!(readl(mt6582_reg(g, MT6582_GPIO_DIN, off, 0)) & BIT(off % 16));
}

static void mt6582_gpio_set(struct gpio_chip *chip, unsigned int off, int val)
{
	struct mt6582_gpio *g = gpiochip_get_data(chip);

	writel(BIT(off % 16),
	       mt6582_reg(g, MT6582_GPIO_DOUT, off,
			  val ? MT6582_REG_SET : MT6582_REG_CLR));
}

static int mt6582_gpio_get_direction(struct gpio_chip *chip, unsigned int off)
{
	struct mt6582_gpio *g = gpiochip_get_data(chip);

	return (readl(mt6582_reg(g, MT6582_GPIO_DIR, off, 0)) & BIT(off % 16)) ?
		GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int mt6582_gpio_direction_input(struct gpio_chip *chip, unsigned int off)
{
	struct mt6582_gpio *g = gpiochip_get_data(chip);

	writel(BIT(off % 16), mt6582_reg(g, MT6582_GPIO_DIR, off, MT6582_REG_CLR));
	return 0;
}

static int mt6582_gpio_direction_output(struct gpio_chip *chip,
					unsigned int off, int val)
{
	struct mt6582_gpio *g = gpiochip_get_data(chip);

	mt6582_gpio_set(chip, off, val);
	writel(BIT(off % 16), mt6582_reg(g, MT6582_GPIO_DIR, off, MT6582_REG_SET));
	return 0;
}

static int mt6582_gpio_to_irq(struct gpio_chip *chip, unsigned int off)
{
	struct mt6582_gpio *g = gpiochip_get_data(chip);

	return mtk_eint_find_irq(&g->eint, off);
}

/* ---- EINT glue: EINT number == GPIO number on the MT6582 ---- */

static int mt6582_xt_get_gpio_n(void *data, unsigned long eint_n,
				unsigned int *gpio_n, struct gpio_chip **gc)
{
	struct mt6582_gpio *g = data;

	*gpio_n = eint_n;
	*gc = &g->chip;
	return 0;
}

static int mt6582_xt_get_gpio_state(void *data, unsigned long eint_n)
{
	struct mt6582_gpio *g = data;

	return mt6582_gpio_get(&g->chip, eint_n);
}

static int mt6582_xt_set_gpio_as_eint(void *data, unsigned long eint_n)
{
	struct mt6582_gpio *g = data;

	/* LK already muxes these pins to EINT function; ensure input. */
	return mt6582_gpio_direction_input(&g->chip, eint_n);
}

static const struct mtk_eint_xt mt6582_eint_xt = {
	.get_gpio_n	= mt6582_xt_get_gpio_n,
	.get_gpio_state	= mt6582_xt_get_gpio_state,
	.set_gpio_as_eint = mt6582_xt_set_gpio_as_eint,
};

static const struct mtk_eint_hw mt6582_eint_hw = {
	.port_mask	= 7,
	.ports		= 6,		/* 169 EINTs -> 6 x 32-bit ports */
	.ap_num		= 169,
	.db_cnt		= 16,		/* hardware debounce for EINT 0..15 */
	.db_time	= debounce_time_mt2701,
};

static int mt6582_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6582_gpio *g;
	u32 ngpio = 169;	/* GPIO number space == EINT space */
	int irq, ret;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;

	g->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(g->base))
		return PTR_ERR(g->base);

	device_property_read_u32(dev, "ngpios", &ngpio);

	g->chip.label = dev_name(dev);
	g->chip.parent = dev;
	g->chip.owner = THIS_MODULE;
	g->chip.base = -1;
	g->chip.ngpio = ngpio;
	g->chip.request = gpiochip_generic_request;
	g->chip.free = gpiochip_generic_free;
	g->chip.get = mt6582_gpio_get;
	g->chip.set = mt6582_gpio_set;
	g->chip.get_direction = mt6582_gpio_get_direction;
	g->chip.direction_input = mt6582_gpio_direction_input;
	g->chip.direction_output = mt6582_gpio_direction_output;
	g->chip.to_irq = mt6582_gpio_to_irq;

	ret = devm_gpiochip_add_data(dev, &g->chip, g);
	if (ret)
		return ret;

	/* EINT irqchip, reusing the mainline mtk-eint library. */
	g->eint.dev = dev;
	g->eint.hw = &mt6582_eint_hw;
	g->eint.pctl = g;
	g->eint.gpio_xlate = &mt6582_eint_xt;

	g->eint.base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(g->eint.base))
		return PTR_ERR(g->eint.base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	g->eint.irq = irq;

	ret = mtk_eint_do_init(&g->eint);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init EINT\n");

	platform_set_drvdata(pdev, g);
	return 0;
}

static int mt6582_gpio_suspend(struct device *dev)
{
	struct mt6582_gpio *g = dev_get_drvdata(dev);

	return mtk_eint_do_suspend(&g->eint);
}

static int mt6582_gpio_resume(struct device *dev)
{
	struct mt6582_gpio *g = dev_get_drvdata(dev);

	return mtk_eint_do_resume(&g->eint);
}

static DEFINE_SIMPLE_DEV_PM_OPS(mt6582_gpio_pm_ops,
				mt6582_gpio_suspend, mt6582_gpio_resume);

static const struct of_device_id mt6582_gpio_of_match[] = {
	{ .compatible = "mediatek,mt6582-gpio" },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6582_gpio_of_match);

static struct platform_driver mt6582_gpio_driver = {
	.probe = mt6582_gpio_probe,
	.driver = {
		.name = "mt6582-gpio",
		.of_match_table = mt6582_gpio_of_match,
		.pm = pm_sleep_ptr(&mt6582_gpio_pm_ops),
	},
};
module_platform_driver(mt6582_gpio_driver);

MODULE_DESCRIPTION("MediaTek MT6582 GPIO + EINT driver");
MODULE_LICENSE("GPL");
