// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal keypad driver for the MediaTek MT6582 (Innioasis Y2 volume keys).
 *
 * The MT6582 keypad controller (phys 0x10011000) auto-scans its matrix when
 * enabled (there is no per-row/col SEL register like the mt6779 IP) and holds
 * the debounced key state in the low 16 bits of KP_MEM1 (a pressed key CLEARS
 * its bit; idle = 0xffff). It raises a GIC interrupt (edge, vendor Linux IRQ
 * 148 = GIC_SPI 116) on every debounced state change; like the mainline
 * mt6779-keypad, the handler just re-reads KP_MEM1 - no ack register.
 *
 * The Y2 populates only two matrix keys: KP_MEM1 bit0 = Vol-Down, bit1 = Vol-Up
 * (confirmed on device). Bit->keycode comes from the DT "linux,keycodes".
 */
#include <linux/bitops.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define MT6582_KP_MEM1		0x04
#define MT6582_KP_EN		0x24	/* bit0 = scan enable */
#define MT6582_KP_NBITS		16

struct mt6582_keypad {
	void __iomem *base;
	struct input_dev *input;
	u16 last;
	unsigned short keycodes[MT6582_KP_NBITS];
	int nkeys;
};

static irqreturn_t mt6582_keypad_irq(int irq, void *dev_id)
{
	struct mt6582_keypad *kp = dev_id;
	u16 state = readl(kp->base + MT6582_KP_MEM1) & 0xffff;
	u16 change = state ^ kp->last;
	int i;

	for (i = 0; i < kp->nkeys; i++) {
		if (!(change & BIT(i)) || !kp->keycodes[i])
			continue;
		/* bit set = released, bit clear = pressed */
		input_report_key(kp->input, kp->keycodes[i], !(state & BIT(i)));
		input_sync(kp->input);
	}

	kp->last = state;
	return IRQ_HANDLED;
}

static int mt6582_keypad_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6582_keypad *kp;
	u32 codes[MT6582_KP_NBITS];
	int i, irq, ret;

	kp = devm_kzalloc(dev, sizeof(*kp), GFP_KERNEL);
	if (!kp)
		return -ENOMEM;

	kp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(kp->base))
		return PTR_ERR(kp->base);

	kp->nkeys = device_property_count_u32(dev, "linux,keycodes");
	if (kp->nkeys <= 0 || kp->nkeys > MT6582_KP_NBITS)
		return dev_err_probe(dev, -EINVAL, "bad linux,keycodes\n");
	ret = device_property_read_u32_array(dev, "linux,keycodes", codes, kp->nkeys);
	if (ret)
		return ret;
	for (i = 0; i < kp->nkeys; i++)
		kp->keycodes[i] = codes[i];

	/* Ensure the matrix is scanning (LK usually leaves it on). */
	writel(readl(kp->base + MT6582_KP_EN) | BIT(0), kp->base + MT6582_KP_EN);

	kp->input = devm_input_allocate_device(dev);
	if (!kp->input)
		return -ENOMEM;
	kp->input->name = "mt6582-keypad";
	kp->input->id.bustype = BUS_HOST;
	for (i = 0; i < kp->nkeys; i++)
		if (kp->keycodes[i])
			input_set_capability(kp->input, EV_KEY, kp->keycodes[i]);

	kp->last = readl(kp->base + MT6582_KP_MEM1) & 0xffff;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL, mt6582_keypad_irq,
					IRQF_ONESHOT, "mt6582-keypad", kp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	return input_register_device(kp->input);
}

static const struct of_device_id mt6582_keypad_of_match[] = {
	{ .compatible = "mediatek,mt6582-keypad" },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6582_keypad_of_match);

static struct platform_driver mt6582_keypad_driver = {
	.probe = mt6582_keypad_probe,
	.driver = {
		.name = "mt6582-keypad",
		.of_match_table = mt6582_keypad_of_match,
	},
};
module_platform_driver(mt6582_keypad_driver);

MODULE_DESCRIPTION("MediaTek MT6582 keypad (volume keys) driver");
MODULE_LICENSE("GPL");
