// SPDX-License-Identifier: GPL-2.0
/*
 * Innioasis Y2 APT32F click-wheel - scroll/rotation input driver.
 *
 * The APT32F touch-wheel controller (i2c @0x51) reports its 5 nav zones as
 * dedicated GPIO/EINT lines (handled by gpio-keys); wheel ROTATION is reported
 * over i2c instead. The controller pulses a data-ready line (GPIO/EINT 55) when
 * a new frame is ready; the frame lives in the chip's register file and is read
 * back starting at register 0:
 *
 *     reg0=0xAA reg1=0x55(valid) reg2=class reg3=nav-idx reg4=scroll-code ...
 *
 * We only handle class 3 (wheel scroll); reg4 selects the direction. Frame
 * layout + the data-ready line were reverse-engineered from the vendor kernel.
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>

#define APT32F_FRAME_LEN	9
#define APT32F_VALID		0x55		/* reg1 */
#define APT32F_CLASS_SCROLL	3		/* reg2 */

struct apt32f_wheel {
	struct i2c_client *client;
	struct input_dev *input;
};

/* reg4 scroll-code -> key. Tap = press+release, like a scroll wheel notch. */
static const unsigned short apt32f_scroll_keys[] = {
	[1] = KEY_UP,
	[2] = KEY_PAGEUP,
	[3] = KEY_DOWN,
	[4] = KEY_PAGEDOWN,
};

static irqreturn_t apt32f_irq(int irq, void *dev_id)
{
	struct apt32f_wheel *w = dev_id;
	u8 reg = 0;
	u8 f[APT32F_FRAME_LEN];
	struct i2c_msg msgs[2] = {
		{ .addr = w->client->addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = w->client->addr, .flags = I2C_M_RD,
		  .len = APT32F_FRAME_LEN, .buf = f },
	};
	unsigned short key;

	/*
	 * Read the frame aligned to reg0: write the register pointer (0) then
	 * repeated-start read the register file. A plain read starts at the
	 * chip's wandering internal pointer and mostly misses the AA/55 header.
	 */
	if (i2c_transfer(w->client->adapter, msgs, 2) != 2)
		return IRQ_HANDLED;

	if (f[1] != APT32F_VALID || f[2] != APT32F_CLASS_SCROLL)
		return IRQ_HANDLED;

	if (f[4] >= ARRAY_SIZE(apt32f_scroll_keys))
		return IRQ_HANDLED;

	key = apt32f_scroll_keys[f[4]];
	if (!key)
		return IRQ_HANDLED;

	input_report_key(w->input, key, 1);
	input_sync(w->input);
	input_report_key(w->input, key, 0);
	input_sync(w->input);

	return IRQ_HANDLED;
}

static int apt32f_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct apt32f_wheel *w;
	int i, ret;

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "no data-ready irq\n");

	w = devm_kzalloc(dev, sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->client = client;

	w->input = devm_input_allocate_device(dev);
	if (!w->input)
		return -ENOMEM;

	w->input->name = "APT32F click-wheel";
	w->input->id.bustype = BUS_I2C;
	w->input->dev.parent = dev;
	for (i = 0; i < ARRAY_SIZE(apt32f_scroll_keys); i++)
		if (apt32f_scroll_keys[i])
			input_set_capability(w->input, EV_KEY, apt32f_scroll_keys[i]);

	ret = input_register_device(w->input);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL, apt32f_irq,
					IRQF_ONESHOT, "apt32f-wheel", w);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	return 0;
}

static const struct of_device_id apt32f_of_match[] = {
	{ .compatible = "innioasis,apt32f-wheel" },
	{ },
};
MODULE_DEVICE_TABLE(of, apt32f_of_match);

static struct i2c_driver apt32f_driver = {
	.probe = apt32f_probe,
	.driver = {
		.name = "apt32f-wheel",
		.of_match_table = apt32f_of_match,
	},
};
module_i2c_driver(apt32f_driver);

MODULE_DESCRIPTION("Innioasis Y2 APT32F click-wheel scroll driver");
MODULE_LICENSE("GPL");
