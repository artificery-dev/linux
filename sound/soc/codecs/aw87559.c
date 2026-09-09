// SPDX-License-Identifier: GPL-2.0-only
/*
 * aw87559.c  --  Awinic AW87559 audio power amplifier
 *
 * A class-D speaker amplifier configured over I2C (8-bit registers) with an
 * enable line. The register tables are the ones the Innioasis Y2's shipping
 * kernel writes (recovered from its aw87559_on/aw87559_off); the OCA72559 is
 * the drop-in alternative that vendor driver also handles, with its own table.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#define AW87559_REG_CHIPID	0x00
#define AW87559_REG_SYSCTRL	0x01

#define AW87559_CHIPID		0x5a
#define OCA72559_CHIPID		0x09

#define AW87559_ID_RETRIES	5

struct aw87559_priv {
	struct regmap *regmap;
	struct device *dev;
	struct gpio_desc *enable_gpio;
	unsigned int chipid;
};

/* aw87559_on(): the vendor's full register image, SYSCTRL last */
static const struct reg_sequence aw87559_on_seq[] = {
	{ 0x62, 0xb5 }, { 0x78, 0x39 }, { 0x79, 0xe5 }, { 0x77, 0xc1 },
	{ 0x77, 0xc1 }, { 0x78, 0x7a }, { 0x79, 0x6c }, { 0x77, 0x81 },
	{ 0x66, 0x69 }, { 0x58, 0xbc },
	{ 0x02, 0x16 }, { 0x03, 0x0a }, { 0x04, 0x80 }, { 0x05, 0x08 },
	{ 0x06, 0x0c }, { 0x07, 0x85 }, { 0x08, 0x8e }, { 0x09, 0x04 },
	{ 0x0a, 0x08 }, { 0x0b, 0x4a }, { 0x0c, 0x03 }, { 0x0d, 0x77 },
	{ 0x0e, 0x7a }, { 0x0f, 0xa3 }, { 0x10, 0x58 },
	{ 0x60, 0x26 }, { 0x61, 0x15 }, { 0x63, 0x5a }, { 0x64, 0xd5 },
	{ 0x65, 0x57 }, { 0x67, 0x28 }, { 0x68, 0x35 }, { 0x69, 0x98 },
	{ 0x70, 0x1c }, { 0x71, 0x9c }, { 0x72, 0x33 }, { 0x73, 0x40 },
	{ 0x74, 0x6c },
	{ 0x01, 0x30 }, { 0x01, 0x78 },
};

/* oca72559_on() */
static const struct reg_sequence oca72559_on_seq[] = {
	{ 0x02, 0x09 }, { 0x03, 0x0a }, { 0x04, 0x02 }, { 0x05, 0x09 },
	{ 0x06, 0x01 }, { 0x07, 0x54 }, { 0x08, 0x4e }, { 0x09, 0x0b },
	{ 0x0a, 0x10 }, { 0x0b, 0x4f }, { 0x0c, 0x00 }, { 0x0d, 0xdd },
	{ 0x0f, 0x23 },
	{ 0x01, 0x38 }, { 0x01, 0x7c },
};

/* Pull the enable line as the vendor does: low, 10 ms, high, 100 ms. */
static void aw87559_power_pulse(struct aw87559_priv *aw)
{
	gpiod_set_value_cansleep(aw->enable_gpio, 0);
	msleep(10);
	gpiod_set_value_cansleep(aw->enable_gpio, 1);
	msleep(100);
}

static int aw87559_read_id(struct aw87559_priv *aw, unsigned int *id)
{
	int ret = regmap_read(aw->regmap, AW87559_REG_CHIPID, id);

	if (ret)
		return ret;
	if (*id != AW87559_CHIPID && *id != OCA72559_CHIPID)
		return -ENODEV;
	return 0;
}

static int aw87559_enable(struct aw87559_priv *aw)
{
	unsigned int id;
	int i, ret;

	for (i = 0; i <= AW87559_ID_RETRIES; i++) {
		aw87559_power_pulse(aw);
		ret = aw87559_read_id(aw, &id);
		if (!ret)
			break;
		dev_warn(aw->dev, "no valid chip id after power-up (%d), retry %d\n",
			 ret, i + 1);
	}
	if (ret) {
		gpiod_set_value_cansleep(aw->enable_gpio, 0);
		return ret;
	}

	if (id == OCA72559_CHIPID)
		ret = regmap_multi_reg_write(aw->regmap, oca72559_on_seq,
					     ARRAY_SIZE(oca72559_on_seq));
	else
		ret = regmap_multi_reg_write(aw->regmap, aw87559_on_seq,
					     ARRAY_SIZE(aw87559_on_seq));
	if (ret)
		return ret;

	/* the vendor path waits this long before unmuting the DAC */
	msleep(80);
	return 0;
}

static void aw87559_disable(struct aw87559_priv *aw)
{
	unsigned int id;

	/* aw87559_off(): mute in SYSCTRL, then drop the enable line */
	if (!aw87559_read_id(aw, &id))
		regmap_write(aw->regmap, AW87559_REG_SYSCTRL, 0x30);
	gpiod_set_value_cansleep(aw->enable_gpio, 0);
	usleep_range(500, 1000);
}

static int aw87559_drv_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct aw87559_priv *aw = snd_soc_component_get_drvdata(c);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		return aw87559_enable(aw);
	case SND_SOC_DAPM_PRE_PMD:
		aw87559_disable(aw);
		return 0;
	default:
		WARN(1, "Unexpected event");
		return -EINVAL;
	}
}

static const struct snd_soc_dapm_widget aw87559_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_OUT_DRV_E("DRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       aw87559_drv_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route aw87559_dapm_routes[] = {
	{ "DRV", NULL, "IN" },
	{ "OUT", NULL, "DRV" },
};

static const struct snd_soc_component_driver aw87559_component_driver = {
	.dapm_widgets = aw87559_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw87559_dapm_widgets),
	.dapm_routes = aw87559_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(aw87559_dapm_routes),
};

static const struct regmap_config aw87559_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int aw87559_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw87559_priv *aw;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw->dev = dev;
	aw->regmap = devm_regmap_init_i2c(client, &aw87559_regmap_config);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(dev, PTR_ERR(aw->regmap),
				     "failed to init regmap\n");

	aw->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(aw->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(aw->enable_gpio),
				     "failed to get the enable gpio\n");

	/* the chip only answers with its enable line up; look, then sleep */
	aw87559_power_pulse(aw);
	ret = aw87559_read_id(aw, &aw->chipid);
	gpiod_set_value_cansleep(aw->enable_gpio, 0);
	if (ret)
		return dev_err_probe(dev, ret, "chip id 0x%02x not recognised\n",
				     aw->chipid);

	dev_info(dev, "%s (chip id 0x%02x)\n",
		 aw->chipid == OCA72559_CHIPID ? "OCA72559" : "AW87559",
		 aw->chipid);

	i2c_set_clientdata(client, aw);

	return devm_snd_soc_register_component(dev, &aw87559_component_driver,
					       NULL, 0);
}

static const struct of_device_id aw87559_of_match[] = {
	{ .compatible = "awinic,aw87559" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw87559_of_match);

static const struct i2c_device_id aw87559_i2c_id[] = {
	{ "aw87559" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw87559_i2c_id);

static struct i2c_driver aw87559_i2c_driver = {
	.driver = {
		.name = "aw87559",
		.of_match_table = aw87559_of_match,
	},
	.probe = aw87559_i2c_probe,
	.id_table = aw87559_i2c_id,
};
module_i2c_driver(aw87559_i2c_driver);

MODULE_DESCRIPTION("Awinic AW87559 Amplifier Driver");
MODULE_LICENSE("GPL v2");
