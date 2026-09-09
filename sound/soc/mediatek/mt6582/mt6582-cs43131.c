// SPDX-License-Identifier: GPL-2.0
/*
 * mt6582-cs43131.c  --  Innioasis Y2: MT6582 AFE with the CS43131 DAC
 *
 * The Y2 hangs a Cirrus CS43131 headphone DAC off the AFE's I2S output. The
 * DAC is the I2S clock consumer and runs its own 22.5792 MHz crystal, so the
 * codec's PLL takes care of the 48 kHz family. An Awinic AW87559 class-D
 * amplifier sits behind the DAC's output for the speaker; it is an aux
 * component here, switched with the "Speaker" pin.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../../codecs/cs43130.h"

#define MT6582_CS43131_XTAL	22579200

enum {
	DAI_LINK_PLAYBACK,
	DAI_LINK_CODEC_I2S,
};

/*
 * The speaker takes the DAC's output through the amplifier, and the stock
 * driver puts the DAC into its mono differential mode for that (PCM path
 * control 2 = 0x05: both outputs carry the left channel, differential) and
 * back to plain stereo (0x00) for the headphones. Done here, on the speaker
 * pin, so the two never fight: the DAC is stereo whenever the speaker is off.
 */
#define CS43131_PCM_PATH_MONO_DIFF	0x05
#define CS43131_PCM_PATH_STEREO		0x00
#define CS43131_PCM_PATH_MASK		0x07

static int mt6582_cs43131_spk_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = w->dapm->card;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_component *codec;
	unsigned int mode;

	rtd = snd_soc_get_pcm_runtime(card, &card->dai_link[DAI_LINK_CODEC_I2S]);
	if (!rtd)
		return -ENODEV;
	codec = snd_soc_rtd_to_codec(rtd, 0)->component;

	/* SND_SOC_DAPM_SPK widgets fire POST_PMU and PRE_PMD */
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mode = CS43131_PCM_PATH_MONO_DIFF;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mode = CS43131_PCM_PATH_STEREO;
		break;
	default:
		return 0;
	}

	return snd_soc_component_update_bits(codec, CS43130_PCM_PATH_CTL_2,
					     CS43131_PCM_PATH_MASK, mode);
}

static const struct snd_soc_dapm_widget mt6582_cs43131_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", mt6582_cs43131_spk_event),
};

static const struct snd_soc_dapm_route mt6582_cs43131_routes[] = {
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
	{"PA IN", NULL, "HPOUTA"},
	{"PA IN", NULL, "HPOUTB"},
	{"Speaker", NULL, "PA OUT"},
};

static const struct snd_kcontrol_new mt6582_cs43131_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static int mt6582_cs43131_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int sclk;

	/* the AFE sends 32-bit words: 64 bit clocks per frame */
	sclk = params_rate(params) * 32 * 2;
	return snd_soc_dai_set_sysclk(codec_dai, 0, sclk, SND_SOC_CLOCK_IN);
}

static const struct snd_soc_ops mt6582_cs43131_ops = {
	.hw_params = mt6582_cs43131_hw_params,
};

static int mt6582_cs43131_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = snd_soc_rtd_to_codec(rtd, 0)->component;

	/* MCLK is the crystal on the DAC's XTAL pins */
	return snd_soc_component_set_sysclk(component, 0, CS43130_MCLK_SRC_EXT,
					    MT6582_CS43131_XTAL,
					    SND_SOC_CLOCK_IN);
}

SND_SOC_DAILINK_DEFS(playback,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(codec,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "cs43130-asp-pcm")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link mt6582_cs43131_dais[] = {
	/* Front End DAI link */
	[DAI_LINK_PLAYBACK] = {
		.name = "cs43131 Playback",
		.stream_name = "cs43131 Playback",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dynamic = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(playback),
	},
	/* Back End DAI link */
	[DAI_LINK_CODEC_I2S] = {
		.name = "Codec",
		.no_pcm = 1,
		.init = mt6582_cs43131_init,
		.ops = &mt6582_cs43131_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(codec),
	},
};

static struct snd_soc_aux_dev mt6582_cs43131_aux_devs[] = {
	{
		.dlc = COMP_EMPTY(),
	},
};

/* Headphones by default: the speaker pin is raised by whoever owns policy. */
static int mt6582_cs43131_late_probe(struct snd_soc_card *card)
{
	if (card->num_aux_devs)
		snd_soc_dapm_disable_pin(&card->dapm, "Speaker");
	return 0;
}

static struct snd_soc_card mt6582_cs43131_card = {
	.name = "y2-cs43131",
	.owner = THIS_MODULE,
	.late_probe = mt6582_cs43131_late_probe,
	.dai_link = mt6582_cs43131_dais,
	.num_links = ARRAY_SIZE(mt6582_cs43131_dais),
	.controls = mt6582_cs43131_controls,
	.num_controls = ARRAY_SIZE(mt6582_cs43131_controls),
	.dapm_widgets = mt6582_cs43131_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6582_cs43131_widgets),
	.dapm_routes = mt6582_cs43131_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6582_cs43131_routes),
};

static int mt6582_cs43131_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6582_cs43131_card;
	struct device_node *platform_node, *amp_node;
	struct snd_soc_dai_link *dai_link;
	int i, ret;

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}

	for_each_card_prelinks(card, i, dai_link) {
		if (dai_link->platforms->name)
			continue;
		dai_link->platforms->of_node = platform_node;
	}

	mt6582_cs43131_dais[DAI_LINK_CODEC_I2S].codecs[0].of_node =
		of_parse_phandle(pdev->dev.of_node, "mediatek,audio-codec", 0);
	if (!mt6582_cs43131_dais[DAI_LINK_CODEC_I2S].codecs[0].of_node) {
		dev_err(&pdev->dev,
			"Property 'audio-codec' missing or invalid\n");
		ret = -EINVAL;
		goto put_platform_node;
	}

	/* the speaker amplifier is optional: no amp, no "Speaker" path */
	amp_node = of_parse_phandle(pdev->dev.of_node, "mediatek,audio-amp", 0);
	if (amp_node) {
		mt6582_cs43131_aux_devs[0].dlc.of_node = amp_node;
		card->aux_dev = mt6582_cs43131_aux_devs;
		card->num_aux_devs = ARRAY_SIZE(mt6582_cs43131_aux_devs);
	} else {
		/* keep only the headphone routes */
		card->num_dapm_routes = 2;
		card->num_dapm_widgets = 1;
		card->num_controls = 1;
	}

	card->dev = &pdev->dev;

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err_probe(&pdev->dev, ret, "snd_soc_register_card fail\n");

	of_node_put(amp_node);
	of_node_put(mt6582_cs43131_dais[DAI_LINK_CODEC_I2S].codecs[0].of_node);
put_platform_node:
	of_node_put(platform_node);
	return ret;
}

static const struct of_device_id mt6582_cs43131_dt_match[] = {
	{ .compatible = "mediatek,mt6582-cs43131", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6582_cs43131_dt_match);

static struct platform_driver mt6582_cs43131_driver = {
	.driver = {
		   .name = "mt6582-cs43131",
		   .of_match_table = mt6582_cs43131_dt_match,
		   .pm = &snd_soc_pm_ops,
	},
	.probe = mt6582_cs43131_dev_probe,
};

module_platform_driver(mt6582_cs43131_driver);

MODULE_DESCRIPTION("MT6582 CS43131 ALSA SoC machine driver (Innioasis Y2)");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mt6582-cs43131");
