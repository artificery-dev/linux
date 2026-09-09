// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6582 ALSA SoC AFE platform driver
 *
 * A cut-down variant of the MT8173 driver for the MT8135-generation AFE in the
 * MT6582: one playback memory interface (DL1) feeding the "2nd I2S out", the
 * I2S whose pads leave the chip (AFE_I2S_CON1 only feeds the on-chip ADDA),
 * driven from the 26 MHz audio clock (no audio PLL on this SoC).
 *
 * Register layout from the vendor 3.4 driver (drivers/misc/mediatek/sound/
 * mt6582/AudDrv_Afe.h); the memif/irq framework is the shared mediatek code.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include "mt6582-afe-common.h"
#include "../common/mtk-base-afe.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-afe-fe-dai.h"

static const unsigned int mt6582_afe_backup_list[] = {
	AUDIO_TOP_CON0,
	AFE_CONN0,
	AFE_DAC_CON1,
	AFE_DL1_BASE,
	AFE_DL1_END,
	AFE_I2S_CON3,
	FPGA_CFG1,
	AFE_DAC_CON0,
};

struct mt6582_afe_private {
	struct clk *clocks[MT6582_CLK_NUM];
};

static const struct snd_pcm_hardware mt6582_afe_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.buffer_bytes_max = 256 * 1024,
	.period_bytes_min = 512,
	.period_bytes_max = 128 * 1024,
	.periods_min = 2,
	.periods_max = 256,
	.fifo_size = 0,
};

struct mt6582_afe_rate {
	unsigned int rate;
	unsigned int regvalue;
};

/* The fs encoding shared by DAC_CON1, I2S_CON1 and IRQ_MCU_CON. */
static const struct mt6582_afe_rate mt6582_afe_i2s_rates[] = {
	{ .rate = 8000, .regvalue = 0 },
	{ .rate = 11025, .regvalue = 1 },
	{ .rate = 12000, .regvalue = 2 },
	{ .rate = 16000, .regvalue = 4 },
	{ .rate = 22050, .regvalue = 5 },
	{ .rate = 24000, .regvalue = 6 },
	{ .rate = 32000, .regvalue = 8 },
	{ .rate = 44100, .regvalue = 9 },
	{ .rate = 48000, .regvalue = 10 },
};

static int mt6582_afe_i2s_fs(unsigned int sample_rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt6582_afe_i2s_rates); i++)
		if (mt6582_afe_i2s_rates[i].rate == sample_rate)
			return mt6582_afe_i2s_rates[i].regvalue;

	return -EINVAL;
}

static int mt6582_afe_set_i2s(struct mtk_base_afe *afe, unsigned int rate)
{
	unsigned int val;
	int fs = mt6582_afe_i2s_fs(rate);

	if (fs < 0)
		return -EINVAL;

	/* DL1 (I05/I06) into the 2nd I2S out (O00/O01) */
	regmap_update_bits(afe->regmap, AFE_CONN0,
			   AFE_CONN0_I05_O00 | AFE_CONN0_I06_O01,
			   AFE_CONN0_I05_O00 | AFE_CONN0_I06_O01);

	/* 2nd I2S out: 32-bit words, I2S framing, clock provider */
	val = AFE_I2S_CON3_RATE(fs) | AFE_I2S_CON3_FORMAT_I2S |
	      AFE_I2S_CON3_WLEN_32BIT;
	regmap_update_bits(afe->regmap, AFE_I2S_CON3, ~AFE_I2S_CON3_EN, val);
	regmap_update_bits(afe->regmap, FPGA_CFG1, FPGA_CFG1_I2S_OUT,
			   FPGA_CFG1_I2S_OUT);

	return 0;
}

static void mt6582_afe_set_i2s_enable(struct mtk_base_afe *afe, bool enable)
{
	unsigned int val;

	regmap_read(afe->regmap, AFE_I2S_CON3, &val);
	if (!!(val & AFE_I2S_CON3_EN) == enable)
		return;

	regmap_update_bits(afe->regmap, AFE_I2S_CON3, AFE_I2S_CON3_EN,
			   enable ? AFE_I2S_CON3_EN : 0);
}

static int mt6582_afe_i2s_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	if (snd_soc_dai_active(dai))
		return 0;

	regmap_update_bits(afe->regmap, AUDIO_TOP_CON0, AUD_TCON0_PDN_I2S, 0);
	return 0;
}

static void mt6582_afe_i2s_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	if (snd_soc_dai_active(dai))
		return;

	mt6582_afe_set_i2s_enable(afe, false);
	regmap_update_bits(afe->regmap, AUDIO_TOP_CON0, AUD_TCON0_PDN_I2S,
			   AUD_TCON0_PDN_I2S);
}

static int mt6582_afe_i2s_prepare(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime * const runtime = substream->runtime;
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = mt6582_afe_set_i2s(afe, runtime->rate);
	if (ret)
		return ret;

	mt6582_afe_set_i2s_enable(afe, true);
	return 0;
}

static int mt6582_memif_fs(struct snd_pcm_substream *substream,
			   unsigned int rate)
{
	return mt6582_afe_i2s_fs(rate);
}

static int mt6582_irq_fs(struct snd_pcm_substream *substream, unsigned int rate)
{
	return mt6582_afe_i2s_fs(rate);
}

static const struct snd_soc_dai_ops mt6582_afe_i2s_ops = {
	.startup	= mt6582_afe_i2s_startup,
	.shutdown	= mt6582_afe_i2s_shutdown,
	.prepare	= mt6582_afe_i2s_prepare,
};

static struct snd_soc_dai_driver mt6582_afe_pcm_dais[] = {
	/* FE DAI: the DL1 memory interface */
	{
		.name = "DL1",
		.id = MT6582_AFE_MEMIF_DL1,
		.playback = {
			.stream_name = "DL1",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &mtk_afe_fe_ops,
	}, {
	/* BE DAI: the I2S output pads */
		.name = "I2S",
		.id = MT6582_AFE_IO_I2S,
		.playback = {
			.stream_name = "I2S Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &mt6582_afe_i2s_ops,
		.symmetric_rate = 1,
	},
};

static const struct snd_soc_dapm_route mt6582_afe_pcm_routes[] = {
	{"I2S Playback", NULL, "DL1"},
};

static const struct snd_soc_component_driver mt6582_afe_pcm_dai_component = {
	.name = "mt6582-afe-pcm-dai",
	.dapm_routes = mt6582_afe_pcm_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6582_afe_pcm_routes),
	.suspend = mtk_afe_suspend,
	.resume = mtk_afe_resume,
};

static const char *aud_clks[MT6582_CLK_NUM] = {
	[MT6582_CLK_INFRA_AUDIO] = "infra_audio",
	[MT6582_CLK_TOP_AUDINTBUS] = "audintbus",
	[MT6582_CLK_TOP_AUDIO] = "audio",
};

static const struct mtk_base_memif_data memif_data[MT6582_AFE_MEMIF_NUM] = {
	{
		.name = "DL1",
		.id = MT6582_AFE_MEMIF_DL1,
		.reg_ofs_base = AFE_DL1_BASE,
		.reg_ofs_cur = AFE_DL1_CUR,
		.fs_reg = AFE_DAC_CON1,
		.fs_shift = 0,
		.fs_maskbit = 0xf,
		.mono_reg = AFE_DAC_CON1,
		.mono_shift = 21,
		.hd_reg = -1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = 1,
		.msb_reg = -1,
		.agent_disable_reg = -1,
	},
};

static const struct mtk_base_irq_data irq_data[MT6582_AFE_IRQ_NUM] = {
	{
		.id = MT6582_AFE_IRQ_DL1,
		.irq_cnt_reg = AFE_IRQ_CNT1,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_en_reg = AFE_IRQ_MCU_CON,
		.irq_en_shift = 0,
		.irq_fs_reg = AFE_IRQ_MCU_CON,
		.irq_fs_shift = 4,
		.irq_fs_maskbit = 0xf,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = 0,
	},
};

static const struct regmap_config mt6582_afe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = MT6582_AFE_MAX_REGISTER,
	.cache_type = REGCACHE_NONE,
};

static irqreturn_t mt6582_afe_irq_handler(int irq, void *dev_id)
{
	struct mtk_base_afe *afe = dev_id;
	unsigned int reg_value;
	int i, ret;

	ret = regmap_read(afe->regmap, AFE_IRQ_STATUS, &reg_value);
	if (ret) {
		dev_err(afe->dev, "%s irq status err\n", __func__);
		reg_value = AFE_IRQ_STATUS_BITS;
		goto err_irq;
	}

	for (i = 0; i < MT6582_AFE_MEMIF_NUM; i++) {
		struct mtk_base_afe_memif *memif = &afe->memif[i];
		struct mtk_base_afe_irq *irq_p;

		if (memif->irq_usage < 0)
			continue;

		irq_p = &afe->irqs[memif->irq_usage];

		if (!(reg_value & (1 << irq_p->irq_data->irq_clr_shift)))
			continue;

		snd_pcm_period_elapsed(memif->substream);
	}

err_irq:
	/* clear irq; the vendor driver also clears the "all" bit on a stray one */
	regmap_write(afe->regmap, AFE_IRQ_CLR,
		     (reg_value & AFE_IRQ_STATUS_BITS) ?: AFE_IRQ_CLR_ALL);

	return IRQ_HANDLED;
}

static int mt6582_afe_runtime_suspend(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	struct mt6582_afe_private *afe_priv = afe->platform_priv;

	/* disable AFE */
	regmap_update_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AFE_ON, 0);

	/* power the block down */
	regmap_update_bits(afe->regmap, AUDIO_TOP_CON0, AUD_TCON0_POWER_OFF,
			   AUD_TCON0_POWER_OFF);

	clk_disable_unprepare(afe_priv->clocks[MT6582_CLK_TOP_AUDIO]);
	clk_disable_unprepare(afe_priv->clocks[MT6582_CLK_TOP_AUDINTBUS]);
	clk_disable_unprepare(afe_priv->clocks[MT6582_CLK_INFRA_AUDIO]);
	return 0;
}

static int mt6582_afe_runtime_resume(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	struct mt6582_afe_private *afe_priv = afe->platform_priv;
	int ret;

	ret = clk_prepare_enable(afe_priv->clocks[MT6582_CLK_INFRA_AUDIO]);
	if (ret)
		return ret;

	ret = clk_prepare_enable(afe_priv->clocks[MT6582_CLK_TOP_AUDINTBUS]);
	if (ret)
		goto err_infra;

	ret = clk_prepare_enable(afe_priv->clocks[MT6582_CLK_TOP_AUDIO]);
	if (ret)
		goto err_audintbus;

	/* power the block up, exactly as the vendor driver does */
	regmap_write(afe->regmap, AUDIO_TOP_CON0, AUD_TCON0_POWER_ON);

	/* start from a quiet interrupt state */
	regmap_write(afe->regmap, AFE_IRQ_MCU_CON, 0);
	regmap_write(afe->regmap, AFE_IRQ_CLR, AFE_IRQ_STATUS_BITS);

	/* enable AFE */
	regmap_update_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AFE_ON,
			   AFE_DAC_CON0_AFE_ON);
	return 0;

err_audintbus:
	clk_disable_unprepare(afe_priv->clocks[MT6582_CLK_TOP_AUDINTBUS]);
err_infra:
	clk_disable_unprepare(afe_priv->clocks[MT6582_CLK_INFRA_AUDIO]);
	return ret;
}

static int mt6582_afe_init_audio_clk(struct mtk_base_afe *afe)
{
	size_t i;
	struct mt6582_afe_private *afe_priv = afe->platform_priv;

	for (i = 0; i < ARRAY_SIZE(aud_clks); i++) {
		afe_priv->clocks[i] = devm_clk_get(afe->dev, aud_clks[i]);
		if (IS_ERR(afe_priv->clocks[i])) {
			dev_err(afe->dev, "%s devm_clk_get %s fail\n",
				__func__, aud_clks[i]);
			return PTR_ERR(afe_priv->clocks[i]);
		}
	}
	return 0;
}

static int mt6582_afe_pcm_dev_probe(struct platform_device *pdev)
{
	int ret, i;
	int irq_id;
	struct mtk_base_afe *afe;
	struct mt6582_afe_private *afe_priv;
	struct snd_soc_component *comp_pcm;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	afe = devm_kzalloc(&pdev->dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;

	afe->platform_priv = devm_kzalloc(&pdev->dev, sizeof(*afe_priv),
					  GFP_KERNEL);
	afe_priv = afe->platform_priv;
	if (!afe_priv)
		return -ENOMEM;

	afe->dev = &pdev->dev;

	irq_id = platform_get_irq(pdev, 0);
	if (irq_id <= 0)
		return irq_id < 0 ? irq_id : -ENXIO;

	afe->base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(afe->base_addr))
		return PTR_ERR(afe->base_addr);

	afe->regmap = devm_regmap_init_mmio(&pdev->dev, afe->base_addr,
		&mt6582_afe_regmap_config);
	if (IS_ERR(afe->regmap))
		return PTR_ERR(afe->regmap);

	ret = mt6582_afe_init_audio_clk(afe);
	if (ret) {
		dev_err(afe->dev, "mt6582_afe_init_audio_clk fail\n");
		return ret;
	}

	/* memif and irq initialize */
	afe->memif_size = MT6582_AFE_MEMIF_NUM;
	afe->memif = devm_kcalloc(afe->dev, afe->memif_size,
				  sizeof(*afe->memif), GFP_KERNEL);
	if (!afe->memif)
		return -ENOMEM;

	afe->irqs_size = MT6582_AFE_IRQ_NUM;
	afe->irqs = devm_kcalloc(afe->dev, afe->irqs_size,
				 sizeof(*afe->irqs), GFP_KERNEL);
	if (!afe->irqs)
		return -ENOMEM;

	for (i = 0; i < afe->irqs_size; i++) {
		afe->memif[i].data = &memif_data[i];
		afe->irqs[i].irq_data = &irq_data[i];
		afe->irqs[i].irq_occupyed = true;
		afe->memif[i].irq_usage = i;
		afe->memif[i].const_irq = 1;
	}

	afe->mtk_afe_hardware = &mt6582_afe_hardware;
	afe->memif_fs = mt6582_memif_fs;
	afe->irq_fs = mt6582_irq_fs;

	platform_set_drvdata(pdev, afe);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = mt6582_afe_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	afe->reg_back_up_list = mt6582_afe_backup_list;
	afe->reg_back_up_list_num = ARRAY_SIZE(mt6582_afe_backup_list);
	afe->runtime_resume = mt6582_afe_runtime_resume;
	afe->runtime_suspend = mt6582_afe_runtime_suspend;

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &mtk_afe_pcm_platform,
					      NULL, 0);
	if (ret)
		goto err_pm_disable;

	comp_pcm = devm_kzalloc(&pdev->dev, sizeof(*comp_pcm), GFP_KERNEL);
	if (!comp_pcm) {
		ret = -ENOMEM;
		goto err_pm_disable;
	}

	ret = snd_soc_component_initialize(comp_pcm,
					   &mt6582_afe_pcm_dai_component,
					   &pdev->dev);
	if (ret)
		goto err_pm_disable;

#ifdef CONFIG_DEBUG_FS
	comp_pcm->debugfs_prefix = "pcm";
#endif

	ret = snd_soc_add_component(comp_pcm,
				    mt6582_afe_pcm_dais,
				    ARRAY_SIZE(mt6582_afe_pcm_dais));
	if (ret)
		goto err_pm_disable;

	ret = devm_request_irq(afe->dev, irq_id, mt6582_afe_irq_handler,
			       0, "Afe_ISR_Handle", (void *)afe);
	if (ret) {
		dev_err(afe->dev, "could not request_irq\n");
		goto err_cleanup_components;
	}

	dev_info(&pdev->dev, "MT6582 AFE driver initialized.\n");
	return 0;

err_cleanup_components:
	snd_soc_unregister_component(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static void mt6582_afe_pcm_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		mt6582_afe_runtime_suspend(&pdev->dev);
}

static const struct of_device_id mt6582_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt6582-afe-pcm", },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6582_afe_pcm_dt_match);

static const struct dev_pm_ops mt6582_afe_pm_ops = {
	SET_RUNTIME_PM_OPS(mt6582_afe_runtime_suspend,
			   mt6582_afe_runtime_resume, NULL)
};

static struct platform_driver mt6582_afe_pcm_driver = {
	.driver = {
		   .name = "mt6582-afe-pcm",
		   .of_match_table = mt6582_afe_pcm_dt_match,
		   .pm = &mt6582_afe_pm_ops,
	},
	.probe = mt6582_afe_pcm_dev_probe,
	.remove = mt6582_afe_pcm_dev_remove,
};

module_platform_driver(mt6582_afe_pcm_driver);

MODULE_DESCRIPTION("MediaTek MT6582 ALSA SoC AFE platform driver");
MODULE_LICENSE("GPL v2");
