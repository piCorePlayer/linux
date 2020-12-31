// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the PCM5102A codec
 *
 * Author:	Florian Meier <florian.meier@koalo.de>
 *		Copyright 2013
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

struct pcm5102a_priv {
	bool rates_384k;
};

static const u32 pcm5102a_rates[] = {
	8000, 16000, 32000, 44100, 48000, 88200, 96000, 176400, 192000,
	352800, 384000,
};

static const struct snd_pcm_hw_constraint_list pcm5102a_constraint_rates = {
	.count = ARRAY_SIZE(pcm5102a_rates),
	.list = pcm5102a_rates,
};

static int pcm5102a_dai_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct pcm5102a_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	dev_dbg(component->dev, "%s: set rates (8k-384k) constraint\n", __func__);

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &pcm5102a_constraint_rates);
	if (ret != 0) {
		dev_err(component->dev, "%s: Failed to set rates constraint: %d\n",
			__func__, ret);
		return ret;
	}

	if (!priv->rates_384k) {
		dev_info(component->dev,
			 "%s: Limiting sample rate support to 192kHz MAX\n",
			 __func__);

		dev_dbg(component->dev, "%s: set minmax (8k/192k) constraint\n",
			__func__);

		ret = snd_pcm_hw_constraint_minmax(substream->runtime,
						   SNDRV_PCM_HW_PARAM_RATE,
						   8000, 192000);
		if (ret < 0) {
			dev_err(component->dev, "%s: Failed to set minmax "
				"constraint: %d\n", __func__, ret);
			return ret;
		}
	}

	return 0;
}

static const struct snd_soc_dai_ops pcm5102a_dai_ops = {
	.startup = pcm5102a_dai_startup,
};

static struct snd_soc_dai_driver pcm5102a_dai = {
	.name = "pcm5102a-hifi",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE
	},
	.ops = &pcm5102a_dai_ops,
};

static struct snd_soc_component_driver soc_component_dev_pcm5102a = {
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int pcm5102a_probe(struct platform_device *pdev)
{
	struct pcm5102a_priv *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

#ifdef CONFIG_OF
	if (pdev->dev.of_node)
		priv->rates_384k = of_property_read_bool(pdev->dev.of_node,
							 "pcm5102a,384k");
#endif

	dev_set_drvdata(&pdev->dev, priv);

	return devm_snd_soc_register_component(&pdev->dev, &soc_component_dev_pcm5102a,
			&pcm5102a_dai, 1);
}

static const struct of_device_id pcm5102a_of_match[] = {
	{ .compatible = "ti,pcm5102a", },
	{ }
};
MODULE_DEVICE_TABLE(of, pcm5102a_of_match);

static struct platform_driver pcm5102a_codec_driver = {
	.probe		= pcm5102a_probe,
	.driver		= {
		.name	= "pcm5102a-codec",
		.of_match_table = pcm5102a_of_match,
	},
};

module_platform_driver(pcm5102a_codec_driver);

MODULE_DESCRIPTION("ASoC PCM5102A codec driver");
MODULE_AUTHOR("Florian Meier <florian.meier@koalo.de>");
MODULE_LICENSE("GPL v2");
