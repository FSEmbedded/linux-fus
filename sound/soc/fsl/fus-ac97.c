/*
 * F&S WM9715 driver for the NXP i.MX6 CPU configured as AC97 interface
 *
 * Copyright 2016 Patrick Jakob, F&S
 * Author: Patrick Jakob <jakob@fs-net.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <linux/clk.h>
#include "imx-audmux.h"
#define DRV_NAME "fus-wm9715"

struct fus_audio_data {
	struct snd_soc_card *card;
	struct platform_device *codec;
	struct clk *mclk;
};

static struct snd_soc_dai_link_component fs_fabric_dai_codecs = {
	.dai_name = "wm9712-hifi",
	.name = "wm9712-codec",
};

static struct snd_soc_dai_link_component fs_fabric_dai_cpu;
static struct snd_soc_dai_link_component fs_fabric_dai_platform;

static struct snd_soc_dai_link fs_fabric_dai[] = {
{
	.name = "AC97 HiFi",
	.stream_name = "AC97 HiFi",
	.codecs = &fs_fabric_dai_codecs,
	.num_codecs=1,
	.cpus = &fs_fabric_dai_cpu,
	.num_cpus=1,
	.platforms = &fs_fabric_dai_platform,
	.num_platforms=1,
	.dai_fmt = SND_SOC_DAIFMT_AC97,
},
};

static struct snd_soc_card fs_wm9715_card = {
	.name = "fus-wm9715",
	.owner = THIS_MODULE,
	.dai_link = fs_fabric_dai,
	.num_links = ARRAY_SIZE(fs_fabric_dai),
};

static int fus_wm9715_audmux_init(struct device_node *np,
				     struct device *dev)
{
	u32 int_ptcr = 0, ext_ptcr = 0;
	int int_port, ext_port;
	int ret;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(dev, "mux-ext-port missing or invalid\n");
		return ret;
	}
	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the AUDMUX API expects it starts at 0.
	 */
	int_port--;
	ext_port--;

	int_ptcr = IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCLKDIR;
	ext_ptcr = IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(int_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR;

	ret = imx_audmux_v2_configure_port(int_port, int_ptcr,
					   IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(dev, "audmux internal port setup failed\n");
		return ret;
	}

	ret = imx_audmux_v2_configure_port(ext_port, ext_ptcr,
					   IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(dev, "audmux external port setup failed\n");
		return ret;
	}

	return 0;
}

static int fus_wm9715_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fus_audio_data *data;
	struct device_node *cpu_np;
	struct platform_device *cpu_pdev;
	int ret;
	struct snd_soc_dai *cpu;

	data = devm_kzalloc(&pdev->dev, sizeof(struct fus_audio_data),
			     GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	cpu_np = of_parse_phandle(np, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "CPU phandle missing or invalid\n");
		ret = -EINVAL;
		goto codec_put;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find CPU DAI device\n");
		ret = -EPROBE_DEFER;
		goto codec_put;
	}

	data->mclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->mclk)) {
		ret = PTR_ERR(data->mclk);
		dev_err(&pdev->dev, "Failed to get mclock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(data->mclk);
	if (ret)
		return ret;

	/* Common settings for corresponding Freescale CPU DAI driver */
	if (strstr(cpu_np->name, "ssi")) {
		/* Only SSI needs to configure AUDMUX */
		ret = fus_wm9715_audmux_init(np, &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "failed to init audmux\n");
			goto codec_put;
		}
	}

	data->codec = platform_device_alloc("wm9712-codec", -1);
	if (!data->codec) {
		dev_err(&pdev->dev, "Can't allocate wm9712 platform device\n");
		return -ENOMEM;
	}

	/* initialize sound card */
	data->card = &fs_wm9715_card;
	data->card->dev = &pdev->dev;
	data->card->dai_link->cpus->of_node = cpu_np;
	data->card->dai_link->platforms->of_node = cpu_np;

	ret = snd_soc_of_parse_card_name(data->card, "fus,model");
	if (ret)
		goto codec_put;

	/* We have to make sure, that ssi is available befor registering
	 * an other device or we end up in an endless probe-defer-loop */
	cpu = snd_soc_find_dai (data->card->dai_link->cpus);
	if (!cpu) {
		dev_err(&pdev->dev, "failed to find CPU DAI device\n");
		ret = -EPROBE_DEFER;
		goto codec_put;
	}

	/* transfer device node for touchscreen */
	ret = platform_device_add(data->codec);
	if (ret)
		goto codec_put;

	ret = snd_soc_register_card(data->card);

	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto codec_unregister;
	}

	return 0;

codec_unregister:
	platform_device_del(data->codec);
codec_put:
	platform_device_put(data->codec);

	return ret;
}

static int fus_wm9715_remove(struct platform_device *pdev)
{
	struct fus_audio_data *data = platform_get_drvdata(pdev);
	int ret;

	ret = snd_soc_unregister_card(data->card);
	platform_device_unregister(data->codec);

	return ret;
}

static const struct of_device_id fus_audio_match[] = {
	{ .compatible = "fus,imx-audio-wm9715", },
	{}
};
MODULE_DEVICE_TABLE(of, fus_audio_match);

static struct platform_driver fs_wm9715_driver = {
	.probe		= fus_wm9715_probe,
	.remove		= fus_wm9715_remove,
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table    = fus_audio_match,
	},
};
module_platform_driver(fs_wm9715_driver);

MODULE_AUTHOR("Patrick Jakob <jakob@fs-net.de>");
MODULE_DESCRIPTION(DRV_NAME ": fus-ac97 WM9715 driver");
MODULE_LICENSE("GPL");

