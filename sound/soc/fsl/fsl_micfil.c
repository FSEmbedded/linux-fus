// SPDX-License-Identifier: GPL-2.0
// Copyright 2018 NXP

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/dma/imx-dma.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/core.h>

#include "fsl_micfil.h"
#include "fsl_utils.h"

#define MICFIL_OSR_DEFAULT	16

enum quality {
	QUALITY_HIGH,
	QUALITY_MEDIUM,
	QUALITY_LOW,
	QUALITY_VLOW0,
	QUALITY_VLOW1,
	QUALITY_VLOW2,
};

struct fsl_micfil {
	struct platform_device *pdev;
	struct regmap *regmap;
	const struct fsl_micfil_soc_data *soc;
	struct clk *busclk;
	struct clk *mclk;
	struct clk *pll8k_clk;
	struct clk *pll11k_clk;
	struct snd_dmaengine_dai_dma_data dma_params_rx;
	struct sdma_peripheral_config sdmacfg;
	unsigned int dataline;
	char name[32];
	int irq[MICFIL_IRQ_LINES];
	enum quality quality;
	int dc_remover;
};

struct fsl_micfil_soc_data {
	unsigned int fifos;
	unsigned int fifo_depth;
	unsigned int dataline;
	bool imx;
	u64  formats;
};

static struct fsl_micfil_soc_data fsl_micfil_imx8mm = {
	.imx = true,
	.fifos = 8,
	.fifo_depth = 8,
	.dataline =  0xf,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
};

static struct fsl_micfil_soc_data fsl_micfil_imx8mp = {
	.imx = true,
	.fifos = 8,
	.fifo_depth = 32,
	.dataline =  0xf,
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
};

static const struct of_device_id fsl_micfil_dt_ids[] = {
	{ .compatible = "fsl,imx8mm-micfil", .data = &fsl_micfil_imx8mm },
	{ .compatible = "fsl,imx8mp-micfil", .data = &fsl_micfil_imx8mp },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_micfil_dt_ids);

static const char * const micfil_quality_select_texts[] = {
	[QUALITY_HIGH] = "High",
	[QUALITY_MEDIUM] = "Medium",
	[QUALITY_LOW] = "Low",
	[QUALITY_VLOW0] = "VLow0",
	[QUALITY_VLOW1] = "Vlow1",
	[QUALITY_VLOW2] = "Vlow2",
};

static const char * const micfil_hwvad_init_mode[] = {
	"Envelope mode", "Energy mode",
};

static const char * const micfil_hwvad_hpf_texts[] = {
	"Filter bypass",
	"Cut-off @1750Hz",
	"Cut-off @215Hz",
	"Cut-off @102Hz",
};

static const char * const micfil_hwvad_zcd_enable[] = {
	"OFF", "ON",
};

static const char * const micfil_hwvad_zcdauto_enable[] = {
	"OFF", "ON",
};

static const char * const micfil_hwvad_noise_decimation[] = {
	"Disabled", "Enabled",
};

/* when adding new rate text, also add it to the
 * micfil_hwvad_rate_ints
 */
static const char * const micfil_hwvad_rate[] = {
	"48KHz", "44.1KHz",
};

static const int micfil_hwvad_rate_ints[] = {
	48000, 44100,
};

static const char * const micfil_clk_src_texts[] = {
	"Auto", "AudioPLL1", "AudioPLL2", "ExtClk3",
};

/* DC Remover Control
 * Filter Bypassed	1 1
 * Cut-off @21Hz	0 0
 * Cut-off @83Hz	0 1
 * Cut-off @152HZ	1 0
 */
static const char * const micfil_dc_remover_texts[] = {
	"Cut-off @21Hz", "Cut-off @83Hz",
	"Cut-off @152Hz", "Bypass",
};

static const struct soc_enum fsl_micfil_quality_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_quality_select_texts),
			    micfil_quality_select_texts);

static DECLARE_TLV_DB_SCALE(gain_tlv, 0, 100, 0);

static int micfil_set_quality(struct fsl_micfil *micfil)
{
	u32 qsel;

	switch (micfil->quality) {
	case QUALITY_HIGH:
		qsel = MICFIL_QSEL_HIGH_QUALITY;
		break;
	case QUALITY_MEDIUM:
		qsel = MICFIL_QSEL_MEDIUM_QUALITY;
		break;
	case QUALITY_LOW:
		qsel = MICFIL_QSEL_LOW_QUALITY;
		break;
	case QUALITY_VLOW0:
		qsel = MICFIL_QSEL_VLOW0_QUALITY;
		break;
	case QUALITY_VLOW1:
		qsel = MICFIL_QSEL_VLOW1_QUALITY;
		break;
	case QUALITY_VLOW2:
		qsel = MICFIL_QSEL_VLOW2_QUALITY;
		break;
	}

	return regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
				  MICFIL_CTRL2_QSEL,
				  FIELD_PREP(MICFIL_CTRL2_QSEL, qsel));
}

static int micfil_quality_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.integer.value[0] = micfil->quality;

	return 0;
}

static int micfil_quality_set(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(cmpnt);

	micfil->quality = ucontrol->value.integer.value[0];

	return micfil_set_quality(micfil);
}

static const struct snd_kcontrol_new fsl_micfil_snd_controls[] = {
	SOC_SINGLE_SX_TLV("CH0 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(0), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH1 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(1), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH2 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(2), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH3 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(3), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH4 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(4), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH5 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(5), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH6 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(6), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH7 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(7), 0x8, 0xF, gain_tlv),
	SOC_ENUM_EXT("MICFIL Quality Select",
		     fsl_micfil_quality_enum,
		     micfil_quality_get, micfil_quality_set),
};

/* The SRES is a self-negated bit which provides the CPU with the
 * capability to initialize the PDM Interface module through the
 * slave-bus interface. This bit always reads as zero, and this
 * bit is only effective when MDIS is cleared
 */
static int fsl_micfil_reset(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	ret = regmap_clear_bits(micfil->regmap, REG_MICFIL_CTRL1,
				MICFIL_CTRL1_MDIS);
	if (ret)
		return ret;

	ret = regmap_set_bits(micfil->regmap, REG_MICFIL_CTRL1,
			      MICFIL_CTRL1_SRES);
	if (ret)
		return ret;

	/*
	 * SRES is self-cleared bit, but REG_MICFIL_CTRL1 is defined
	 * as non-volatile register, so SRES still remain in regmap
	 * cache after set, that every update of REG_MICFIL_CTRL1,
	 * software reset happens. so clear it explicitly.
	 */
	ret = regmap_clear_bits(micfil->regmap, REG_MICFIL_CTRL1,
				MICFIL_CTRL1_SRES);
	if (ret)
		return ret;

	/*
	 * Set SRES should clear CHnF flags, But even add delay here
	 * the CHnF may not be cleared sometimes, so clear CHnF explicitly.
	 */
	ret = regmap_write_bits(micfil->regmap, REG_MICFIL_STAT, 0xFF, 0xFF);
	if (ret)
		return ret;

	/* w1c */
	regmap_write_bits(micfil->regmap, REG_MICFIL_STAT, 0xFF, 0xFF);

	return 0;
}

static int fsl_micfil_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	unsigned int rates[MICFIL_NUM_RATES] = {8000, 11025, 16000, 22050, 32000, 44100, 48000};
	int i, j, k = 0;
	u64 clk_rate;

	if (!micfil) {
		dev_err(dai->dev, "micfil dai priv_data not set\n");
		return -EINVAL;
	}

	micfil->constraint_rates.list = micfil->constraint_rates_list;
	micfil->constraint_rates.count = 0;

	for (j = 0; j < MICFIL_NUM_RATES; j++) {
		for (i = 0; i < MICFIL_CLK_SRC_NUM; i++) {
			clk_rate = clk_get_rate(micfil->clk_src[i]);
			if (clk_rate != 0 && do_div(clk_rate, rates[j]) == 0) {
				micfil->constraint_rates_list[k++] = rates[j];
				micfil->constraint_rates.count++;
				break;
			}
		}
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				   &micfil->constraint_rates);

	return 0;
}

static int fsl_micfil_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &micfil->pdev->dev;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = fsl_micfil_reset(dev);
		if (ret) {
			dev_err(dev, "failed to soft reset\n");
			return ret;
		}

		/* DMA Interrupt Selection - DISEL bits
		 * 00 - DMA and IRQ disabled
		 * 01 - DMA req enabled
		 * 10 - IRQ enabled
		 * 11 - reserved
		 */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				MICFIL_CTRL1_DISEL,
				FIELD_PREP(MICFIL_CTRL1_DISEL, MICFIL_CTRL1_DISEL_DMA));
		if (ret)
			return ret;

		/* Enable the module */
		ret = regmap_set_bits(micfil->regmap, REG_MICFIL_CTRL1,
				      MICFIL_CTRL1_PDMIEN);
		if (ret)
			return ret;

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* Disable the module */
		ret = regmap_clear_bits(micfil->regmap, REG_MICFIL_CTRL1,
					MICFIL_CTRL1_PDMIEN);
		if (ret)
			return ret;

		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				MICFIL_CTRL1_DISEL,
				FIELD_PREP(MICFIL_CTRL1_DISEL, MICFIL_CTRL1_DISEL_DISABLE));
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fsl_micfil_reparent_rootclk(struct fsl_micfil *micfil, unsigned int sample_rate)
{
	struct device *dev = &micfil->pdev->dev;
	u64 ratio = sample_rate;
	struct clk *clk;
	int ret;

	/* Get root clock */
	clk = micfil->mclk;

	/* Disable clock first, for it was enabled by pm_runtime */
	clk_disable_unprepare(clk);
	fsl_asoc_reparent_pll_clocks(dev, clk, micfil->pll8k_clk,
				     micfil->pll11k_clk, ratio);
	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	return 0;
}

static int fsl_micfil_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	int clk_div = 8;
	int osr = MICFIL_OSR_DEFAULT;
	int ret;
	u32 hwvad_state;

	/* 1. Disable the module */
	ret = regmap_clear_bits(micfil->regmap, REG_MICFIL_CTRL1,
				MICFIL_CTRL1_PDMIEN);
	if (ret)
		return ret;

	/* enable channels */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 0xFF, ((1 << channels) - 1));
	if (ret)
		return ret;

	ret = fsl_micfil_reparent_rootclk(micfil, rate);
	if (ret)
		return ret;

	ret = clk_set_rate(micfil->mclk, rate * clk_div * osr * 8);
	if (ret)
		return ret;

	ret = micfil_set_quality(micfil);
	if (ret)
		return ret;

	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
				 MICFIL_CTRL2_CLKDIV | MICFIL_CTRL2_CICOSR,
				 FIELD_PREP(MICFIL_CTRL2_CLKDIV, clk_div) |
				 FIELD_PREP(MICFIL_CTRL2_CICOSR, 16 - osr));

	micfil->dma_params_rx.peripheral_config = &micfil->sdmacfg;
	micfil->dma_params_rx.peripheral_size = sizeof(micfil->sdmacfg);
	micfil->sdmacfg.n_fifos_src = channels;
	micfil->sdmacfg.sw_done = true;
	micfil->dma_params_rx.maxburst = channels * MICFIL_DMA_MAXBURST_RX;

	return 0;
}

static const struct snd_soc_dai_ops fsl_micfil_dai_ops = {
	.startup = fsl_micfil_startup,
	.trigger = fsl_micfil_trigger,
	.hw_params = fsl_micfil_hw_params,
};

static int fsl_micfil_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct fsl_micfil *micfil = dev_get_drvdata(cpu_dai->dev);
	struct device *dev = cpu_dai->dev;
	unsigned int val = 0;
	int ret, i;

	micfil->quality = QUALITY_VLOW0;

	/* set default gain to 2 */
	regmap_write(micfil->regmap, REG_MICFIL_OUT_CTRL, 0x22222222);

	/* set DC Remover in bypass mode*/
	for (i = 0; i < MICFIL_OUTPUT_CHANNELS; i++)
		val |= MICFIL_DC_BYPASS << MICFIL_DC_CHX_SHIFT(i);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_DC_CTRL,
				 MICFIL_DC_CTRL_CONFIG, val);
	if (ret) {
		dev_err(dev, "failed to set DC Remover mode bits\n");
		return ret;
	}
	micfil->dc_remover = MICFIL_DC_BYPASS;

	snd_soc_dai_init_dma_data(cpu_dai, NULL,
				  &micfil->dma_params_rx);

	/* FIFO Watermark Control - FIFOWMK*/
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_FIFO_CTRL,
			MICFIL_FIFO_CTRL_FIFOWMK,
			FIELD_PREP(MICFIL_FIFO_CTRL_FIFOWMK, micfil->soc->fifo_depth - 1));
	if (ret)
		return ret;

	return 0;
}

static struct snd_soc_dai_driver fsl_micfil_dai = {
	.probe = fsl_micfil_dai_probe,
	.capture = {
		.stream_name = "CPU-Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &fsl_micfil_dai_ops,
};

static const struct snd_soc_component_driver fsl_micfil_component = {
	.name		= "fsl-micfil-dai",
	.controls       = fsl_micfil_snd_controls,
	.num_controls   = ARRAY_SIZE(fsl_micfil_snd_controls),
	.legacy_dai_naming      = 1,
};

/* REGMAP */
static const struct reg_default fsl_micfil_reg_defaults[] = {
	{REG_MICFIL_CTRL1,		0x00000000},
	{REG_MICFIL_CTRL2,		0x00000000},
	{REG_MICFIL_STAT,		0x00000000},
	{REG_MICFIL_FIFO_CTRL,		0x00000007},
	{REG_MICFIL_FIFO_STAT,		0x00000000},
	{REG_MICFIL_DATACH0,		0x00000000},
	{REG_MICFIL_DATACH1,		0x00000000},
	{REG_MICFIL_DATACH2,		0x00000000},
	{REG_MICFIL_DATACH3,		0x00000000},
	{REG_MICFIL_DATACH4,		0x00000000},
	{REG_MICFIL_DATACH5,		0x00000000},
	{REG_MICFIL_DATACH6,		0x00000000},
	{REG_MICFIL_DATACH7,		0x00000000},
	{REG_MICFIL_DC_CTRL,		0x00000000},
	{REG_MICFIL_OUT_CTRL,		0x00000000},
	{REG_MICFIL_OUT_STAT,		0x00000000},
	{REG_MICFIL_VAD0_CTRL1,		0x00000000},
	{REG_MICFIL_VAD0_CTRL2,		0x000A0000},
	{REG_MICFIL_VAD0_STAT,		0x00000000},
	{REG_MICFIL_VAD0_SCONFIG,	0x00000000},
	{REG_MICFIL_VAD0_NCONFIG,	0x80000000},
	{REG_MICFIL_VAD0_NDATA,		0x00000000},
	{REG_MICFIL_VAD0_ZCD,		0x00000004},
};

static bool fsl_micfil_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_CTRL1:
	case REG_MICFIL_CTRL2:
	case REG_MICFIL_STAT:
	case REG_MICFIL_FIFO_CTRL:
	case REG_MICFIL_FIFO_STAT:
	case REG_MICFIL_DATACH0:
	case REG_MICFIL_DATACH1:
	case REG_MICFIL_DATACH2:
	case REG_MICFIL_DATACH3:
	case REG_MICFIL_DATACH4:
	case REG_MICFIL_DATACH5:
	case REG_MICFIL_DATACH6:
	case REG_MICFIL_DATACH7:
	case REG_MICFIL_DC_CTRL:
	case REG_MICFIL_OUT_CTRL:
	case REG_MICFIL_OUT_STAT:
	case REG_MICFIL_VAD0_CTRL1:
	case REG_MICFIL_VAD0_CTRL2:
	case REG_MICFIL_VAD0_STAT:
	case REG_MICFIL_VAD0_SCONFIG:
	case REG_MICFIL_VAD0_NCONFIG:
	case REG_MICFIL_VAD0_NDATA:
	case REG_MICFIL_VAD0_ZCD:
		return true;
	default:
		return false;
	}
}

static bool fsl_micfil_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_CTRL1:
	case REG_MICFIL_CTRL2:
	case REG_MICFIL_STAT:		/* Write 1 to Clear */
	case REG_MICFIL_FIFO_CTRL:
	case REG_MICFIL_FIFO_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_DC_CTRL:
	case REG_MICFIL_OUT_CTRL:
	case REG_MICFIL_OUT_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_VAD0_CTRL1:
	case REG_MICFIL_VAD0_CTRL2:
	case REG_MICFIL_VAD0_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_VAD0_SCONFIG:
	case REG_MICFIL_VAD0_NCONFIG:
	case REG_MICFIL_VAD0_ZCD:
		return true;
	default:
		return false;
	}
}

static bool fsl_micfil_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_STAT:
	case REG_MICFIL_DATACH0:
	case REG_MICFIL_DATACH1:
	case REG_MICFIL_DATACH2:
	case REG_MICFIL_DATACH3:
	case REG_MICFIL_DATACH4:
	case REG_MICFIL_DATACH5:
	case REG_MICFIL_DATACH6:
	case REG_MICFIL_DATACH7:
	case REG_MICFIL_VAD0_STAT:
	case REG_MICFIL_VAD0_NDATA:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config fsl_micfil_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = REG_MICFIL_VAD0_ZCD,
	.reg_defaults = fsl_micfil_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(fsl_micfil_reg_defaults),
	.readable_reg = fsl_micfil_readable_reg,
	.volatile_reg = fsl_micfil_volatile_reg,
	.writeable_reg = fsl_micfil_writeable_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* END OF REGMAP */

static irqreturn_t voice_detected_fn(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct device *dev = &micfil->pdev->dev;
	int ret;

	/* disable hwvad */
	spin_lock(&micfil->hwvad_lock);
	ret = disable_hwvad(dev, true);
	spin_unlock(&micfil->hwvad_lock);

	if (ret)
		dev_err(dev, "Failed to disable HWVAD module: %d\n", ret);

	/* notify userspace that voice was detected */
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);

	return IRQ_HANDLED;
}

static irqreturn_t hwvad_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct device *dev = &micfil->pdev->dev;
	int ret;
	u32 vad0_reg;

	regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &vad0_reg);

	/* The only difference between MICFIL_VAD0_STAT_EF and
	 * MICFIL_VAD0_STAT_IF is that the former requires Write
	 * 1 to Clear. Since both flags are set, it is enough
	 * to only read one of them
	 */
	if (vad0_reg & MICFIL_VAD0_STAT_IF_MASK) {
		/* Write 1 to clear */
		regmap_write_bits(micfil->regmap, REG_MICFIL_VAD0_STAT,
				  MICFIL_VAD0_STAT_IF_MASK,
				  MICFIL_VAD0_STAT_IF);

		/* disable hwvad interrupts */
		ret = configure_hwvad_interrupts(dev, 0);
		if (ret)
			dev_err(dev, "Failed to disable interrupts\n");
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t hwvad_err_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct device *dev = &micfil->pdev->dev;
	u32 vad0_reg;

	regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &vad0_reg);

	if (vad0_reg & MICFIL_VAD0_STAT_INSATF_MASK)
		dev_dbg(dev, "voice activity input overflow/underflow detected\n");

	if (vad0_reg & MICFIL_VAD0_STAT_INITF_MASK)
		dev_dbg(dev, "voice activity dectector is initializing\n");

	return IRQ_HANDLED;
}

static irqreturn_t micfil_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct platform_device *pdev = micfil->pdev;
	u32 stat_reg;
	u32 fifo_stat_reg;
	u32 ctrl1_reg;
	bool dma_enabled;
	int i;

	regmap_read(micfil->regmap, REG_MICFIL_STAT, &stat_reg);
	regmap_read(micfil->regmap, REG_MICFIL_CTRL1, &ctrl1_reg);
	regmap_read(micfil->regmap, REG_MICFIL_FIFO_STAT, &fifo_stat_reg);

	dma_enabled = FIELD_GET(MICFIL_CTRL1_DISEL, ctrl1_reg) == MICFIL_CTRL1_DISEL_DMA;

	/* Channel 0-7 Output Data Flags */
	for (i = 0; i < MICFIL_OUTPUT_CHANNELS; i++) {
		if (stat_reg & MICFIL_STAT_CHXF(i))
			dev_dbg(&pdev->dev,
				"Data available in Data Channel %d\n", i);
		/* if DMA is not enabled, field must be written with 1
		 * to clear
		 */
		if (!dma_enabled)
			regmap_write_bits(micfil->regmap,
					  REG_MICFIL_STAT,
					  MICFIL_STAT_CHXF(i),
					  1);
	}

	for (i = 0; i < MICFIL_FIFO_NUM; i++) {
		if (fifo_stat_reg & MICFIL_FIFO_STAT_FIFOX_OVER(i))
			dev_dbg(&pdev->dev,
				"FIFO Overflow Exception flag for channel %d\n",
				i);

		if (fifo_stat_reg & MICFIL_FIFO_STAT_FIFOX_UNDER(i))
			dev_dbg(&pdev->dev,
				"FIFO Underflow Exception flag for channel %d\n",
				i);
	}

	return IRQ_HANDLED;
}

static irqreturn_t micfil_err_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct platform_device *pdev = micfil->pdev;
	u32 stat_reg;

	regmap_read(micfil->regmap, REG_MICFIL_STAT, &stat_reg);

	if (stat_reg & MICFIL_STAT_BSY_FIL)
		dev_dbg(&pdev->dev, "isr: Decimation Filter is running\n");

	if (stat_reg & MICFIL_STAT_FIR_RDY)
		dev_dbg(&pdev->dev, "isr: FIR Filter Data ready\n");

	if (stat_reg & MICFIL_STAT_LOWFREQF) {
		dev_dbg(&pdev->dev, "isr: ipg_clk_app is too low\n");
		regmap_write_bits(micfil->regmap, REG_MICFIL_STAT,
				  MICFIL_STAT_LOWFREQF, 1);
	}

	return IRQ_HANDLED;
}

static int fsl_set_clock_params(struct device *, unsigned int);

static int enable_hwvad(struct device *dev, bool sync)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	int rate;
	u32 state;

	if (sync)
		pm_runtime_get_sync(dev);

	state = atomic_cmpxchg(&micfil->hwvad_state,
			       MICFIL_HWVAD_OFF,
			       MICFIL_HWVAD_ON);

	/* we should not reenable when sync = true because
	 * this means enable was called for second time by
	 * user. However state = ON and sync = false can only
	 * occur when enable is called from system_resume. In
	 * this case we should enable the hwvad
	 */
	if (sync && state == MICFIL_HWVAD_ON) {
		dev_err(dev, "hwvad already on\n");
		ret = -EBUSY;
		goto enable_error;
	}

	if (micfil->vad_rate_index >= ARRAY_SIZE(micfil_hwvad_rate_ints)) {
		dev_err(dev, "There are more select texts than rates\n");
		ret = -EINVAL;
		goto enable_error;
	}

	rate = micfil_hwvad_rate_ints[micfil->vad_rate_index];

	/* This is required because if an arecord was done,
	 * suspend function will mark regmap as cache only
	 * and reads/writes in volatile regs will fail
	 */
	regcache_cache_only(micfil->regmap, false);
	regcache_mark_dirty(micfil->regmap);
	regcache_sync(micfil->regmap);

	ret = fsl_set_clock_params(dev, rate);
	if (ret)
		goto enable_error;

	ret = fsl_micfil_reset(dev);
	if (ret)
		goto enable_error;

	/* Initialize Hardware Voice Activity */
	ret = init_hwvad(dev);
	if (ret == 0)
		return 0;

enable_error:
	if (state == MICFIL_HWVAD_OFF)
		atomic_cmpxchg(&micfil->hwvad_state,
			       MICFIL_HWVAD_ON, MICFIL_HWVAD_OFF);
	if (sync)
		pm_runtime_put_sync(dev);
	return ret;
}

static int disable_hwvad(struct device *dev, bool sync)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret = 0;
	u32 state;

	/* disable is called with sync = false only from
	 * system suspend and in this case, you should not
	 * change the hwvad_state so we know at system_resume
	 * to reenable hwvad
	 */
	if (sync)
		state = atomic_cmpxchg(&micfil->hwvad_state,
				       MICFIL_HWVAD_ON,
				       MICFIL_HWVAD_OFF);
	else
		state = atomic_read(&micfil->hwvad_state);

	if (state == MICFIL_HWVAD_ON) {
		/* This is required because if an arecord was done,
		 * suspend function will mark regmap as cache only
		 * and reads/writes in volatile regs will fail
		 */
		regcache_cache_only(micfil->regmap, false);
		regcache_mark_dirty(micfil->regmap);
		regcache_sync(micfil->regmap);

		/* Voice Activity Detector Reset */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL1,
					  MICFIL_VAD0_CTRL1_RST_SHIFT,
					  MICFIL_VAD0_CTRL1_RST);

		/* Disable HWVAD */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL1,
					  MICFIL_VAD0_CTRL1_EN_MASK,
					  0);

		/* Disable Signal Filter */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_SCONFIG,
					  MICFIL_VAD0_SCONFIG_SFILEN_MASK,
					  0);

		/* Signal Maximum Enable */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_SCONFIG,
					  MICFIL_VAD0_SCONFIG_SMAXEN_MASK,
					  0);

		/* Enable pre-filter Noise & Signal */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL2,
					  MICFIL_VAD0_CTRL2_PREFEN_MASK,
					  0);

		/* Noise Decimation Enable */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_NCONFIG,
					  MICFIL_VAD0_NCONFIG_NDECEN_MASK,
					  0);

		/* disable the module and clock only if recording
		 * is not done in parallel
		 */
		state = atomic_read(&micfil->recording_state);
		if (state == MICFIL_RECORDING_OFF) {
		/* Disable MICFIL module */
			ret |= regmap_update_bits(micfil->regmap,
						  REG_MICFIL_CTRL1,
						  MICFIL_CTRL1_PDMIEN_MASK,
						  0);
		}

		if (sync)
			pm_runtime_put_sync(dev);
	} else {
		ret = -EPERM;
		dev_err(dev, "HWVAD is not enabled %d\n", ret);
	}

	return ret;
}

static ssize_t micfil_hwvad_handler(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct kobject *nand_kobj = kobj->parent;
	struct device *dev = container_of(nand_kobj, struct device, kobj);
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	unsigned long vad_channel;
	int ret;

	ret = kstrtoul(buf, 16, &vad_channel);
	if (ret < 0)
		return -EINVAL;

	spin_lock(&micfil->hwvad_lock);
	if (vad_channel <= 7) {
		micfil->vad_channel = vad_channel;
		ret = enable_hwvad(dev, true);
	} else {
		micfil->vad_channel = -1;
		ret = disable_hwvad(dev, true);
	}
	spin_unlock(&micfil->hwvad_lock);

	if (ret) {
		dev_err(dev, "Failed to %s hwvad: %d\n",
			vad_channel <= 7 ? "enable" : "disable", ret);
		return ret;
	}

	return count;
}

static struct kobj_attribute hwvad_en_attr = __ATTR(enable,
						   0660,
						   NULL,
						   micfil_hwvad_handler);

static int fsl_micfil_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_micfil *micfil;
	struct resource *res;
	void __iomem *regs;
	int ret, i;

	micfil = devm_kzalloc(&pdev->dev, sizeof(*micfil), GFP_KERNEL);
	if (!micfil)
		return -ENOMEM;

	micfil->pdev = pdev;
	strncpy(micfil->name, np->name, sizeof(micfil->name) - 1);

	micfil->soc = of_device_get_match_data(&pdev->dev);

	/* ipg_clk is used to control the registers
	 * ipg_clk_app is used to operate the filter
	 */
	micfil->mclk = devm_clk_get(&pdev->dev, "ipg_clk_app");
	if (IS_ERR(micfil->mclk)) {
		dev_err(&pdev->dev, "failed to get core clock: %ld\n",
			PTR_ERR(micfil->mclk));
		return PTR_ERR(micfil->mclk);
	}

	micfil->busclk = devm_clk_get(&pdev->dev, "ipg_clk");
	if (IS_ERR(micfil->busclk)) {
		dev_err(&pdev->dev, "failed to get ipg clock: %ld\n",
			PTR_ERR(micfil->busclk));
		return PTR_ERR(micfil->busclk);
	}

	fsl_asoc_get_pll_clocks(&pdev->dev, &micfil->pll8k_clk,
				&micfil->pll11k_clk);

	/* init regmap */
	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	micfil->regmap = devm_regmap_init_mmio(&pdev->dev,
					       regs,
					       &fsl_micfil_regmap_config);
	if (IS_ERR(micfil->regmap)) {
		dev_err(&pdev->dev, "failed to init MICFIL regmap: %ld\n",
			PTR_ERR(micfil->regmap));
		return PTR_ERR(micfil->regmap);
	}

	/* dataline mask for RX */
	ret = of_property_read_u32_index(np,
					 "fsl,dataline",
					 0,
					 &micfil->dataline);
	if (ret)
		micfil->dataline = 1;

	if (micfil->dataline & ~micfil->soc->dataline) {
		dev_err(&pdev->dev, "dataline setting error, Mask is 0x%X\n",
			micfil->soc->dataline);
		return -EINVAL;
	}

	/* get IRQs */
	for (i = 0; i < MICFIL_IRQ_LINES; i++) {
		micfil->irq[i] = platform_get_irq(pdev, i);
		if (micfil->irq[i] < 0)
			return micfil->irq[i];
	}

	/* Digital Microphone interface interrupt */
	ret = devm_request_irq(&pdev->dev, micfil->irq[0],
			       micfil_isr, IRQF_SHARED,
			       micfil->name, micfil);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim mic interface irq %u\n",
			micfil->irq[0]);
		return ret;
	}

	/* Digital Microphone interface error interrupt */
	ret = devm_request_irq(&pdev->dev, micfil->irq[1],
			       micfil_err_isr, IRQF_SHARED,
			       micfil->name, micfil);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim mic interface error irq %u\n",
			micfil->irq[1]);
		return ret;
	}

	micfil->slave_mode = false;

	micfil->dma_params_rx.chan_name = "rx";
	micfil->dma_params_rx.addr = res->start + REG_MICFIL_DATACH0;
	micfil->dma_params_rx.maxburst = MICFIL_DMA_MAXBURST_RX;

	platform_set_drvdata(pdev, micfil);

	pm_runtime_enable(&pdev->dev);
	regcache_cache_only(micfil->regmap, true);

	/*
	 * Register platform component before registering cpu dai for there
	 * is not defer probe for platform component in snd_soc_add_pcm_runtime().
	 */
	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "failed to pcm register\n");
		return ret;
	}

	fsl_micfil_dai.capture.formats = micfil->soc->formats;

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_micfil_component,
					      &fsl_micfil_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register component %s\n",
			fsl_micfil_component.name);
		return ret;
	}

	/* create sysfs entry used to enable hwvad from userspace */
	micfil->hwvad_kobject = kobject_create_and_add("hwvad",
						       &pdev->dev.kobj);
	if (!micfil->hwvad_kobject)
		return -ENOMEM;

	ret = sysfs_create_file(micfil->hwvad_kobject,
				&hwvad_en_attr.attr);
	if (ret) {
		dev_err(&pdev->dev, "failed to create file for hwvad_enable\n");
		kobject_put(micfil->hwvad_kobject);
		return -ENOMEM;
	}

	return 0;
}

static int __maybe_unused fsl_micfil_runtime_suspend(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	u32 state;

	state = atomic_read(&micfil->hwvad_state);
	if (state == MICFIL_HWVAD_ON)
		return 0;

	regcache_cache_only(micfil->regmap, true);

	clk_disable_unprepare(micfil->mclk);
	clk_disable_unprepare(micfil->busclk);

	return 0;
}

static int __maybe_unused fsl_micfil_runtime_resume(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 state;

	state = atomic_read(&micfil->hwvad_state);

	/* enable mclk only if the hwvad is not enabled
	 * When hwvad is enabled, clock won't be disabled
	 * in suspend since hwvad and recording share the
	 * same clock
	 */
	if (state == MICFIL_HWVAD_ON)
		return 0;

	ret = clk_prepare_enable(micfil->busclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(micfil->mclk);
	if (ret < 0) {
		clk_disable_unprepare(micfil->busclk);
		return ret;
	}

	regcache_cache_only(micfil->regmap, false);
	regcache_mark_dirty(micfil->regmap);
	regcache_sync(micfil->regmap);

	return 0;
}

static int __maybe_unused fsl_micfil_suspend(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 state;

	state = atomic_read(&micfil->hwvad_state);

	if (state == MICFIL_HWVAD_ON) {
		dev_err(dev, "Disabling hwvad on suspend");
		ret = disable_hwvad(dev, false);
		if (ret)
			dev_warn(dev, "Failed to disable hwvad");
	}

	pm_runtime_force_suspend(dev);

	return 0;
}

static int __maybe_unused fsl_micfil_resume(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 state;

	pm_runtime_force_resume(dev);

	state = atomic_read(&micfil->hwvad_state);
	if (state == MICFIL_HWVAD_ON) {
		dev_err(dev, "Enabling hwvad on resume");
		ret = enable_hwvad(dev, false);
		if (ret)
			dev_warn(dev, "Failed to re-enable hwvad");
	}

	return 0;
}

static const struct dev_pm_ops fsl_micfil_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_micfil_runtime_suspend,
			   fsl_micfil_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_micfil_suspend,
				fsl_micfil_resume)
};

static struct platform_driver fsl_micfil_driver = {
	.probe = fsl_micfil_probe,
	.driver = {
		.name = "fsl-micfil-dai",
		.pm = &fsl_micfil_pm_ops,
		.of_match_table = fsl_micfil_dt_ids,
	},
};
module_platform_driver(fsl_micfil_driver);

MODULE_AUTHOR("Cosmin-Gabriel Samoila <cosmin.samoila@nxp.com>");
MODULE_DESCRIPTION("NXP PDM Microphone Interface (MICFIL) driver");
MODULE_LICENSE("GPL v2");
