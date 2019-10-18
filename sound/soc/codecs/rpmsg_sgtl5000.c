/*
 *
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 */
#include <linux/module.h>
#include <linux/string.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "../fsl/fsl_rpmsg_i2s.h"
#include "sgtl5000.h"

#define SGTL5000_SUPPLY_NUM 3


/* sgtl5000 private structure in codec */
struct sgtl5000_priv {
	int sysclk;	/* sysclk rate */
	int master;	/* i2s master or not */
	int fmt;	/* i2s data format */
	int num_supplies;
	struct regmap *regmap;
	struct clk *mclk;
	int revision;
	int mono2both;
	int no_standby;
	u8 micbias_resistor;
	u8 micbias_voltage;
	u8 lrclk_strength;
};

/*
 * mic_bias power on/off share the same register bits with
 * output impedance of mic bias, when power on mic bias, we
 * need reclaim it to impedance value.
 * 0x0 = Powered off
 * 0x1 = 2Kohm
 * 0x2 = 4Kohm
 * 0x3 = 8Kohm
 */
static int mic_bias_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* change mic bias resistor */
		snd_soc_update_bits(codec, SGTL5000_CHIP_MIC_CTRL,
			SGTL5000_BIAS_R_MASK,
			0x2 << SGTL5000_BIAS_R_SHIFT);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SGTL5000_CHIP_MIC_CTRL,
				SGTL5000_BIAS_R_MASK, 0);
		break;
	}
	return 0;
}

static void power_vag_up(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, SGTL5000_CHIP_ANA_POWER,
		SGTL5000_VAG_POWERUP, SGTL5000_VAG_POWERUP);
}


static void power_vag_down(struct snd_soc_codec *codec)
{
	const u32 mask = SGTL5000_DAC_POWERUP | SGTL5000_ADC_POWERUP;

	/*
	 * Don't clear VAG_POWERUP, when both DAC and ADC are
	 * operational to prevent inadvertently starving the
	 * other one of them.
	 */
	if ((snd_soc_read(codec, SGTL5000_CHIP_ANA_POWER) &
			mask) != mask) {
		snd_soc_update_bits(codec, SGTL5000_CHIP_ANA_POWER,
			SGTL5000_VAG_POWERUP, 0);
		msleep(400);
	}
}

static int power_vag_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		power_vag_up(codec);
		msleep(400);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		power_vag_down(codec);
		break;
	default:
		break;
	}

	return 0;
}

/* input sources for ADC */
static const char *adc_mux_text[] = {
	"MIC_IN", "LINE_IN"
};

static SOC_ENUM_SINGLE_DECL(adc_enum,
			    SGTL5000_CHIP_ANA_CTRL, 2,
			    adc_mux_text);

static const struct snd_kcontrol_new adc_mux =
SOC_DAPM_ENUM("Capture_Mux Capture Switch", adc_enum);

/* input sources for DAC */
static const char *dac_mux_text[] = {
	"DAC", "LINE_IN"
};

static SOC_ENUM_SINGLE_DECL(dac_enum,
			    SGTL5000_CHIP_ANA_CTRL, 6,
			    dac_mux_text);

static const struct snd_kcontrol_new dac_mux =
SOC_DAPM_ENUM("Headphone_Mux Playback Switch", dac_enum);

static const struct snd_soc_dapm_widget sgtl5000_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("LINE_IN"),
	SND_SOC_DAPM_INPUT("MIC_IN"),

	SND_SOC_DAPM_OUTPUT("HP_OUT"),
	SND_SOC_DAPM_OUTPUT("LINE_OUT"),

	SND_SOC_DAPM_SUPPLY("Mic Bias", SGTL5000_CHIP_MIC_CTRL, 8, 0,
			    mic_bias_event,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA("HP", SGTL5000_CHIP_ANA_POWER, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LO", SGTL5000_CHIP_ANA_POWER, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("Capture Mux", SND_SOC_NOPM, 0, 0, &adc_mux),
	SND_SOC_DAPM_MUX("Headphone Mux", SND_SOC_NOPM, 0, 0, &dac_mux),

	/* aif for i2s input */
	SND_SOC_DAPM_AIF_IN("AIFIN", "Playback",
				0, SGTL5000_CHIP_DIG_POWER,
				0, 0),

	/* aif for i2s output */
	SND_SOC_DAPM_AIF_OUT("AIFOUT", "Capture",
				0, SGTL5000_CHIP_DIG_POWER,
				1, 0),

	SND_SOC_DAPM_ADC("ADC", "Capture", SGTL5000_CHIP_ANA_POWER, 1, 0),
	SND_SOC_DAPM_DAC("DAC", "Playback", SGTL5000_CHIP_ANA_POWER, 3, 0),

	SND_SOC_DAPM_PRE("VAG_POWER_PRE", power_vag_event),
	SND_SOC_DAPM_POST("VAG_POWER_POST", power_vag_event),
};

/* routes for sgtl5000 */
static const struct snd_soc_dapm_route sgtl5000_dapm_routes[] = {
	{"Capture Mux", "LINE_IN", "LINE_IN"},	/* line_in --> adc_mux */
	{"Capture Mux", "MIC_IN", "MIC_IN"},	/* mic_in --> adc_mux */

	{"ADC", NULL, "Capture Mux"},		/* adc_mux --> adc */
	{"AIFOUT", NULL, "ADC"},		/* adc --> i2s_out */

	{"DAC", NULL, "AIFIN"},			/* i2s-->dac,skip audio mux */
	{"Headphone Mux", "DAC", "DAC"},	/* dac --> hp_mux */
	{"LO", NULL, "DAC"},			/* dac --> line_out */

	{"Headphone Mux", "LINE_IN", "LINE_IN"},/* line_in --> hp_mux */
	{"HP", NULL, "Headphone Mux"},		/* hp_mux --> hp */

	{"LINE_OUT", NULL, "LO"},
	{"HP_OUT", NULL, "HP"},
};

/* custom function to fetch info of PCM playback volume */
static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xfc - 0x3c;
	return 0;
}

/*
 * custom function to get of PCM playback volume
 *
 * dac volume register
 * 15-------------8-7--------------0
 * | R channel vol | L channel vol |
 *  -------------------------------
 *
 * PCM volume with 0.5017 dB steps from 0 to -90 dB
 *
 * register values map to dB
 * 0x3B and less = Reserved
 * 0x3C = 0 dB
 * 0x3D = -0.5 dB
 * 0xF0 = -90 dB
 * 0xFC and greater = Muted
 *
 * register value map to userspace value
 *
 * register value	0x3c(0dB)	  0xf0(-90dB)0xfc
 *			------------------------------
 * userspace value	0xc0			     0
 */
static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	int reg;
	int l;
	int r;

	reg = snd_soc_read(codec, SGTL5000_CHIP_DAC_VOL);

	/* get left channel volume */
	l = (reg & SGTL5000_DAC_VOL_LEFT_MASK) >> SGTL5000_DAC_VOL_LEFT_SHIFT;

	/* get right channel volume */
	r = (reg & SGTL5000_DAC_VOL_RIGHT_MASK) >> SGTL5000_DAC_VOL_RIGHT_SHIFT;

	/* make sure value fall in (0x3c,0xfc) */
	l = clamp(l, 0x3c, 0xfc);
	r = clamp(r, 0x3c, 0xfc);

	/* invert it and map to userspace value */
	l = 0xfc - l;
	r = 0xfc - r;

	ucontrol->value.integer.value[0] = l;
	ucontrol->value.integer.value[1] = r;

	return 0;
}

/*
 * custom function to put of PCM playback volume
 *
 * dac volume register
 * 15-------------8-7--------------0
 * | R channel vol | L channel vol |
 *  -------------------------------
 *
 * PCM volume with 0.5017 dB steps from 0 to -90 dB
 *
 * register values map to dB
 * 0x3B and less = Reserved
 * 0x3C = 0 dB
 * 0x3D = -0.5 dB
 * 0xF0 = -90 dB
 * 0xFC and greater = Muted
 *
 * userspace value map to register value
 *
 * userspace value	0xc0			     0
 *			------------------------------
 * register value	0x3c(0dB)	0xf0(-90dB)0xfc
 */
static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	int reg;
	int l;
	int r;

	l = ucontrol->value.integer.value[0];
	r = ucontrol->value.integer.value[1];

	/* make sure userspace volume fall in (0, 0xfc-0x3c) */
	l = clamp(l, 0, 0xfc - 0x3c);
	r = clamp(r, 0, 0xfc - 0x3c);

	/* invert it, get the value can be set to register */
	l = 0xfc - l;
	r = 0xfc - r;

	/* shift to get the register value */
	reg = l << SGTL5000_DAC_VOL_LEFT_SHIFT |
		r << SGTL5000_DAC_VOL_RIGHT_SHIFT;

	snd_soc_write(codec, SGTL5000_CHIP_DAC_VOL, reg);

	return 0;
}
static const DECLARE_TLV_DB_SCALE(capture_6db_attenuate, -600, 600, 0);

/* tlv for mic gain, 0db 20db 30db 40db */
static const DECLARE_TLV_DB_RANGE(mic_gain_tlv,
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 3, TLV_DB_SCALE_ITEM(2000, 1000, 0)
);

/* tlv for hp volume, -51.5db to 12.0db, step .5db */
static const DECLARE_TLV_DB_SCALE(headphone_volume, -5150, 50, 0);

/* tlv for lineout volume, 31 steps of .5db each */
static const DECLARE_TLV_DB_SCALE(lineout_volume, -1550, 50, 0);

static const struct snd_kcontrol_new rpmsg_sgtl5000_snd_controls[] = {

/* SOC_DOUBLE_S8_TLV with invert */
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "PCM Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = dac_info_volsw,
		.get = dac_get_volsw,
		.put = dac_put_volsw,
	},

	SOC_DOUBLE("ADC Capture Volume", SGTL5000_CHIP_ANA_ADC_CTRL, 0, 4, 0xf, 0),
	SOC_SINGLE_TLV("ADC_Attenuate_(-6dB) Capture Switch",
			SGTL5000_CHIP_ANA_ADC_CTRL,
			8, 1, 0, capture_6db_attenuate),
	SOC_SINGLE("ADC_ZCD Capture Switch", SGTL5000_CHIP_ANA_CTRL, 1, 1, 0),
	/* ###TODO: Find better solution for Mic Bias enable.*/
	SOC_SINGLE("Mic_Enable Capture Switch", SGTL5000_CHIP_MIC_CTRL, 9, 1, 0),

	SOC_DOUBLE_TLV("Headphone Playback Volume",
			SGTL5000_CHIP_ANA_HP_CTRL,
			0, 8,
			0x7f, 1,
			headphone_volume),
	SOC_SINGLE("Headphone Playback Switch", SGTL5000_CHIP_ANA_CTRL,
			4, 1, 1),
	SOC_SINGLE("Headphone_ZCD Playback Switch", SGTL5000_CHIP_ANA_CTRL,
			5, 1, 0),

	SOC_SINGLE_TLV("Mic Capture Volume", SGTL5000_CHIP_MIC_CTRL,
			0, 3, 0, mic_gain_tlv),

	SOC_DOUBLE_TLV("Line Out Playback Volume",
			SGTL5000_CHIP_LINE_OUT_VOL,
			SGTL5000_LINE_OUT_VOL_LEFT_SHIFT,
			SGTL5000_LINE_OUT_VOL_RIGHT_SHIFT,
			0x1f, 1,
			lineout_volume),
	SOC_SINGLE("Line Out Playback Switch", SGTL5000_CHIP_ANA_CTRL, 8, 1, 1),
};


/* mute the codec used by alsa core */
static int sgtl5000_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 adcdac_ctrl = SGTL5000_DAC_MUTE_LEFT | SGTL5000_DAC_MUTE_RIGHT;

	snd_soc_update_bits(codec, SGTL5000_CHIP_ADCDAC_CTRL,
			adcdac_ctrl, mute ? adcdac_ctrl : 0);

	return 0;
}

static const struct snd_soc_dai_ops sgtl5000_ops = {
	.digital_mute = sgtl5000_digital_mute,
};

#define RPMSG_RATES (SNDRV_PCM_RATE_8000_48000)

#define RPMSG_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE )


static struct snd_soc_dai_driver rpmsg_sgtl5000_codec_dai = {
	.name = "rpmsg-sgtl5000-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = RPMSG_RATES,
		.formats = RPMSG_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = RPMSG_RATES,
		.formats = RPMSG_FORMATS,
	},
	.ops = &sgtl5000_ops,
	.symmetric_rates = 1,
};


static unsigned int rpmsg_sgtl5000_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = snd_soc_codec_get_drvdata(codec);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[RPMSG_AUDIO_I2C];
	int err, reg_val;

	mutex_lock(&i2s_info->i2c_lock);
	rpmsg->param.buffer_addr = reg;
	rpmsg->header.cmd = GET_CODEC_VALUE;
	err = i2s_info->send_message(rpmsg, i2s_info);
	reg_val = rpmsg->param.buffer_size;
	mutex_unlock(&i2s_info->i2c_lock);
	if (err)
		return 0;

	return reg_val;
}

static int rpmsg_sgtl5000_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = snd_soc_codec_get_drvdata(codec);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[RPMSG_AUDIO_I2C];
	int err;

	mutex_lock(&i2s_info->i2c_lock);
	rpmsg->param.buffer_addr = reg;
	rpmsg->param.buffer_size = val;
	rpmsg->header.cmd = SET_CODEC_VALUE;
	err = i2s_info->send_message(rpmsg, i2s_info);
	mutex_unlock(&i2s_info->i2c_lock);
	if (err)
		return err;

	return 0;
}

/*
 * This precalculated table contains all (vag_val * 100 / lo_calcntrl) results
 * to select an appropriate lo_vol_* in SGTL5000_CHIP_LINE_OUT_VOL
 * The calculatation was done for all possible register values which
 * is the array index and the following formula: 10^((idx−15)/40) * 100
 */
static const u8 vol_quot_table[] = {
	42, 45, 47, 50, 53, 56, 60, 63,
	67, 71, 75, 79, 84, 89, 94, 100,
	106, 112, 119, 126, 133, 141, 150, 158,
	168, 178, 188, 200, 211, 224, 237, 251
};

/*
 * sgtl5000 has 3 internal power supplies:
 * 1. VAG, normally set to vdda/2
 * 2. charge pump, set to different value
 *	according to voltage of vdda and vddio
 * 3. line out VAG, normally set to vddio/2
 *
 * and should be set according to:
 * 1. vddd provided by external or not
 * 2. vdda and vddio voltage value. > 3.1v or not
 */
static int sgtl5000_set_power_regs(struct snd_soc_codec *codec)
{
	int vddd;
	int vdda;
	int vddio;
	u16 ana_pwr;
	u16 lreg_ctrl;
	int vag;
	int lo_vag;
	int vol_quot;
	int lo_vol;
	size_t i;

	/* Hard coded for Picocoremx7ULP
	   TODO: Use devicetree or move to Cortex-M4*/
	vdda  = 3000000;
	vddio = 3300000;
	vddd  = 1800000;

	vdda  = vdda / 1000;
	vddio = vddio / 1000;
	vddd  = vddd / 1000;

	if (vdda <= 0 || vddio <= 0 || vddd < 0) {
		dev_err(codec->dev, "regulator voltage not set correctly\n");

		return -EINVAL;
	}

	/* according to datasheet, maximum voltage of supplies */
	if (vdda > 3600 || vddio > 3600 || vddd > 1980) {
		dev_err(codec->dev,
			"exceed max voltage vdda %dmV vddio %dmV vddd %dmV\n",
			vdda, vddio, vddd);

		return -EINVAL;
	}

	/* reset value */
	ana_pwr = snd_soc_read(codec, SGTL5000_CHIP_ANA_POWER);
	ana_pwr |= SGTL5000_DAC_STEREO |
			SGTL5000_ADC_STEREO |
			SGTL5000_REFTOP_POWERUP;
	lreg_ctrl = snd_soc_read(codec, SGTL5000_CHIP_LINREG_CTRL);

	if (vddio < 3100 && vdda < 3100) {
		/* enable internal oscillator used for charge pump */
		snd_soc_update_bits(codec, SGTL5000_CHIP_CLK_TOP_CTRL,
					SGTL5000_INT_OSC_EN,
					SGTL5000_INT_OSC_EN);
		/* Enable VDDC charge pump */
		ana_pwr |= SGTL5000_VDDC_CHRGPMP_POWERUP;
	} else if (vddio >= 3100 && vdda >= 3100) {
		ana_pwr &= ~SGTL5000_VDDC_CHRGPMP_POWERUP;
		/* VDDC use VDDIO rail */
		lreg_ctrl |= SGTL5000_VDDC_ASSN_OVRD;
		lreg_ctrl |= SGTL5000_VDDC_MAN_ASSN_VDDIO <<
			    SGTL5000_VDDC_MAN_ASSN_SHIFT;
	}

	snd_soc_write(codec, SGTL5000_CHIP_LINREG_CTRL, lreg_ctrl);

	snd_soc_write(codec, SGTL5000_CHIP_ANA_POWER, ana_pwr);

	/*
	 * set ADC/DAC VAG to vdda / 2,
	 * should stay in range (0.8v, 1.575v)
	 */
	vag = vdda / 2;
	if (vag <= SGTL5000_ANA_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_ANA_GND_BASE + SGTL5000_ANA_GND_STP *
		 (SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT))
		vag = SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT;
	else
		vag = (vag - SGTL5000_ANA_GND_BASE) / SGTL5000_ANA_GND_STP;

	snd_soc_update_bits(codec, SGTL5000_CHIP_REF_CTRL,
			SGTL5000_ANA_GND_MASK, vag << SGTL5000_ANA_GND_SHIFT);

	/* set line out VAG to vddio / 2, in range (0.8v, 1.675v) */
	lo_vag = vddio / 2;
	if (lo_vag <= SGTL5000_LINE_OUT_GND_BASE)
		lo_vag = 0;
	else if (lo_vag >= SGTL5000_LINE_OUT_GND_BASE +
		SGTL5000_LINE_OUT_GND_STP * SGTL5000_LINE_OUT_GND_MAX)
		lo_vag = SGTL5000_LINE_OUT_GND_MAX;
	else
		lo_vag = (lo_vag - SGTL5000_LINE_OUT_GND_BASE) /
		    SGTL5000_LINE_OUT_GND_STP;

	snd_soc_update_bits(codec, SGTL5000_CHIP_LINE_OUT_CTRL,
			SGTL5000_LINE_OUT_CURRENT_MASK |
			SGTL5000_LINE_OUT_GND_MASK,
			lo_vag << SGTL5000_LINE_OUT_GND_SHIFT |
			SGTL5000_LINE_OUT_CURRENT_360u <<
				SGTL5000_LINE_OUT_CURRENT_SHIFT);

	/*
	 * Set lineout output level in range (0..31)
	 * the same value is used for right and left channel
	 *
	 * Searching for a suitable index solving this formula:
	 * idx = 40 * log10(vag_val / lo_cagcntrl) + 15
	 */
	vol_quot = (vag * 100) / lo_vag;
	lo_vol = 0;
	for (i = 0; i < ARRAY_SIZE(vol_quot_table); i++) {
		if (vol_quot >= vol_quot_table[i])
			lo_vol = i;
		else
			break;
	}

	snd_soc_update_bits(codec, SGTL5000_CHIP_LINE_OUT_VOL,
		SGTL5000_LINE_OUT_VOL_RIGHT_MASK |
		SGTL5000_LINE_OUT_VOL_LEFT_MASK,
		lo_vol << SGTL5000_LINE_OUT_VOL_RIGHT_SHIFT |
		lo_vol << SGTL5000_LINE_OUT_VOL_LEFT_SHIFT);

	return 0;
}

static int rpmsg_sgtl5000_probe(struct snd_soc_codec *codec)
{
	int ret;
		/* power up sgtl5000 */
	ret = sgtl5000_set_power_regs(codec);

	/* disable small pop, introduce 200ms delay in turning off */
	snd_soc_update_bits(codec, SGTL5000_CHIP_REF_CTRL,
				SGTL5000_SMALL_POP, 0);

	/* disable short cut detector */
	snd_soc_write(codec, SGTL5000_CHIP_SHORT_CTRL, 0);

	/*
	 * set i2s as default input of sound switch
	 * TODO: add sound switch to control and dapm widge.
	 */
	snd_soc_write(codec, SGTL5000_CHIP_SSS_CTRL,
			SGTL5000_DAC_SEL_I2S_IN << SGTL5000_DAC_SEL_SHIFT);
	snd_soc_write(codec, SGTL5000_CHIP_DIG_POWER,
			SGTL5000_ADC_EN | SGTL5000_DAC_EN);

	/* enable dac volume ramp by default */
	snd_soc_write(codec, SGTL5000_CHIP_ADCDAC_CTRL,
			SGTL5000_DAC_VOL_RAMP_EN |
			SGTL5000_DAC_MUTE_RIGHT |
			SGTL5000_DAC_MUTE_LEFT);

	snd_soc_write(codec, SGTL5000_CHIP_ANA_CTRL,
			SGTL5000_HP_ZCD_EN |
			SGTL5000_ADC_ZCD_EN);

	/* Set Mic bias voltage */
	snd_soc_update_bits(codec, SGTL5000_CHIP_MIC_CTRL,
			SGTL5000_BIAS_VOLT_MASK,
			0x0 << SGTL5000_BIAS_VOLT_SHIFT);
	/* Disable DAP */
	snd_soc_write(codec, SGTL5000_DAP_CTRL, 0);

	return 0;
}

static struct snd_soc_codec_driver rpmsg_sgtl5000_codec = {
	.probe = rpmsg_sgtl5000_probe,
	.read = rpmsg_sgtl5000_read,
	.write = rpmsg_sgtl5000_write,
	.component_driver = {
		.controls		= rpmsg_sgtl5000_snd_controls,
		.num_controls		= ARRAY_SIZE(rpmsg_sgtl5000_snd_controls),
		.dapm_widgets		= sgtl5000_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(sgtl5000_dapm_widgets),
		.dapm_routes		= sgtl5000_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(sgtl5000_dapm_routes),
	},
};

static int rpmsg_sgtl5000_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(pdev->dev.parent);
	int ret;
	ret = snd_soc_register_codec(dev,
					&rpmsg_sgtl5000_codec,
					&rpmsg_sgtl5000_codec_dai,
					1);
	if (ret) {
		dev_err(dev, "%s: snd_soc_register_codec() failed (%d)\n",
			__func__, ret);
		return ret;
	}

	dev_set_drvdata(dev, rpmsg_i2s);

	return 0;
}

static int rpmsg_sgtl5000_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver rpmsg_sgtl5000_codec_driver = {
	.driver = {
		.name = RPMSG_CODEC_DRV_NAME,
	},
	.probe = rpmsg_sgtl5000_codec_probe,
	.remove = rpmsg_sgtl5000_codec_remove,
};

module_platform_driver(rpmsg_sgtl5000_codec_driver);

MODULE_DESCRIPTION("rpmsg sgtl5000 Codec Driver");
MODULE_LICENSE("GPL");
