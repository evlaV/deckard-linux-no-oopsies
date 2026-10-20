// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * max98390.c  --  MAX98390 ALSA Soc Audio driver
 *
 * Copyright (C) 2020 Maxim Integrated Products
 *
 */

#include <linux/acpi.h>
#include <linux/cdev.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "max98390.h"

static const struct reg_default max98390_reg_defaults[] = {
	{MAX98390_INT_EN1, 0xf0},
	{MAX98390_INT_EN2, 0x00},
	{MAX98390_INT_EN3, 0x00},
	{MAX98390_INT_FLAG_CLR1, 0x00},
	{MAX98390_INT_FLAG_CLR2, 0x00},
	{MAX98390_INT_FLAG_CLR3, 0x00},
	{MAX98390_IRQ_CTRL, 0x01},
	{MAX98390_CLK_MON, 0x6d},
	{MAX98390_DAT_MON, 0x03},
	{MAX98390_WDOG_CTRL, 0x00},
	{MAX98390_WDOG_RST, 0x00},
	{MAX98390_MEAS_ADC_THERM_WARN_THRESH, 0x75},
	{MAX98390_MEAS_ADC_THERM_SHDN_THRESH, 0x8c},
	{MAX98390_MEAS_ADC_THERM_HYSTERESIS, 0x08},
	{MAX98390_PIN_CFG, 0x55},
	{MAX98390_PCM_RX_EN_A, 0x00},
	{MAX98390_PCM_RX_EN_B, 0x00},
	{MAX98390_PCM_TX_EN_A, 0x00},
	{MAX98390_PCM_TX_EN_B, 0x00},
	{MAX98390_PCM_TX_HIZ_CTRL_A, 0xff},
	{MAX98390_PCM_TX_HIZ_CTRL_B, 0xff},
	{MAX98390_PCM_CH_SRC_1, 0x00},
	{MAX98390_PCM_CH_SRC_2, 0x00},
	{MAX98390_PCM_CH_SRC_3, 0x00},
	{MAX98390_PCM_MODE_CFG, 0xc0},
	{MAX98390_PCM_MASTER_MODE, 0x1c},
	{MAX98390_PCM_CLK_SETUP, 0x44},
	{MAX98390_PCM_SR_SETUP, 0x08},
	{MAX98390_ICC_RX_EN_A, 0x00},
	{MAX98390_ICC_RX_EN_B, 0x00},
	{MAX98390_ICC_TX_EN_A, 0x00},
	{MAX98390_ICC_TX_EN_B, 0x00},
	{MAX98390_ICC_HIZ_MANUAL_MODE, 0x00},
	{MAX98390_ICC_TX_HIZ_EN_A, 0x00},
	{MAX98390_ICC_TX_HIZ_EN_B, 0x00},
	{MAX98390_ICC_LNK_EN, 0x00},
	{MAX98390_R2039_AMP_DSP_CFG, 0x0f},
	{MAX98390_R203A_AMP_EN, 0x81},
	{MAX98390_TONE_GEN_DC_CFG, 0x00},
	{MAX98390_SPK_SRC_SEL, 0x00},
	{MAX98390_SSM_CFG, 0x85},
	{MAX98390_MEAS_EN, 0x03},
	{MAX98390_MEAS_DSP_CFG, 0x0f},
	{MAX98390_BOOST_CTRL0, 0x1c},
	{MAX98390_BOOST_CTRL3, 0x01},
	{MAX98390_BOOST_CTRL1, 0x40},
	{MAX98390_MEAS_ADC_CFG, 0x07},
	{MAX98390_MEAS_ADC_BASE_MSB, 0x00},
	{MAX98390_MEAS_ADC_BASE_LSB, 0x23},
	{MAX98390_ADC_CH0_DIVIDE, 0x00},
	{MAX98390_ADC_CH1_DIVIDE, 0x00},
	{MAX98390_ADC_CH2_DIVIDE, 0x00},
	{MAX98390_ADC_CH0_FILT_CFG, 0x00},
	{MAX98390_ADC_CH1_FILT_CFG, 0x00},
	{MAX98390_ADC_CH2_FILT_CFG, 0x00},
	{MAX98390_PWR_GATE_CTL, 0x2c},
	{MAX98390_BROWNOUT_EN, 0x00},
	{MAX98390_BROWNOUT_INFINITE_HOLD, 0x00},
	{MAX98390_BROWNOUT_INFINITE_HOLD_CLR, 0x00},
	{MAX98390_BROWNOUT_LVL_HOLD, 0x00},
	{MAX98390_BROWNOUT_LVL1_THRESH, 0x00},
	{MAX98390_BROWNOUT_LVL2_THRESH, 0x00},
	{MAX98390_BROWNOUT_LVL3_THRESH, 0x00},
	{MAX98390_BROWNOUT_LVL4_THRESH, 0x00},
	{MAX98390_BROWNOUT_THRESH_HYSTERYSIS, 0x00},
	{MAX98390_BROWNOUT_AMP_LIMITER_ATK_REL, 0x1f},
	{MAX98390_BROWNOUT_AMP_GAIN_ATK_REL, 0x00},
	{MAX98390_BROWNOUT_AMP1_CLIP_MODE, 0x00},
	{MAX98390_BROWNOUT_LVL1_CUR_LIMIT, 0x00},
	{MAX98390_BROWNOUT_LVL1_AMP1_CTRL1, 0x00},
	{MAX98390_BROWNOUT_LVL1_AMP1_CTRL2, 0x00},
	{MAX98390_BROWNOUT_LVL1_AMP1_CTRL3, 0x00},
	{MAX98390_BROWNOUT_LVL2_CUR_LIMIT, 0x00},
	{MAX98390_BROWNOUT_LVL2_AMP1_CTRL1, 0x00},
	{MAX98390_BROWNOUT_LVL2_AMP1_CTRL2, 0x00},
	{MAX98390_BROWNOUT_LVL2_AMP1_CTRL3, 0x00},
	{MAX98390_BROWNOUT_LVL3_CUR_LIMIT, 0x00},
	{MAX98390_BROWNOUT_LVL3_AMP1_CTRL1, 0x00},
	{MAX98390_BROWNOUT_LVL3_AMP1_CTRL2, 0x00},
	{MAX98390_BROWNOUT_LVL3_AMP1_CTRL3, 0x00},
	{MAX98390_BROWNOUT_LVL4_CUR_LIMIT, 0x00},
	{MAX98390_BROWNOUT_LVL4_AMP1_CTRL1, 0x00},
	{MAX98390_BROWNOUT_LVL4_AMP1_CTRL2, 0x00},
	{MAX98390_BROWNOUT_LVL4_AMP1_CTRL3, 0x00},
	{MAX98390_BROWNOUT_ILIM_HLD, 0x00},
	{MAX98390_BROWNOUT_LIM_HLD, 0x00},
	{MAX98390_BROWNOUT_CLIP_HLD, 0x00},
	{MAX98390_BROWNOUT_GAIN_HLD, 0x00},
	{MAX98390_ENV_TRACK_VOUT_HEADROOM, 0x0f},
	{MAX98390_ENV_TRACK_BOOST_VOUT_DELAY, 0x80},
	{MAX98390_ENV_TRACK_REL_RATE, 0x07},
	{MAX98390_ENV_TRACK_HOLD_RATE, 0x07},
	{MAX98390_ENV_TRACK_CTRL, 0x01},
	{MAX98390_BOOST_BYPASS1, 0x49},
	{MAX98390_BOOST_BYPASS2, 0x2b},
	{MAX98390_BOOST_BYPASS3, 0x08},
	{MAX98390_FET_SCALING1, 0x00},
	{MAX98390_FET_SCALING2, 0x03},
	{MAX98390_FET_SCALING3, 0x00},
	{MAX98390_FET_SCALING4, 0x07},
	{MAX98390_SPK_SPEEDUP, 0x00},
	{DSMIG_WB_DRC_RELEASE_TIME_1, 0x00},
	{DSMIG_WB_DRC_RELEASE_TIME_2, 0x00},
	{DSMIG_WB_DRC_ATTACK_TIME_1, 0x00},
	{DSMIG_WB_DRC_ATTACK_TIME_2, 0x00},
	{DSMIG_WB_DRC_COMPRESSION_RATIO, 0x00},
	{DSMIG_WB_DRC_COMPRESSION_THRESHOLD, 0x00},
	{DSMIG_WB_DRC_MAKEUPGAIN, 0x00},
	{DSMIG_WB_DRC_NOISE_GATE_THRESHOLD, 0x00},
	{DSMIG_WBDRC_HPF_ENABLE, 0x00},
	{DSMIG_WB_DRC_TEST_SMOOTHER_OUT_EN, 0x00},
	{DSMIG_PPR_THRESHOLD, 0x00},
	{DSM_STEREO_BASS_CHANNEL_SELECT, 0x00},
	{DSM_TPROT_THRESHOLD_BYTE0, 0x00},
	{DSM_TPROT_THRESHOLD_BYTE1, 0x00},
	{DSM_TPROT_ROOM_TEMPERATURE_BYTE0, 0x00},
	{DSM_TPROT_ROOM_TEMPERATURE_BYTE1, 0x00},
	{DSM_TPROT_RECIP_RDC_ROOM_BYTE0, 0x00},
	{DSM_TPROT_RECIP_RDC_ROOM_BYTE1, 0x00},
	{DSM_TPROT_RECIP_RDC_ROOM_BYTE2, 0x00},
	{DSM_TPROT_RECIP_TCONST_BYTE0, 0x00},
	{DSM_TPROT_RECIP_TCONST_BYTE1, 0x00},
	{DSM_TPROT_RECIP_TCONST_BYTE2, 0x00},
	{DSM_THERMAL_ATTENUATION_SETTINGS, 0x00},
	{DSM_THERMAL_PILOT_TONE_ATTENUATION, 0x00},
	{DSM_TPROT_PG_TEMP_THRESH_BYTE0, 0x00},
	{DSM_TPROT_PG_TEMP_THRESH_BYTE1, 0x00},
	{DSMIG_DEBUZZER_THRESHOLD, 0x00},
	{DSMIG_DEBUZZER_ALPHA_COEF_TEST_ONLY, 0x08},
	{DSM_VOL_ENA, 0x20},
	{DSM_VOL_CTRL, 0xa0},
	{DSMIG_EN, 0x00},
	{MAX98390_R23E1_DSP_GLOBAL_EN, 0x00},
	{MAX98390_R23FF_GLOBAL_EN, 0x00},
};

static int max98390_dai_set_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);
	unsigned int mode;
	unsigned int format;
	unsigned int invert = 0;

	dev_dbg(component->dev, "%s: fmt 0x%08X\n", __func__, fmt);

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		mode = MAX98390_PCM_MASTER_MODE_SLAVE;
		break;
	case SND_SOC_DAIFMT_CBP_CFP:
		max98390->provider = true;
		mode = MAX98390_PCM_MASTER_MODE_MASTER;
		break;
	default:
		dev_err(component->dev, "DAI clock mode unsupported\n");
		return -EINVAL;
	}

	regmap_update_bits(max98390->regmap,
		MAX98390_PCM_MASTER_MODE,
		MAX98390_PCM_MASTER_MODE_MASK,
		mode);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		invert = MAX98390_PCM_MODE_CFG_PCM_BCLKEDGE;
		break;
	default:
		dev_err(component->dev, "DAI invert mode unsupported\n");
		return -EINVAL;
	}

	regmap_update_bits(max98390->regmap,
		MAX98390_PCM_MODE_CFG,
		MAX98390_PCM_MODE_CFG_PCM_BCLKEDGE,
		invert);

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format = MAX98390_PCM_FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format = MAX98390_PCM_FORMAT_LJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		format = MAX98390_PCM_FORMAT_TDM_MODE1;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		format = MAX98390_PCM_FORMAT_TDM_MODE0;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(max98390->regmap,
		MAX98390_PCM_MODE_CFG,
		MAX98390_PCM_MODE_CFG_FORMAT_MASK,
		format << MAX98390_PCM_MODE_CFG_FORMAT_SHIFT);

	return 0;
}

static int max98390_get_bclk_sel(int bclk)
{
	int i;
	/* BCLKs per LRCLK */
	static int bclk_sel_table[] = {
		32, 48, 64, 96, 128, 192, 256, 320, 384, 512,
	};
	/* match BCLKs per LRCLK */
	for (i = 0; i < ARRAY_SIZE(bclk_sel_table); i++) {
		if (bclk_sel_table[i] == bclk)
			return i + 2;
	}
	return 0;
}

static int max98390_set_clock(struct snd_soc_component *component,
		struct snd_pcm_hw_params *params)
{
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);
	/* codec MCLK rate in master mode */
	static int rate_table[] = {
		5644800, 6000000, 6144000, 6500000,
		9600000, 11289600, 12000000, 12288000,
		13000000, 19200000,
	};
	/* BCLK/LRCLK ratio calculation */
	int blr_clk_ratio = params_channels(params)
		* snd_pcm_format_width(params_format(params));
	int value;

	if (max98390->provider) {
		int i;
		/* match rate to closest value */
		for (i = 0; i < ARRAY_SIZE(rate_table); i++) {
			if (rate_table[i] >= max98390->sysclk)
				break;
		}
		if (i == ARRAY_SIZE(rate_table)) {
			dev_err(component->dev, "failed to find proper clock rate.\n");
			return -EINVAL;
		}

		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_MASTER_MODE,
			MAX98390_PCM_MASTER_MODE_MCLK_MASK,
			i << MAX98390_PCM_MASTER_MODE_MCLK_RATE_SHIFT);
	}

	if (!max98390->tdm_mode) {
		/* BCLK configuration */
		value = max98390_get_bclk_sel(blr_clk_ratio);
		if (!value) {
			/*
			 * Returning -ENOTSUPP instead of -EINVAL avoids
			 * an unnecessary dev_err message in the caller.
			 */
			return -ENOTSUPP;
		}

		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_CLK_SETUP,
			MAX98390_PCM_CLK_SETUP_BSEL_MASK,
			value);
	}
	return 0;
}

static int max98390_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component =
		dai->component;
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	unsigned int sampling_rate;
	unsigned int chan_sz;

	/* pcm mode configuration */
	switch (snd_pcm_format_width(params_format(params))) {
	case 16:
		chan_sz = MAX98390_PCM_MODE_CFG_CHANSZ_16;
		break;
	case 24:
		chan_sz = MAX98390_PCM_MODE_CFG_CHANSZ_24;
		break;
	case 32:
		chan_sz = MAX98390_PCM_MODE_CFG_CHANSZ_32;
		break;
	default:
		dev_err(component->dev, "format unsupported %d\n",
			params_format(params));
		goto err;
	}

	regmap_update_bits(max98390->regmap,
		MAX98390_PCM_MODE_CFG,
		MAX98390_PCM_MODE_CFG_CHANSZ_MASK, chan_sz);

	dev_dbg(component->dev, "format supported %d",
		params_format(params));

	/* sampling rate configuration */
	switch (params_rate(params)) {
	case 8000:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_8000;
		break;
	case 11025:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_11025;
		break;
	case 12000:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_12000;
		break;
	case 16000:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_16000;
		break;
	case 22050:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_22050;
		break;
	case 24000:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_24000;
		break;
	case 32000:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_32000;
		break;
	case 44100:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_44100;
		break;
	case 48000:
		sampling_rate = MAX98390_PCM_SR_SET1_SR_48000;
		break;
	default:
		dev_err(component->dev, "rate %d not supported\n",
			params_rate(params));
		goto err;
	}

	/* set DAI_SR to correct LRCLK frequency */
	regmap_update_bits(max98390->regmap,
		MAX98390_PCM_SR_SETUP,
		MAX98390_PCM_SR_SET1_SR_MASK,
		sampling_rate);

	return max98390_set_clock(component, params);
err:
	return -EINVAL;
}

static int max98390_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	int bsel;
	unsigned int chan_sz;

	if (!tx_mask && !rx_mask && !slots && !slot_width)
		max98390->tdm_mode = false;
	else
		max98390->tdm_mode = true;

	dev_dbg(component->dev,
		"Tdm mode : %d\n", max98390->tdm_mode);

	/* BCLK configuration */
	bsel = max98390_get_bclk_sel(slots * slot_width);
	if (!bsel) {
		dev_err(component->dev, "BCLK %d not supported\n",
			slots * slot_width);
		return -EINVAL;
	}

	regmap_update_bits(max98390->regmap,
		MAX98390_PCM_CLK_SETUP,
		MAX98390_PCM_CLK_SETUP_BSEL_MASK,
		bsel);

	/* Channel size configuration */
	switch (slot_width) {
	case 16:
		chan_sz = MAX98390_PCM_MODE_CFG_CHANSZ_16;
		break;
	case 24:
		chan_sz = MAX98390_PCM_MODE_CFG_CHANSZ_24;
		break;
	case 32:
		chan_sz = MAX98390_PCM_MODE_CFG_CHANSZ_32;
		break;
	default:
		dev_err(component->dev, "format unsupported %d\n",
			slot_width);
		return -EINVAL;
	}

	regmap_update_bits(max98390->regmap,
		MAX98390_PCM_MODE_CFG,
		MAX98390_PCM_MODE_CFG_CHANSZ_MASK, chan_sz);

	/* Rx slot configuration */
	regmap_write(max98390->regmap,
		MAX98390_PCM_RX_EN_A,
		rx_mask & 0xFF);
	regmap_write(max98390->regmap,
		MAX98390_PCM_RX_EN_B,
		(rx_mask & 0xFF00) >> 8);

	/* Tx slot Hi-Z configuration */
	regmap_write(max98390->regmap,
		MAX98390_PCM_TX_HIZ_CTRL_A,
		~tx_mask & 0xFF);
	regmap_write(max98390->regmap,
		MAX98390_PCM_TX_HIZ_CTRL_B,
		(~tx_mask & 0xFF00) >> 8);

	return 0;
}

static int max98390_dai_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	max98390->sysclk = freq;
	return 0;
}

static void max98390_do_mute(struct max98390_priv *max98390)
{
	if (max98390->mute) {
		regmap_write(max98390->regmap, MAX98390_R23FF_GLOBAL_EN, 0x00);
		/* Might need a 400us or 6ms sleep here based
		 * DSM_VOL_RMP_DN_BYP values, we pick the higher number
		 * to be sure EN=1 after this can latch */
		regmap_update_bits(max98390->regmap,
			MAX98390_R203A_AMP_EN,
			MAX98390_AMP_EN_MASK, 0);
		usleep_range(6000, 8000);
	} else {
		regmap_update_bits(max98390->regmap,
			MAX98390_R203A_AMP_EN,
			MAX98390_AMP_EN_MASK, 1);
		regmap_write(max98390->regmap, MAX98390_R23FF_GLOBAL_EN, 0x01);
		/* Might need a 750us or 10ms sleep here based on SPK_SPEEDUP
		 * and DSM_VOL_RMP_UP_BYP values, we pick the higher number
		 * to be sure tON is complete */
		usleep_range(10000, 12000);
	}

	/* EN=1 sometimes fails to latch, lets try to handle that */
	if (!max98390->mute) {
		struct snd_soc_component *component = max98390->component;
		unsigned int val = 0;
		int ret;

		regmap_read(max98390->regmap, MAX98390_R23FF_GLOBAL_EN, &val);
		if (!(val & MAX98390_GLOBAL_EN_MASK)) {
			dev_warn(component->dev, "EN=1 latch failed, retry\n");
			ret = regmap_write(max98390->regmap,
					MAX98390_R23FF_GLOBAL_EN, 0x01);
			usleep_range(10000, 12000);

			regmap_read(max98390->regmap,
					MAX98390_R23FF_GLOBAL_EN, &val);
			if (!(val & MAX98390_GLOBAL_EN_MASK)) {
				dev_warn(component->dev, "EN=1 latch failed"
						"(%d), gave up\n", ret);
			}
		}
	}
}

/*
 * Interrupt decode, indexed by bit position within the latched flag banks.
 * *_STATE is set by a rising edge on the corresponding *_RAW bit and cleared
 * only by writing 1 to *_CLR, so a condition that is still present does not
 * re-latch and the level-triggered line needs no storm protection.
 *
 * The thermal stages are worth reporting because the device acts on them
 * itself: it disables the Class-D amplifier above the shutdown threshold, and
 * after such a shutdown clears EN once the die drops back below the warning
 * threshold. The driver has no other way to learn that the part has put itself
 * into software shutdown.
 */
static const char * const max98390_int1_msg[8] = {
	"brownout: BDE back to normal",
	"brownout: BDE active",
	"brownout: level change",
	"brownout: entered level 4",
	"thermal warning cleared: die below warn threshold",
	"thermal warning: die above warn threshold",
	"thermal shutdown cleared: die below shutdown threshold",
	"thermal shutdown: die above shutdown threshold",
};

static const char * const max98390_int2_msg[8] = {
	"watchdog timer at 75%",
	"watchdog timer expired",
	"boost input current limit reached",
	"power-down done",
	"power-up done",
	"power-up failed",
	"speaker overcurrent: amp current limit exceeded",
	"boost UVLO: output below minimum voltage",
};

static const char * const max98390_int3_msg[8] = {
	"BCLK rate error",
	"BCLK rate error recovered",
	"LRCLK rate error",
	"LRCLK rate error recovered",
	"frame error: BCLK/LRCLK ratio does not match BSEL",
	"frame error recovered",
	"data monitor: DC detected in input",
	NULL,	/* reserved */
};

/*
 * The two thermal sources the driver acts on by itself. The shutdown is worth
 * reporting because the device disables the Class-D amplifier on its own, and
 * the end of the warning excursion is what tells us it has also cleared EN;
 * see max98390_thermal_rearm().
 */
#define MAX98390_INT1_THERMSHDN_START	BIT(7)
#define MAX98390_INT1_THERMWARN_END	BIT(4)
#define MAX98390_INT1_ESSENTIAL		(MAX98390_INT1_THERMSHDN_START | \
					 MAX98390_INT1_THERMWARN_END)

/*
 * Log any data or clock errors. These are non-routine and make sense to flag
 * as a log. The corresponding monitors might auto restart or not, and those
 * are deemed less interesting in the interest of keeping logging to a minimum.
 * Note that these will not be triggered unless the clock/data monitors are
 * enabled.
 */
#define MAX98390_INT3_DAT_MON_ERR	BIT(6)
#define MAX98390_INT3_PCM_FRAME_ERR	BIT(4)
#define MAX98390_INT3_LRCLK_ERR		BIT(2)
#define MAX98390_INT3_BCLK_ERR		BIT(0)
#define MAX98390_INT3_ESSENTIAL		(MAX98390_INT3_DAT_MON_ERR | \
					 MAX98390_INT3_PCM_FRAME_ERR | \
					 MAX98390_INT3_LRCLK_ERR | \
					 MAX98390_INT3_BCLK_ERR)

/*
 * A source the driver cannot keep up with would otherwise loop indefinitely: a
 * level-triggered line that is not cleared re-asserts as soon as the threaded
 * handler unmasks it, and because each pass returns IRQ_HANDLED the core's own
 * spurious detector never engages. Give up rather than tie up the i2c bus.
 */
#define MAX98390_IRQ_STORM_INTERVAL	HZ
#define MAX98390_IRQ_STORM_BURST	100

/*
 * Events that are the ordinary outcome of an operation rather than the
 * consequence of a fault, logged at debug level.
 *
 * The return-to-normal reports are deliberately excluded. Each one is the
 * trailing edge of a corresponding error - a clock recovery is only generated
 * after a clock error, and the brownout engine can only re-enter level 0 having
 * left it - so keeping both ends at warning level is what makes the duration of
 * a fault readable from the log.
 */
#define MAX98390_INT1_ROUTINE	0
#define MAX98390_INT2_ROUTINE	(BIT(4) | BIT(3))
#define MAX98390_INT3_ROUTINE	0

/*
 * The masks default to the two sources the driver acts on, so this reports
 * nothing new unless one is widened. They are read-only at runtime and can only
 * be set at module load or on the kernel command line.
 */
static unsigned int max98390_int_en1 = MAX98390_INT1_ESSENTIAL;
static unsigned int max98390_int_en2;
static unsigned int max98390_int_en3 = MAX98390_INT3_ESSENTIAL;

module_param_named(int_en1, max98390_int_en1, uint, 0444);
MODULE_PARM_DESC(int_en1,
	"INT_EN1 mask: thermal and brownout (default 0x90, 0xff for all)");
module_param_named(int_en2, max98390_int_en2, uint, 0444);
MODULE_PARM_DESC(int_en2,
	"INT_EN2 mask: supply, overcurrent, watchdog (default 0, 0xff for all)");
module_param_named(int_en3, max98390_int_en3, uint, 0444);
MODULE_PARM_DESC(int_en3,
	"INT_EN3 mask: clocking, bit 7 reserved (default 0, 0x7f for all)");
/* Live status, to accompany a fault report. */
static void max98390_log_state(struct max98390_priv *max98390)
{
	unsigned int gate = 0, bde = 0, vbat = 0, die = 0, gain = 0;
	unsigned int rdc_hi = 0, rdc_lo = 0, coil_hi = 0, coil_lo = 0;

	regmap_read(max98390->regmap, MAX98390_PWR_GATE_STATUS, &gate);
	regmap_read(max98390->regmap, MAX98390_BROWNOUT_STATUS, &bde);
	regmap_read(max98390->regmap, MAX98390_MEAS_ADC_CH0_READ, &vbat);
	regmap_read(max98390->regmap, MAX98390_MEAS_ADC_CH2_READ, &die);
	regmap_read(max98390->regmap, DSM_THERMAL_GAIN, &gain);
	regmap_read(max98390->regmap, THERMAL_RDC_RD_BACK_BYTE1, &rdc_hi);
	regmap_read(max98390->regmap, THERMAL_RDC_RD_BACK_BYTE0, &rdc_lo);
	regmap_read(max98390->regmap, THERMAL_COILTEMP_RD_BACK_BYTE1, &coil_hi);
	regmap_read(max98390->regmap, THERMAL_COILTEMP_RD_BACK_BYTE0, &coil_lo);

	dev_warn_ratelimited(max98390->dev,
		"state: pwrgate=0x%02x bde=0x%02x thermgain=0x%02x vbat_adc=0x%02x die=%dC rdc=%u coil=%dC\n",
		gate, bde, gain, vbat,
		/* die temperature: T = code * 1.28 - 29 */
		((int)die * 128 - 2900) / 100,
		/* Rdc readback is 14 bit, 1 LSB = 11/768 ohm */
		((rdc_hi & 0x3f) << 8) | rdc_lo,
		/* coil temperature is <4,12> fixed point, 100/4096 C per LSB */
		((int)(s16)((coil_hi << 8) | coil_lo) * 100) / 4096);
}

/*
 * The device clears EN itself at the end of a thermal shutdown, once the die has
 * cooled back below the thermal warning threshold, and does not restart on its
 * own.
 *
 * THERMWARN_END is also generated by an excursion that only crossed the warning
 * threshold, for which the device takes no action at all and the amplifier keeps
 * running. So test EN rather than inferring from the interrupt which stage of
 * the sequence the part went through - it is volatile, so this reads the device.
 *
 * EN reading back as 0 here is unambiguous: the thermal circuit takes its input
 * from the measurement ADC, and the part refuses to enable at all unless the
 * temperature channel is running (PWRUP_FAIL is raised instead), so a die
 * temperature excursion cannot be observed while the part is shut down.
 *
 * The restart itself is handed to mute_work rather than writing EN here: it
 * already implements the documented SPK_EN-then-EN order, and the workqueue
 * guarantees it will not run concurrently with itself.
 */
static void max98390_thermal_rearm(struct max98390_priv *max98390)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(max98390->component);
	unsigned int val;

	if (regmap_read(max98390->regmap, MAX98390_R23FF_GLOBAL_EN, &val))
		return;

	if (val & MAX98390_GLOBAL_EN_MASK)
		return;

	snd_soc_dapm_mutex_lock(dapm);

	if (max98390->mute) {
		snd_soc_dapm_mutex_unlock(dapm);
		return;
	}

	dev_warn_ratelimited(max98390->dev,
			     "restarting after thermal shutdown\n");

	max98390_do_mute(max98390);

	snd_soc_dapm_mutex_unlock(dapm);
}

static irqreturn_t max98390_irq(int irq, void *data)
{
	static const struct {
		unsigned int flag;
		unsigned int clr;
		const char * const *msg;
		unsigned int routine;
	} bank[] = {
		{ MAX98390_INT_FLAG1, MAX98390_INT_FLAG_CLR1,
		  max98390_int1_msg, MAX98390_INT1_ROUTINE },
		{ MAX98390_INT_FLAG2, MAX98390_INT_FLAG_CLR2,
		  max98390_int2_msg, MAX98390_INT2_ROUTINE },
		{ MAX98390_INT_FLAG3, MAX98390_INT_FLAG_CLR3,
		  max98390_int3_msg, MAX98390_INT3_ROUTINE },
	};
	struct max98390_priv *max98390 = data;
	irqreturn_t ret = IRQ_NONE;
	unsigned int flag, int1 = 0;
	bool fault = false;
	int i, bit;

	if (!__ratelimit(&max98390->storm)) {
		dev_err(max98390->dev, "interrupt storm, disabling irq %d\n",
			max98390->irq);
		disable_irq_nosync(max98390->irq);
		return IRQ_HANDLED;
	}

	for (i = 0; i < ARRAY_SIZE(bank); i++) {
		if (regmap_read(max98390->regmap, bank[i].flag, &flag))
			continue;
		if (!flag)
			continue;

		if (bank[i].flag == MAX98390_INT_FLAG1)
			int1 = flag;

		for (bit = 0; bit < 8; bit++) {
			if (!(flag & BIT(bit)) || !bank[i].msg[bit])
				continue;

			if (bank[i].routine & BIT(bit)) {
				dev_dbg_ratelimited(max98390->dev, "%s\n",
						    bank[i].msg[bit]);
			} else {
				dev_warn_ratelimited(max98390->dev, "%s\n",
						     bank[i].msg[bit]);
				fault = true;
			}
		}

		/*
		 * Clear only what was read, so anything latching in between
		 * re-asserts IRQ instead of being lost.
		 */
		regmap_write(max98390->regmap, bank[i].clr, flag);
		ret = IRQ_HANDLED;
	}

	if (fault)
		max98390_log_state(max98390);

	if (int1 & MAX98390_INT1_THERMWARN_END)
		max98390_thermal_rearm(max98390);

	return ret;
}

/*
 * Must run after the software reset, which returns IRQ_CTRL and the INT_EN
 * masks to their defaults.
 */
static void max98390_irq_setup(struct snd_soc_component *component)
{
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	if (max98390->irq <= 0)
		return;

	/*
	 * IRQ_EN, active low, open-drain. Open-drain is required wherever the
	 * IO supply of this device is higher than that of the pin it drives,
	 * because the high level is then produced by a pull-up on the host
	 * side instead of by this device. It is also the reset default.
	 */
	regmap_write(max98390->regmap, MAX98390_IRQ_CTRL, 0x01);

	/*
	 * IRQ_EN and the thermal bits of INT_EN1 are set out of reset, so the
	 * part has been latching flags since then. Discard them so the first
	 * interrupt reported is one that happened after the masks were set.
	 */
	regmap_write(max98390->regmap, MAX98390_INT_FLAG_CLR1, 0xff);
	regmap_write(max98390->regmap, MAX98390_INT_FLAG_CLR2, 0xff);
	regmap_write(max98390->regmap, MAX98390_INT_FLAG_CLR3, 0xff);

	regmap_write(max98390->regmap, MAX98390_INT_EN1, max98390_int_en1);
	regmap_write(max98390->regmap, MAX98390_INT_EN2, max98390_int_en2);
	regmap_write(max98390->regmap, MAX98390_INT_EN3, max98390_int_en3);

	dev_info(component->dev, "irq %d armed, en=0x%02x/0x%02x/0x%02x\n",
		 max98390->irq, max98390_int_en1, max98390_int_en2,
		 max98390_int_en3);

	/*
	 * Only ever touched from the threaded handler, which cannot run
	 * concurrently with itself, so the state needs no locking of its own.
	 */
	ratelimit_state_init(&max98390->storm, MAX98390_IRQ_STORM_INTERVAL,
			     MAX98390_IRQ_STORM_BURST);

	/* Requested with IRQF_NO_AUTOEN; safe to enable now. */
	enable_irq(max98390->irq);
}


static int max98390_dac_mute(struct snd_soc_dai *dai, int mute, int stream)
{
        struct snd_soc_component *component = dai->component;
        struct max98390_priv *max98390 =
                snd_soc_component_get_drvdata(component);

        if (max98390->mute != mute) {
                max98390->mute = mute;
                max98390_do_mute(max98390);
        }

        return 0;
}

static const struct snd_soc_dai_ops max98390_dai_ops = {
	.set_sysclk = max98390_dai_set_sysclk,
	.set_fmt = max98390_dai_set_fmt,
	.hw_params = max98390_dai_hw_params,
	.set_tdm_slot = max98390_dai_tdm_slot,
	.mute_stream = max98390_dac_mute,
};

static int max98390_dac_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}

static const char * const max98390_switch_text[] = {
	"Left", "Right", "LeftRight"};

static const char * const max98390_boost_voltage_text[] = {
	"6.5V", "6.625V", "6.75V", "6.875V", "7V", "7.125V", "7.25V", "7.375V",
	"7.5V", "7.625V", "7.75V", "7.875V", "8V", "8.125V", "8.25V", "8.375V",
	"8.5V", "8.625V", "8.75V", "8.875V", "9V", "9.125V", "9.25V", "9.375V",
	"9.5V", "9.625V", "9.75V", "9.875V", "10V"
};

static SOC_ENUM_SINGLE_DECL(max98390_boost_voltage,
		MAX98390_BOOST_CTRL0, 0,
		max98390_boost_voltage_text);

static DECLARE_TLV_DB_SCALE(max98390_spk_tlv, 300, 300, 0);
static DECLARE_TLV_DB_SCALE(max98390_digital_tlv, -8000, 50, 0);
#define MAX98390_DSM_VOL_MAX 184

static const char * const max98390_current_limit_text[] = {
	"0.00A", "0.50A", "1.00A", "1.05A", "1.10A", "1.15A", "1.20A", "1.25A",
	"1.30A", "1.35A", "1.40A", "1.45A", "1.50A", "1.55A", "1.60A", "1.65A",
	"1.70A", "1.75A", "1.80A", "1.85A", "1.90A", "1.95A", "2.00A", "2.05A",
	"2.10A", "2.15A", "2.20A", "2.25A", "2.30A", "2.35A", "2.40A", "2.45A",
	"2.50A", "2.55A", "2.60A", "2.65A", "2.70A", "2.75A", "2.80A", "2.85A",
	"2.90A", "2.95A", "3.00A", "3.05A", "3.10A", "3.15A", "3.20A", "3.25A",
	"3.30A", "3.35A", "3.40A", "3.45A", "3.50A", "3.55A", "3.60A", "3.65A",
	"3.70A", "3.75A", "3.80A", "3.85A", "3.90A", "3.95A", "4.00A", "4.05A",
	"4.10A"
};

static SOC_ENUM_SINGLE_DECL(max98390_current_limit,
		MAX98390_BOOST_CTRL1, 0,
		max98390_current_limit_text);

static int max98390_ref_rdc_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	max98390->ref_rdc_value = ucontrol->value.integer.value[0];

	regmap_write(max98390->regmap, DSM_TPROT_RECIP_RDC_ROOM_BYTE0,
		max98390->ref_rdc_value & 0x000000ff);
	regmap_write(max98390->regmap, DSM_TPROT_RECIP_RDC_ROOM_BYTE1,
		(max98390->ref_rdc_value >> 8) & 0x000000ff);
	regmap_write(max98390->regmap, DSM_TPROT_RECIP_RDC_ROOM_BYTE2,
		(max98390->ref_rdc_value >> 16) & 0x000000ff);

	return 0;
}

static int max98390_ref_rdc_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = max98390->ref_rdc_value;

	return 0;
}

static int max98390_ambient_temp_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	max98390->ambient_temp_value = ucontrol->value.integer.value[0];

	regmap_write(max98390->regmap, DSM_TPROT_ROOM_TEMPERATURE_BYTE1,
		(max98390->ambient_temp_value >> 8) & 0x000000ff);
	regmap_write(max98390->regmap, DSM_TPROT_ROOM_TEMPERATURE_BYTE0,
		(max98390->ambient_temp_value) & 0x000000ff);

	return 0;
}

static int max98390_ambient_temp_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = max98390->ambient_temp_value;

	return 0;
}

static int max98390_adaptive_rdc_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);

	dev_warn(component->dev, "Put adaptive rdc not supported\n");

	return 0;
}

static int max98390_adaptive_rdc_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int rdc, rdc0;
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	regmap_read(max98390->regmap, THERMAL_RDC_RD_BACK_BYTE1, &rdc);
	regmap_read(max98390->regmap, THERMAL_RDC_RD_BACK_BYTE0, &rdc0);
	ucontrol->value.integer.value[0] = rdc0 | rdc << 8;

	return 0;
}

static int max98390_dsm_calib_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	/* Do nothing */
	return 0;
}

static int max98390_dsm_calib_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct max98390_priv *max98390 = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(component);
	unsigned int rdc, rdc_cal_result, rdc_integer, rdc_factor, temp, val;

	snd_soc_dapm_mutex_lock(dapm);

	regmap_read(max98390->regmap, MAX98390_R23FF_GLOBAL_EN, &val);
	if (!val) {
		/* Enable the codec for the duration of calibration readout */
		regmap_update_bits(max98390->regmap, MAX98390_R203A_AMP_EN,
				   MAX98390_AMP_EN_MASK, 1);
		regmap_write(max98390->regmap, MAX98390_R23FF_GLOBAL_EN, 0x01);
	}

	regmap_read(max98390->regmap, THERMAL_RDC_RD_BACK_BYTE1, &rdc);
	regmap_read(max98390->regmap, THERMAL_RDC_RD_BACK_BYTE0, &rdc_cal_result);
	regmap_read(max98390->regmap, MAX98390_MEAS_ADC_CH2_READ, &temp);

	if (!val) {
		/* Disable the codec if it was disabled */
		regmap_write(max98390->regmap, MAX98390_R23FF_GLOBAL_EN, 0x00);
		regmap_update_bits(max98390->regmap, MAX98390_R203A_AMP_EN,
				   MAX98390_AMP_EN_MASK, 0);
	}

	snd_soc_dapm_mutex_unlock(dapm);

	rdc_cal_result |= (rdc << 8) & 0x0000FFFF;
	if (rdc_cal_result)
		max98390->ref_rdc_value = 268435456U / rdc_cal_result;

	max98390->ambient_temp_value = temp * 52 - 1188;

	rdc_integer =  rdc_cal_result * 937  / 65536;
	rdc_factor = ((rdc_cal_result * 937 * 100) / 65536) - (rdc_integer * 100);

	dev_info(component->dev,
		 "rdc resistance about %d.%02d ohm, reg=0x%X temp reg=0x%X\n",
		 rdc_integer, rdc_factor, rdc_cal_result, temp);

	return 0;
}

static const struct snd_kcontrol_new max98390_snd_controls[] = {
	SOC_SINGLE_TLV("Speaker Volume", MAX98390_R203D_SPK_GAIN,
		0, 6, 0,
		max98390_spk_tlv),
	SOC_SINGLE("Ramp Up Bypass Switch", MAX98390_R2039_AMP_DSP_CFG,
		MAX98390_AMP_DSP_CFG_RMP_UP_SHIFT, 1, 0),
	SOC_SINGLE("Ramp Down Bypass Switch", MAX98390_R2039_AMP_DSP_CFG,
		MAX98390_AMP_DSP_CFG_RMP_DN_SHIFT, 1, 0),
	SOC_SINGLE("Boost Clock Phase", MAX98390_BOOST_CTRL3,
		MAX98390_BOOST_CLK_PHASE_CFG_SHIFT, 3, 0),
	SOC_ENUM("Boost Output Voltage", max98390_boost_voltage),
	SOC_ENUM("Current Limit", max98390_current_limit),
	SOC_SINGLE_EXT("DSM Rdc", SND_SOC_NOPM, 0, 0xffffff, 0,
		max98390_ref_rdc_get, max98390_ref_rdc_put),
	SOC_SINGLE_EXT("DSM Ambient Temp", SND_SOC_NOPM, 0, 0xffff, 0,
		max98390_ambient_temp_get, max98390_ambient_temp_put),
	SOC_SINGLE_EXT("DSM Adaptive Rdc", SND_SOC_NOPM, 0, 0xffff, 0,
		max98390_adaptive_rdc_get, max98390_adaptive_rdc_put),
	SOC_SINGLE_EXT("DSM Calibration", SND_SOC_NOPM, 0, 1, 0,
		max98390_dsm_calib_get, max98390_dsm_calib_put),
};

static const struct soc_enum dai_sel_enum =
	SOC_ENUM_SINGLE(MAX98390_PCM_CH_SRC_1,
		MAX98390_PCM_RX_CH_SRC_SHIFT,
		3, max98390_switch_text);

static const struct snd_kcontrol_new max98390_dai_controls =
	SOC_DAPM_ENUM("DAI Sel", dai_sel_enum);

static const struct snd_soc_dapm_widget max98390_dapm_widgets[] = {
	SND_SOC_DAPM_DAC_E("Amp Enable", "HiFi Playback",
		SND_SOC_NOPM, 0, 0, max98390_dac_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("DAI Sel Mux", SND_SOC_NOPM, 0, 0,
		&max98390_dai_controls),
	SND_SOC_DAPM_OUTPUT("BE_OUT"),
};

static const struct snd_soc_dapm_route max98390_audio_map[] = {
	/* Plabyack */
	{"DAI Sel Mux", "Left", "Amp Enable"},
	{"DAI Sel Mux", "Right", "Amp Enable"},
	{"DAI Sel Mux", "LeftRight", "Amp Enable"},
	{"BE_OUT", NULL, "DAI Sel Mux"},
};

static bool max98390_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX98390_SOFTWARE_RESET ... MAX98390_INT_EN3:
	case MAX98390_IRQ_CTRL ... MAX98390_WDOG_CTRL:
	case MAX98390_MEAS_ADC_THERM_WARN_THRESH
		... MAX98390_BROWNOUT_INFINITE_HOLD:
	case MAX98390_BROWNOUT_LVL_HOLD ... DSMIG_DEBUZZER_THRESHOLD:
	case DSM_VOL_ENA ... MAX98390_R24FF_REV_ID:
		return true;
	default:
		return false;
	}
};

static bool max98390_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX98390_SOFTWARE_RESET ... MAX98390_INT_EN3:
	case MAX98390_R23FF_GLOBAL_EN:
	case MAX98390_MEAS_ADC_CH0_READ ... MAX98390_MEAS_ADC_CH2_READ:
	case MAX98390_PWR_GATE_STATUS ... MAX98390_BROWNOUT_STATUS:
	case MAX98390_BROWNOUT_LOWEST_STATUS:
	case MAX98390_ENV_TRACK_BOOST_VOUT_READ:
	case DSM_STBASS_HPF_B0_BYTE0 ... DSM_DEBUZZER_ATTACK_TIME_BYTE2:
	case THERMAL_RDC_RD_BACK_BYTE1 ... DSMIG_DEBUZZER_THRESHOLD:
	case DSM_THERMAL_GAIN ... DSM_WBDRC_GAIN:
		return true;
	default:
		return false;
	}
}

#define MAX98390_RATES SNDRV_PCM_RATE_8000_48000

#define MAX98390_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver max98390_dai[] = {
	{
		.name = "max98390-aif1",
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MAX98390_RATES,
			.formats = MAX98390_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MAX98390_RATES,
			.formats = MAX98390_FORMATS,
		},
		.ops = &max98390_dai_ops,
	}
};

static int max98390_dsm_init(struct snd_soc_component *component)
{
	int ret;
	int param_size, param_start_addr;
	const char *vendor, *product;
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);
	const struct firmware *fw;
	char *dsm_param;

	vendor = dmi_get_system_info(DMI_SYS_VENDOR);
	product = dmi_get_system_info(DMI_PRODUCT_NAME);

	// First check if filename has been set yet
	if(strncmp(max98390->filename, "dsm_param", strlen("dsm_param"))){
		printk("setting up default file\n");
		if (!strcmp(max98390->dsm_param_name, "default")) {
			if (vendor && product) {
				snprintf(max98390->filename, sizeof(max98390->filename),
					"dsm_param_%s_%s.bin", vendor, product);
			} else {
				sprintf(max98390->filename, "dsm_param.bin");
			}
		} else {
			snprintf(max98390->filename, sizeof(max98390->filename), "%s",
				max98390->dsm_param_name);
		}
	}

	ret = request_firmware(&fw, max98390->filename, component->dev);
	if (ret) {
		dev_err(component->dev, "Requested Firmware %s Not Found! Trying default\n", max98390->filename);
		ret = request_firmware(&fw, "dsm_param.bin", component->dev);
		if (ret) {
			ret = request_firmware(&fw, "dsmparam.bin",
				component->dev);
			if (ret) {
				dev_err(component->dev, "Firmware Not Found!\n");
				goto err;
			}
		}
	}

	dev_info(component->dev, "Found Firmware %s", max98390->filename);

	dev_dbg(component->dev,
		"max98390: param fw size %zd\n",
		fw->size);
	if (fw->size < MAX98390_DSM_PARAM_MIN_SIZE) {
		dev_err(component->dev,
			"param fw is invalid.\n");
		ret = -EINVAL;
		goto err_alloc;
	}
	dsm_param = (char *)fw->data;
	param_start_addr = (dsm_param[0] & 0xff) | (dsm_param[1] & 0xff) << 8;
	param_size = (dsm_param[2] & 0xff) | (dsm_param[3] & 0xff) << 8;
	if (param_size > MAX98390_DSM_PARAM_MAX_SIZE ||
		param_start_addr < MAX98390_IRQ_CTRL ||
		fw->size < param_size + MAX98390_DSM_PAYLOAD_OFFSET) {
		dev_err(component->dev,
			"param fw is invalid.\n");
		ret = -EINVAL;
		goto err_alloc;
	}
	regmap_write(max98390->regmap, MAX98390_R203A_AMP_EN, 0x80);
	dsm_param += MAX98390_DSM_PAYLOAD_OFFSET;
	regmap_bulk_write(max98390->regmap, param_start_addr,
		dsm_param, param_size);
	regmap_write(max98390->regmap, MAX98390_R23E1_DSP_GLOBAL_EN, 0x01);

err_alloc:
	release_firmware(fw);
err:
	return ret;
}

static void max98390_init_regs(struct snd_soc_component *component)
{
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	regmap_write(max98390->regmap, MAX98390_CLK_MON, 0x6f);
	regmap_write(max98390->regmap, MAX98390_DAT_MON, 0x00);
	regmap_write(max98390->regmap, MAX98390_PWR_GATE_CTL, 0x00);
	regmap_write(max98390->regmap, MAX98390_PCM_RX_EN_A, 0x03);
	regmap_write(max98390->regmap, MAX98390_ENV_TRACK_VOUT_HEADROOM, 0x0e);
	regmap_write(max98390->regmap, MAX98390_BOOST_BYPASS1, 0x46);
	regmap_write(max98390->regmap, MAX98390_FET_SCALING3, 0x03);

	/* voltage, current slot configuration */
	regmap_write(max98390->regmap,
		MAX98390_PCM_CH_SRC_2,
		(max98390->i_l_slot << 4 |
		max98390->v_l_slot)&0xFF);

	if (max98390->v_l_slot < 8) {
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_HIZ_CTRL_A,
			1 << max98390->v_l_slot, 0);
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_EN_A,
			1 << max98390->v_l_slot,
			1 << max98390->v_l_slot);
	} else {
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_HIZ_CTRL_B,
			1 << (max98390->v_l_slot - 8), 0);
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_EN_B,
			1 << (max98390->v_l_slot - 8),
			1 << (max98390->v_l_slot - 8));
	}

	if (max98390->i_l_slot < 8) {
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_HIZ_CTRL_A,
			1 << max98390->i_l_slot, 0);
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_EN_A,
			1 << max98390->i_l_slot,
			1 << max98390->i_l_slot);
	} else {
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_HIZ_CTRL_B,
			1 << (max98390->i_l_slot - 8), 0);
		regmap_update_bits(max98390->regmap,
			MAX98390_PCM_TX_EN_B,
			1 << (max98390->i_l_slot - 8),
			1 << (max98390->i_l_slot - 8));
	}
}

/* Applies maxim,dsm_volume_max on top of whatever the DSM params wrote */
static void max98390_dsm_vol_clip(struct max98390_priv *max98390)
{
	unsigned int val;

	if (!max98390->dsm_vol_max)
		return;

	regmap_read(max98390->regmap, DSM_VOL_CTRL, &val);
	if (val > max98390->dsm_vol_max)
		regmap_write(max98390->regmap, DSM_VOL_CTRL, max98390->dsm_vol_max);
}

static int max98390_add_dsm_vol_control(struct snd_soc_component *component)
{
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);
	struct snd_kcontrol_new kctl = SOC_SINGLE_TLV("Digital Volume",
		DSM_VOL_CTRL, 0, MAX98390_DSM_VOL_MAX, 0, max98390_digital_tlv);

	/* Create a per-instance copy of the soc_mixer_control struct ... */
	max98390->dsm_vol_mc = *(struct soc_mixer_control *)kctl.private_value;
	/* ... so that we can override the platform_max per instance */
	max98390->dsm_vol_mc.platform_max = max98390->dsm_vol_max;
	/* And the kctl uses our per-instance structure */
	kctl.private_value = (unsigned long)&max98390->dsm_vol_mc;

	return snd_soc_add_component_controls(component, &kctl, 1);
}

static int max98390_probe(struct snd_soc_component *component)
{
	struct max98390_priv *max98390 =
		snd_soc_component_get_drvdata(component);

	regmap_write(max98390->regmap, MAX98390_SOFTWARE_RESET, 0x01);
	/* Sleep reset settle time */
	msleep(20);

	/* Amp init setting */
	max98390_init_regs(component);
	/* Update dsm bin param */
	max98390_dsm_init(component);
	max98390_dsm_vol_clip(max98390);

	/* Dsm Setting */
	if (max98390->ref_rdc_value) {
		regmap_write(max98390->regmap, DSM_TPROT_RECIP_RDC_ROOM_BYTE0,
			max98390->ref_rdc_value & 0x000000ff);
		regmap_write(max98390->regmap, DSM_TPROT_RECIP_RDC_ROOM_BYTE1,
			(max98390->ref_rdc_value >> 8) & 0x000000ff);
		regmap_write(max98390->regmap, DSM_TPROT_RECIP_RDC_ROOM_BYTE2,
			(max98390->ref_rdc_value >> 16) & 0x000000ff);
	}
	if (max98390->ambient_temp_value) {
		regmap_write(max98390->regmap, DSM_TPROT_ROOM_TEMPERATURE_BYTE1,
			(max98390->ambient_temp_value >> 8) & 0x000000ff);
		regmap_write(max98390->regmap, DSM_TPROT_ROOM_TEMPERATURE_BYTE0,
			(max98390->ambient_temp_value) & 0x000000ff);
	}

    /* save a pointer to the component in priv for use with sysfs entries */
    max98390->component = component;

	max98390_irq_setup(component);

	return max98390_add_dsm_vol_control(component);
}

#if 0 
static int max98390_suspend(struct device *dev)
{
	struct max98390_priv *max98390 = dev_get_drvdata(dev);

	dev_dbg(dev, "%s:Enter\n", __func__);

	regcache_cache_only(max98390->regmap, true);
	regcache_mark_dirty(max98390->regmap);

	return 0;
}

static int max98390_resume(struct device *dev)
{
	struct max98390_priv *max98390 = dev_get_drvdata(dev);

	dev_dbg(dev, "%s:Enter\n", __func__);

	regcache_cache_only(max98390->regmap, false);
	regcache_sync(max98390->regmap);

	return 0;
}

static const struct dev_pm_ops max98390_pm = {
	SYSTEM_SLEEP_PM_OPS(max98390_suspend, max98390_resume)
};
#endif

static const struct snd_soc_component_driver soc_codec_dev_max98390 = {
	.probe			= max98390_probe,
	.controls		= max98390_snd_controls,
	.num_controls		= ARRAY_SIZE(max98390_snd_controls),
	.dapm_widgets		= max98390_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(max98390_dapm_widgets),
	.dapm_routes		= max98390_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(max98390_audio_map),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct regmap_config max98390_regmap = {
	.reg_bits         = 16,
	.val_bits         = 8,
	.max_register     = MAX98390_R24FF_REV_ID,
	.reg_defaults     = max98390_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(max98390_reg_defaults),
	.readable_reg	  = max98390_readable_register,
	.volatile_reg	  = max98390_volatile_reg,
	.cache_type       = REGCACHE_RBTREE,
};

static void max98390_slot_config(struct i2c_client *i2c,
	struct max98390_priv *max98390)
{
	int value;
	struct device *dev = &i2c->dev;

	if (!device_property_read_u32(dev, "maxim,vmon-slot-no", &value))
		max98390->v_l_slot = value & 0xF;
	else
		max98390->v_l_slot = 0;

	if (!device_property_read_u32(dev, "maxim,imon-slot-no", &value))
		max98390->i_l_slot = value & 0xF;
	else
		max98390->i_l_slot = 1;
}

/* sysfs helper for loading different firmware files */
static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max98390_priv *max98390 = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%s", max98390->filename);
}

static ssize_t firmware_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct max98390_priv *max98390 = dev_get_drvdata(dev);

	// Max size 128 taken from init function
	snprintf(max98390->filename, 128, "%.*s",
				(int)min(count, 128 - 1), buf);

	dev_info(dev, "MAX98390 firmware set to %s", max98390->filename);
	return count;
}

static const char * const load_modes[] = {
	"load", "unload"
};

static ssize_t load_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret, mode;
	struct max98390_priv *max98390 = dev_get_drvdata(dev);
	struct snd_soc_component *component = max98390->component;

	mode = sysfs_match_string(load_modes, buf);
	if (mode < 0) {
		return mode;
	}

	ret = max98390_dsm_init(component);
	if ( ret ) {
		return ret;
	}
	max98390_dsm_vol_clip(max98390);

	return count;
}

static DEVICE_ATTR_RW(firmware);
static DEVICE_ATTR_WO(load);

static struct attribute *max98390_attrs[] = {
	&dev_attr_firmware.attr,
	&dev_attr_load.attr,
	NULL,
};

static const struct attribute_group max98390_attr_group = {
	.name = "max98390",
	.attrs = max98390_attrs,
};

/*
 * The interrupt line may be described either as an interrupt, in which case the
 * i2c core has already resolved it into client->irq, or as a plain GPIO. A GPIO
 * specifier carries no interrupt trigger - GPIO_ACTIVE_LOW is a signal
 * polarity, not an interrupt type - so one has to be supplied for that path.
 *
 * Returns the interrupt number, 0 if the board describes none, or a negative
 * error. -EPROBE_DEFER means the GPIO controller has not registered yet and the
 * caller must retry.
 */
static int max98390_get_irq(struct i2c_client *i2c, unsigned long *irq_flags)
{
	struct gpio_desc *gpio;
	int irq;

	if (i2c->irq)
		return i2c->irq;

	gpio = devm_gpiod_get_optional(&i2c->dev, "irq", GPIOD_IN);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);
	if (!gpio)
		return 0;

	irq = gpiod_to_irq(gpio);
	if (irq < 0)
		return irq;

	*irq_flags |= IRQF_TRIGGER_LOW;

	return irq;
}

static int max98390_i2c_probe(struct i2c_client *i2c)
{
	int ret = 0;
	int reg = 0;
	int irq;
	unsigned long irq_flags = IRQF_ONESHOT | IRQF_NO_AUTOEN;

	struct max98390_priv *max98390 = NULL;
	struct i2c_adapter *adapter = i2c->adapter;
	struct gpio_desc *reset_gpio;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE |
					      I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&i2c->dev, "I2C check functionality failed\n");
		return -ENXIO;
	}

	max98390 = devm_kzalloc(&i2c->dev, sizeof(*max98390), GFP_KERNEL);
	if (!max98390) {
		ret = -ENOMEM;
		return ret;
	}
	i2c_set_clientdata(i2c, max98390);

	ret = device_property_read_u32(&i2c->dev, "maxim,temperature_calib",
				       &max98390->ambient_temp_value);
	if (ret) {
		dev_info(&i2c->dev,
			 "no optional property 'temperature_calib' found, default:\n");
	}
	ret = device_property_read_u32(&i2c->dev, "maxim,r0_calib",
				       &max98390->ref_rdc_value);
	if (ret) {
		dev_info(&i2c->dev,
			 "no optional property 'r0_calib' found, default:\n");
	}

	dev_info(&i2c->dev,
		"%s: r0_calib: 0x%x,temperature_calib: 0x%x",
		__func__, max98390->ref_rdc_value,
		max98390->ambient_temp_value);

	ret = device_property_read_string(&i2c->dev, "maxim,dsm_param_name",
				       &max98390->dsm_param_name);
	if (ret)
		max98390->dsm_param_name = "default";

	if (!device_property_read_u32(&i2c->dev, "maxim,dsm_volume_max",
				      &max98390->dsm_vol_max) &&
	    (!max98390->dsm_vol_max ||
	     max98390->dsm_vol_max > MAX98390_DSM_VOL_MAX)) {
		dev_warn(&i2c->dev, "ignoring invalid dsm_volume_max %u\n",
			 max98390->dsm_vol_max);
		max98390->dsm_vol_max = 0;
	}

	max98390->mute = 1;

	/* voltage/current slot configuration */
	max98390_slot_config(i2c, max98390);

	/* regmap initialization */
	max98390->regmap = devm_regmap_init_i2c(i2c, &max98390_regmap);
	if (IS_ERR(max98390->regmap)) {
		ret = PTR_ERR(max98390->regmap);
		dev_err(&i2c->dev,
			"Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	reset_gpio = devm_gpiod_get_optional(&i2c->dev,
					     "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(&i2c->dev, PTR_ERR(reset_gpio),
				     "Failed to get reset gpio\n");

	/* Power on device */
	if (reset_gpio) {
		usleep_range(1000, 2000);
		/* bring out of reset */
		gpiod_set_value_cansleep(reset_gpio, 0);
		usleep_range(1000, 2000);
	}

	/* Check Revision ID */
	ret = regmap_read(max98390->regmap,
		MAX98390_R24FF_REV_ID, &reg);
	if (ret) {
		dev_err(&i2c->dev,
			"ret=%d, Failed to read: 0x%02X\n",
			ret, MAX98390_R24FF_REV_ID);
		return ret;
	}
	dev_info(&i2c->dev, "MAX98390 revisionID: 0x%02X\n", reg);

	max98390->dev = &i2c->dev;

	irq = max98390_get_irq(i2c, &irq_flags);
	if (irq == -EPROBE_DEFER)
		return dev_err_probe(&i2c->dev, irq,
				     "interrupt not available yet\n");
	if (irq < 0) {
		/*
		 * The line is described but unusable, which is a board or
		 * binding error. Report it and carry on unmonitored rather
		 * than refusing to bring up the amplifier.
		 */
		dev_warn(&i2c->dev, "ignoring unusable interrupt: %d\n", irq);
	} else if (irq > 0) {
		max98390->irq = irq;
		/*
		 * Requested disabled and enabled by max98390_irq_setup(). That
		 * cannot run before the component probe, because the software
		 * reset there returns IRQ_CTRL and the INT_EN masks to their
		 * defaults. Until it has run the line may already be asserted.
		 */
		ret = devm_request_threaded_irq(&i2c->dev, irq, NULL,
						max98390_irq, irq_flags,
						"max98390", max98390);
		if (ret) {
			/* Not fatal, the amplifier just goes unmonitored. */
			dev_err(&i2c->dev, "failed to request irq %d: %d\n",
				irq, ret);
			max98390->irq = 0;
		}
	}

	ret = devm_snd_soc_register_component(&i2c->dev,
			&soc_codec_dev_max98390,
			max98390_dai, ARRAY_SIZE(max98390_dai));

    /* Setup sysfs files */
    dev_set_drvdata(&i2c->dev, max98390);
    ret = sysfs_create_group(&i2c->dev.kobj, &max98390_attr_group);
    if (ret < 0) {
        dev_err(&i2c->dev, "failed to create MAX98390 sysfs files\n");
        return ret;
    }

	return ret;
}

static const struct i2c_device_id max98390_i2c_id[] = {
	{ .name = "max98390" },
	{ }
};

MODULE_DEVICE_TABLE(i2c, max98390_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id max98390_of_match[] = {
	{ .compatible = "maxim,max98390", },
	{}
};
MODULE_DEVICE_TABLE(of, max98390_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id max98390_acpi_match[] = {
	{ "MX98390", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, max98390_acpi_match);
#endif

static struct i2c_driver max98390_i2c_driver = {
	.driver = {
		.name = "max98390",
		.of_match_table = of_match_ptr(max98390_of_match),
		.acpi_match_table = ACPI_PTR(max98390_acpi_match),
		//.pm = pm_ptr(&max98390_pm),
	},
	.probe = max98390_i2c_probe,
	.id_table = max98390_i2c_id,
};

module_i2c_driver(max98390_i2c_driver)

MODULE_DESCRIPTION("ALSA SoC MAX98390 driver");
MODULE_AUTHOR("Steve Lee <steves.lee@maximintegrated.com>");
MODULE_LICENSE("GPL");
