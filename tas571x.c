/*
 * TAS571x amplifier audio driver
 *
 * Copyright (C) 2015 Google, Inc.
 * Copyright (c) 2013 Daniel Mack <zonque@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/stddef.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <linux/gpio.h>

#include "tas571x.h"

#define TAS571X_MAX_SUPPLIES	6

#define PMX_DEBUG
#ifdef PMX_DEBUG
#define pmx_dbg(fmt, ...) \
		printk(KERN_CRIT "[PMX][%s|%d]: " fmt, __func__, __LINE__, ##__VA_ARGS__)
#else
#define pmx_dbg(fmt, ...) do {} while(0)
#endif

#define GPIO_HIGH				1
#define GPIO_LOW				0

#define FPS_SDN_GPIO			55
#define PDN_GPIO				57
#define RESET_GPIO				58

#define TAS5716_BUS_ADDR		4
#define TAS5716_SLAVE_ADDR		0x1b	// 0x36 >> 1

#define NO_OF_BYTES_1			1
#define NO_OF_BYTES_4 			NO_OF_BYTES_1 * 4
#define NO_OF_BYTES_8			NO_OF_BYTES_1 * 8
#define NO_OF_BYTES_20			NO_OF_BYTES_1 * 20

struct i2c_client *amp_i2c_client;

struct _tas5716_init_data {
	unsigned int reg;
	unsigned int val;
};

struct tas571x_chip {
	const char			*const *supply_names;
	int				num_supply_names;
	const struct snd_kcontrol_new	*controls;
	int				num_controls;
	const struct regmap_config	*regmap_config;
	int				vol_reg_size;
};

struct tas571x_private {
	const struct tas571x_chip	*chip;
	struct regmap				*regmap;
	struct regulator_bulk_data	supplies[TAS571X_MAX_SUPPLIES];
	struct clk					*mclk;
	unsigned int				format;
	struct gpio_desc			*reset_gpio;
	struct gpio_desc			*pdn_gpio;
	struct snd_soc_codec_driver	codec_driver;
};

static int tas571x_register_size(unsigned int reg)
{
	switch (reg) {
	case TAS5716_CLK_CTRL_REG:
	case TAS5716_DEVICE_ID_REG:
	case TAS5716_ERR_STATUS_REG:
	case TAS5716_SYS_CTRL1_REG:
	case TAS5716_SDI_REG:
	case TAS5716_SYS_CTRL2_REG:
	case TAS5716_SOFT_MUTE_REG:
	case TAS5716_MASTER_VOL_REG:
	case TAS5716_CH1_VOL_REG:
	case TAS5716_CH2_VOL_REG:
	case TAS5716_CH3_VOL_REG:
	case TAS5716_CH4_VOL_REG:
	case TAS5716_HP_VOL_REG:
	case TAS5716_CH6_VOL_REG:
	case TAS5716_VOL_CFG_REG:
	case TAS5716_MOD_LIMIT_REG:
	case TAS5716_IC_DELAY_CH1_REG:
	case TAS5716_IC_DELAY_CH2_REG:
	case TAS5716_IC_DELAY_CH3_REG:
	case TAS5716_IC_DELAY_CH4_REG:
	case TAS5716_IC_DELAY_CH5_REG:
	case TAS5716_IC_DELAY_CH6_REG:
	case TAS5716_OFFSET_REG:
	case TAS5716_PWM_SHUTDOWN_REG:
	case TAS5716_START_STOP_PERIOD_REG:
	case TAS5716_OSC_TRIM_REG:
	case TAS5716_BKND_ERR_REG:
		return NO_OF_BYTES_1;	// byte
	case TAS5716_INPUT_MUX_REG:
	case TAS5716_CH6_INPUT_MUX2_REG:
	case TAS5716_AM_TUNED_FREQ_REG:
	case TAS5716_PWM_MUX_REG:
	case TAS5716_1G_REG:
	case TAS5716_SCALE_REG:
	case TAS5716_DRC1_T_REG:
	case TAS5716_DRC1_K_REG:
	case TAS5716_DRC1_O_REG:
	case TAS5716_DRC2_T_REG:
	case TAS5716_DRC2_K_REG:
	case TAS5716_DRC2_O_REG:
	case TAS5716_DRC_CTRL_REG:
		return NO_OF_BYTES_4;	// byte
	case TAS5716_DRC1_AE_REG:
	case TAS5716_DRC1_AA_REG:
	case TAS5716_DRC1_AD_REG:
	case TAS5716_DRC2_AE_REG:
	case TAS5716_DRC2_AA_REG:
	case TAS5716_DRC2_AD_REG:
		return NO_OF_BYTES_8;	// byte
	case TAS5716_CH6BQ_LOUD_REG:
	case TAS5716_CH6BQ_POST_VOL_REG:
	case TAS5716_CH1_BQ0_REG:
	case TAS5716_CH1_BQ1_REG:
	case TAS5716_CH1_BQ2_REG:
	case TAS5716_CH1_BQ3_REG:
	case TAS5716_CH1_BQ4_REG:
	case TAS5716_CH1_BQ5_REG:
	case TAS5716_CH1_BQ6_REG:
	case TAS5716_CH2_BQ0_REG:
	case TAS5716_CH2_BQ1_REG:
	case TAS5716_CH2_BQ2_REG:
	case TAS5716_CH2_BQ3_REG:
	case TAS5716_CH2_BQ4_REG:
	case TAS5716_CH2_BQ5_REG:
	case TAS5716_CH2_BQ6_REG:
	case TAS5716_CH6_BQ0_REG:
	case TAS5716_CH6_BQ1_REG:
		return NO_OF_BYTES_20;	// byte
	default:
		return NO_OF_BYTES_1;
	}
}

static int tas571x_reg_write(void *context, unsigned int reg,
			     unsigned int value)
{
	struct i2c_client *client = (struct i2c_client *) context;
	struct tas571x_private *priv = i2c_get_clientdata(client);
	unsigned int i, size;
	uint8_t buf[5];
	int ret;

	// pmx_dbg ("reg: 0x%02x, value: 0x%08x\n", reg, value);

	size = tas571x_register_size(reg);
	buf[0] = reg;

	for (i = size; i >= 1; --i) {
		buf[i] = value;
		value >>= 8;
	}

	ret = i2c_master_send(client, buf, size + 1);
	if (ret == size + 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tas571x_reg_read(void *context, unsigned int reg,
			    unsigned int *value)
{
	struct i2c_client *client = (struct i2c_client *) context;
	struct tas571x_private *priv = i2c_get_clientdata(client);
	uint8_t send_buf, recv_buf[4];
	struct i2c_msg msgs[2];
	unsigned int size;
	unsigned int i;
	int ret;

	size = tas571x_register_size(reg);
	send_buf = reg;

	msgs[0].addr = client->addr;
	msgs[0].len = sizeof(send_buf);
	msgs[0].buf = &send_buf;
	msgs[0].flags = 0;

	msgs[1].addr = client->addr;
	msgs[1].len = size;
	msgs[1].buf = recv_buf;
	msgs[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*value = 0;

	for (i = 0; i < size; i++) {
		*value <<= 8;
		*value |= recv_buf[i];
	}
	
	return 0;
}

static int tas571x_set_dai_fmt(struct snd_soc_dai *dai, unsigned int format)
{
	struct tas571x_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	pmx_dbg ("BEGIN\n");
	priv->format = format;
	return 0;
}

static int tas571x_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct tas571x_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	u32 val;

	pmx_dbg ("BEGIN\n");
	
	switch (priv->format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		val = 0x00;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = 0x03;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = 0x06;
		break;
	default:
		return -EINVAL;
	}

	if (snd_pcm_format_width(params_format(params)) >= 24)
		val += 2;
	else if (snd_pcm_format_width(params_format(params)) >= 20)
		val += 1;

	return regmap_update_bits(priv->regmap, TAS571X_SDI_REG,
				  TAS571X_SDI_FMT_MASK, val);
}

static int tas571x_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct tas571x_private *priv = snd_soc_codec_get_drvdata(codec);
	int ret;

	pmx_dbg ("BEGIN\n");

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			gpio_set_value(PDN_GPIO, 0);
			usleep_range(5000, 6000);

			regcache_cache_only(priv->regmap, false);
			ret = regcache_sync(priv->regmap);
			if (ret)
				return ret;
		}
		break;
	case SND_SOC_BIAS_OFF:
		regcache_cache_only(priv->regmap, true);
		gpio_set_value(PDN_GPIO, 1);

//		if (!IS_ERR(priv->mclk))
//			clk_disable_unprepare(priv->mclk);
		break;
	}

	return 0;
}

static const struct snd_soc_dai_ops tas571x_dai_ops = {
	.set_fmt	= tas571x_set_dai_fmt,
	.hw_params	= tas571x_hw_params,
};

static const char *const tas5711_supply_names[] = {
	"AVDD",
	"DVDD",
	"PVDD_A",
	"PVDD_B",
	"PVDD_C",
	"PVDD_D",
};

static const DECLARE_TLV_DB_SCALE(tas5711_volume_tlv, -10350, 50, 1);

static const struct snd_kcontrol_new tas5711_controls[] = {
	SOC_SINGLE_TLV("Master Volume",
		       TAS571X_MVOL_REG,
		       0, 0xff, 1, tas5711_volume_tlv),
	SOC_DOUBLE_R_TLV("Speaker Volume",
			 TAS571X_CH1_VOL_REG,
			 TAS571X_CH2_VOL_REG,
			 0, 0xff, 1, tas5711_volume_tlv),
	SOC_DOUBLE("Speaker Switch",
		   TAS571X_SOFT_MUTE_REG,
		   TAS571X_SOFT_MUTE_CH1_SHIFT, TAS571X_SOFT_MUTE_CH2_SHIFT,
		   1, 1),
};

static const struct reg_default tas5711_reg_defaults[] = {
	{ 0x04, 0x05 },
	{ 0x05, 0x40 },
	{ 0x06, 0x00 },
	{ 0x07, 0xff },
	{ 0x08, 0x30 },
	{ 0x09, 0x30 },
	{ 0x1b, 0x82 },
};

static const struct regmap_config tas5711_regmap_config = {
	.reg_bits				= 8,
	.val_bits				= 32,
	.max_register			= 0xff,
	.reg_read				= tas571x_reg_read,
	.reg_write				= tas571x_reg_write,
	.reg_defaults			= tas5711_reg_defaults,
	.num_reg_defaults		= ARRAY_SIZE(tas5711_reg_defaults),
	.cache_type				= REGCACHE_RBTREE,
};

static const struct tas571x_chip tas5711_chip = {
	.supply_names			= tas5711_supply_names,
	.num_supply_names		= ARRAY_SIZE(tas5711_supply_names),
	.controls				= tas5711_controls,
	.num_controls			= ARRAY_SIZE(tas5711_controls),
	.regmap_config			= &tas5711_regmap_config,
	.vol_reg_size			= 1,
};

static const char *const tas5717_supply_names[] = {
	"AVDD",
	"DVDD",
	"HPVDD",
	"PVDD_AB",
	"PVDD_CD",
};

static const DECLARE_TLV_DB_SCALE(tas5717_volume_tlv, -10375, 25, 0);

static const struct snd_kcontrol_new tas5717_controls[] = {
	/* MVOL LSB is ignored - see comments in tas571x_i2c_probe() */
	SOC_SINGLE_TLV("Master Volume",
		       TAS571X_MVOL_REG, 1, 0x1ff, 1,
		       tas5717_volume_tlv),
	SOC_DOUBLE_R_TLV("Speaker Volume",
			 TAS571X_CH1_VOL_REG, TAS571X_CH2_VOL_REG,
			 1, 0x1ff, 1, tas5717_volume_tlv),
	SOC_DOUBLE("Speaker Switch",
		   TAS571X_SOFT_MUTE_REG,
		   TAS571X_SOFT_MUTE_CH1_SHIFT, TAS571X_SOFT_MUTE_CH2_SHIFT,
		   1, 1),
};

static const struct reg_default tas5717_reg_defaults[] = {
	{ 0x04, 0x05 },
	{ 0x05, 0x40 },
	{ 0x06, 0x00 },
	// { 0x07, 0x03ff },
	{ 0x07, 0xff },
	{ 0x08, 0x30 },
	{ 0x09, 0x30 },
	{ 0x1b, 0x82 },
};

#if 1 // for tas5716
static const char *const tas5716_supply_names[] = {
	"AVDD",
	"DVDD",
	"PVCC",
};

static const struct reg_default tas5716_reg_defaults[] = {
	{ TAS5716_CLK_CTRL_REG			, 0x6C },
	{ TAS5716_DEVICE_ID_REG			, 0x28 },
	{ TAS5716_ERR_STATUS_REG		, 0x00 },
	{ TAS5716_SYS_CTRL1_REG			, 0xA0 },
	{ TAS5716_SDI_REG				, 0x05 },
	{ TAS5716_SYS_CTRL2_REG			, 0x40 },
	{ TAS5716_SOFT_MUTE_REG			, 0x00 },
	{ TAS5716_MASTER_VOL_REG		, 0xff },
	{ TAS5716_CH1_VOL_REG			, 0x30 },
	{ TAS5716_CH2_VOL_REG			, 0x30 },
	{ TAS5716_CH3_VOL_REG			, 0x30 },
	{ TAS5716_CH4_VOL_REG			, 0x30 },
	{ TAS5716_HP_VOL_REG			, 0x30 },
	{ TAS5716_CH6_VOL_REG			, 0x30 },
	{ TAS5716_VOL_CFG_REG			, 0x91 },
	{ TAS5716_MOD_LIMIT_REG			, 0x02 },
	{ TAS5716_IC_DELAY_CH1_REG		, 0x4C },
	{ TAS5716_IC_DELAY_CH2_REG		, 0x34 },
	{ TAS5716_IC_DELAY_CH3_REG		, 0x1C },
	{ TAS5716_IC_DELAY_CH4_REG		, 0x64 },
	{ TAS5716_IC_DELAY_CH5_REG		, 0xB0 },
	{ TAS5716_IC_DELAY_CH6_REG		, 0x90 },
	{ TAS5716_OFFSET_REG			, 0x00 },
	{ TAS5716_PWM_SHUTDOWN_REG		, 0x30 },
	{ TAS5716_START_STOP_PERIOD_REG	, 0x0A },
	{ TAS5716_OSC_TRIM_REG			, 0x82 },
	{ TAS5716_BKND_ERR_REG			, 0x02 },
};

static const struct regmap_config tas5716_regmap_config = {
	.reg_bits			= 8,
	.val_bits			= 32,
	.max_register		= 0xff,
	.reg_read			= tas571x_reg_read,
	.reg_write			= tas571x_reg_write,
	.reg_defaults		= tas5716_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(tas5716_reg_defaults),
	.cache_type			= REGCACHE_RBTREE,
};

/* This entry is reused for tas5716 as the software interface is identical. */
static const struct tas571x_chip tas5716_chip = {
	.supply_names		= tas5716_supply_names,
	.num_supply_names	= ARRAY_SIZE(tas5716_supply_names),
	.controls			= tas5717_controls,
	.num_controls		= ARRAY_SIZE(tas5717_controls),
	.regmap_config		= &tas5716_regmap_config,
	.vol_reg_size		= 2,
};
#endif

static const struct regmap_config tas5717_regmap_config = {
	.reg_bits			= 8,
	.val_bits			= 32,
	.max_register		= 0xff,
	.reg_read			= tas571x_reg_read,
	.reg_write			= tas571x_reg_write,
	.reg_defaults		= tas5717_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(tas5717_reg_defaults),
	.cache_type			= REGCACHE_RBTREE,
};

/* This entry is reused for tas5719 as the software interface is identical. */
static const struct tas571x_chip tas5717_chip = {
	.supply_names			= tas5717_supply_names,
	.num_supply_names		= ARRAY_SIZE(tas5717_supply_names),
	.controls				= tas5717_controls,
	.num_controls			= ARRAY_SIZE(tas5717_controls),
	.regmap_config			= &tas5717_regmap_config,
	.vol_reg_size			= 2,
};

static const struct snd_soc_dapm_widget tas571x_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACR", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT_A"),
	SND_SOC_DAPM_OUTPUT("OUT_B"),
	SND_SOC_DAPM_OUTPUT("OUT_C"),
	SND_SOC_DAPM_OUTPUT("OUT_D"),
};

static const struct snd_soc_dapm_route tas571x_dapm_routes[] = {
	{ "DACL",  NULL, "Playback" },
	{ "DACR",  NULL, "Playback" },
	{ "OUT_A", NULL, "DACL" },
	{ "OUT_B", NULL, "DACL" },
	{ "OUT_C", NULL, "DACR" },
	{ "OUT_D", NULL, "DACR" },
};

static const struct snd_soc_codec_driver tas571x_codec = {
	.set_bias_level 	= tas571x_set_bias_level,
	.idle_bias_off 		= true,
	.dapm_widgets 		= tas571x_dapm_widgets,
	.num_dapm_widgets 	= ARRAY_SIZE(tas571x_dapm_widgets),
	.dapm_routes 		= tas571x_dapm_routes,
	.num_dapm_routes 	= ARRAY_SIZE(tas571x_dapm_routes),
};

static struct snd_soc_dai_driver tas571x_dai = {
	.name = "tas571x-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &tas571x_dai_ops,
};

static const struct of_device_id tas571x_of_match[];

int gpio_setup ()
{
	int ret = -EIO;
	
	/* PDN high */
	if (gpio_is_valid(PDN_GPIO)) {
		ret = gpio_request(PDN_GPIO, "power_down");
		if (ret < 0) {
 			pmx_dbg ("can't request enable gpio %d\n", PDN_GPIO);
			goto oops;
		}

		ret = gpio_direction_output(PDN_GPIO, GPIO_HIGH);
		if (ret < 0) {
			pmx_dbg ("can't request output direction enable gpio %d\n", PDN_GPIO);
			goto oops;
		}
	}

	/* RESET high */
	if (gpio_is_valid(RESET_GPIO)) {
		ret = gpio_request(RESET_GPIO, "reset_pin");
		if (ret < 0) {
			pmx_dbg ("can't request enable gpio %d\n", RESET_GPIO);
			goto oops;
		}

		/* wait 100 us */
		usleep_range(50, 150);
		
		ret = gpio_direction_output(RESET_GPIO, GPIO_HIGH);
		if (ret < 0) {
			pmx_dbg ("can't request output direction enable gpio %d\n", RESET_GPIO);
			goto oops;
		}

		/* wait 13.5 ms after RESET goes high */
		usleep_range(13000, 15000);
	}

	/* FPS_SDN high */
	if (gpio_is_valid(FPS_SDN_GPIO)) {
		ret = gpio_request(FPS_SDN_GPIO, "fps_sdn_pin");
		if (ret < 0) {
			pmx_dbg ("can't request enable gpio %d\n", FPS_SDN_GPIO);
			goto oops;
		}

		/* wait 100 us */
		usleep_range(50, 150);
		
		ret = gpio_direction_output(FPS_SDN_GPIO, GPIO_HIGH);
		if (ret < 0) {
			pmx_dbg ("can't request output direction enable gpio %d\n", FPS_SDN_GPIO);
			goto oops;
		}

		/* wait 13.5 ms after FPS_SDN goes high */
		usleep_range(13000, 15000);
	}

	return 0;

oops:
	return -EIO;
}

static const struct _tas5716_init_data tas5716_init_data[] =
{
	/*	Register address				   Value			Comment		*/
	{ TAS5716_OSC_TRIM_REG			, 0x00 },		// 0x1b
	{ TAS5716_SYS_CTRL1_REG			, 0xa0 },		// 0x03
	{ TAS5716_SDI_REG				, 0x05 },		// 0x04
	{ TAS5716_SYS_CTRL2_REG			, 0x20 },		// 0x05
	{ TAS5716_SOFT_MUTE_REG			, 0x00 },		// 0x06
	{ TAS5716_MASTER_VOL_REG		, 0x40 },		// 0x07
	{ TAS5716_CH1_VOL_REG			, 0x40 },		// 0x08
	{ TAS5716_CH2_VOL_REG			, 0x40 },		// 0x09
	{ TAS5716_CH3_VOL_REG			, 0x30 },		// 0x0a
	{ TAS5716_CH4_VOL_REG			, 0x30 },		// 0x0b
	{ TAS5716_HP_VOL_REG			, 0x30 },		// 0x0c
	{ TAS5716_CH6_VOL_REG			, 0x30 },		// 0x0d
	{ TAS5716_MOD_LIMIT_REG			, 0x02 },		// 0x10
	{ TAS5716_IC_DELAY_CH1_REG		, 0x4c },		// 0x11
	{ TAS5716_IC_DELAY_CH2_REG		, 0x34 },		// 0x12
	{ TAS5716_IC_DELAY_CH3_REG		, 0x1c },		// 0x13
	{ TAS5716_IC_DELAY_CH4_REG		, 0x64 },		// 0x14
	{ TAS5716_IC_DELAY_CH5_REG		, 0xd0 },		// 0x15
	{ TAS5716_IC_DELAY_CH6_REG		, 0x90 },		// 0x16
	{ TAS5716_OFFSET_REG			, 0x00 },		// 0x17
	{ TAS5716_PWM_SHUTDOWN_REG		, 0x00 },		// 0x19
	{ TAS5716_BKND_ERR_REG			, 0x02 },		// 0x1c
	{ TAS5716_INPUT_MUX_REG			, 0x00897778 },	// 0x20
	{ TAS5716_CH6_INPUT_MUX2_REG	, 0x00004003 },	// 0x21
	{ TAS5716_AM_TUNED_FREQ_REG		, 0x00 },		// 0x22
	{ TAS5716_PWM_MUX_REG			, 0x01021345 },	// 0x25
	{ TAS5716_1G_REG				, 0x00800000 },	// 0x26
};

static void i2c_dump_reg_value (unsigned int reg, uint8_t val[])
{
	int i=0;
	int sz = tas571x_register_size (reg);

	pmx_dbg ("reg: 0x%02x\n", reg);
	
	switch (sz) {
	case NO_OF_BYTES_1:
		pmx_dbg ("val[%d]: 0x%02x\n", i, val[0]);
		break;
	case NO_OF_BYTES_4:
		//for (i=0; i<sz; i++) {
		//	pmx_dbg ("val: 0x%08x\n", val[0]);
		//}
		break;
	case NO_OF_BYTES_8:
		for (i=0; i<sz; i++) {
			pmx_dbg ("val[%d]: 0x%02x\n", i, val[i]);
		}
		break;
	case NO_OF_BYTES_20:
		for (i=0; i<sz; i++) {
			pmx_dbg ("val[%d]: 0x%02x\n", i, val[i]);
		}
		break;
	default:
		break;
	}

	return;
}

static int i2c_single_write (struct regmap *map, unsigned int reg, unsigned int val)
{
	int r = regmap_write (map, reg, val);
	if (r < 0)
		pmx_dbg ("regmap_write failed with reg (0x%02x)\n", reg);

	pmx_dbg ("reg: 0x%02x with val: 0x%08x\n", reg, val);
	return r;
}

static int i2c_single_read (struct regmap *map, unsigned int reg)
{
	unsigned int val;
	int r = regmap_read (map, reg, &val);
	if (r < 0)
		pmx_dbg ("regmap_read failed with reg (0x%x)\n", reg);
	else {
		i2c_dump_reg_value (reg, &val);
	}
	return r;
}

// support 8/20 bytes
static int i2c_multiple_bytes_read (struct i2c_client *client, struct regmap *map
											, unsigned int reg, uint8_t val[])
{
	int i, j, ret, reg_sz;
	uint8_t u8_reg = reg;
	uint8_t recv_buf[20] = {0};
	struct i2c_msg msgs[2];

	reg_sz = tas571x_register_size (reg);

	/* device addr + R/W bit + sub addr */
	msgs[0].addr	= client->addr;
	msgs[0].len		= sizeof(u8_reg);
	msgs[0].buf		= &u8_reg;
	msgs[0].flags	= 0;

	/* device addr + R/W bit + recv buf */
	msgs[1].addr	= client->addr;
	msgs[1].len		= reg_sz;
	msgs[1].buf		= recv_buf;
	msgs[1].flags	= I2C_M_RD;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	for (i = 0; i < reg_sz; i++) {
		val[i] |= recv_buf[i];
	}
	return 0;
}

static int i2c_multiple_bytes_write (struct i2c_client *client, struct regmap *map
											, unsigned int reg, uint8_t val[])
{
	int i, j, ret, reg_sz;
	uint8_t u8_reg = reg;
	uint8_t send_buf[21] = {0};
	struct i2c_msg msgs[2];

	reg_sz = tas571x_register_size (reg);

	send_buf[0] = u8_reg;
	
	for (i=1; i<reg_sz+1; i++) {
		send_buf[i] = val[i-1];
	}

	/* device addr + R/W bit + sub addr + data */
	msgs[0].addr	= client->addr;
	msgs[0].len		= reg_sz+1;
	msgs[0].buf		= send_buf;
	msgs[0].flags	= 0;
	
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));	
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	return 0;
}

// Need to verify again
static void i2c_single_write_read_verify (struct regmap *map, unsigned int reg, unsigned int val)
{
	i2c_single_write (map, reg , val);
	i2c_single_read (map, reg);
}

// Need to verify again
static void i2c_multi_write_read_verify (struct i2c_client *client, struct regmap *map, unsigned int reg, uint8_t val[])
{
	// i2c_single_write (map, reg , val);
	i2c_multiple_bytes_read (client, map, reg, val);

#ifdef _DEBUG_
	// i2c_dump_reg_value (reg, val);
#endif
}

#ifdef CONFIG_SND_REALTEK
#define RTK_GPIO_REG_BASE	0x18000000

void init_i2s_pin_mux ()
{
	void __iomem *io = ioremap(RTK_GPIO_REG_BASE, 1024);
	
	writel(0x00000005, io + 0x0374);
	writel(0x000000ff, io + 0x0368);
	writel(0x84000015, io + 0x7314);
	writel(0x55505a69, io + 0x7310);

	mdelay (50);
	
	writel(0x3580c35d, io + 0x0000);
	writel(0x42940679, io + 0x000c);

	mdelay (50);
	
	writel(0x00000001, io + 0x0080);
	writel(0x000000ff, io + 0x0120);

	mdelay (50);

	writel(0x00000005, io + 0x0124);
	writel(0x00000003, io + 0x0128);
	
	mdelay (50);
	
	writel(0x10efe436, io + 0x0010);
	writel(0x0000b001, io + 0x6000);

	iounmap(io);
}
#endif

// debug only
int myreg = 0;
module_param(myreg, int, 0);

static int tas571x_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int ret, i, sz;
	struct tas571x_private *priv;
	struct device *dev = &client->dev;
	struct tas571x_chip *chip = (struct tas571x_chip *) client->dev.platform_data;

#ifdef CONFIG_SND_REALTEK
	// init i2s config/pin mux
	init_i2s_pin_mux ();
#endif

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	
	i2c_set_clientdata(client, priv);

	/* regmap initialize */
	priv->regmap = devm_regmap_init (dev, NULL, client, chip->regmap_config);
	if (IS_ERR(priv->regmap)) {
		pmx_dbg ("devm_regmap_init failed!!!\n");
		return PTR_ERR(priv->regmap);
	}

	/* gpio pin setup */
	if (gpio_setup() != 0)
		return -EIO;

	/* i2c command setup */
	sz = sizeof (tas5716_init_data) / sizeof (struct _tas5716_init_data);
	for (i=0; i<sz; i++) {
		/* less than 4 bytes we can use single write */
		if (tas571x_register_size (tas5716_init_data[i].reg) <= NO_OF_BYTES_4) {
#ifndef _DEBUG_
			i2c_single_write (priv->regmap, tas5716_init_data[i].reg
			 							, tas5716_init_data[i].val);

#else
			i2c_single_write_read_verify (priv->regmap, tas5716_init_data[i].reg
													, tas5716_init_data[i].val);
#endif

			if (tas5716_init_data[i].reg == TAS5716_OSC_TRIM_REG) {
				/* Wait 50 ms while the part acquires lock */
				usleep_range (50000, 55000);
			}
		/* more than 4 bytes we should use multibytes method */
		} else {
			// TBD - wait for audio parameters from Davis
		}
	}

#if 0	// multibyte test
	pmx_dbg ("myreg : 0x%02x\n", myreg);

	// how to read multi bytes - 8 (0x0080000000000000)
	uint8_t v8bi[8] = {0};
	uint8_t v8bo[8] = {0};
	i2c_multiple_bytes_read (client, priv->regmap, TAS5716_DRC1_AE_REG, &v8bi);

	pmx_dbg ("--------------------------\n");
	for (i=0; i<8; i++) {
		pmx_dbg ("v8bi[%d]: 0x%02x\n", i, v8bi[i]);
	}
	pmx_dbg ("--------------------------\n");
	
	v8bi[1] = 0x40;	// from 0x00
	i2c_multiple_bytes_write (client, priv->regmap, TAS5716_DRC1_AE_REG, v8bi);
	i2c_multiple_bytes_read (client, priv->regmap, TAS5716_DRC1_AE_REG, &v8bo);

	pmx_dbg ("--------------------------\n");
	for (i=0; i<8; i++) {
		pmx_dbg ("v8bo[%d]: 0x%02x\n", i, v8bo[i]);
	}
	pmx_dbg ("--------------------------\n");

	// how to read multi bytes - 20
	uint8_t v20bi[20] = {0};
	uint8_t v20bo[20] = {0};
	i2c_multiple_bytes_read (client, priv->regmap, TAS5716_CH1_BQ0_REG, &v20bi);

	pmx_dbg ("--------------------------\n");
	for (i=0; i<20; i++) {
		pmx_dbg ("v20bi[%d]: 0x%02x\n", i, v20bi[i]);
	}
	pmx_dbg ("--------------------------\n");
	
	v20bi[1] = 0x40;	// from 0x80
	i2c_multiple_bytes_write (client, priv->regmap, TAS5716_CH1_BQ0_REG, v20bi);
	i2c_multiple_bytes_read (client, priv->regmap, TAS5716_CH1_BQ0_REG, &v20bo);

	pmx_dbg ("--------------------------\n");
	for (i=0; i<20; i++) {
		pmx_dbg ("v20bo[%d]: 0x%02x\n", i, v20bo[i]);
	}
	pmx_dbg ("--------------------------\n");

#endif

	memcpy(&priv->codec_driver, &tas571x_codec, sizeof(priv->codec_driver));
	priv->codec_driver.controls = chip->controls;
	priv->codec_driver.num_controls = chip->num_controls;
	
	regcache_cache_only(priv->regmap, true);
	gpio_set_value(RESET_GPIO, 1);
	
	pmx_dbg ("snd_soc_register_codec okay!!!\n");
	return snd_soc_register_codec(&client->dev, &priv->codec_driver,
				      &tas571x_dai, 1);
}

static int tas571x_i2c_remove(struct i2c_client *client)
{
	pmx_dbg ("\n");
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct of_device_id tas571x_of_match[] = {
	{ .compatible = "ti,tas5711", .data = &tas5711_chip, },
	{ .compatible = "ti,tas5717", .data = &tas5717_chip, },
	{ .compatible = "ti,tas5719", .data = &tas5717_chip, },
	{ .compatible = "ti,tas5716", .data = &tas5716_chip, },
	{ }
};
MODULE_DEVICE_TABLE(of, tas571x_of_match);

static const struct i2c_device_id tas571x_i2c_id[] = {
	{ "tas5711", 0 },
	{ "tas5717", 0 },
	{ "tas5719", 0 },
	{ "tas5716", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tas571x_i2c_id);

static struct i2c_driver tas571x_i2c_driver = {
	.driver = {
		.name = "tas571x",
		.of_match_table = of_match_ptr(tas571x_of_match),
	},
	.probe = tas571x_i2c_probe,
	.remove = tas571x_i2c_remove,
	.id_table = tas571x_i2c_id,
};

static int __init tas571x_init(void)
{
	static struct i2c_board_info info  = {
		.type = "tas5716",
		.addr = TAS5716_SLAVE_ADDR,
		.platform_data = &tas5716_chip,
	};
	
	pmx_dbg ("TAS5716_BUS_ADDR %d\n", TAS5716_BUS_ADDR);
	pmx_dbg ("TAS5716_SLAVE_ADDR 0x%02x\n", TAS5716_SLAVE_ADDR);

	i2c_add_driver (&tas571x_i2c_driver);
	
	amp_i2c_client = i2c_new_device (i2c_get_adapter(TAS5716_BUS_ADDR), &info);
	if (!amp_i2c_client) {
		pmx_dbg("i2c_new_device failed!\n");
		return -1;
	}

	/* success */
	return 0;
}

static void __exit tas571x_exit(void)
{
	pmx_dbg("\n");
	gpio_free (PDN_GPIO);
	gpio_free (RESET_GPIO);
	gpio_free (FPS_SDN_GPIO);

	i2c_unregister_device (amp_i2c_client);
	i2c_del_driver (&tas571x_i2c_driver);
}

fs_initcall(tas571x_init);
module_exit(tas571x_exit);

MODULE_DESCRIPTION("ASoC TAS571x driver");
MODULE_AUTHOR("Kevin Cernekee <cernekee@chromium.org>");
MODULE_LICENSE("GPL");

