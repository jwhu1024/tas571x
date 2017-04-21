/*
 * TAS571x amplifier audio driver
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _TAS571X_H
#define _TAS571X_H

#if 1 // for tas5716
/* device registers */
#define TAS5716_CLK_CTRL_REG				0x00
#define TAS5716_DEVICE_ID_REG				0x01
#define TAS5716_ERR_STATUS_REG				0x02
#define TAS5716_SYS_CTRL1_REG				0x03
#define TAS5716_SDI_REG						0x04
#define TAS5716_SYS_CTRL2_REG				0x05
#define TAS5716_SOFT_MUTE_REG				0x06
#define TAS5716_MASTER_VOL_REG				0x07
#define TAS5716_CH1_VOL_REG					0x08
#define TAS5716_CH2_VOL_REG					0x09
#define TAS5716_CH3_VOL_REG					0x0a
#define TAS5716_CH4_VOL_REG					0x0b
#define TAS5716_HP_VOL_REG					0x0c
#define TAS5716_CH6_VOL_REG					0x0d
#define TAS5716_VOL_CFG_REG					0x0e
#define TAS5716_RESERVED0_REG				0x0f
#define TAS5716_MOD_LIMIT_REG				0x10
#define TAS5716_IC_DELAY_CH1_REG			0x11
#define TAS5716_IC_DELAY_CH2_REG			0x12
#define TAS5716_IC_DELAY_CH3_REG			0x13
#define TAS5716_IC_DELAY_CH4_REG			0x14
#define TAS5716_IC_DELAY_CH5_REG			0x15
#define TAS5716_IC_DELAY_CH6_REG			0x16
#define TAS5716_OFFSET_REG					0x17
#define TAS5716_RESERVED1_REG				0x18
#define TAS5716_PWM_SHUTDOWN_REG			0x19
#define TAS5716_START_STOP_PERIOD_REG		0x1a
#define TAS5716_OSC_TRIM_REG				0x1b
#define TAS5716_BKND_ERR_REG				0x1c
#define TAS5716_RESERVED2_REG				0x1d
#define TAS5716_RESERVED3_REG				0x1e
#define TAS5716_RESERVED4_REG				0x1f
#define TAS5716_INPUT_MUX_REG				0x20
#define TAS5716_CH6_INPUT_MUX2_REG			0x21
#define TAS5716_AM_TUNED_FREQ_REG			0x22
#define TAS5716_CH6BQ_LOUD_REG				0x23
#define TAS5716_CH6BQ_POST_VOL_REG			0x24
#define TAS5716_PWM_MUX_REG					0x25
#define TAS5716_1G_REG						0x26
#define TAS5716_RESERVED5_REG				0x27
#define TAS5716_SCALE_REG					0x28
#define TAS5716_CH1_BQ0_REG					0x29
#define TAS5716_CH1_BQ1_REG					0x2a
#define TAS5716_CH1_BQ2_REG					0x2b
#define TAS5716_CH1_BQ3_REG					0x2c
#define TAS5716_CH1_BQ4_REG					0x2d
#define TAS5716_CH1_BQ5_REG					0x2e
#define TAS5716_CH1_BQ6_REG					0x2f
#define TAS5716_CH2_BQ0_REG					0x30
#define TAS5716_CH2_BQ1_REG					0x31
#define TAS5716_CH2_BQ2_REG					0x32
#define TAS5716_CH2_BQ3_REG					0x33
#define TAS5716_CH2_BQ4_REG					0x34
#define TAS5716_CH2_BQ5_REG					0x35
#define TAS5716_CH2_BQ6_REG					0x36
#define TAS5716_CH6_BQ0_REG					0x37
#define TAS5716_CH6_BQ1_REG					0x38
#define TAS5716_RESERVED6_REG				0x39
#define TAS5716_DRC1_AE_REG					0x3a
#define TAS5716_DRC1_AA_REG					0x3b
#define TAS5716_DRC1_AD_REG					0x3c
#define TAS5716_DRC2_AE_REG					0x3d
#define TAS5716_DRC2_AA_REG					0x3e
#define TAS5716_DRC2_AD_REG					0x3f
#define TAS5716_DRC1_T_REG					0x40
#define TAS5716_DRC1_K_REG					0x41
#define TAS5716_DRC1_O_REG					0x42
#define TAS5716_DRC2_T_REG					0x43
#define TAS5716_DRC2_K_REG					0x44
#define TAS5716_DRC2_O_REG					0x45
#define TAS5716_DRC_CTRL_REG				0x46
#define TAS5716_RESERVED7_REG				0x47
#define TAS5716_RESERVED8_REG				0x48
#define TAS5716_RESERVED9_REG				0x49

//#else
/* device registers */
#define TAS571X_SDI_REG			0x04
#define TAS571X_SDI_FMT_MASK		0x0f

#define TAS571X_SYS_CTRL_2_REG		0x05
#define TAS571X_SYS_CTRL_2_SDN_MASK	0x40

#define TAS571X_SOFT_MUTE_REG		0x06
#define TAS571X_SOFT_MUTE_CH1_SHIFT	0
#define TAS571X_SOFT_MUTE_CH2_SHIFT	1
#define TAS571X_SOFT_MUTE_CH3_SHIFT	2

#define TAS571X_MVOL_REG		0x07
#define TAS571X_CH1_VOL_REG		0x08
#define TAS571X_CH2_VOL_REG		0x09

#define TAS571X_OSC_TRIM_REG		0x1b
#endif
#endif /* _TAS571X_H */
