// SPDX-License-Identifier: GPL-2.0
/*
 * gc8034 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
 * V0.0X01.0X04 add quick stream on/off
 * V0.0X01.0X05 add function g_mbus_config
 * V0.0X01.0X06
 * 1. add 2lane support.
 * 2. add some debug info.
 * 3. adjust gc8034_g_mbus_config function.
 * V0.0X01.0X07 support get channel info
 * V0.0X01.0X08
 * 1. default support 2lane full 30fps.
 * 2. default support rk otp spec.
 * V0.0X01.0X09 adjust supply sequence to suit spec
 */
//#define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/of_graph.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x09)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define GC8034_LANES			4
#define GC8034_BITS_PER_SAMPLE		10
#define GC8034_MIPI_FREQ_336MHZ		336000000U
#define GC8034_MIPI_FREQ_634MHZ		634000000U

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define GC8034_PIXEL_RATE		288000000
#define GC8034_XVCLK_FREQ		24000000

#define CHIP_ID				0x8044
#define GC8034_REG_CHIP_ID_H		0xf0
#define GC8034_REG_CHIP_ID_L		0xf1

#define GC8034_REG_SET_PAGE		0xfe
#define GC8034_SET_PAGE_ZERO		0x00

#define GC8034_REG_CTRL_MODE		0x3f
#define GC8034_MODE_SW_STANDBY		0x00
#define GC8034_MODE_STREAMING		0xd0

#define GC8034_REG_EXPOSURE_H		0x03
#define GC8034_REG_EXPOSURE_L		0x04
#define GC8034_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 8) & 0x7F)	/* 4 Bits */
#define GC8034_FETCH_LOW_BYTE_EXP(VAL)	((VAL) & 0xFF)	/* 8 Bits */
#define	GC8034_EXPOSURE_MIN		4
#define	GC8034_EXPOSURE_STEP		1
#define GC8034_VTS_MAX			0x1fff

#define GC8034_REG_AGAIN		0xb6
#define GC8034_REG_DGAIN_INT		0xb1
#define GC8034_REG_DGAIN_FRAC		0xb2
#define GC8034_GAIN_MIN			64
#define GC8034_GAIN_MAX			1092
#define GC8034_GAIN_STEP		1
#define GC8034_GAIN_DEFAULT		64

#define GC8034_REG_VTS_H		0x07
#define GC8034_REG_VTS_L		0x08

#define REG_NULL			0xFF

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define GC8034_NAME			"gc8034_i2c"
#define GC8034_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SRGGB10_1X10

/* choose 2lane support full 30fps or 15fps */
#define GC8034_2LANE_30FPS

static const char * const gc8034_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
	"avdd",		/* Analog power */
};

#define GC8034_NUM_SUPPLIES ARRAY_SIZE(gc8034_supply_names)


struct gc8034_dd {
	u16 x;
	u16 y;
	u16 t;
};

struct regval {
	u8 addr;
	u8 val;
};

struct gc8034_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 mipi_freq_idx;
	const struct regval *global_reg_list;
	const struct regval *reg_list;
//	u32 vc[PAD_MAX];
};

struct gc8034 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*power_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[GC8034_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*link_freq;
	struct mutex		mutex;
	bool			streaming;
	unsigned int		lane_num;
	unsigned int		cfg_num;
	unsigned int		pixel_rate;
	bool			power_on;
	const struct gc8034_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32 Dgain_ratio;
};

#define to_gc8034(sd) container_of(sd, struct gc8034, subdev)

#undef GC8034_MIRROR_NORMAL
#undef GC8034_MIRROR_H
#undef GC8034_MIRROR_V
#undef GC8034_MIRROR_HV
/* If you use the otp function, keep the otp_drv ->
 * gc8034_common_otp_drv.h consistent.
 */
#define GC8034_MIRROR_NORMAL

#if defined(GC8034_MIRROR_NORMAL)
	#define GC8034_MIRROR	0xc0
	#define BINNING_STARTY	0x04
	#define BINNING_STARTX	0x05
	#define FULL_STARTY	0x08
	#define FULL_STARTX	0x09
#elif defined(GC8034_MIRROR_H)
	#define GC8034_MIRROR	0xc1
	#define BINNING_STARTY	0x04
	#define BINNING_STARTX	0x04
	#define FULL_STARTY	0x08
	#define FULL_STARTX	0x08
#elif defined(GC8034_MIRROR_V)
	#define GC8034_MIRROR	0xc2
	#define BINNING_STARTY	0x05
	#define BINNING_STARTX	0x05
	#define FULL_STARTY	0x09
	#define FULL_STARTX	0x09
#elif defined(GC8034_MIRROR_HV)
	#define GC8034_MIRROR	0xc3
	#define BINNING_STARTY	0x05
	#define BINNING_STARTX	0x04
	#define FULL_STARTY	0x09
	#define FULL_STARTX	0x08
#else
	#define GC8034_MIRROR	0xc0
	#define BINNING_STARTY	0x04
	#define BINNING_STARTX	0x05
	#define FULL_STARTY	0x08
	#define FULL_STARTX	0x09
#endif

/*
 * Xclk 24Mhz
 */
static const struct regval gc8034_global_regs_2lane[] = {
#ifdef GC8034_2LANE_30FPS
	/* SYS */
	{0xf2, 0x00},
	{0xf4, 0x90},
	{0xf5, 0x3d},
	{0xf6, 0x44},
	{0xf8, 0x63},
	{0xfa, 0x42},
	{0xf9, 0x00},
	{0xf7, 0x95},
	{0xfc, 0x00},
	{0xfc, 0x00},
	{0xfc, 0xea},
	{0xfe, 0x03},
	{0x03, 0x9a},
	{0xfc, 0xee},
	{0xfe, 0x00},
	{0x88, 0x03},

	/*Cisctl&Analog*/
	{0xfe, 0x00},
	{0x03, 0x08},
	{0x04, 0xc6},
	{0x05, 0x02},
	{0x06, 0x16},
	{0x07, 0x00},
	{0x08, 0x10},
	{0x0a, 0x3a}, //row start
	{0x0b, 0x00},
	{0x0c, 0x04}, //col start
	{0x0d, 0x09},
	{0x0e, 0xa0}, //win_height 2464
	{0x0f, 0x0c},
	{0x10, 0xd4}, //win_width 3284
	{0x17, GC8034_MIRROR},
	{0x18, 0x02},
	{0x19, 0x17},
	{0x1e, 0x50},
	{0x1f, 0x80},
	{0x21, 0x4c},
	{0x25, 0x00},
	{0x28, 0x4a},
	{0x2d, 0x89},
	{0xca, 0x02},
	{0xcb, 0x00},
	{0xcc, 0x39},
	{0xce, 0xd0},
	{0xcf, 0x93},
	{0xd0, 0x1b},
	{0xd1, 0xaa},
	{0xd2, 0xcb},
	{0xd8, 0x40},
	{0xd9, 0xff},
	{0xda, 0x0e},
	{0xdb, 0xb0},
	{0xdc, 0x0e},
	{0xde, 0x08},
	{0xe4, 0xc6},
	{0xe5, 0x08},
	{0xe6, 0x10},
	{0xed, 0x2a},
	{0xfe, 0x02},
	{0x59, 0x02},
	{0x5a, 0x04},
	{0x5b, 0x08},
	{0x5c, 0x20},
	{0xfe, 0x00},
	{0x1a, 0x09},
	{0x1d, 0x13},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfe, 0x10},
	{0xfe, 0x00},

	/* Gamma */
	{0xfe, 0x00},
	{0x20, 0x54},
	{0x33, 0x82},
	{0xfe, 0x01},
	{0xdf, 0x06},
	{0xe7, 0x18},
	{0xe8, 0x20},
	{0xe9, 0x16},
	{0xea, 0x17},
	{0xeb, 0x50},
	{0xec, 0x6c},
	{0xed, 0x9b},
	{0xee, 0xd8},

	/*ISP*/
	{0xfe, 0x00},
	{0x80, 0x13},
	{0x84, 0x01},
	{0x89, 0x03},
	{0x8d, 0x03},
	{0x8f, 0x14},
	{0xad, 0x00},
	{0x66, 0x0c},
	{0xbc, 0x09},
	{0xc2, 0x7f},
	{0xc3, 0xff},

	/*Crop window*/
	{0x90, 0x01},
	{0x92, FULL_STARTY},
	{0x94, FULL_STARTX},
	{0x95, 0x09},
	{0x96, 0x90},
	{0x97, 0x0c},
	{0x98, 0xc0},

	/*Gain*/
	{0xb0, 0x90},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb6, 0x00},

	/*BLK*/
	{0xfe, 0x00},
	{0x40, 0x22},
	{0x41, 0x20},
	{0x42, 0x02},
	{0x43, 0x08},
	{0x4e, 0x0f},
	{0x4f, 0xf0},
	{0x58, 0x80},
	{0x59, 0x80},
	{0x5a, 0x80},
	{0x5b, 0x80},
	{0x5c, 0x00},
	{0x5d, 0x00},
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x6b, 0x01},
	{0x6c, 0x00},
	{0x6d, 0x0c},

	/*WB offset*/
	{0xfe, 0x01},
	{0xbf, 0x40},

	/*Dark Sun*/
	{0xfe, 0x01},
	{0x68, 0x77},

	/*DPC*/
	{0xfe, 0x01},
	{0x60, 0x00},
	{0x61, 0x10},
	{0x62, 0x60},
	{0x63, 0x30},
	{0x64, 0x00},

	/* LSC */
	{0xfe, 0x01},
	{0xa8, 0x60},
	{0xa2, 0xd1},
	{0xc8, 0x57},
	{0xa1, 0xb8},
	{0xa3, 0x91},
	{0xc0, 0x50},
	{0xd0, 0x05},
	{0xd1, 0xb2},
	{0xd2, 0x1f},
	{0xd3, 0x00},
	{0xd4, 0x00},
	{0xd5, 0x00},
	{0xd6, 0x00},
	{0xd7, 0x00},
	{0xd8, 0x00},
	{0xd9, 0x00},
	{0xa4, 0x10},
	{0xa5, 0x20},
	{0xa6, 0x60},
	{0xa7, 0x80},
	{0xab, 0x18},
	{0xc7, 0xc0},

	/*ABB*/
	{0xfe, 0x01},
	{0x20, 0x02},
	{0x21, 0x02},
	{0x23, 0x42},

	/*MIPI*/
	{0xfe, 0x03},
	{0x01, 0x07},
	{0x02, 0x04},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0xf0}, //lwc 3264*5/4
	{0x13, 0x0f},
	{0x15, 0x10}, //LP
	{0x16, 0x29},
	{0x17, 0xff},
	{0x18, 0x01},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x0c},
	{0x22, 0x0e},
	{0x23, 0x45},
	{0x24, 0x01},
	{0x25, 0x1c},
	{0x26, 0x0b},
	{0x29, 0x0e},
	{0x2a, 0x1d},
	{0x2b, 0x0b},
	{0xfe, 0x00},
	//{0x3f, 0x91},
	{0x3f, 0x00},
#else
	/*SYS*/
	{0xf2, 0x00},
	{0xf4, 0x80},
	{0xf5, 0x19},
	{0xf6, 0x44},
	{0xf7, 0x95}, //pll enable
	{0xf8, 0x63}, //pll mode
	{0xf9, 0x00},
	{0xfa, 0x45},
	{0xfc, 0xfe},

	/*Cisctl&Analog*/
	{0xfe, 0x00},
	{0x03, 0x08},
	{0x04, 0xc6},
	{0x05, 0x02},
	{0x06, 0x16},
	{0x07, 0x00},
	{0x08, 0x10},
	{0x0a, 0x3a}, //row start
	{0x0b, 0x00},
	{0x0c, 0x04}, //col start
	{0x0d, 0x09},
	{0x0e, 0xa0}, //win_height 2464
	{0x0f, 0x0c},
	{0x10, 0xd4}, //win_width 3284
	{0x17, GC8034_MIRROR},
	{0x18, 0x02},
	{0x19, 0x17},
	{0x1e, 0x50},
	{0x1f, 0x80},
	{0x21, 0x4c},
	{0x25, 0x00},
	{0x28, 0x4a},
	{0x2d, 0x89},
	{0xca, 0x02},
	{0xcb, 0x00},
	{0xcc, 0x39},
	{0xce, 0xd0},
	{0xcf, 0x93},
	{0xd0, 0x1b},
	{0xd1, 0xaa},
	{0xd2, 0xcb},
	{0xd8, 0x40},
	{0xd9, 0xff},
	{0xda, 0x0e},
	{0xdb, 0xb0},
	{0xdc, 0x0e},
	{0xde, 0x08},
	{0xe4, 0xc6},
	{0xe5, 0x08},
	{0xe6, 0x10},
	{0xed, 0x2a},
	{0xfe, 0x02},
	{0x59, 0x02},
	{0x5a, 0x04},
	{0x5b, 0x08},
	{0x5c, 0x20},
	{0xfe, 0x00},
	{0x1a, 0x09},
	{0x1d, 0x13},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfe, 0x10},
	{0xfe, 0x00},

	/* Gamma */
	{0xfe, 0x00},
	{0x20, 0x54},
	{0x33, 0x82},
	{0xfe, 0x01},
	{0xdf, 0x06},
	{0xe7, 0x18},
	{0xe8, 0x20},
	{0xe9, 0x16},
	{0xea, 0x17},
	{0xeb, 0x50},
	{0xec, 0x6c},
	{0xed, 0x9b},
	{0xee, 0xd8},

	/*ISP*/
	{0xfe, 0x00},
	{0x80, 0x13},
	{0x84, 0x01},
	{0x89, 0x03},
	{0x8d, 0x03},
	{0x8f, 0x14},
	{0xad, 0x00},

	/*Crop window*/
	{0x90, 0x01},
	{0x92, FULL_STARTY},
	{0x94, FULL_STARTX},
	{0x95, 0x09},
	{0x96, 0x90},
	{0x97, 0x0c},
	{0x98, 0xc0},

	/*Gain*/
	{0xb0, 0x90},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb6, 0x00},

	/*BLK*/
	{0xfe, 0x00},
	{0x40, 0x22},
	{0x43, 0x03}, //add_offset
	{0x4e, 0x00}, //row_bits[15:8]
	{0x4f, 0x3c}, //row_bits[7:0]
	{0x58, 0x80}, //dark current ratio
	{0x59, 0x80},
	{0x5a, 0x80},
	{0x5b, 0x80},
	{0x5c, 0x00},
	{0x5d, 0x00},
	{0x5e, 0x00},
	{0x5f, 0x00},

	/*WB offset*/
	{0xfe, 0x01},
	{0xbf, 0x40},

	/*Dark Sun*/
	{0xfe, 0x01},
	{0x68, 0x77},

	/*DPC*/
	{0xfe, 0x01},
	{0x60, 0x15},
	{0x61, 0x10},
	{0x62, 0x60},
	{0x63, 0x48},
	{0x64, 0x02},

	/*LSC*/
	{0xfe, 0x01},
	{0xa0, 0x10}, //[6]segment_width[8], 0x[5:4]segment_height[9:8]
	{0xa8, 0x60}, //segment_height[7:0]
	{0xa2, 0xd1}, //height_ratio[7:0]
	{0xc8, 0x5b}, //[7:4]height_ratio[11:8]
	{0xa1, 0xb8}, //segment_width[7:0]
	{0xa3, 0x91}, //width_ratio[7:0]
	{0xc0, 0x50}, //[7:4]width_ratio[11:8]
	{0xd0, 0x05}, //segment_width_end[11:8]
	{0xd1, 0xb2}, //segment_width_end[7:0]
	{0xd2, 0x1f}, //col_segment
	{0xd3, 0x00}, //row_num_start[7:0]
	{0xd4, 0x00}, //[5:4]row_num_start[9:8] [3:0]col_seg_start
	{0xd5, 0x00}, //[7:2]col_num_start[7:2]
	{0xd6, 0x00}, //[2:0]col_num_start[10:8]
	{0xd7, 0x00}, //row_seg_start
	{0xd8, 0x00}, //col_cal_start[7:0]
	{0xd9, 0x00}, //[2:0]col_cal_start[10:8]

	/*ABB*/
	{0xfe, 0x01},
	{0x20, 0x02},
	{0x21, 0x02},
	{0x23, 0x43},

	/*MIPI*/
	{0xfe, 0x03},
	{0x01, 0x07},
	{0x02, 0x07},
	{0x03, 0x92},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0xf0}, //lwc 3264*5/4
	{0x13, 0x0f},
	{0x15, 0x10}, //LP
	{0x16, 0x29},
	{0x17, 0xff},
	{0x18, 0x01},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x05},
	{0x22, 0x05},
	{0x23, 0x16},
	{0x24, 0x00},
	{0x25, 0x12},
	{0x26, 0x07},
	{0x29, 0x07},
	{0x2a, 0x08},
	{0x2b, 0x07},
	{0xfe, 0x00},
	//{0x3f, 0x91},
	{0x3f, 0x00},
#endif
	{REG_NULL, 0x00},
};

#ifndef GC8034_2LANE_30FPS
/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 672Mbps
 */
static const struct regval gc8034_1632x1224_regs_2lane[] = {
	/*SYS*/
	{0xf2, 0x00},
	{0xf4, 0x80},
	{0xf5, 0x19},
	{0xf6, 0x44},
	{0xf8, 0x63},
	{0xfa, 0x45},
	{0xf9, 0x00},
	{0xf7, 0x95},
	{0xfc, 0x00},
	{0xfc, 0x00},
	{0xfc, 0xea},
	{0xfe, 0x03},
	{0x03, 0x9a},
	{0xfc, 0xee},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfe, 0x10},
	{0xfe, 0x00},

	/*ISP*/
	{0xfe, 0x00},
	{0x80, 0x10},
	{0xad, 0x30},
	{0x66, 0x2c},
	{0xbc, 0x49},

	/*Crop window*/
	{0x90, 0x01},
	{0x92, BINNING_STARTY}, //crop y
	{0x94, BINNING_STARTX}, //crop x
	{0x95, 0x04},
	{0x96, 0xc8},
	{0x97, 0x06},
	{0x98, 0x60},

	/*MIPI*/
	{0xfe, 0x03},
	{0x01, 0x07},
	{0x02, 0x03},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0xf8},
	{0x13, 0x07},
	{0x15, 0x10}, //LP mode
	{0x16, 0x29},
	{0x17, 0xff},
	{0x18, 0x01},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x05},
	{0x22, 0x06},
	{0x23, 0x16},
	{0x24, 0x00},
	{0x25, 0x12},
	{0x26, 0x07},
	{0x29, 0x07},
	{0x2a, 0x08},
	{0x2b, 0x07},
	{0xfe, 0x00},
	{0x3f, 0x00},

	{REG_NULL, 0x00},
};
#endif

/*
 * Xclk 24Mhz
 * max_framerate 15fps
 * mipi_datarate per lane 672Mbps
 */
static const struct regval gc8034_3264x2448_regs_2lane[] = {
#ifdef GC8034_2LANE_30FPS
	/* SYS */
	{0xf2, 0x00},
	{0xf4, 0x90},
	{0xf5, 0x3d},
	{0xf6, 0x44},
	{0xf8, 0x63},
	{0xfa, 0x42},
	{0xf9, 0x00},
	{0xf7, 0x95},
	{0xfc, 0x00},
	{0xfc, 0x00},
	{0xfc, 0xea},
	{0xfe, 0x03},
	{0x03, 0x9a},
	{0xfc, 0xee},
	{0xfe, 0x00},
	{0x3f, 0x00},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfe, 0x10},
	{0xfe, 0x00},

	/* ISP */
	{0xfe, 0x00},
	{0x80, 0x13},
	{0xad, 0x00},
	{0x66, 0x0c},
	{0xbc, 0x06},

	/* Crop window */
	{0x90, 0x01},
	{0x92, FULL_STARTY},
	{0x94, FULL_STARTX},
	{0x95, 0x09},
	{0x96, 0x90},
	{0x97, 0x0c},
	{0x98, 0xc0},

	/* MIPI */
	{0xfe, 0x03},
	{0x01, 0x07},
	{0x02, 0x04},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0xf0}, //lwc 3264*5/4
	{0x13, 0x0f},
	{0x15, 0x10}, //LP
	{0x16, 0x29},
	{0x17, 0xff},
	{0x18, 0x01},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x0c},
	{0x22, 0x0c},
	{0x23, 0x56},
	{0x24, 0x00},
	{0x25, 0x1c},
	{0x26, 0x0b},
	{0x29, 0x0e},
	{0x2a, 0x1d},
	{0x2b, 0x0b},
	{0xfe, 0x00},
	//{0x3f, 0x91},
	{0x3f, 0x00},
#else
	/*SYS*/
	{0xf2, 0x00},
	{0xf4, 0x80},
	{0xf5, 0x19},
	{0xf6, 0x44},
	{0xf7, 0x95}, //pll enable
	{0xf8, 0x63}, //pll mode
	{0xf9, 0x00},
	{0xfa, 0x45},
	{0xfc, 0x00},
	{0xfc, 0x00},
	{0xfc, 0xfe},

	/* ISP */
	{0xfe, 0x00},
	{0x80, 0x13},
	{0xad, 0x00},
	{0x66, 0x0c},
	{0xbc, 0x09},

	/* Crop window */
	{0x90, 0x01},
	{0x92, FULL_STARTY},
	{0x94, FULL_STARTX},
	{0x95, 0x09},
	{0x96, 0x90},
	{0x97, 0x0c},
	{0x98, 0xc0},

	/* MIPI */
	{0xfe, 0x03},
	{0x01, 0x07},
	{0x02, 0x03},
	{0x03, 0x92},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0xf0}, //lwc 3264*5/4
	{0x13, 0x0f},
	{0x15, 0x10}, //LP
	{0x16, 0x29},
	{0x17, 0xff},
	{0x18, 0x01},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x05},
	{0x22, 0x05},
	{0x23, 0x16},
	{0x24, 0x00},
	{0x25, 0x12},
	{0x26, 0x07},
	{0x29, 0x07},
	{0x2a, 0x08},
	{0x2b, 0x07},
	{0xfe, 0x00},
	//{0x3f, 0x91},
	{0x3f, 0x00},
#endif
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 */
static const struct regval gc8034_global_regs_4lane[] = {
	/*SYS*/
	{0xf2, 0x00},
	{0xf4, 0x80},
	{0xf5, 0x19},
	{0xf6, 0x44},
	{0xf8, 0x63},
	{0xfa, 0x45},
	{0xf9, 0x00},
	{0xf7, 0x9d},
	{0xfc, 0x00},
	{0xfc, 0x00},
	{0xfc, 0xea},
	{0xfe, 0x03},
	{0x03, 0x9a},
	{0x18, 0x07},
	{0x01, 0x07},
	{0xfc, 0xee},
	/*Cisctl&Analog*/
	{0xfe, 0x00},
	{0x03, 0x08},
	{0x04, 0xc6},
	{0x05, 0x02},
	{0x06, 0x16},
	{0x07, 0x00},
	{0x08, 0x10},
	{0x0a, 0x3a},
	{0x0b, 0x00},
	{0x0c, 0x04},
	{0x0d, 0x09},
	{0x0e, 0xa0},
	{0x0f, 0x0c},
	{0x10, 0xd4},
	{0x17, 0xc0},
	{0x18, 0x02},
	{0x19, 0x17},
	{0x1e, 0x50},
	{0x1f, 0x80},
	{0x21, 0x4c},
	{0x25, 0x00},
	{0x28, 0x4a},
	{0x2d, 0x89},
	{0xca, 0x02},
	{0xcb, 0x00},
	{0xcc, 0x39},
	{0xce, 0xd0},
	{0xcf, 0x93},
	{0xd0, 0x19},
	{0xd1, 0xaa},
	{0xd2, 0xcb},
	{0xd8, 0x40},
	{0xd9, 0xff},
	{0xda, 0x0e},
	{0xdb, 0xb0},
	{0xdc, 0x0e},
	{0xde, 0x08},
	{0xe4, 0xc6},
	{0xe5, 0x08},
	{0xe6, 0x10},
	{0xed, 0x2a},
	{0xfe, 0x02},
	{0x59, 0x02},
	{0x5a, 0x04},
	{0x5b, 0x08},
	{0x5c, 0x20},
	{0xfe, 0x00},
	{0x1a, 0x09},
	{0x1d, 0x13},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfe, 0x10},
	{0xfe, 0x00},
	/*Gamma*/
	{0xfe, 0x00},
	{0x20, 0x55},
	{0x33, 0x83},
	{0xfe, 0x01},
	{0xdf, 0x06},
	{0xe7, 0x18},
	{0xe8, 0x20},
	{0xe9, 0x16},
	{0xea, 0x17},
	{0xeb, 0x50},
	{0xec, 0x6c},
	{0xed, 0x9b},
	{0xee, 0xd8},
	/*ISP*/
	{0xfe, 0x00},
	{0x80, 0x10},
	{0x84, 0x01},
	{0x88, 0x03},
	{0x89, 0x03},
	{0x8d, 0x03},
	{0x8f, 0x14},
	{0xad, 0x30},
	{0x66, 0x2c},
	{0xbc, 0x49},
	{0xc2, 0x7f},
	{0xc3, 0xff},
	/*Crop window*/
	{0x90, 0x01},
	{0x92, 0x08},
	{0x94, 0x09},
	{0x95, 0x04},
	{0x96, 0xc8},
	{0x97, 0x06},
	{0x98, 0x60},
	/*Gain*/
	{0xb0, 0x90},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb6, 0x00},
	/*BLK*/
	{0xfe, 0x00},
	{0x40, 0x22},
	{0x41, 0x20},
	{0x42, 0x02},
	{0x43, 0x08},
	{0x4e, 0x0f},
	{0x4f, 0xf0},
	{0x58, 0x80},
	{0x59, 0x80},
	{0x5a, 0x80},
	{0x5b, 0x80},
	{0x5c, 0x00},
	{0x5d, 0x00},
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x6b, 0x01},
	{0x6c, 0x00},
	{0x6d, 0x0c},
	/*WB offset*/
	{0xfe, 0x01},
	{0xbf, 0x40},
	/*Dark Sun*/
	{0xfe, 0x01},
	{0x68, 0x77},
	/*DPC*/
	{0xfe, 0x01},
	{0x60, 0x00},
	{0x61, 0x10},
	{0x62, 0x28},
	{0x63, 0x10},
	{0x64, 0x02},
	/*LSC*/
	{0xfe, 0x01},
	{0xa8, 0x60},
	{0xa2, 0xd1},
	{0xc8, 0x57},
	{0xa1, 0xb8},
	{0xa3, 0x91},
	{0xc0, 0x50},
	{0xd0, 0x05},
	{0xd1, 0xb2},
	{0xd2, 0x1f},
	{0xd3, 0x00},
	{0xd4, 0x00},
	{0xd5, 0x00},
	{0xd6, 0x00},
	{0xd7, 0x00},
	{0xd8, 0x00},
	{0xd9, 0x00},
	{0xa4, 0x10},
	{0xa5, 0x20},
	{0xa6, 0x60},
	{0xa7, 0x80},
	{0xab, 0x18},
	{0xc7, 0xc0},
	/*ABB*/
	{0xfe, 0x01},
	{0x20, 0x02},
	{0x21, 0x02},
	{0x23, 0x42},
	/*MIPI*/
	{0xfe, 0x03},
	{0x02, 0x03},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0xf8},
	{0x13, 0x07},
	{0x15, 0x10},
	{0x16, 0x29},
	{0x17, 0xff},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x02},
	{0x22, 0x03},
	{0x23, 0x0a},
	{0x24, 0x00},
	{0x25, 0x12},
	{0x26, 0x04},
	{0x29, 0x04},
	{0x2a, 0x02},
	{0x2b, 0x04},
	{0xfe, 0x00},
	{0x3f, 0x00},

	/*SYS*/
	{0xf2, 0x00},
	{0xf4, 0x80},
	{0xf5, 0x19},
	{0xf6, 0x44},
	{0xf8, 0x63},
	{0xfa, 0x45},
	{0xf9, 0x00},
	{0xf7, 0x95},
	{0xfc, 0x00},
	{0xfc, 0x00},
	{0xfc, 0xea},
	{0xfe, 0x03},
	{0x03, 0x9a},
	{0x18, 0x07},
	{0x01, 0x07},
	{0xfc, 0xee},
	/*ISP*/
	{0xfe, 0x00},
	{0x80, 0x13},
	{0xad, 0x00},
	/*Crop window*/
	{0x90, 0x01},
	{0x92, 0x08},
	{0x94, 0x09},
	{0x95, 0x09},
	{0x96, 0x90},
	{0x97, 0x0c},
	{0x98, 0xc0},
	/*DPC*/
	{0xfe, 0x01},
	{0x62, 0x60},
	{0x63, 0x48},
	/*MIPI*/
	{0xfe, 0x03},
	{0x02, 0x03},
	{0x04, 0x80},
	{0x11, 0x2b},
	{0x12, 0xf0},
	{0x13, 0x0f},
	{0x15, 0x10},
	{0x16, 0x29},
	{0x17, 0xff},
	{0x19, 0xaa},
	{0x1a, 0x02},
	{0x21, 0x05},
	{0x22, 0x06},
	{0x23, 0x2b},
	{0x24, 0x00},
	{0x25, 0x12},
	{0x26, 0x07},
	{0x29, 0x07},
	{0x2a, 0x12},
	{0x2b, 0x07},
	{0xfe, 0x00},
	{0x3f, 0x00},

	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 656Mbps
 */
static const struct regval gc8034_3264x2448_regs_4lane[] = {
	{REG_NULL, 0x00},
};

static const struct gc8034_mode supported_modes_2lane[] = {
#ifdef GC8034_2LANE_30FPS
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0900,
		.hts_def = 0x0858 * 2,
		.vts_def = 0x09c0,
		.mipi_freq_idx = 1,
		.global_reg_list = gc8034_global_regs_2lane,
		.reg_list = gc8034_3264x2448_regs_2lane,
	//	.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
#else
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x09a0,
		.hts_def = 0x0858 * 2,
		.vts_def = 0x09c4,
		.mipi_freq_idx = 0,
		.global_reg_list = gc8034_global_regs_2lane,
		.reg_list = gc8034_3264x2448_regs_2lane,
	//	.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.width = 1632,
		.height = 1224,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x09a0,
		.hts_def = 0x0858 * 2,
		.vts_def = 0x09c4,
		.mipi_freq_idx = 0,
		.global_reg_list = gc8034_global_regs_2lane,
		.reg_list = gc8034_1632x1224_regs_2lane,
	//	.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
#endif
};

static const struct gc8034_mode supported_modes_4lane[] = {
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x08c6,
		.hts_def = 0x10b0,
		.vts_def = 0x09c0,
		.mipi_freq_idx = 0,
		.global_reg_list = gc8034_global_regs_4lane,
		.reg_list = gc8034_3264x2448_regs_4lane,
	//	.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const struct gc8034_mode *supported_modes;

static const s64 link_freq_menu_items[] = {
	GC8034_MIPI_FREQ_336MHZ,
	GC8034_MIPI_FREQ_634MHZ
};

/* Write registers up to 4 at a time */
static int gc8034_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"gc8034 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int gc8034_write_array(struct i2c_client *client,
	const struct regval *regs)
{
	u32 i = 0;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = gc8034_write_reg(client, regs[i].addr, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int gc8034_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_err(&client->dev,
		"gc8034 read reg:0x%x failed !\n", reg);

	return ret;
}

static int gc8034_get_reso_dist(const struct gc8034_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		abs(mode->height - framefmt->height);
}

static const struct gc8034_mode *
gc8034_find_best_fit(struct gc8034 *gc8034,
		     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < gc8034->cfg_num; i++) {
		dist = gc8034_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int gc8034_set_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_format *fmt)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	const struct gc8034_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc8034->mutex);

	mode = gc8034_find_best_fit(gc8034, fmt);
	fmt->format.code = GC8034_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc8034->mutex);
		return -ENOTTY;
#endif
	} else {
		gc8034->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc8034->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc8034->vblank, vblank_def,
					 GC8034_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(gc8034->vblank, vblank_def);
		__v4l2_ctrl_s_ctrl(gc8034->link_freq, mode->mipi_freq_idx);
	}

	mutex_unlock(&gc8034->mutex);

	return 0;
}

static int gc8034_get_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_format *fmt)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	const struct gc8034_mode *mode = gc8034->cur_mode;

	mutex_lock(&gc8034->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
#else
		mutex_unlock(&gc8034->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = GC8034_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc8034->mutex);

	return 0;
}

static int gc8034_enum_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = GC8034_MEDIA_BUS_FMT;

	return 0;
}

static int gc8034_enum_frame_sizes(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc8034 *gc8034 = to_gc8034(sd);

	if (fse->index >= gc8034->cfg_num)
		return -EINVAL;

	if (fse->code != GC8034_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int gc8034_g_frame_interval(struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *fi)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	const struct gc8034_mode *mode = gc8034->cur_mode;

	mutex_lock(&gc8034->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc8034->mutex);

	return 0;
}

static int __gc8034_start_stream(struct gc8034 *gc8034)
{
	int ret;

	ret = gc8034_write_array(gc8034->client, gc8034->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&gc8034->mutex);
	ret = v4l2_ctrl_handler_setup(&gc8034->ctrl_handler);
	mutex_lock(&gc8034->mutex);
	ret |= gc8034_write_reg(gc8034->client,
		GC8034_REG_SET_PAGE,
		GC8034_SET_PAGE_ZERO);
	if (2 == gc8034->lane_num) {
		ret |= gc8034_write_reg(gc8034->client,
			GC8034_REG_CTRL_MODE,
			0x91);
	} else {
		ret |= gc8034_write_reg(gc8034->client,
			GC8034_REG_CTRL_MODE,
			GC8034_MODE_STREAMING);
	}
	return ret;
}

static int __gc8034_stop_stream(struct gc8034 *gc8034)
{
	int ret;

	ret = gc8034_write_reg(gc8034->client,
		GC8034_REG_SET_PAGE,
		GC8034_SET_PAGE_ZERO);
	ret |= gc8034_write_reg(gc8034->client,
		GC8034_REG_CTRL_MODE,
		GC8034_MODE_SW_STANDBY);

	return ret;
}

static int gc8034_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	struct i2c_client *client = gc8034->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				gc8034->cur_mode->width,
				gc8034->cur_mode->height,
		DIV_ROUND_CLOSEST(gc8034->cur_mode->max_fps.denominator,
		gc8034->cur_mode->max_fps.numerator));

	mutex_lock(&gc8034->mutex);
	on = !!on;
	if (on == gc8034->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc8034_start_stream(gc8034);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc8034_stop_stream(gc8034);
		pm_runtime_put(&client->dev);
	}

	gc8034->streaming = on;

unlock_and_return:
	mutex_unlock(&gc8034->mutex);

	return ret;
}

static int gc8034_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	struct i2c_client *client = gc8034->client;
	const struct gc8034_mode *mode = gc8034->cur_mode;
	int ret = 0;

	dev_info(&client->dev, "%s(%d) on(%d)\n", __func__, __LINE__, on);
	mutex_lock(&gc8034->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc8034->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = gc8034_write_array(gc8034->client, mode->global_reg_list);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc8034->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc8034->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc8034->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc8034_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC8034_XVCLK_FREQ / 1000 / 1000);
}

static int gc8034_enable_regulators(struct gc8034 *gc8034,
				    struct regulator_bulk_data *consumers)
{
	int i, j;
	int ret = 0;
	struct device *dev = &gc8034->client->dev;
	int num_consumers = GC8034_NUM_SUPPLIES;

	for (i = 0; i < num_consumers; i++) {

		ret = regulator_enable(consumers[i].consumer);
		if (ret < 0) {
			dev_err(dev, "Failed to enable regulator: %s\n",
				consumers[i].supply);
			goto err;
		}
	}
	return 0;
err:
	for (j = 0; j < i; j++)
		regulator_disable(consumers[j].consumer);

	return ret;
}

static int __gc8034_power_on(struct gc8034 *gc8034)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc8034->client->dev;

	if (!IS_ERR(gc8034->power_gpio))
		gpiod_set_value_cansleep(gc8034->power_gpio, 1);

	usleep_range(1000, 2000);

	if (!IS_ERR_OR_NULL(gc8034->pins_default)) {
		ret = pinctrl_select_state(gc8034->pinctrl,
					   gc8034->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(gc8034->xvclk, GC8034_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc8034->xvclk) != GC8034_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

	if (!IS_ERR(gc8034->reset_gpio))
		gpiod_set_value_cansleep(gc8034->reset_gpio, 1);

	ret = gc8034_enable_regulators(gc8034, gc8034->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(100, 200);
	ret = clk_prepare_enable(gc8034->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	usleep_range(1000, 1100);
	if (!IS_ERR(gc8034->pwdn_gpio))
		gpiod_set_value_cansleep(gc8034->pwdn_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(gc8034->reset_gpio))
		gpiod_set_value_cansleep(gc8034->reset_gpio, 0);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc8034_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(gc8034->xvclk);

	return ret;
}

static void __gc8034_power_off(struct gc8034 *gc8034)
{
	int ret;

	if (!IS_ERR(gc8034->pwdn_gpio))
		gpiod_set_value_cansleep(gc8034->pwdn_gpio, 1);

	if (!IS_ERR(gc8034->reset_gpio))
		gpiod_set_value_cansleep(gc8034->reset_gpio, 1);

	clk_disable_unprepare(gc8034->xvclk);
	if (!IS_ERR_OR_NULL(gc8034->pins_sleep)) {
		ret = pinctrl_select_state(gc8034->pinctrl,
					   gc8034->pins_sleep);
		if (ret < 0)
			dev_dbg(&gc8034->client->dev, "could not set pins\n");
	}
	if (!IS_ERR(gc8034->power_gpio))
		gpiod_set_value_cansleep(gc8034->power_gpio, 0);

	regulator_bulk_disable(GC8034_NUM_SUPPLIES, gc8034->supplies);
}

static int gc8034_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc8034 *gc8034 = to_gc8034(sd);

	return __gc8034_power_on(gc8034);
}

static int gc8034_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc8034 *gc8034 = to_gc8034(sd);

	__gc8034_power_off(gc8034);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc8034_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc8034 *gc8034 = to_gc8034(sd);
	struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, fh->state, 0);
	const struct gc8034_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc8034->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = GC8034_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc8034->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int gc8034_enum_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc8034 *gc8034 = to_gc8034(sd);

	if (fie->index >= gc8034->cfg_num)
		return -EINVAL;

	if (fie->code != GC8034_MEDIA_BUS_FMT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

#if 0
static int gc8034_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct gc8034 *sensor = to_gc8034(sd);
	struct device *dev = &sensor->client->dev;

	dev_info(dev, "%s(%d) enter!\n", __func__, __LINE__);

	if (2 == sensor->lane_num) {
		config->type = V4L2_MBUS_CSI2_DPHY;
		config->flags = V4L2_MBUS_CSI2_2_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else if (4 == sensor->lane_num) {
		config->type = V4L2_MBUS_CSI2_DPHY;
		config->flags = V4L2_MBUS_CSI2_4_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		dev_err(&sensor->client->dev,
			"unsupported lane_num(%d)\n", sensor->lane_num);
	}

	return 0;
}
#endif

static const struct dev_pm_ops gc8034_pm_ops = {
	SET_RUNTIME_PM_OPS(gc8034_runtime_suspend,
			gc8034_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc8034_internal_ops = {
	.open = gc8034_open,
};
#endif

static const struct v4l2_subdev_core_ops gc8034_core_ops = {
	.s_power = gc8034_s_power,
};

static const struct v4l2_subdev_video_ops gc8034_video_ops = {
	.s_stream = gc8034_s_stream,
	.g_frame_interval = gc8034_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc8034_pad_ops = {
	.enum_mbus_code = gc8034_enum_mbus_code,
	.enum_frame_size = gc8034_enum_frame_sizes,
	.enum_frame_interval = gc8034_enum_frame_interval,
	.get_fmt = gc8034_get_fmt,
	.set_fmt = gc8034_set_fmt,
//	.get_mbus_config = gc8034_g_mbus_config,
};

static const struct v4l2_subdev_ops gc8034_subdev_ops = {
	.core	= &gc8034_core_ops,
	.video	= &gc8034_video_ops,
	.pad	= &gc8034_pad_ops,
};

static int gc8034_set_exposure_reg(struct gc8034 *gc8034, u32 exposure)
{
	int ret = 0;
	u32 cal_shutter = 0;

	cal_shutter = exposure >> 1;
	cal_shutter = cal_shutter << 1;

	gc8034->Dgain_ratio = 256 * exposure / cal_shutter;
	ret = gc8034_write_reg(gc8034->client,
		GC8034_REG_SET_PAGE, GC8034_SET_PAGE_ZERO);
	ret |= gc8034_write_reg(gc8034->client,
		GC8034_REG_EXPOSURE_H,
		GC8034_FETCH_HIGH_BYTE_EXP(cal_shutter));
	ret |= gc8034_write_reg(gc8034->client,
		GC8034_REG_EXPOSURE_L,
		GC8034_FETCH_LOW_BYTE_EXP(cal_shutter));
	return ret;
}

#define MAX_AG_INDEX		9
#define AGC_REG_NUM		14
#define MEAG_INDEX		7

u16 gain_level[MAX_AG_INDEX] = {
		0x0040, /* 1.000*/
		0x0058, /* 1.375*/
		0x007d, /* 1.950*/
		0x00ad, /* 2.700*/
		0x00f3, /* 3.800*/
		0x0159, /* 5.400*/
		0x01ea, /* 7.660*/
		0x02ac, /*10.688*/
		0x03c2, /*15.030*/
};

u8 agc_register[MAX_AG_INDEX][AGC_REG_NUM] = {
	/* fullsize */
	{ 0x00, 0x55, 0x83, 0x01, 0x06, 0x18, 0x20,
		0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8, 0x00 },
	{ 0x00, 0x55, 0x83, 0x01, 0x06, 0x18, 0x20,
		0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8, 0x00 },
	{ 0x00, 0x4e, 0x84, 0x01, 0x0c, 0x2e, 0x2d,
		0x15, 0x19, 0x47, 0x70, 0x9f, 0xd8, 0x00 },
	{ 0x00, 0x51, 0x80, 0x01, 0x07, 0x28, 0x32,
		0x22, 0x20, 0x49, 0x70, 0x91, 0xd9, 0x00 },
	{ 0x00, 0x4d, 0x83, 0x01, 0x0f, 0x3b, 0x3b,
		0x1c, 0x1f, 0x47, 0x6f, 0x9b, 0xd3, 0x00 },
	{ 0x00, 0x50, 0x83, 0x01, 0x08, 0x35, 0x46,
		0x1e, 0x22, 0x4c, 0x70, 0x9a, 0xd2, 0x00 },
	{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a,
		0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 },
	{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a,
		0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 },
	{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a,
		0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 }
};

static int gc8034_set_gain_reg(struct gc8034 *gc8034, u32 a_gain)
{
	int ret = 0;
	u32 temp_gain = 0;
	int gain_index = 0;
	u32 Dgain_ratio = 0;

	Dgain_ratio = gc8034->Dgain_ratio;
	for (gain_index = MEAG_INDEX - 1; gain_index >= 0; gain_index--) {
		if (a_gain >= gain_level[gain_index]) {
			ret = gc8034_write_reg(gc8034->client,
				GC8034_REG_SET_PAGE, GC8034_SET_PAGE_ZERO);
			ret |= gc8034_write_reg(gc8034->client,
				0xb6, gain_index);
			temp_gain = 256 * a_gain / gain_level[gain_index];
			temp_gain = temp_gain * Dgain_ratio / 256;
			ret |= gc8034_write_reg(gc8034->client,
				0xb1, temp_gain >> 8);
			ret |= gc8034_write_reg(gc8034->client,
				0xb2, temp_gain & 0xff);

			ret |= gc8034_write_reg(gc8034->client, 0xfe,
				agc_register[gain_index][0]);
			ret |= gc8034_write_reg(gc8034->client, 0x20,
				agc_register[gain_index][1]);
			ret |= gc8034_write_reg(gc8034->client, 0x33,
				agc_register[gain_index][2]);
			ret |= gc8034_write_reg(gc8034->client, 0xfe,
				agc_register[gain_index][3]);
			ret |= gc8034_write_reg(gc8034->client, 0xdf,
				agc_register[gain_index][4]);
			ret |= gc8034_write_reg(gc8034->client, 0xe7,
				agc_register[gain_index][5]);
			ret |= gc8034_write_reg(gc8034->client, 0xe8,
				agc_register[gain_index][6]);
			ret |= gc8034_write_reg(gc8034->client, 0xe9,
				agc_register[gain_index][7]);
			ret |= gc8034_write_reg(gc8034->client, 0xea,
				agc_register[gain_index][8]);
			ret |= gc8034_write_reg(gc8034->client, 0xeb,
				agc_register[gain_index][9]);
			ret |= gc8034_write_reg(gc8034->client, 0xec,
				agc_register[gain_index][10]);
			ret |= gc8034_write_reg(gc8034->client, 0xed,
				agc_register[gain_index][11]);
			ret |= gc8034_write_reg(gc8034->client, 0xee,
				agc_register[gain_index][12]);
			ret |= gc8034_write_reg(gc8034->client, 0xfe,
				agc_register[gain_index][13]);
			break;
		}
	}
	return ret;
}

static const struct regval gc8034_global_regs_test_pattern[] = {
	{0xfc, 0x00},
	{0xf4, 0x80},
	{0xf5, 0x19},
	{0xf8, 0x63},
	{0xfa, 0x45},
	{0xfc, 0x00},
	{0xfc, 0xfe},
	{0xfe, 0x03},
	{0x21, 0x05},
	{0x22, 0x06},
	{0x23, 0x16},
	{0x25, 0x12},
	{0x26, 0x07},
	{0x29, 0x07},
	{0x2a, 0x08},
	{0x2b, 0x07},
	{0xfe, 0x00},
	{0x8c, 0x01},
};

static int gc8034_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc8034 *gc8034 = container_of(ctrl->handler,
					struct gc8034, ctrl_handler);
	struct i2c_client *client = gc8034->client;
	s64 max;
	int ret = 0;
	s32 temp;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc8034->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(gc8034->exposure,
					 gc8034->exposure->minimum, max,
					 gc8034->exposure->step,
					 gc8034->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		dev_dbg(&client->dev, "set exposure value 0x%x\n", ctrl->val);
		ret = gc8034_set_exposure_reg(gc8034, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set analog gain value 0x%x\n", ctrl->val);
		ret = gc8034_set_gain_reg(gc8034, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vb value 0x%x\n", ctrl->val);
		/* VB = VTS - 2448 -36, according android8.1 driver */
		temp = ctrl->val + gc8034->cur_mode->height - 2448 - 36;
		ret = gc8034_write_reg(gc8034->client,
					GC8034_REG_SET_PAGE,
					GC8034_SET_PAGE_ZERO);
		ret |= gc8034_write_reg(gc8034->client,
					GC8034_REG_VTS_H,
					(temp >> 8) & 0xff);
		ret |= gc8034_write_reg(gc8034->client,
					GC8034_REG_VTS_L,
					temp & 0xff);
		break;
	case V4L2_CID_TEST_PATTERN:
		int i;
		for (i = 0; i < ARRAY_SIZE(gc8034_global_regs_test_pattern); i++) {
			ret = gc8034_write_reg(gc8034->client, gc8034_global_regs_test_pattern[i].addr,
							gc8034_global_regs_test_pattern[i].val);
		}
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc8034_ctrl_ops = {
	.s_ctrl = gc8034_set_ctrl,
};

static int gc8034_initialize_controls(struct gc8034 *gc8034)
{
	const struct gc8034_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc8034->ctrl_handler;
	mode = gc8034->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc8034->mutex;

	gc8034->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_menu_items) - 1, 0,
				link_freq_menu_items);
	v4l2_ctrl_s_ctrl(gc8034->link_freq, mode->mipi_freq_idx);

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			0, gc8034->pixel_rate, 1, gc8034->pixel_rate);

	h_blank = mode->hts_def - mode->width;
	gc8034->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc8034->hblank)
		gc8034->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc8034->vblank = v4l2_ctrl_new_std(handler, &gc8034_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC8034_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	gc8034->exposure = v4l2_ctrl_new_std(handler, &gc8034_ctrl_ops,
				V4L2_CID_EXPOSURE, GC8034_EXPOSURE_MIN,
				exposure_max, GC8034_EXPOSURE_STEP,
				mode->exp_def);

	gc8034->anal_gain = v4l2_ctrl_new_std(handler, &gc8034_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, GC8034_GAIN_MIN,
				GC8034_GAIN_MAX, GC8034_GAIN_STEP,
				GC8034_GAIN_DEFAULT);
	if (handler->error) {
		ret = handler->error;
		dev_err(&gc8034->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc8034->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int gc8034_check_sensor_id(struct gc8034 *gc8034,
				struct i2c_client *client)
{
	struct device *dev = &gc8034->client->dev;
	u16 id = 0;
	u8 reg_H = 0;
	u8 reg_L = 0;
	int ret;

	ret = gc8034_read_reg(client, GC8034_REG_CHIP_ID_H, &reg_H);
	ret |= gc8034_read_reg(client, GC8034_REG_CHIP_ID_L, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected gc%04x sensor\n", id);
	return ret;
}

static int gc8034_configure_regulators(struct gc8034 *gc8034)
{
	unsigned int i;

	for (i = 0; i < GC8034_NUM_SUPPLIES; i++)
		gc8034->supplies[i].supply = gc8034_supply_names[i];

	return devm_regulator_bulk_get(&gc8034->client->dev,
		GC8034_NUM_SUPPLIES,
		gc8034->supplies);
}

static int gc8034_parse_of(struct gc8034 *gc8034)
{
	struct device *dev = &gc8034->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;
	unsigned int fps;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	gc8034->lane_num = rval;
	if (4 == gc8034->lane_num) {
		gc8034->cur_mode = &supported_modes_4lane[0];
		supported_modes = supported_modes_4lane;
		gc8034->cfg_num = ARRAY_SIZE(supported_modes_4lane);
		/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		fps = DIV_ROUND_CLOSEST(gc8034->cur_mode->max_fps.denominator,
					gc8034->cur_mode->max_fps.numerator);
		gc8034->pixel_rate = gc8034->cur_mode->vts_def *
				     gc8034->cur_mode->hts_def * fps;

		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
			 gc8034->lane_num, gc8034->pixel_rate);
	} else if (2 == gc8034->lane_num) {
		gc8034->cur_mode = &supported_modes_2lane[0];
		supported_modes = supported_modes_2lane;
		gc8034->cfg_num = ARRAY_SIZE(supported_modes_2lane);
		/*pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		fps = DIV_ROUND_CLOSEST(gc8034->cur_mode->max_fps.denominator,
					gc8034->cur_mode->max_fps.numerator);
		gc8034->pixel_rate = gc8034->cur_mode->vts_def *
				     gc8034->cur_mode->hts_def * fps;
		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
			 gc8034->lane_num, gc8034->pixel_rate);
	} else {
		dev_err(dev, "unsupported lane_num(%d)\n", gc8034->lane_num);
		return -1;
	}

	return 0;
}

static int gc8034_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc8034 *gc8034;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc8034 = devm_kzalloc(dev, sizeof(*gc8034), GFP_KERNEL);
	if (!gc8034)
		return -ENOMEM;

	gc8034->client = client;

	gc8034->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(gc8034->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	gc8034->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(gc8034->power_gpio))
		dev_warn(dev, "Failed to get power-gpios, maybe no use\n");
	gc8034->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc8034->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc8034->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc8034->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = gc8034_configure_regulators(gc8034);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	ret = gc8034_parse_of(gc8034);
	if (ret != 0)
		return -EINVAL;

	gc8034->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gc8034->pinctrl)) {
		gc8034->pins_default =
			pinctrl_lookup_state(gc8034->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gc8034->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		gc8034->pins_sleep =
			pinctrl_lookup_state(gc8034->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gc8034->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&gc8034->mutex);

	sd = &gc8034->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc8034_subdev_ops);
	ret = gc8034_initialize_controls(gc8034);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc8034_power_on(gc8034);
	if (ret)
		goto err_free_handler;

	ret = gc8034_check_sensor_id(gc8034, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc8034_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc8034->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc8034->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc8034->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc8034->module_index, facing,
		 GC8034_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__gc8034_power_off(gc8034);
err_free_handler:
	v4l2_ctrl_handler_free(&gc8034->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc8034->mutex);

	return ret;
}

static void gc8034_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc8034 *gc8034 = to_gc8034(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc8034->ctrl_handler);
	mutex_destroy(&gc8034->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc8034_power_off(gc8034);
	pm_runtime_set_suspended(&client->dev);

	return;
}

static const struct of_device_id gc8034_of_match[] = {
	{ .compatible = "galaxycore,gc8034" },
        { .compatible = "galaxycore,gc8034" },
	{},
};
MODULE_DEVICE_TABLE(of, gc8034_of_match);

static const struct i2c_device_id gc8034_match_id[] = {
	{ "galaxycore,gc8034", 0},
	{ },
};

static struct i2c_driver gc8034_i2c_driver = {
	.driver = {
		.name = GC8034_NAME,
		.pm = &gc8034_pm_ops,
		.of_match_table = gc8034_of_match,
	},
	.probe		= &gc8034_probe,
	.remove		= &gc8034_remove,
	.id_table	= gc8034_match_id,
};

module_i2c_driver(gc8034_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore gc8034 sensor driver");
MODULE_AUTHOR("99degree <https://github.com/99degree>");
MODULE_LICENSE("GPL v2");
