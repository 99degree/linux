// SPDX-License-Identifier: GPL-2.0
/*
 * gc02m1.c - gc02m1 sensor driver based on imx214 sensor driver by below
 *
 * Copyright 2018 Qtechnology A/S
 * Ricardo Ribalda <ribalda@kernel.org>
 
 * Copyright 2023 99degree <https://github.com/99degree>
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define GC02M1_DEFAULT_CLK_FREQ	24000000
#define GC02M1_DEFAULT_LINK_FREQ 480000000
#define GC02M1_DEFAULT_PIXEL_RATE ((GC02M1_DEFAULT_LINK_FREQ * 8LL) / 10)
#define GC02M1_FPS 30
#define GC02M1_MBUS_CODE MEDIA_BUS_FMT_SRGGB10_1X10

/*
 * REGISTER BASE
 */
#define GC02M1_SENSOR_SHUTTER_H				0x03
#define GC02M1_SENSOR_SHUTTER_H_MASK		GENMASK(5, 0) /* 0x3f */
#define GC02M1_SENSOR_SHUTTER_L				0x04

#define GC02M1_SENSOR_MIRROR				0x17
#define GC02M1_SENSOR_MIRROR_NO_FLIP		0x80
#define GC02M1_SENSOR_MIRROR_H_FLIP			0x81
#define GC02M1_SENSOR_MIRROR_V_FLIP			0x82
#define GC02M1_SENSOR_MIRROR_HV_FLIP		0x83

#define GC02M1_SENSOR_TEST_PATTERN			0x8c
#define GC02M1_SENSOR_TEST_PATTERN_ENABLE	0x11
#define GC02M1_SENSOR_TEST_PATTERN_DISABLE	0x10
		
#define GC02M1_SENSOR_FRAME_LENGTH_H		0x41  //framelength
#define GC02M1_SENSOR_FRAME_LENGTH_L		0x42  //framelegth
	
#define GC02M1_SENSOR_AGAIN_H				0xb1
#define GC02M1_SENSOR_AGAIN_H_MASK			GENMASK(4, 0)  /* total [0x1f 0xff], 13 bits */
#define GC02M1_SENSOR_AGAIN_H_BITS_SHIFT	0x03  // total 0x1fff, 13 bits 
#define GC02M1_SENSOR_AGAIN_L				0xb2
#define GC02M1_SENSOR_AGAIN_STEP			0xb6  // [0..16]
#define GC02M1_SENSOR_AGAIN_STEP_BITS		0x12

#define GC02M1_SENSOR_ID_H					0xf0
#define GC02M1_SENSOR_ID_L					0xf1
#define GC02M1_SENSOR_DUMMY_ENABLE			0xfe // write 0x01 then write 0x00
#define GC02M1_SENSOR_STREAMING_BASE		0x100
/* SENSOR PRIVATE INFO FOR GAIN SETTING */
//#define GC02M1_SENSOR_GAIN_BASE             0x400
//#define GC02M1_SENSOR_GAIN_MAX              (12 * GC02M1_SENSOR_GAIN_BASE)
//#define GC02M1_SENSOR_GAIN_MAX_VALID_INDEX  16
//#define GC02M1_SENSOR_GAIN_MAP_SIZE         16
//#define GC02M1_SENSOR_DGAIN_BASE            0x400

#if 0
/*
 * to simplify the transform scale, the uint16 a_gain should shift to 13 bit; 
 * but the max a_gain is 10337(0x2861) which is between 13 bit 0x1fff to 14 bit (0x3fff)
 * as simplified form, so grab AGAIN val = source_a_gain >> 3 bit, 
 * again step val = source a_gain > 13.
 * otherwise need to do a nlogm...which is not desired.
 */
uint16 GC02M1_AGC_Param[GC02M1_SENSOR_GAIN_MAX_VALID_INDEX][2] = {
		{  1024,  0 },
		{  1536,  1 },
		{  2035,  2 },
		{  2519,  3 },
		{  3165,  4 },
		{  3626,  5 },
		{  4147,  6 },
		{  4593,  7 },
		{  5095,  8 },
		{  5697,  9 },
		{  6270, 10 },
		{  6714, 11 },
		{  7210, 12 },
		{  7686, 13 },
		{  8214, 14 },
		{ 10337, 15 },
};
#endif

static const char * const gc02m1_supply_name[] = {
	"vdda",
	"vddd",
	"vdddo",
};

#define GC02M1_NUM_SUPPLIES ARRAY_SIZE(gc02m1_supply_name)

struct gc02m1 {
	struct device *dev;
	struct clk *xclk;
	struct regmap *regmap;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *unit_size;

	struct regulator_bulk_data	supplies[GC02M1_NUM_SUPPLIES];

	struct gpio_desc *enable_gpio;

	/*
	 * Serialize control access, get/set format, get selection
	 * and start streaming.
	 */
	struct mutex mutex;
};

struct reg_8 {
	u16 addr;
	u8 val;
};

enum {
	GC02M1_TABLE_WAIT_MS = 0,
	GC02M1_TABLE_END,
	GC02M1_MAX_RETRIES,
	GC02M1_WAIT_MS
};

static const struct reg_8 mode_table_common[] = {
	/*system*/
	{0xfc, 0x01},
	{0xf4, 0x41},
	{0xf5, 0xc0},
	{0xf6, 0x44},
	{0xf8, 0x38},
	{0xf9, 0x82},
	{0xfa, 0x00},
	{0xfd, 0x80},
	{0xfc, 0x81},
	{0xfe, 0x03},
	{0x01, 0x0b},
	{0xf7, 0x01},
	{0xfc, 0x80},
	{0xfc, 0x80},
	{0xfc, 0x80},
	{0xfc, 0x8e},

	/*CISCTL*/
	{0xfe, 0x00},
	{0x87, 0x09},
	{0xee, 0x72},
	{0xfe, 0x01},
	{0x8c, 0x90},
	{0xfe, 0x00},
	{0x90, 0x00},
	{0x03, 0x04},
	{0x04, 0x7d},
	{0x41, 0x04},
	{0x42, 0xf4},
	{0x05, 0x04},
	{0x06, 0x48},
	{0x07, 0x00},
	{0x08, 0x18},
	{0x9d, 0x18},
	{0x09, 0x00},
	{0x0a, 0x02},
	{0x0d, 0x04},
	{0x0e, 0xbc},
	{0x17, 0x80}, /* use default 0x80, let ioctl to handle V_flip and H_flip */
	{0x19, 0x04},
	{0x24, 0x00},
	{0x56, 0x20},
	{0x5b, 0x00},
	{0x5e, 0x01},

	/*analog Register width*/
	{0x21, 0x3c},
	{0x44, 0x20},
	{0xcc, 0x01},

	/*analog mode*/
	{0x1a, 0x04},
	{0x1f, 0x11},
	{0x27, 0x30},
	{0x2b, 0x00},
	{0x33, 0x00},
	{0x53, 0x90},
	{0xe6, 0x50},

	/*analog voltage*/
	{0x39, 0x07},
	{0x43, 0x04},
	{0x46, 0x2a},
	{0x7c, 0xa0},
	{0xd0, 0xbe},
	{0xd1, 0x60},
	{0xd2, 0x40},
	{0xd3, 0xf3},
	{0xde, 0x1d},

	/*analog current*/
	{0xcd, 0x05},
	{0xce, 0x6f},

	/*CISCTL RESET*/
	{0xfc, 0x88},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfc, 0x8e},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfc, 0x88},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfc, 0x8e},
	{0xfe, 0x04},
	{0xe0, 0x01},
	{0xfe, 0x00},

	/*ISP*/
	{0xfe, 0x01},
	{0x53, 0x44},
	{0x87, 0x53},
	{0x89, 0x03},

	/*Gain*/
	{0xfe, 0x00},
	{0xb0, 0x74},
	{0xb1, 0x04},
	{0xb2, 0x00},
	{0xb6, 0x00},
	{0xfe, 0x04},
	{0xd8, 0x00},
	{0xc0, 0x40},
	{0xc0, 0x00},
	{0xc0, 0x00},
	{0xc0, 0x00},
	{0xc0, 0x60},
	{0xc0, 0x00},
	{0xc0, 0xc0},
	{0xc0, 0x2a},
	{0xc0, 0x80},
	{0xc0, 0x00},
	{0xc0, 0x00},
	{0xc0, 0x40},
	{0xc0, 0xa0},
	{0xc0, 0x00},
	{0xc0, 0x90},
	{0xc0, 0x19},
	{0xc0, 0xc0},
	{0xc0, 0x00},
	{0xc0, 0xD0},
	{0xc0, 0x2F},
	{0xc0, 0xe0},
	{0xc0, 0x00},
	{0xc0, 0x90},
	{0xc0, 0x39},
	{0xc0, 0x00},
	{0xc0, 0x01},
	{0xc0, 0x20},
	{0xc0, 0x04},
	{0xc0, 0x20},
	{0xc0, 0x01},
	{0xc0, 0xe0},
	{0xc0, 0x0f},
	{0xc0, 0x40},
	{0xc0, 0x01},
	{0xc0, 0xe0},
	{0xc0, 0x1a},
	{0xc0, 0x60},
	{0xc0, 0x01},
	{0xc0, 0x20},
	{0xc0, 0x25},
	{0xc0, 0x80},
	{0xc0, 0x01},
	{0xc0, 0xa0},
	{0xc0, 0x2c},
	{0xc0, 0xa0},
	{0xc0, 0x01},
	{0xc0, 0xe0},
	{0xc0, 0x32},
	{0xc0, 0xc0},
	{0xc0, 0x01},
	{0xc0, 0x20},
	{0xc0, 0x38},
	{0xc0, 0xe0},
	{0xc0, 0x01},
	{0xc0, 0x60},
	{0xc0, 0x3c},
	{0xc0, 0x00},
	{0xc0, 0x02},
	{0xc0, 0xa0},
	{0xc0, 0x40},
	{0xc0, 0x80},
	{0xc0, 0x02},
	{0xc0, 0x18},
	{0xc0, 0x5c},
	{0xfe, 0x00},
	{0x9f, 0x10},

	/*BLK*/
	{0xfe, 0x00},
	{0x26, 0x20},
	{0xfe, 0x01},
	{0x40, 0x22},
	{0x46, 0x7f},
	{0x49, 0x0f},
	{0x4a, 0xf0},
	{0xfe, 0x04},
	{0x14, 0x80},
	{0x15, 0x80},
	{0x16, 0x80},
	{0x17, 0x80},

	/*ant _blooming*/
	{0xfe, 0x01},
	{0x41, 0x20},
	{0x4c, 0x00},
	{0x4d, 0x0c},
	{0x44, 0x08},
	{0x48, 0x03},

	/*Window 1600X1200*/
	{0xfe, 0x01},
	{0x90, 0x01},
	{0x91, 0x00},
	{0x92, 0x06},
	{0x93, 0x00},
	{0x94, 0x06},
	{0x95, 0x04},
	{0x96, 0xb0},
	{0x97, 0x06},
	{0x98, 0x40},

	/*mipi*/
	{0xfe, 0x03},
	{0x01, 0x23},
	{0x03, 0xce},
	{0x04, 0x48},
	{0x15, 0x00},
	{0x21, 0x10},
	{0x22, 0x05},
	{0x23, 0x20},
	{0x25, 0x20},
	{0x26, 0x08},
	{0x29, 0x06},
	{0x2a, 0x0a},
	{0x2b, 0x08},

	/*out*/
	{0xfe, 0x01},
	{0x8c, 0x10},
	{0xfe, 0x00},
	{0x3e, 0x00},
	{GC02M1_TABLE_WAIT_MS, 10},
//	{0x0138, 0x01},
	{GC02M1_TABLE_END, 0x00},
};

/*
static void set_dummy(void)
{
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x41, (imgsensor.frame_length >> 8) & 0x3f);
	write_cmos_sensor(0x42, imgsensor.frame_length & 0xff);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
}
*/

static const struct reg_8 mode_1600x1200[] = {
	/* mode */
	{0xfe, 0x00},
	{0x3e, 0x90},

	/* TODO: change sub script */

	{GC02M1_TABLE_WAIT_MS, 10},
//	{0x0138, 0x01},
	{GC02M1_TABLE_END, 0x00},
};

//https://github.com/MotorolaMobilityLLC/kernel-mtk/blob/6482b999eba3b7b23a3f21da5bc2b603d12353e4/drivers/misc/mediatek/imgsensor/src/common/v1_1/mot_devonn_gc02m1_mipi_raw/mot_devonn_gc02m1mipiraw_Sensor.c
static const struct reg_8 mode_1600x1200_custom1[] = {
	{0x41, 0x06},//30fps:0x04
	{0x42, 0x3c},//30fps:0xf4
	{0x07, 0x01},//30fps: 0x00
	{0x08, 0x60},//30fps: 0x18
	{0x3e, 0x90},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0x80, 0x00},
	{0x82, 0x08},
	{0x83, 0x0a},
	{0x88, 0x00},
	{0x89, 0x04},
	{0x8a, 0x00},
	{0x8b, 0x12},
	{0x7f, 0x29},
	{0x85, 0x51},
	{0xfe, 0x00},
	{GC02M1_TABLE_WAIT_MS, 10},
//	{0x0138, 0x01},
	{GC02M1_TABLE_END, 0x00},
};

/*
 * Declare modes in order, from biggest
 * to smallest height.
 */
static const struct gc02m1_mode {
	u32 width;
	u32 height;
	const struct reg_8 *reg_table;
} gc02m1_modes[] = {
	{
		.width = 1600,
		.height = 1200,
		.reg_table = mode_1600x1200,
	},
	{
		.width = 1600,
		.height = 1200,
		.reg_table = mode_1600x1200_custom1,
	},
};

static inline struct gc02m1 *to_gc02m1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc02m1, sd);
}

static int __maybe_unused gc02m1_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret;

	ret = regulator_bulk_enable(GC02M1_NUM_SUPPLIES, gc02m1->supplies);
	if (ret < 0) {
		dev_err(gc02m1->dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	usleep_range(2000, 3000);

	ret = clk_prepare_enable(gc02m1->xclk);
	if (ret < 0) {
		regulator_bulk_disable(GC02M1_NUM_SUPPLIES, gc02m1->supplies);
		dev_err(gc02m1->dev, "clk prepare enable failed\n");
		return ret;
	}

	gpiod_set_value_cansleep(gc02m1->enable_gpio, 1);
	usleep_range(12000, 15000);

	return 0;
}

static int __maybe_unused gc02m1_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	gpiod_set_value_cansleep(gc02m1->enable_gpio, 0);

	clk_disable_unprepare(gc02m1->xclk);

	regulator_bulk_disable(GC02M1_NUM_SUPPLIES, gc02m1->supplies);
	usleep_range(10, 20);

	return 0;
}

static int gc02m1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = GC02M1_MBUS_CODE;

	return 0;
}

static int gc02m1_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != GC02M1_MBUS_CODE)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(gc02m1_modes))
		return -EINVAL;

	fse->min_width = fse->max_width = gc02m1_modes[fse->index].width;
	fse->min_height = fse->max_height = gc02m1_modes[fse->index].height;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc02m1_s_register(struct v4l2_subdev *subdev,
			     const struct v4l2_dbg_register *reg)
{
	struct gc02m1 *gc02m1 = container_of(subdev, struct gc02m1, sd);

	return regmap_write(gc02m1->regmap, reg->reg, reg->val);
}

static int gc02m1_g_register(struct v4l2_subdev *subdev,
			     struct v4l2_dbg_register *reg)
{
	struct gc02m1 *gc02m1 = container_of(subdev, struct gc02m1, sd);
	unsigned int aux;
	int ret;

	reg->size = 1;
	ret = regmap_read(gc02m1->regmap, reg->reg, &aux);
	reg->val = aux;

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops gc02m1_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc02m1_g_register,
	.s_register = gc02m1_s_register,
#endif
};

static struct v4l2_mbus_framefmt *
__gc02m1_get_pad_format(struct gc02m1 *gc02m1,
			struct v4l2_subdev_state *sd_state,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&gc02m1->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &gc02m1->fmt;
	default:
		return NULL;
	}
}

static int gc02m1_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *format)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	mutex_lock(&gc02m1->mutex);
	format->format = *__gc02m1_get_pad_format(gc02m1, sd_state,
						  format->pad,
						  format->which);
	mutex_unlock(&gc02m1->mutex);

	return 0;
}

static struct v4l2_rect *
__gc02m1_get_pad_crop(struct gc02m1 *gc02m1,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&gc02m1->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &gc02m1->crop;
	default:
		return NULL;
	}
}

static int gc02m1_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *format)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	const struct gc02m1_mode *mode;

	mutex_lock(&gc02m1->mutex);

	__crop = __gc02m1_get_pad_crop(gc02m1, sd_state, format->pad,
				       format->which);

	mode = v4l2_find_nearest_size(gc02m1_modes,
				      ARRAY_SIZE(gc02m1_modes), width, height,
				      format->format.width,
				      format->format.height);

	__crop->width = mode->width;
	__crop->height = mode->height;

	__format = __gc02m1_get_pad_format(gc02m1, sd_state, format->pad,
					   format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = GC02M1_MBUS_CODE;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;
	__format->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(__format->colorspace);
	__format->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
				__format->colorspace, __format->ycbcr_enc);
	__format->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(__format->colorspace);

	format->format = *__format;

	mutex_unlock(&gc02m1->mutex);

	return 0;
}

static int gc02m1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	mutex_lock(&gc02m1->mutex);
	sel->r = *__gc02m1_get_pad_crop(gc02m1, sd_state, sel->pad,
					sel->which);
	mutex_unlock(&gc02m1->mutex);
	return 0;
}

static int gc02m1_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = gc02m1_modes[0].width;
	fmt.format.height = gc02m1_modes[0].height;

	gc02m1_set_format(subdev, sd_state, &fmt);

	return 0;
}

static int gc02m1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc02m1 *gc02m1 = container_of(ctrl->handler,
					     struct gc02m1, ctrls);
	u8 vals[2];
	int ret;

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (!pm_runtime_get_if_in_use(gc02m1->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
#if 0
		/* set framelength */
		if(ctrl->val > 61053)
			vals[1] = 0xff;
			vals[0] = 0x70;			
		} else /* long exposure */ {
			vals[1] = 0x0b;
			vals[0] = 0x9c;				
		}
		ret = regmap_bulk_write(gc02m1->regmap, 0x200, vals, 2);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
#endif
		/* shutter */
		vals[1] = ctrl->val;
		vals[0] = (ctrl->val >> 8) & GC02M1_SENSOR_SHUTTER_H_MASK;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_SHUTTER_H, vals, 2);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		ret = 0;
		break;
		
	case V4L2_CID_GAIN:
		vals[0] = 1;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_DUMMY_ENABLE, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);	
	
		//TODO: update framelength here
	
		vals[0] = 0;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_DUMMY_ENABLE, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);

		vals[0] = (ctrl->val >> GC02M1_SENSOR_AGAIN_STEP_BITS);
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_AGAIN_STEP, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		
		vals[1] = (ctrl->val >> GC02M1_SENSOR_AGAIN_H_BITS_SHIFT) & 0xff; /* valid bits 0x1fff*/
		vals[0] = (ctrl->val >> (8 + GC02M1_SENSOR_AGAIN_H_BITS_SHIFT)) & GC02M1_SENSOR_AGAIN_H_MASK;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_AGAIN_H, vals, 2);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		ret = 0;		
		break;
		
	/* todo check if oxfe is needed */
	case V4L2_CID_VFLIP:
		/* play safe, put 00 to 0xfe*/
		vals[0] = 0;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_DUMMY_ENABLE, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		
		vals[0] = GC02M1_SENSOR_MIRROR_V_FLIP;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_MIRROR, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		ret = 0;
		break;
		
	case V4L2_CID_HFLIP:
		/* play safe, put 00 to 0xfe*/
		vals[0] = 0;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_DUMMY_ENABLE, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		
		vals[0] = GC02M1_SENSOR_MIRROR_H_FLIP;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_MIRROR, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		ret = 0;
		break;
		
	/* TODO: need to disable test pattern, in case, shutdown the sensor instead */
	case V4L2_CID_TEST_PATTERN:
		vals[0] = 1;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_DUMMY_ENABLE, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);		
		
		vals[0] = GC02M1_SENSOR_TEST_PATTERN_ENABLE;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_TEST_PATTERN, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);
		
		vals[0] = 0;
		ret = regmap_bulk_write(gc02m1->regmap, GC02M1_SENSOR_DUMMY_ENABLE, vals, 1);
		if (ret < 0)
			dev_err(gc02m1->dev, "Error %d\n", ret);			
		ret = 0;
		break;
		
	default:
		ret = -EINVAL;
	}

	pm_runtime_put(gc02m1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc02m1_ctrl_ops = {
	.s_ctrl = gc02m1_set_ctrl,
};

#define MAX_CMD 4
static int gc02m1_write_table(struct gc02m1 *gc02m1,
			      const struct reg_8 table[])
{
	u8 vals[MAX_CMD];
	int i;
	int ret;

	for (; table->addr != GC02M1_TABLE_END ; table++) {
		if (table->addr == GC02M1_TABLE_WAIT_MS) {
			usleep_range(table->val * 1000,
				     table->val * 1000 + 500);
			continue;
		}

		for (i = 0; i < MAX_CMD; i++) {
			if (table[i].addr != (table[0].addr + i))
				break;
			vals[i] = table[i].val;
		}

		ret = regmap_bulk_write(gc02m1->regmap, table->addr, vals, i);

		if (ret) {
			dev_err(gc02m1->dev, "write_table error: %d\n", ret);
			return ret;
		}

		table += i - 1;
	}

	return 0;
}

static int gc02m1_start_streaming(struct gc02m1 *gc02m1)
{
	const struct gc02m1_mode *mode;
	int ret;

	mutex_lock(&gc02m1->mutex);
	ret = gc02m1_write_table(gc02m1, mode_table_common);
	if (ret < 0) {
		dev_err(gc02m1->dev, "could not sent common table %d\n", ret);
		goto error;
	}

	mode = v4l2_find_nearest_size(gc02m1_modes,
				ARRAY_SIZE(gc02m1_modes), width, height,
				gc02m1->fmt.width, gc02m1->fmt.height);
	ret = gc02m1_write_table(gc02m1, mode->reg_table);
	if (ret < 0) {
		dev_err(gc02m1->dev, "could not sent mode table %d\n", ret);
		goto error;
	}
	ret = __v4l2_ctrl_handler_setup(&gc02m1->ctrls);
	if (ret < 0) {
		dev_err(gc02m1->dev, "could not sync v4l2 controls\n");
		goto error;
	}
	ret = regmap_write(gc02m1->regmap, GC02M1_SENSOR_STREAMING_BASE, 1);
	if (ret < 0) {
		dev_err(gc02m1->dev, "could not sent start table %d\n", ret);
		goto error;
	}

	mutex_unlock(&gc02m1->mutex);
	return 0;

error:
	mutex_unlock(&gc02m1->mutex);
	return ret;
}

static int gc02m1_stop_streaming(struct gc02m1 *gc02m1)
{
	int ret;

	ret = regmap_write(gc02m1->regmap, GC02M1_SENSOR_STREAMING_BASE, 0);
	if (ret < 0)
		dev_err(gc02m1->dev, "could not sent stop table %d\n",	ret);

	return ret;
}

static int gc02m1_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct gc02m1 *gc02m1 = to_gc02m1(subdev);
	int ret;

	if (enable) {
		ret = pm_runtime_resume_and_get(gc02m1->dev);
		if (ret < 0)
			return ret;

		ret = gc02m1_start_streaming(gc02m1);
		if (ret < 0)
			goto err_rpm_put;
	} else {
		ret = gc02m1_stop_streaming(gc02m1);
		if (ret < 0)
			goto err_rpm_put;
		pm_runtime_put(gc02m1->dev);
	}

	return 0;

err_rpm_put:
	pm_runtime_put(gc02m1->dev);
	return ret;
}

static int gc02m1_g_frame_interval(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_frame_interval *fival)
{
	fival->interval.numerator = 1;
	fival->interval.denominator = GC02M1_FPS;

	return 0;
}

static int gc02m1_enum_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	const struct gc02m1_mode *mode;

	if (fie->index != 0)
		return -EINVAL;

	mode = v4l2_find_nearest_size(gc02m1_modes,
				ARRAY_SIZE(gc02m1_modes), width, height,
				fie->width, fie->height);

	fie->code = GC02M1_MBUS_CODE;
	fie->width = mode->width;
	fie->height = mode->height;
	fie->interval.numerator = 1;
	fie->interval.denominator = GC02M1_FPS;

	return 0;
}

static const struct v4l2_subdev_video_ops gc02m1_video_ops = {
	.s_stream = gc02m1_s_stream,
	.g_frame_interval = gc02m1_g_frame_interval,
	.s_frame_interval = gc02m1_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc02m1_subdev_pad_ops = {
	.enum_mbus_code = gc02m1_enum_mbus_code,
	.enum_frame_size = gc02m1_enum_frame_size,
	.enum_frame_interval = gc02m1_enum_frame_interval,
	.get_fmt = gc02m1_get_format,
	.set_fmt = gc02m1_set_format,
	.get_selection = gc02m1_get_selection,
	.init_cfg = gc02m1_entity_init_cfg,
};

static const struct v4l2_subdev_ops gc02m1_subdev_ops = {
	.core = &gc02m1_core_ops,
	.video = &gc02m1_video_ops,
	.pad = &gc02m1_subdev_pad_ops,
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int gc02m1_get_regulators(struct device *dev, struct gc02m1 *gc02m1)
{
	unsigned int i;

	for (i = 0; i < GC02M1_NUM_SUPPLIES; i++)
		gc02m1->supplies[i].supply = gc02m1_supply_name[i];

	return devm_regulator_bulk_get(dev, GC02M1_NUM_SUPPLIES,
				       gc02m1->supplies);
}

static int gc02m1_parse_fwnode(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned int i;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	if (ret) {
		dev_err(dev, "parsing endpoint node failed\n");
		goto done;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == GC02M1_DEFAULT_LINK_FREQ)
			break;

	if (i == bus_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequencies %d not supported, Please review your DT\n",
			GC02M1_DEFAULT_LINK_FREQ);
		ret = -EINVAL;
		goto done;
	}

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(endpoint);
	return ret;
}

static int gc02m1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc02m1 *gc02m1;
	static const s64 link_freq[] = {
		GC02M1_DEFAULT_LINK_FREQ,
	};
	static const struct v4l2_area unit_size = {
		.width = 1120,
		.height = 1120,
	};
	int ret;

	ret = gc02m1_parse_fwnode(dev);
	if (ret)
		return ret;

	gc02m1 = devm_kzalloc(dev, sizeof(*gc02m1), GFP_KERNEL);
	if (!gc02m1)
		return -ENOMEM;

	gc02m1->dev = dev;

	gc02m1->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(gc02m1->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(gc02m1->xclk);
	}

	ret = clk_set_rate(gc02m1->xclk, GC02M1_DEFAULT_CLK_FREQ);
	if (ret) {
		dev_err(dev, "could not set xclk frequency\n");
		return ret;
	}

	ret = gc02m1_get_regulators(dev, gc02m1);
	if (ret < 0) {
		dev_err(dev, "cannot get regulators\n");
		return ret;
	}

	/* TODO: is reset2 or enable gpio? */
	gc02m1->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(gc02m1->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(gc02m1->enable_gpio);
	}

	gc02m1->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(gc02m1->regmap)) {
		dev_err(dev, "regmap init failed\n");
		return PTR_ERR(gc02m1->regmap);
	}

	v4l2_i2c_subdev_init(&gc02m1->sd, client, &gc02m1_subdev_ops);

	/*
	 * Enable power initially, to avoid warnings
	 * from clk_disable on power_off
	 */
	gc02m1_power_on(gc02m1->dev);

	pm_runtime_set_active(gc02m1->dev);
	pm_runtime_enable(gc02m1->dev);
	pm_runtime_idle(gc02m1->dev);

	v4l2_ctrl_handler_init(&gc02m1->ctrls, 3);

	gc02m1->pixel_rate = v4l2_ctrl_new_std(&gc02m1->ctrls, NULL,
					       V4L2_CID_PIXEL_RATE, 0,
					       GC02M1_DEFAULT_PIXEL_RATE, 1,
					       GC02M1_DEFAULT_PIXEL_RATE);
	gc02m1->link_freq = v4l2_ctrl_new_int_menu(&gc02m1->ctrls, NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq) - 1,
						   0, link_freq);
	if (gc02m1->link_freq)
		gc02m1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

/* TODO check!!! */
	gc02m1->exposure = v4l2_ctrl_new_std(&gc02m1->ctrls, &gc02m1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     0, 3184, 1, 0x0c70);

	gc02m1->unit_size = v4l2_ctrl_new_std_compound(&gc02m1->ctrls,
				NULL,
				V4L2_CID_UNIT_CELL_SIZE,
				v4l2_ctrl_ptr_create((void *)&unit_size));
	ret = gc02m1->ctrls.error;
	if (ret) {
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto free_ctrl;
	}

	gc02m1->sd.ctrl_handler = &gc02m1->ctrls;
	mutex_init(&gc02m1->mutex);
	gc02m1->ctrls.lock = &gc02m1->mutex;

	gc02m1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc02m1->pad.flags = MEDIA_PAD_FL_SOURCE;
	gc02m1->sd.dev = &client->dev;
	gc02m1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&gc02m1->sd.entity, 1, &gc02m1->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	gc02m1_entity_init_cfg(&gc02m1->sd, NULL);

	ret = v4l2_async_register_subdev_sensor(&gc02m1->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	return 0;

free_entity:
	media_entity_cleanup(&gc02m1->sd.entity);
free_ctrl:
	mutex_destroy(&gc02m1->mutex);
	v4l2_ctrl_handler_free(&gc02m1->ctrls);
	pm_runtime_disable(gc02m1->dev);

	return ret;
}

static void gc02m1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	v4l2_async_unregister_subdev(&gc02m1->sd);
	media_entity_cleanup(&gc02m1->sd.entity);
	v4l2_ctrl_handler_free(&gc02m1->ctrls);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&gc02m1->mutex);
}

static const struct of_device_id gc02m1_of_match[] = {
	{ .compatible = "samsung,gc02m1" },
	{ }
};
MODULE_DEVICE_TABLE(of, gc02m1_of_match);

static const struct dev_pm_ops gc02m1_pm_ops = {
	SET_RUNTIME_PM_OPS(gc02m1_power_off, gc02m1_power_on, NULL)
};

static struct i2c_driver gc02m1_i2c_driver = {
	.driver = {
		.of_match_table = gc02m1_of_match,
		.pm = &gc02m1_pm_ops,
		.name  = "gc02m1",
	},
	.probe = gc02m1_probe,
	.remove = gc02m1_remove,
};

module_i2c_driver(gc02m1_i2c_driver);

MODULE_DESCRIPTION("samsung gc02m1 Camera driver");
MODULE_AUTHOR("Ricardo Ribalda <ribalda@kernel.org>");
MODULE_AUTHOR("99degree <https://github.com/99degree>");
MODULE_LICENSE("GPL v2");
