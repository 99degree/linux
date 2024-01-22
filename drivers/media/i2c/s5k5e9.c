// SPDX-License-Identifier: GPL-2.0
/*
 * s5k5e9.c - s5k5e9 sensor driver based on imx214 sensor driver by below
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

#define S5K5E9_DEFAULT_CLK_FREQ			24000000
#define S5K5E9_DEFAULT_LINK_FREQ		480000000
#define S5K5E9_DEFAULT_PIXEL_RATE 		((S5K5E9_DEFAULT_LINK_FREQ * 8LL) / 10)
#define S5K5E9_FPS				30
#define S5K5E9_MBUS_CODE 			MEDIA_BUS_FMT_SRGGB10_1X10
#define S5K5E9_SHUTTER_BASE_VAL_ONE_SEC		61053
#define S5K5E9_INT_MAX				5


#define S5K5E9_REG_SENSOR_ID_L			0x0000
#define S5K5E9_REG_SENSOR_ID_H			0x0001
#define S5K5E9_SENSOR_ID_VAL			0x559b

#define S5K5E9_REG_FRAMECNT			0x0005

#define S5K5E9_REG_MODE_SELECT			0x0100
#define S5K5E9_MODE_STANDBY			0x0
#define S5K5E9_MODE_STREAMING			0x1

#define S5K5E9_REG_GAIN_H			0x0204
#define S5K5E9_REG_GAIN_L			0x0205
#define S5K5E9_REG_GAIN_BIT_SHIFT		0x1

#define S5K5E9_REG_UPDATE_DUMMY			0x3200
#define S5K5E9_REG_UPDATE_DUMMY_VAL		0x00

#define S5K5E9_REG_TEST_PATTERN			0x0601
#define S5K5E9_REG_TEST_PATTERN_ENABLE		0x2
#define S5K5E9_REG_TEST_PATTERN_DISABLE		0x0

/* TODO: directly from old imx214, need to train */
/* Exposure control */
#define S5K5E9_REG_EXPOSURE			0x0202
#define S5K5E9_EXPOSURE_MIN			0
#define S5K5E9_EXPOSURE_MAX			3184
#define S5K5E9_EXPOSURE_STEP			1
#define S5K5E9_EXPOSURE_DEFAULT			3184

/* S5K5E9 native and active pixel array size */
#define S5K5E9_NATIVE_WIDTH			4224U
#define S5K5E9_NATIVE_HEIGHT			3136U
#define S5K5E9_PIXEL_ARRAY_LEFT			8U
#define S5K5E9_PIXEL_ARRAY_TOP			8U
#define S5K5E9_PIXEL_ARRAY_WIDTH		4208U
#define S5K5E9_PIXEL_ARRAY_HEIGHT		3120U

static const char * const s5k5e9_supply_name[] = {
	"vdda",
	"vddd",
	"vdddo",
};

#define S5K5E9_NUM_SUPPLIES ARRAY_SIZE(s5k5e9_supply_name)

struct s5k5e9 {
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
	S5K5E9_TABLE_WAIT_MS = 0, /* treat as register addr SENSOR_ID_L */
	S5K5E9_TABLE_END, /* treat as register addr SENSOR_ID_H */
	S5K5E9_MAX_RETRIES,
	S5K5E9_WAIT_MS
};

/*From s5k5e9_mode_tbls.h*/
static const struct reg_8 mode_2592x1940[] = {
	{0x0100, 0x00},
	{0x0136, 0x13},
	{0x0137, 0x33},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x59},
	{0x030d, 0x03},
	{0x030e, 0x00},
	{0x030f, 0x89},
	{0x3c1f, 0x00},
	{0x3c17, 0x00},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x01},
	{0x0820, 0x03},
	{0x0821, 0x6c},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3929, 0x0f},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x08},
	{0x0348, 0x0a},
	{0x0349, 0x27},
	{0x034a, 0x07},
	{0x034b, 0x9f},
	{0x034c, 0x0a},
	{0x034d, 0x20},
	{0x034e, 0x07},
	{0x034f, 0x98},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0101, 0x00},
	{0x0340, 0x07},
	{0x0341, 0xee},
	{0x0342, 0x0c},
	{0x0343, 0x28},
	{0x0200, 0x0b},
	{0x0201, 0x9c},
	{0x0202, 0x00},
	{0x0203, 0x02},
	{0x30b8, 0x2e},
	{0x30ba, 0x36},
	{0x0104, 0x00},
	{0x0340, 0x07},
	{0x0341, 0xee},
	{0x0202, 0x00},
	{0x0203, 0xa9},
	{0x0204, 0x00},
	{0x0205, 0x20},
	{0x0104, 0x00},
	{S5K5E9_TABLE_WAIT_MS, 10},
//	{0x0138, 0x01},
	{S5K5E9_TABLE_END, 0x00}
};

static const struct reg_8 mode_1920x1080[] = {
/* TODO: disable device tree and capture again for change resolution */
	{S5K5E9_TABLE_WAIT_MS, 10},
//	{0x0138, 0x01},
	{S5K5E9_TABLE_END, 0x00}
};

static const struct reg_8 mode_1280x720[] = {
	/* mode */
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0x5F},
	{0x030D, 0x04},
	{0x030E, 0x00},
	{0x030F, 0x92},
	{0x3C1F, 0x00},
	{0x3C17, 0x00},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x01},
	{0x0820, 0x03},
	{0x0821, 0x6C},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3929, 0x0F},
	{0x0344, 0x00},
	{0x0345, 0x18},
	{0x0346, 0x01},
	{0x0347, 0x04},
	{0x0348, 0x0A},
	{0x0349, 0x17},
	{0x034A, 0x06},
	{0x034B, 0xA3},
	{0x034C, 0x05},
	{0x034D, 0x00},
	{0x034E, 0x02},
	{0x034F, 0xD0},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x03},
	{0x0101, 0x00},
	{0x0340, 0x0F},
	{0x0341, 0xE4},
	{0x0342, 0x0C},
	{0x0343, 0x28},
	{0x0200, 0x0B},
	{0x0201, 0x9C},
	{0x0202, 0x00},
	{0x0203, 0x02},
	{0x30B8, 0x2A},
	{0x30BA, 0x2E},
	{0x0100, 0x01},
	{S5K5E9_TABLE_WAIT_MS, 10},
/* TODO: check what is this */
//	{0x0138, 0x01},
	{S5K5E9_TABLE_END, 0x00}
};


static const struct reg_8 mode_table_common[] = {
	{0x0100, 0x00},
	{0x3b45, 0x01},
	{0x0b05, 0x01},
	{0x392f, 0x01},
	{0x3930, 0x00},
	{0x3924, 0x7f},
	{0x3925, 0xfd},
	{0x3c08, 0xff},
	{0x3c09, 0xff},
	{0x3c0a, 0x05}, //allwinner init script do not have this
	{0x3c31, 0xff},
	{0x3c32, 0xff},
	{0x3290, 0x10},
	{0x3200, 0x01},
	{0x3074, 0x06},
	{0x3075, 0x2f},
	{0x308a, 0x20},
	{0x308b, 0x08},
	{0x308c, 0x0b},
	{0x3081, 0x07},
	{0x307b, 0x85},
	{0x307a, 0x0a},
	{0x3079, 0x0a},
	{0x306e, 0x71},
	{0x306f, 0x28},
	{0x301f, 0x20},
	{0x3012, 0x4e},
	{0x306b, 0x9a},
	{0x3091, 0x16},
	{0x30c4, 0x06},
	{0x306a, 0x79},
	{0x30b0, 0xff},
	{0x306d, 0x08},
	{0x3084, 0x16},
	{0x3070, 0x0f},
	{0x30c2, 0x05},
	{0x3069, 0x87},
	{0x3c0f, 0x00},
//	{0x0a02, 0x3f}, //my dump does not have this
	{0x3083, 0x14},
	{0x3080, 0x08},
	{0x3c34, 0xea},
	{0x3c35, 0x5c},
	{S5K5E9_TABLE_END, 0x00}
};

/*
 * Declare modes in order, from biggest
 * to smallest height.
 */
static const struct s5k5e9_mode {
	u32 width;
	u32 height;
	u32 linelength;
	u32 framelength;
	const struct reg_8 *reg_table;
} s5k5e9_modes[] = {
	{
		.width = 2592,
		.height = 1940,
		.linelength = 3112,
		.framelength = 2030,
		.reg_table = mode_2592x1940,
	},
#if 0
	/* this is depending on framelength and linelength, and is not support yet */
	{
		.width = 1280,
		.height = 720,
		/* TODO: find out real framelength and linelength */
		.linelength = 3112,
		.framelength = 2030,
		.reg_table = mode_1280x720,
	},
#endif
};

static inline struct s5k5e9 *to_s5k5e9(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5e9, sd);
}

static int __maybe_unused s5k5e9_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	int ret;
	int id[2];

	ret = devm_regulator_bulk_get_enable(dev, S5K5E9_NUM_SUPPLIES, s5k5e9_supply_name);
	if (ret < 0) {
		dev_err(s5k5e9->dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	usleep_range(2000, 3000);

	ret = clk_prepare_enable(s5k5e9->xclk);
	if (ret < 0) {
		dev_err(s5k5e9->dev, "clk prepare enable failed\n");
		return ret;
	}

	/* it is an CAM0_RST_N gpio pin */
	gpiod_set_value_cansleep(s5k5e9->enable_gpio, 0);
	usleep_range(12000, 15000);

	gpiod_set_value_cansleep(s5k5e9->enable_gpio, 1);
	usleep_range(12000, 15000);

	gpiod_set_value_cansleep(s5k5e9->enable_gpio, 0);
	usleep_range(12000, 15000);

	ret = regmap_read(s5k5e9->regmap, S5K5E9_REG_SENSOR_ID_H, &id[1]);
	ret = regmap_read(s5k5e9->regmap, S5K5E9_REG_SENSOR_ID_L,&id[0]);
	if ((id[1] | (id[0] << 8)) != S5K5E9_SENSOR_ID_VAL) {
		dev_err(dev, "sensor init 1 failed ret = 0x%x\n", id[0] << 8 | id[1]);
		return -EIO;
	}

	return 0;
}

static int __maybe_unused s5k5e9_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);

	gpiod_set_value_cansleep(s5k5e9->enable_gpio, 0);

	clk_disable_unprepare(s5k5e9->xclk);

	usleep_range(10, 20);

	return 0;
}

static int s5k5e9_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = S5K5E9_MBUS_CODE;

	return 0;
}

static int s5k5e9_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != S5K5E9_MBUS_CODE)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(s5k5e9_modes))
		return -EINVAL;

	fse->min_width = fse->max_width = s5k5e9_modes[fse->index].width;
	fse->min_height = fse->max_height = s5k5e9_modes[fse->index].height;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int s5k5e9_s_register(struct v4l2_subdev *subdev,
			     const struct v4l2_dbg_register *reg)
{
	struct s5k5e9 *s5k5e9 = container_of(subdev, struct s5k5e9, sd);

	return regmap_write(s5k5e9->regmap, reg->reg, reg->val);
}

static int s5k5e9_g_register(struct v4l2_subdev *subdev,
			     struct v4l2_dbg_register *reg)
{
	struct s5k5e9 *s5k5e9 = container_of(subdev, struct s5k5e9, sd);
	unsigned int aux;
	int ret;

	reg->size = 1;
	ret = regmap_read(s5k5e9->regmap, reg->reg, &aux);
	reg->val = aux;

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops s5k5e9_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = s5k5e9_g_register,
	.s_register = s5k5e9_s_register,
#endif
};

static struct v4l2_mbus_framefmt *
__s5k5e9_get_pad_format(struct s5k5e9 *s5k5e9,
			struct v4l2_subdev_state *sd_state,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_format(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &s5k5e9->fmt;
	default:
		return NULL;
	}
}

static int s5k5e9_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *format)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);

	mutex_lock(&s5k5e9->mutex);
	format->format = *__s5k5e9_get_pad_format(s5k5e9, sd_state,
						  format->pad,
						  format->which);
	mutex_unlock(&s5k5e9->mutex);

	return 0;
}

static struct v4l2_rect *
__s5k5e9_get_pad_crop(struct s5k5e9 *s5k5e9,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &s5k5e9->crop;
	default:
		return NULL;
	}
}

static int s5k5e9_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *format)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	const struct s5k5e9_mode *mode;

	mutex_lock(&s5k5e9->mutex);

	__crop = __s5k5e9_get_pad_crop(s5k5e9, sd_state, format->pad,
				       format->which);

	mode = v4l2_find_nearest_size(s5k5e9_modes,
				      ARRAY_SIZE(s5k5e9_modes), width, height,
				      format->format.width,
				      format->format.height);

	__crop->width = mode->width;
	__crop->height = mode->height;

	__format = __s5k5e9_get_pad_format(s5k5e9, sd_state, format->pad,
					   format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = S5K5E9_MBUS_CODE;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;
	__format->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(__format->colorspace);
	__format->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
				__format->colorspace, __format->ycbcr_enc);
	__format->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(__format->colorspace);

	format->format = *__format;

	mutex_unlock(&s5k5e9->mutex);

	return 0;
}

static int s5k5e9_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		mutex_lock(&s5k5e9->mutex);
		sel->r = *__s5k5e9_get_pad_crop(s5k5e9, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&s5k5e9->mutex);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = S5K5E9_NATIVE_WIDTH;
		sel->r.height = S5K5E9_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = S5K5E9_PIXEL_ARRAY_TOP;
		sel->r.left = S5K5E9_PIXEL_ARRAY_LEFT;
		sel->r.width = S5K5E9_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5K5E9_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int s5k5e9_entity_init_state(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = { };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = s5k5e9_modes[0].width;
	fmt.format.height = s5k5e9_modes[0].height;

	s5k5e9_set_format(subdev, sd_state, &fmt);

	return 0;
}

static int s5k5e9_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e9 *s5k5e9 = container_of(ctrl->handler,
					     struct s5k5e9, ctrls);
	u8 vals[2];
	int ret;

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (!pm_runtime_get_if_in_use(s5k5e9->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* TODO: Long exposure > 1s, is yet to support */
#if 0
		if(ctrl->val > 61053) {
			wr_pair[0] =
				{0x0340, ((ctrl->val + 5) >> 8 ) & 0xff }; //frame_h
			wr_pair[1] =
				{0x0341, (ctrl->val + 5) & 0xff}; //frame_l
                        wr_pair[2] =
				{0x0342, 0xFF}; //line_h
                        wr_pair[3] =
				{0x0343, 0XFC}; //line_l
                        wr_pair[4] =
				{0x0200, 0xFF};
                        wr_pair[5] =
				{0x0201, 0X70};
                        wr_pair[6] =
				{0x0202, (ctrl->val >> 8 ) & 0xff };
                        wr_pair[7] =
				{0x0203, ctrl->val & 0xff};
			};
#endif

		regmap_write(s5k5e9->regmap, 0x0340, (s5k5e9_modes[0].framelength >> 8 ) & 0xff);
		regmap_write(s5k5e9->regmap, 0x0341, s5k5e9_modes[0].framelength & 0xff);
		regmap_write(s5k5e9->regmap, 0x0342, (s5k5e9_modes[0].linelength >> 8 ) & 0xff);
		regmap_write(s5k5e9->regmap, 0x0343, s5k5e9_modes[0].linelength & 0xff);
		regmap_write(s5k5e9->regmap, 0x0200, 0xff);
		regmap_write(s5k5e9->regmap, 0x0201, 0x70);
		regmap_write(s5k5e9->regmap, 0x0202, (ctrl->val >> 8 ) & 0xff);
		ret = regmap_write(s5k5e9->regmap, 0x0203, ctrl->val & 0xff);

		if (ret < 0) {
			dev_err(s5k5e9->dev, "Error %d\n", ret);
			break;
		}

		ret = 0;
		break;

	case V4L2_CID_GAIN:
		vals[1] = (ctrl->val >> S5K5E9_REG_GAIN_BIT_SHIFT) & 0xff; /* valid bits 0x1fff*/
		vals[0] = (ctrl->val >> (8 + S5K5E9_REG_GAIN_BIT_SHIFT)) & 0xff;
		ret = regmap_bulk_write(s5k5e9->regmap, S5K5E9_REG_GAIN_H, vals, 2);
		if (ret < 0)
			dev_err(s5k5e9->dev, "Error %d\n", ret);
		ret = 0;
		break;

	case V4L2_CID_TEST_PATTERN: /* this is not the */
		vals[0] = S5K5E9_REG_TEST_PATTERN_ENABLE;
		ret = regmap_bulk_write(s5k5e9->regmap, S5K5E9_REG_TEST_PATTERN, vals, 1);
		if (ret < 0)
			dev_err(s5k5e9->dev, "Error %d\n", ret);

		vals[0] = 0;
		ret = regmap_bulk_write(s5k5e9->regmap, S5K5E9_REG_UPDATE_DUMMY, vals, 1);
		if (ret < 0)
			dev_err(s5k5e9->dev, "Error %d\n", ret);
		ret = 0;
		break;

	default:
		ret = -EINVAL;
	}

	pm_runtime_put(s5k5e9->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k5e9_ctrl_ops = {
	.s_ctrl = s5k5e9_set_ctrl,
};

static int s5k5e9_ctrls_init(struct s5k5e9 *s5k5e9)
{
	static const s64 link_freq[] = {
		S5K5E9_DEFAULT_LINK_FREQ
	};
	static const struct v4l2_area unit_size = {
		.width = 1120,
		.height = 1120,
	};
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;

	ret = v4l2_fwnode_device_parse(s5k5e9->dev, &props);
	if (ret < 0)
		return ret;

	ctrl_hdlr = &s5k5e9->ctrls;
	ret = v4l2_ctrl_handler_init(&s5k5e9->ctrls, 6);
	if (ret)
		return ret;

	s5k5e9->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, NULL,
					       V4L2_CID_PIXEL_RATE, 0,
					       S5K5E9_DEFAULT_PIXEL_RATE, 1,
					       S5K5E9_DEFAULT_PIXEL_RATE);

	s5k5e9->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq) - 1,
						   0, link_freq);
	if (s5k5e9->link_freq)
		s5k5e9->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k5e9->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e9_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K5E9_EXPOSURE_MIN,
					     S5K5E9_EXPOSURE_MAX,
					     S5K5E9_EXPOSURE_STEP,
					     S5K5E9_EXPOSURE_DEFAULT);

	s5k5e9->unit_size = v4l2_ctrl_new_std_compound(ctrl_hdlr,
				NULL,
				V4L2_CID_UNIT_CELL_SIZE,
				v4l2_ctrl_ptr_create((void *)&unit_size));

	v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k5e9_ctrl_ops, &props);

	ret = ctrl_hdlr->error;
	if (ret) {
		v4l2_ctrl_handler_free(ctrl_hdlr);
		dev_err(s5k5e9->dev, "failed to add controls: %d\n", ret);
		return ret;
	}

	s5k5e9->sd.ctrl_handler = ctrl_hdlr;

	return 0;
};

#define MAX_CMD 4
static int s5k5e9_write_table(struct s5k5e9 *s5k5e9,
			      const struct reg_8 table[])
{
	u8 vals[MAX_CMD];
	int i;
	int ret;

	for (; table->addr != S5K5E9_TABLE_END ; table++) {
		if (table->addr == S5K5E9_TABLE_WAIT_MS) {
			usleep_range(table->val * 1000,
				     table->val * 1000 + 500);
			continue;
		}

		for (i = 0; i < MAX_CMD; i++) {
			if (table[i].addr != (table[0].addr + i))
				break;
			vals[i] = table[i].val;
		}

		ret = regmap_bulk_write(s5k5e9->regmap, table->addr, vals, i);

		if (ret) {
			dev_err(s5k5e9->dev, "write_table error: %d\n", ret);
			return ret;
		}

		table += i - 1;
	}

	return 0;
}

static int s5k5e9_start_streaming(struct s5k5e9 *s5k5e9)
{
	const struct s5k5e9_mode *mode;
	int ret;

	mutex_lock(&s5k5e9->mutex);
	ret = s5k5e9_write_table(s5k5e9, mode_table_common);
	if (ret < 0) {
		dev_err(s5k5e9->dev, "could not sent common table %d\n", ret);
		goto error;
	}

	mode = v4l2_find_nearest_size(s5k5e9_modes,
				ARRAY_SIZE(s5k5e9_modes), width, height,
				s5k5e9->fmt.width, s5k5e9->fmt.height);
	ret = s5k5e9_write_table(s5k5e9, mode->reg_table);
	if (ret < 0) {
		dev_err(s5k5e9->dev, "could not sent mode table %d\n", ret);
		goto error;
	}
	ret = __v4l2_ctrl_handler_setup(&s5k5e9->ctrls);
	if (ret < 0) {
		dev_err(s5k5e9->dev, "could not sync v4l2 controls\n");
		goto error;
	}
	ret = regmap_write(s5k5e9->regmap, S5K5E9_REG_MODE_SELECT, S5K5E9_MODE_STREAMING);
	if (ret < 0) {
		dev_err(s5k5e9->dev, "could not sent start table %d\n", ret);
		goto error;
	}

	mutex_unlock(&s5k5e9->mutex);
	return 0;

error:
	mutex_unlock(&s5k5e9->mutex);
	return ret;
}

static int s5k5e9_stop_streaming(struct s5k5e9 *s5k5e9)
{
	int ret;

	ret = regmap_write(s5k5e9->regmap, S5K5E9_REG_MODE_SELECT, S5K5E9_MODE_STANDBY);
	if (ret < 0)
		dev_err(s5k5e9->dev, "could not sent stop table %d\n",	ret);

	return ret;
}

static int s5k5e9_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(subdev);
	int ret;

	if (enable) {
		ret = pm_runtime_resume_and_get(s5k5e9->dev);
		if (ret < 0)
			return ret;

		ret = s5k5e9_start_streaming(s5k5e9);
		if (ret < 0)
			goto err_rpm_put;
	} else {
		ret = s5k5e9_stop_streaming(s5k5e9);
		if (ret < 0)
			goto err_rpm_put;
		pm_runtime_put(s5k5e9->dev);
	}

	return 0;

err_rpm_put:
	pm_runtime_put(s5k5e9->dev);
	return ret;
}

static int s5k5e9_get_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_interval *fival)
{
	/*
	 * FIXME: Implement support for V4L2_SUBDEV_FORMAT_TRY, using the V4L2
	 * subdev active state API.
	 */
	if (fival->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	fival->interval.numerator = 1;
	fival->interval.denominator = S5K5E9_FPS;

	return 0;
}

static int s5k5e9_enum_frame_interval(struct v4l2_subdev *subdev,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	const struct s5k5e9_mode *mode;

	if (fie->index != 0)
		return -EINVAL;

	mode = v4l2_find_nearest_size(s5k5e9_modes,
				ARRAY_SIZE(s5k5e9_modes), width, height,
				fie->width, fie->height);

	fie->code = S5K5E9_MBUS_CODE;
	fie->width = mode->width;
	fie->height = mode->height;
	fie->interval.numerator = 1;
	fie->interval.denominator = S5K5E9_FPS;

	return 0;
}

static const struct v4l2_subdev_video_ops s5k5e9_video_ops = {
	.s_stream = s5k5e9_s_stream,
};

static const struct v4l2_subdev_pad_ops s5k5e9_subdev_pad_ops = {
	.enum_mbus_code = s5k5e9_enum_mbus_code,
	.enum_frame_size = s5k5e9_enum_frame_size,
	.enum_frame_interval = s5k5e9_enum_frame_interval,
	.get_fmt = s5k5e9_get_format,
	.set_fmt = s5k5e9_set_format,
	.get_selection = s5k5e9_get_selection,
	.get_frame_interval = s5k5e9_get_frame_interval,
	.set_frame_interval = s5k5e9_get_frame_interval,
};

static const struct v4l2_subdev_ops s5k5e9_subdev_ops = {
	.core = &s5k5e9_core_ops,
	.video = &s5k5e9_video_ops,
	.pad = &s5k5e9_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5e9_internal_ops = {
	.init_state = s5k5e9_entity_init_state,
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int s5k5e9_get_regulators(struct device *dev, struct s5k5e9 *s5k5e9)
{
	/*
	 * since in power_on() there is a caall to bulk_get_optional()
	 * then there is no need to call again and again
	*/
	return 0;
#if 0
        unsigned int i;

	for (i = 0; i < S5K5E9_NUM_SUPPLIES; i++)
		s5k5e9->supplies[i].supply = s5k5e9_supply_name[i];

	return devm_regulator_bulk_get(dev, S5K5E9_NUM_SUPPLIES,
				       s5k5e9->supplies);
#endif
}

static int s5k5e9_parse_fwnode(struct device *dev)
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
		if (bus_cfg.link_frequencies[i] == S5K5E9_DEFAULT_LINK_FREQ)
			break;

	if (i == bus_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequencies %d not supported, Please review your DT\n",
			S5K5E9_DEFAULT_LINK_FREQ);
		ret = -EINVAL;
		goto done;
	}

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	fwnode_handle_put(endpoint);
	return ret;
}

static int s5k5e9_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k5e9 *s5k5e9;
	int i, ret;

	ret = s5k5e9_parse_fwnode(dev);
	if (ret)
		return ret;

	s5k5e9 = devm_kzalloc(dev, sizeof(*s5k5e9), GFP_KERNEL);
	if (!s5k5e9)
		return -ENOMEM;

	s5k5e9->dev = dev;

	s5k5e9->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(s5k5e9->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(s5k5e9->xclk);
	}

	ret = clk_set_rate(s5k5e9->xclk, S5K5E9_DEFAULT_CLK_FREQ);
	if (ret) {
		dev_err(dev, "could not set xclk frequency\n");
		return ret;
	}

	ret = s5k5e9_get_regulators(dev, s5k5e9);
	if (ret < 0) {
		dev_err(dev, "cannot get regulators\n");
		return ret;
	}

	s5k5e9->enable_gpio = devm_gpiod_get_index_optional(dev, "enable", i, GPIOD_OUT_LOW);
	if (IS_ERR(s5k5e9->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return -EIO;
	}

	s5k5e9->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(s5k5e9->regmap)) {
		dev_err(dev, "regmap init failed\n");
		return PTR_ERR(s5k5e9->regmap);
	}

	i2c_set_clientdata(client, s5k5e9);
	v4l2_i2c_subdev_init(&s5k5e9->sd, client, &s5k5e9_subdev_ops);
	s5k5e9->sd.internal_ops = &s5k5e9_internal_ops;

	/*
	 * Enable power initially, to avoid warnings
	 * from clk_disable on power_off
	 */
	ret = s5k5e9_power_on(s5k5e9->dev);
	if (ret < 0)
		goto error_power_off;

	pm_runtime_set_active(s5k5e9->dev);
	pm_runtime_enable(s5k5e9->dev);
	pm_runtime_idle(s5k5e9->dev);

	ret = s5k5e9_ctrls_init(s5k5e9);
	if (ret < 0)
		goto error_power_off;

	mutex_init(&s5k5e9->mutex);
	s5k5e9->ctrls.lock = &s5k5e9->mutex;

	s5k5e9->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k5e9->pad.flags = MEDIA_PAD_FL_SOURCE;
	s5k5e9->sd.dev = &client->dev;
	s5k5e9->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&s5k5e9->sd.entity, 1, &s5k5e9->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	s5k5e9_entity_init_state(&s5k5e9->sd, NULL);

	ret = v4l2_async_register_subdev_sensor(&s5k5e9->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	dev_info(dev, "probe successed!");
	return 0;

free_entity:
	media_entity_cleanup(&s5k5e9->sd.entity);
free_ctrl:
	mutex_destroy(&s5k5e9->mutex);
	v4l2_ctrl_handler_free(&s5k5e9->ctrls);
error_power_off:
	pm_runtime_disable(s5k5e9->dev);

	return ret;
}

static void s5k5e9_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);

	v4l2_async_unregister_subdev(&s5k5e9->sd);
	media_entity_cleanup(&s5k5e9->sd.entity);
	v4l2_ctrl_handler_free(&s5k5e9->ctrls);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&s5k5e9->mutex);
}

static const struct of_device_id s5k5e9_of_match[] = {
	{ .compatible = "samsung,s5k5e9" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k5e9_of_match);

static const struct dev_pm_ops s5k5e9_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k5e9_power_off, s5k5e9_power_on, NULL)
};

static struct i2c_driver s5k5e9_i2c_driver = {
	.driver = {
		.of_match_table = s5k5e9_of_match,
		.pm = &s5k5e9_pm_ops,
		.name  = "s5k5e9",
	},
	.probe = s5k5e9_probe,
	.remove = s5k5e9_remove,
};

module_i2c_driver(s5k5e9_i2c_driver);

MODULE_DESCRIPTION("samsung s5k5e9 Camera driver");
MODULE_AUTHOR("Ricardo Ribalda <ribalda@kernel.org>");
MODULE_AUTHOR("99degree <https://github.com/99degree>");
MODULE_LICENSE("GPL v2");
