// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2022 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define NT36675_VREG_MAX                3

struct nt36675_tianma {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data vregs[NT36675_VREG_MAX];
	bool prepared;
};

static inline struct nt36675_tianma *to_nt36675_tianma(struct drm_panel *panel)
{
	return container_of(panel, struct nt36675_tianma, panel);
}

#ifdef ENABLE_REGULATOR
static int nt36675_tianma_init_vregs(struct nt36675_tianma *nt, struct device *dev)
{
        int ret;

        nt->vregs[0].supply = "vddio";
        nt->vregs[1].supply = "lab";
        nt->vregs[2].supply = "ibb";
        ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(nt->vregs),
                                      nt->vregs);
        if (ret < 0)
                return ret;

	/* vddio */
        ret = regulator_is_supported_voltage(nt->vregs[0].consumer,
                                             1800000, 1904000);
        if (!ret)
                return -EINVAL;

	/* lab */
        ret = regulator_is_supported_voltage(nt->vregs[1].consumer,
                                             4600000, 6000000);
        if (!ret)
                return -EINVAL;

	/* ibb */
        ret = regulator_is_supported_voltage(nt->vregs[2].consumer,
                                             4600000, 6000000);
        if (!ret)
                return -EINVAL;

        return 0;
}
#endif

#define dsi_dcs_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static void nt36675_tianma_reset(struct nt36675_tianma *ctx)
{
//	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
//	usleep_range(10000, 11000);
//	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
//	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
        gpiod_set_value_cansleep(ctx->reset_gpio, 0);
        usleep_range(10000, 11000);
}

static int nt36675_tianma_on(struct nt36675_tianma *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	dsi_dcs_write_seq(dsi, 0xff, 0x10);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	dsi_dcs_write_seq(dsi, 0x3b, 0x03, 0x1e, 0x0a, 0x04, 0x04);
	dsi_dcs_write_seq(dsi, 0xb0, 0x00);

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear on: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x00b8);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);
	dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	dsi_dcs_write_seq(dsi, 0xff, 0x27);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	dsi_dcs_write_seq(dsi, 0x07, 0x01);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_VSYNC_TIMING, 0x25);
	dsi_dcs_write_seq(dsi, 0xff, 0x23);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	dsi_dcs_write_seq(dsi, 0x0a, 0x20);
	dsi_dcs_write_seq(dsi, 0x0b, 0x20);
	dsi_dcs_write_seq(dsi, 0x0c, 0x20);
	dsi_dcs_write_seq(dsi, 0x0d, 0x2a);
	dsi_dcs_write_seq(dsi, 0x10, 0x50);
	dsi_dcs_write_seq(dsi, 0x11, 0x01);
	dsi_dcs_write_seq(dsi, 0x12, 0x95);
	dsi_dcs_write_seq(dsi, 0x15, 0x68);
	dsi_dcs_write_seq(dsi, 0x16, 0x0b);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_ROWS, 0xff);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_PARTIAL_COLUMNS, 0xff);
	dsi_dcs_write_seq(dsi, 0x32, 0xff);
	dsi_dcs_write_seq(dsi, 0x33, 0xfe);
	dsi_dcs_write_seq(dsi, 0x34, 0xfd);
	dsi_dcs_write_seq(dsi, 0x35, 0xfa);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_ADDRESS_MODE, 0xf6);
	dsi_dcs_write_seq(dsi, 0x37, 0xf2);
	dsi_dcs_write_seq(dsi, 0x38, 0xf0);
	dsi_dcs_write_seq(dsi, 0x39, 0xee);

	ret = mipi_dsi_dcs_set_pixel_format(dsi, 0xec);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0x3b, 0xea);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_3D_CONTROL, 0xe8);
	dsi_dcs_write_seq(dsi, 0x3f, 0xe7);
	dsi_dcs_write_seq(dsi, MIPI_DCS_SET_VSYNC_TIMING, 0xe6);
	dsi_dcs_write_seq(dsi, 0x41, 0xe5);
	dsi_dcs_write_seq(dsi, 0xa0, 0x11);
	dsi_dcs_write_seq(dsi, 0xff, 0x10);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(80);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	usleep_range(5000, 6000);

	dsi_dcs_write_seq(dsi, 0xff, 0x27);
	dsi_dcs_write_seq(dsi, 0xfb, 0x01);
	dsi_dcs_write_seq(dsi, 0x3f, 0x01);
	dsi_dcs_write_seq(dsi, 0x43, 0x08);
	dsi_dcs_write_seq(dsi, 0xff, 0x10);

	return 0;
}

static int nt36675_tianma_off(struct nt36675_tianma *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	dsi_dcs_write_seq(dsi, 0xff, 0x10);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(140);

	return 0;
}

static int nt36675_tianma_prepare(struct drm_panel *panel)
{
	struct nt36675_tianma *ctx = to_nt36675_tianma(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

#ifdef ENABLE_REGULATOR
	//TODO: change value
        ret = regulator_enable(ctx->vregs[0].consumer);
        if (ret)
                return ret;
        usleep_range(2000, 5000);

        ret = regulator_enable(ctx->vregs[1].consumer);
        if (ret)
                goto end;

        ret = regulator_enable(ctx->vregs[2].consumer);
        if (ret)
                goto end;
#endif
	nt36675_tianma_reset(ctx);

	ret = nt36675_tianma_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	ctx->prepared = true;
	return 0;

end:
	regulator_disable(ctx->vregs[0].consumer);
	return ret;
}

static int nt36675_tianma_unprepare(struct drm_panel *panel)
{
	struct nt36675_tianma *ctx = to_nt36675_tianma(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = nt36675_tianma_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode nt36675_tianma_mode = {
	.clock = (1080 + 20 + 4 + 22) * (2400 + 10 + 2 + 30) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 20,
	.hsync_end = 1080 + 20 + 4,
	.htotal = 1080 + 20 + 4 + 22,
	.vdisplay = 2400,
	.vsync_start = 2400 + 10,
	.vsync_end = 2400 + 10 + 2,
	.vtotal = 2400 + 10 + 2 + 30,
	.width_mm = 69,
	.height_mm = 154,
};

static int nt36675_tianma_get_modes(struct drm_panel *panel,
				    struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &nt36675_tianma_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs nt36675_tianma_panel_funcs = {
	.prepare = nt36675_tianma_prepare,
	.unprepare = nt36675_tianma_unprepare,
	.get_modes = nt36675_tianma_get_modes,
};

static int nt36675_tianma_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt36675_tianma *ctx;
	int ret;

	dev_info(dev, "%s", __func__);
	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
#ifdef ENABLE_REGULATOR
        ret = nt36675_tianma_init_vregs(ctx, dev);
        if (ret)
                return dev_err_probe(dev, ret, "Regulator init failure.\n");
#endif
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &nt36675_tianma_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	pr_info("nt36675_tianma_probe ok!\n");
	return 0;
}

static void nt36675_tianma_remove(struct mipi_dsi_device *dsi)
{
	struct nt36675_tianma *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

}

static const struct of_device_id nt36675_tianma_of_match[] = {
	{ .compatible = "mdss,nt36675-tianma" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt36675_tianma_of_match);

static struct mipi_dsi_driver nt36675_tianma_driver = {
	.probe = nt36675_tianma_probe,
	.remove = nt36675_tianma_remove,
	.driver = {
		.name = "panel-nt36675-tianma",
		.of_match_table = nt36675_tianma_of_match,
	},
};
module_mipi_dsi_driver(nt36675_tianma_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for nt36675 video mode dsi tianma panel");
MODULE_LICENSE("GPL v2");
