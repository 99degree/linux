// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2022 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct s6d6ft0_tianma_fhd {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline
struct s6d6ft0_tianma_fhd *to_s6d6ft0_tianma_fhd(struct drm_panel *panel)
{
	return container_of(panel, struct s6d6ft0_tianma_fhd, panel);
}

#define dsi_dcs_write_seq(dsi, seq...) do {				\
		static const u8 d[] = { seq };				\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static void s6d6ft0_tianma_fhd_reset(struct s6d6ft0_tianma_fhd *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(40);
}

static int s6d6ft0_tianma_fhd_on(struct s6d6ft0_tianma_fhd *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi_dcs_write_seq(dsi, 0x9f, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	dsi_dcs_write_seq(dsi, 0x55);
	dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	dsi_dcs_write_seq(dsi, 0x73, 0x94);
	dsi_dcs_write_seq(dsi, 0xea,
			  0x00, 0x73, 0x11, 0x1b, 0x23, 0x2a, 0x40, 0x59, 0x70,
			  0x6e, 0xa1, 0x86, 0x92, 0x9f, 0xab, 0xb8, 0x4d, 0x59,
			  0x65, 0x7f, 0x00, 0x73, 0x11, 0x1b, 0x23, 0x2a, 0x40,
			  0x59, 0x70, 0x6e, 0xa1, 0x86, 0x92, 0x9f, 0xab, 0xb8,
			  0x4d, 0x59, 0x65, 0x7f, 0x00, 0x73, 0x11, 0x1b, 0x23,
			  0x2a, 0x40, 0x59, 0x70, 0x6e, 0xa1, 0x86, 0x92, 0x9f,
			  0xab, 0xb8, 0x4d, 0x59, 0x65, 0x7f);
	dsi_dcs_write_seq(dsi, 0xeb,
			  0x00, 0x73, 0x11, 0x1b, 0x23, 0x2a, 0x40, 0x59, 0x70,
			  0x6e, 0xa1, 0x86, 0x92, 0x9f, 0xab, 0xb8, 0x4d, 0x59,
			  0x65, 0x7f, 0x00, 0x73, 0x11, 0x1b, 0x23, 0x2a, 0x40,
			  0x59, 0x70, 0x6e, 0xa1, 0x86, 0x92, 0x9f, 0xab, 0xb8,
			  0x4d, 0x59, 0x65, 0x7f, 0x00, 0x73, 0x11, 0x1b, 0x23,
			  0x2a, 0x40, 0x59, 0x70, 0x6e, 0xa1, 0x86, 0x92, 0x9f,
			  0xab, 0xb8, 0x4d, 0x59, 0x65, 0x7f);
	dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	dsi_dcs_write_seq(dsi, 0x9f, 0x5a, 0x5a);

	return 0;
}

static int s6d6ft0_tianma_fhd_off(struct s6d6ft0_tianma_fhd *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi_dcs_write_seq(dsi, 0x9f, 0xa5, 0xa5);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(80);

	dsi_dcs_write_seq(dsi, 0xf0, 0x5a, 0x5a);
	dsi_dcs_write_seq(dsi, 0x73, 0x90);
	dsi_dcs_write_seq(dsi, 0xf0, 0xa5, 0xa5);
	dsi_dcs_write_seq(dsi, 0x24);
	dsi_dcs_write_seq(dsi, 0x9f, 0x5a, 0x5a);

	return 0;
}

static int s6d6ft0_tianma_fhd_prepare(struct drm_panel *panel)
{
	struct s6d6ft0_tianma_fhd *ctx = to_s6d6ft0_tianma_fhd(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	s6d6ft0_tianma_fhd_reset(ctx);

	ret = s6d6ft0_tianma_fhd_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int s6d6ft0_tianma_fhd_unprepare(struct drm_panel *panel)
{
	struct s6d6ft0_tianma_fhd *ctx = to_s6d6ft0_tianma_fhd(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = s6d6ft0_tianma_fhd_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode s6d6ft0_tianma_fhd_mode = {
	.clock = (1080 + 229 + 4 + 4) * (2160 + 8 + 2 + 6) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 229,
	.hsync_end = 1080 + 229 + 4,
	.htotal = 1080 + 229 + 4 + 4,
	.vdisplay = 2160,
	.vsync_start = 2160 + 8,
	.vsync_end = 2160 + 8 + 2,
	.vtotal = 2160 + 8 + 2 + 6,
	.width_mm = 0,
	.height_mm = 0,
};

static int s6d6ft0_tianma_fhd_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &s6d6ft0_tianma_fhd_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs s6d6ft0_tianma_fhd_panel_funcs = {
	.prepare = s6d6ft0_tianma_fhd_prepare,
	.unprepare = s6d6ft0_tianma_fhd_unprepare,
	.get_modes = s6d6ft0_tianma_fhd_get_modes,
};

static int s6d6ft0_tianma_fhd_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6d6ft0_tianma_fhd *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

	drm_panel_init(&ctx->panel, dev, &s6d6ft0_tianma_fhd_panel_funcs,
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

	return 0;
}

static void s6d6ft0_tianma_fhd_remove(struct mipi_dsi_device *dsi)
{
	struct s6d6ft0_tianma_fhd *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

}

static const struct of_device_id s6d6ft0_tianma_fhd_of_match[] = {
	{ .compatible = "mdss,s6d6ft0-tianma-fhd" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6d6ft0_tianma_fhd_of_match);

static struct mipi_dsi_driver s6d6ft0_tianma_fhd_driver = {
	.probe = s6d6ft0_tianma_fhd_probe,
	//.remove = s6d6ft0_tianma_fhd_remove,
	.driver = {
		.name = "panel-s6d6ft0-tianma-fhd",
		.of_match_table = s6d6ft0_tianma_fhd_of_match,
	},
};
module_mipi_dsi_driver(s6d6ft0_tianma_fhd_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for s6d6ff0 tianma fhd video mode dsi panel");
MODULE_LICENSE("GPL v2");
