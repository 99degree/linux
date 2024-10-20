// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct ft8756_huaxing {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

struct ft8756_match_data {
	int parse_cmdline;
};

static inline struct ft8756_huaxing *to_ft8756_huaxing(struct drm_panel *panel)
{
	return container_of(panel, struct ft8756_huaxing, panel);
}

static void ft8756_huaxing_reset(struct ft8756_huaxing *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(4000, 5000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int ft8756_huaxing_on(struct ft8756_huaxing *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x87, 0x56, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x87, 0x56);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0xca,
			       0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			       0x80, 0x80, 0x80, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x90);
	mipi_dsi_dcs_write_seq(dsi, 0xca,
			       0xfe, 0xff, 0x66, 0xf6, 0xff, 0x66, 0xfb, 0xff,
			       0x32);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x00b8);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0xb5);
	mipi_dsi_dcs_write_seq(dsi, 0xca, 0x04);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(90);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	usleep_range(5000, 6000);

	return 0;
}

static int ft8756_huaxing_off(struct ft8756_huaxing *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

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
	msleep(150);

	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0xf7, 0x5a, 0xa5, 0x95, 0x27);

	return 0;
}

static int ft8756_huaxing_prepare(struct drm_panel *panel)
{
	struct ft8756_huaxing *ctx = to_ft8756_huaxing(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ft8756_huaxing_reset(ctx);

	ret = ft8756_huaxing_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int ft8756_huaxing_unprepare(struct drm_panel *panel)
{
	struct ft8756_huaxing *ctx = to_ft8756_huaxing(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = ft8756_huaxing_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode ft8756_huaxing_mode = {
	.clock = (1080 + 20 + 4 + 20) * (2400 + 8 + 4 + 32) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 20,
	.hsync_end = 1080 + 20 + 4,
	.htotal = 1080 + 20 + 4 + 20,
	.vdisplay = 2400,
	.vsync_start = 2400 + 8,
	.vsync_end = 2400 + 8 + 4,
	.vtotal = 2400 + 8 + 4 + 32,
	.width_mm = 69,
	.height_mm = 154,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ft8756_huaxing_get_modes(struct drm_panel *panel,
				    struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ft8756_huaxing_mode);
}

static const struct drm_panel_funcs ft8756_huaxing_panel_funcs = {
	.prepare = ft8756_huaxing_prepare,
	.unprepare = ft8756_huaxing_unprepare,
	.get_modes = ft8756_huaxing_get_modes,
};

static int ft8756_huaxing_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	static const struct ft8756_match_data *match_data;
	struct ft8756_huaxing *ctx;
	int ret;

	match_data = of_device_get_match_data(dev);
        if (match_data && match_data->parse_cmdline) {
                char *path = "/chosen";
                struct device_node *dt_node;
                const char *bootargs;

                dt_node = of_find_node_by_path(path);
                if (!dt_node) {
                        dev_err(dev, "Failed to find device-tree node: %s\n", path);
                        return -ENODEV;
                }

                if (!of_property_read_string(dt_node, "bootargs", &bootargs))
                        if (!strstr(bootargs, "ft87"))
				return -ENODEV;

		dev_info(dev, "Found focaltech/huaxing panel as specified in chosen/bootargs.");
	}

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
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	drm_panel_init(&ctx->panel, dev, &ft8756_huaxing_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void ft8756_huaxing_remove(struct mipi_dsi_device *dsi)
{
	struct ft8756_huaxing *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct ft8756_match_data cmdline_data = {
        .parse_cmdline = true,
};

static const struct of_device_id ft8756_huaxing_of_match[] = {
	{ .compatible = "mdss,ft8756-huaxing", .data = &cmdline_data }, // FIXME
	{ .compatible = "focaltech,ft8756-huaxing", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ft8756_huaxing_of_match);

static struct mipi_dsi_driver ft8756_huaxing_driver = {
	.probe = ft8756_huaxing_probe,
	.remove = ft8756_huaxing_remove,
	.driver = {
		.name = "panel-ft8756-huaxing",
		.of_match_table = ft8756_huaxing_of_match,
	},
};
module_mipi_dsi_driver(ft8756_huaxing_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for ft8756 video mode dsi huaxing panel");
MODULE_LICENSE("GPL");
