// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2013, The Linux Foundation. All rights reserved.

static const struct drm_display_mode nt36675_tianma_mode_simple = {
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

static const struct panel_desc_dsi nt36675_tianma_simple = {
	.desc = {
		.modes = &nt36675_tianma_mode_simple,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 69,
			.height = 154,
		},
		.connector_type = DRM_MODE_CONNECTOR_DSI,
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};
