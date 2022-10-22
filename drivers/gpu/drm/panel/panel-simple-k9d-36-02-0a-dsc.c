// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2013, The Linux Foundation. All rights reserved.

static const struct drm_display_mode k9d_36_02_0a_dsc_mode = {
	.clock = (1080 + 16 + 8 + 8) * (2400 + 1212 + 4 + 8) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 16,
	.hsync_end = 1080 + 16 + 8,
	.htotal = 1080 + 16 + 8 + 8,
	.vdisplay = 2400,
	.vsync_start = 2400 + 1212,
	.vsync_end = 2400 + 1212 + 4,
	.vtotal = 2400 + 1212 + 4 + 8,
	.width_mm = 683,
	.height_mm = 1517,
};

static const struct panel_desc_dsi k9d_36_02_0a_dsc = {
	.desc = {
		.modes = &k9d_36_02_0a_dsc_mode,
		.num_modes = 1,
		.bpc = 10,
		.size = {
			.width = 683,
			.height = 1517,
		},
		.connector_type = DRM_MODE_CONNECTOR_DSI,
	},
	.flags = MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_CLOCK_NON_CONTINUOUS |
		 MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};
