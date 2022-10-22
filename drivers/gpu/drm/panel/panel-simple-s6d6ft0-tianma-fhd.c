// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2013, The Linux Foundation. All rights reserved.

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

static const struct panel_desc_dsi s6d6ft0_tianma_fhd = {
	.desc = {
		.modes = &s6d6ft0_tianma_fhd_mode,
		.num_modes = 1,
		.bpc = 8,
		.size = {
			.width = 0,
			.height = 0,
		},
		.connector_type = DRM_MODE_CONNECTOR_DSI,
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};
