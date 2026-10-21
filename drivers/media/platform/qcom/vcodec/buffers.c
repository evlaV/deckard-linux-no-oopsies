// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/videodev2.h>

#include "buffers.h"

u32 video_raw_buffer_size(u32 colorformat,
			  u32 pix_width,
			  u32 pix_height)
{
	u32 size = 0;
	u32 y_plane, uv_plane, y_stride,
	uv_stride, y_sclines, uv_sclines;
	u32 y_ubwc_plane = 0, uv_ubwc_plane = 0;
	u32 y_meta_stride = 0, y_meta_scanlines = 0;
	u32 uv_meta_stride = 0, uv_meta_scanlines = 0;
	u32 y_meta_plane = 0, uv_meta_plane = 0;

	if (!pix_width || !pix_height)
		goto invalid_input;

	switch (colorformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		y_stride = ALIGN(pix_width, 128);
		uv_stride = ALIGN(pix_width, 128);
		y_sclines = ALIGN(pix_height, 32);
		uv_sclines = ALIGN((pix_height + 1) >> 1, 16);
		y_plane = y_stride * y_sclines;
		uv_plane = uv_stride * uv_sclines;
		size = y_plane + uv_plane;
		break;
	case V4L2_PIX_FMT_QC08C:
		y_stride = ALIGN(pix_width, 128);
		uv_stride = ALIGN(pix_width, 128);
		y_sclines = ALIGN(pix_height, 32);
		uv_sclines = ALIGN((pix_height + 1) >> 1, 32);
		y_meta_stride = ALIGN(DIV_ROUND_UP(pix_width, 32), 64);
		uv_meta_stride = ALIGN(DIV_ROUND_UP((pix_width + 1) >> 1, 16), 64);
		y_ubwc_plane =
			ALIGN(y_stride * y_sclines, 4096);
		uv_ubwc_plane =
			ALIGN(uv_stride * uv_sclines, 4096);
		y_meta_scanlines =
			ALIGN(DIV_ROUND_UP(pix_height, 8), 16);
		y_meta_plane =
			ALIGN(y_meta_stride * y_meta_scanlines, 4096);
		uv_meta_scanlines =
			ALIGN(DIV_ROUND_UP((pix_height + 1) >> 1, 8), 16);
		uv_meta_plane =
			ALIGN(uv_meta_stride * uv_meta_scanlines, 4096);
		size = (y_ubwc_plane + uv_ubwc_plane + y_meta_plane +
			uv_meta_plane);
		break;
	case V4L2_PIX_FMT_QC10C:
		y_stride =
			ALIGN(ALIGN(pix_width, 192) * 4 / 3, 256);
		uv_stride =
			ALIGN(ALIGN(pix_width, 192) * 4 / 3, 256);
		y_sclines =
			ALIGN(pix_height, 16);
		uv_sclines =
			ALIGN((pix_height + 1) >> 1, 16);
		y_ubwc_plane =
			ALIGN(y_stride * y_sclines, 4096);
		uv_ubwc_plane =
			ALIGN(uv_stride * uv_sclines, 4096);
		y_meta_stride =
			ALIGN(DIV_ROUND_UP(pix_width, 48), 64);
		y_meta_scanlines =
			ALIGN(DIV_ROUND_UP(pix_height, 4), 16);
		y_meta_plane =
			ALIGN(y_meta_stride * y_meta_scanlines, 4096);
		uv_meta_stride =
			ALIGN(DIV_ROUND_UP((pix_width + 1) >> 1, 24), 64);
		uv_meta_scanlines =
			ALIGN(DIV_ROUND_UP((pix_height + 1) >> 1, 4), 16);
		uv_meta_plane =
			ALIGN(uv_meta_stride * uv_meta_scanlines, 4096);

		size = y_ubwc_plane + uv_ubwc_plane + y_meta_plane +
			uv_meta_plane;
		break;
	case V4L2_PIX_FMT_P010:
		y_stride = ALIGN(pix_width * 2, 128);
		uv_stride = ALIGN(pix_width * 2, 128);
		y_sclines = ALIGN(pix_height, 32);
		uv_sclines = ALIGN((pix_height + 1) >> 1, 16);
		y_plane = y_stride * y_sclines;
		uv_plane = uv_stride * uv_sclines;

		size = y_plane + uv_plane;
		break;
	default:
		break;
	}

invalid_input:

	return ALIGN(size, 4096);
}
