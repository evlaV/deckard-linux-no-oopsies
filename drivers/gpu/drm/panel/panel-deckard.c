// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023-2024, The Valve Ltd. All rights reserved.
 * Copyright (c) 2024, The Linaro Ltd. All rights reserved.
 */

#include <linux/backlight.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

//#define NO_DSC

/* panel specific MIPI DSI CMD */
#define LPM02648C_MF_CMD_ACCESS_PROTECT		0xb0
#define LPM02648C_SEQ_CTL			0xd6
#define LPM02648C_DSI_CTL			0xb6
#define LPM02648C_DISP_MODE			0xb7
#define LPM02648C_GEN_OUTPIN_SET		0xb9
#define LPM02648C_DISP_SET1			0xc0
#define LPM02648C_DISP_SET2			0xf1
#define LPM02648C_DISP_SET3			0xc6
#define LPM02648C_DISP_SET3_2			0xcd
#define LPM02648C_DISP_SET4			0xcf
#define LPM02648C_DISP_SET5			0xec
#define LPM02648C_DISP_SET6			0xef
#define LPM02648C_TE_GPIO_CTL			0xbe
#define LPM02648C_PPS_SET			0xe6
#define LPM02648C_MFG_INFO			0xbf
#define 	VENDOR_OFFSET			27
#define 	VCOM120_OFFSET			13
#define 	VCOM90_OFFSET			15
#define LPM02648C_GAMMA_SET			0xc7
#define 	GAMMA_SET_NUM_BYTES		76
#define 	GAMMA_VGMN_BASE			19
#define 	GAMMA_CODE_MAX			0x3ff

// Highest gamma_black_mode. 0 is factory gamma, 1 is the fitted near-black curve (BOE panels only).
#define GAMMA_BLACK_MODE_MAX		1

// Per-eye shift of node 17, in codes, from two testers matching the eyes by eye.
#define GAMMA_BLACK_MODE_BIAS_LEFT	(-12)
#define GAMMA_BLACK_MODE_BIAS_RIGHT	2

// Fixed point: Q62 for q and q^t, Q40 for q^t inside the fit, Q32 for codes, A and C.
#define GAMMA_BLACK_MODE_Q_FRAC		62
#define GAMMA_BLACK_MODE_Z_FRAC		40
#define GAMMA_BLACK_MODE_FRAC		32

// q = exp(-B) for the fit's B grid: B = 1e-4 + i * (0.2 - 1e-4) / 2000, i = 0..2000.
#define GAMMA_BLACK_MODE_GRID_STEPS	2000
#define GAMMA_BLACK_MODE_Q_FIRST	0x3ffe5c972fa63a06ULL	// exp(-1e-4)
#define GAMMA_BLACK_MODE_Q_RATIO	0x3ffe5cccde2bb286ULL	// exp(-(0.2 - 1e-4) / 2000)

// Brightness settings
#define BRIGHTNESS_DEFAULT 256
#define BRIGHTNESS_MAX_120 459

// Panel settings
#define VFP		700
#define VID_VS_DELAY	1112
#define VBP		23
#define GPO1_TES1_144_SHARP	150
#define GPO1_TES1_120_SHARP	100
#define GPO1_TES1_108_SHARP	75
#define GPO1_TES1_144_BOE	3435
#define GPO1_TES1_120_BOE	4068
#define GPO1_TES1_108_BOE	4619
#define GPO1_TES1_144_JDI	GPO1_TES1_144_BOE
#define GPO1_TES1_120_JDI	GPO1_TES1_120_BOE
#define GPO1_TES1_96		5290
#define GPO1_TES1_90		5446
#define GPO1_TEW1			0
#define RTN 59

// Panel Vendor types
#define VENDOR_SHARP	'S'
#define VENDOR_JDI		'J'
#define VENDOR_BOE		'B'

/*
 * Camera capture signal on panel tester is GPIO2 routed out TE pin
 * Need 100usec of offest on either side, so we use 50 lines at
 * 2usec / internal line time
 */
#define CAMERA_TRIGGER_OFFSET 50

static const char * const regulator_names[] = {
	"vdda",
};

static const char * const preboost_driver_names[] = {
	"preboost",
};

static const char * const backlight_driver_names[] = {
	"backlight-left",
	"backlight-right",
};

static const char * const lcd_bias_names[] = {
	"left-pos",
	"left-neg",
	"right-pos",
	"right-neg"
};

typedef enum
{
	LEFT_POS = 0,
	LEFT_NEG,
	RIGHT_POS,
	RIGHT_NEG,
} lpm026m648c_bias_t;

#define PANEL_MFG_INFO_NUM_BYTES 40

typedef enum
{
	MODE_120 = 0,
    MODE_144,
	MODE_108,
	MODE_96,
	MODE_90,
	MODE_80,
	MODE_72,
} lpm026m648c_modes_t;

struct lpm026m648c_ctx {
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data power_supplies[ARRAY_SIZE(regulator_names)];
	struct regulator_bulk_data preboost_drivers[ARRAY_SIZE(preboost_driver_names)];
	struct regulator_bulk_data backlight_drivers[ARRAY_SIZE(backlight_driver_names)];
	struct regulator_bulk_data lcd_bias_regulators[ARRAY_SIZE(lcd_bias_names)];

	struct gpio_descs *reset_gpios;
	struct gpio_desc *enp_gpio;
	struct gpio_desc *enn_gpio;

	struct backlight_device *backlight;
	struct mipi_dsi_device *dsi[2];
	u8 panel_mfg_info[PANEL_MFG_INFO_NUM_BYTES*2];
	u8 panel_read_data[512];
	int panel_read_id, panel_read_reg_addr, panel_read_num_bytes;

	struct drm_dsc_config dsc_cfg;
	bool is_dual_panel;
	bool dphy;
	bool low_fps_support;
	bool prepared;
	bool enabled;

	char panel_vendor;

	uint16_t vcom120[2];
	uint16_t vcom90[2];

	// Serializes gamma writes from sysfs against prepare/unprepare.
	struct mutex gamma_lock;
	unsigned int gamma_black_mode;
	// Cleared before power off, so sysfs can't write gamma to panels that are shutting down.
	bool gamma_safe_to_write;
	bool factory_gamma_valid[2];
	u8 factory_gamma[2][GAMMA_SET_NUM_BYTES];

	lpm026m648c_modes_t current_mode;

	struct delayed_work check_display_power;
};

static inline struct lpm026m648c_ctx *panel_to_ctx(struct drm_panel *panel)
{
	return container_of(panel, struct lpm026m648c_ctx, panel);
}

// Display modes are set based on a 701361 kHz pixel clock.  This allows
// use to achieve 7 digit precision for integer refresh rates.
// If we wanted exact integer refresh rates, we could bump the pixel clock to
// 720576 kHz.

static const struct drm_display_mode modes[7] = {
	{
		.name = "  2160x2160_120",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 447 + 1 + 20) * 120 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 447,
		.vsync_end = 2160 + 447 + 1,
		.vtotal = 2160 + 447 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,

	},
	{
		.name = "  2160x2160_144",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 9 + 1 + 20) * 144 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 9,
		.vsync_end = 2160 + 9 + 1,
		.vtotal = 2160 + 9 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,

	},
	{
		.name = "  2160x2160_108",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 739 + 1 + 20) * 108 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 739,
		.vsync_end = 2160 + 739 + 1,
		.vtotal = 2160 + 739 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,
	},
	{
		.name = "  2160x2160_96",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 1104.0 + 1 + 20) * 96 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 1104.0,
		.vsync_end = 2160 + 1104.0 + 1,
		.vtotal = 2160 + 1104.0 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,
	},
	{
		.name = "  2160x2160_90",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 1323 + 1 + 20) * 90 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 1323,
		.vsync_end = 2160 + 1323 + 1,
		.vtotal = 2160 + 1323 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,
	},
	{
		.name = "  2160x2160_80",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 1761 + 1 + 20) * 80 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 1761,
		.vsync_end = 2160 + 1761 + 1,
		.vtotal = 2160 + 1761 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,
	},
	{
		.name = "  2160x2160_72",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 2199 + 1 + 20) * 72 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 2199,
		.vsync_end = 2160 + 2199 + 1,
		.vtotal = 2160 + 2199 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,
	},
};

#define lpm026m648c_dsi_write_seq(ctx, cmd, seq...)					\
	do {										\
		u8 d[] = {cmd, seq};							\
		struct mipi_dsi_multi_context dsi_ctx0 = { .dsi = ctx->dsi[0] };	\
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx0, d, ARRAY_SIZE(d));		\
		if (ctx->is_dual_panel) {						\
			struct mipi_dsi_multi_context dsi_ctx1 = { .dsi = ctx->dsi[1] };\
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx1, d, ARRAY_SIZE(d));	\
		}									\
	} while (0)

#define lpm026m648c_dsi_write_buffer(ctx, d)						\
	do {										\
		struct mipi_dsi_multi_context dsi_ctx0 = { .dsi = ctx->dsi[0] };	\
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx0, d, ARRAY_SIZE(d));		\
		if (ctx->is_dual_panel) {						\
			struct mipi_dsi_multi_context dsi_ctx1 = { .dsi = ctx->dsi[1] };\
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx1, d, ARRAY_SIZE(d));	\
		}									\
	} while (0)

#define BE16(x) (x)>>8, (x)&0xff
#define LE16(x) (x)&0xff, (x)>>8

static int dsi_populate_dsc_params(struct lpm026m648c_ctx *ctx)
{
	int ret;
	struct drm_dsc_config *dsc = &ctx->dsc_cfg;

	dsc->simple_422 = 0;
	dsc->convert_rgb = 1;
	dsc->vbr_enable = 0;

	drm_dsc_set_const_params(dsc);
	drm_dsc_set_rc_buf_thresh(dsc);

	/* handle only bpp = bpc = 8, pre-SCR panels */
	ret = drm_dsc_setup_rc_params(dsc, DRM_DSC_1_1_PRE_SCR);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev, "failed to setup dsc params\n");

	dsc->initial_scale_value = drm_dsc_initial_scale_value(dsc);
	dsc->line_buf_depth = dsc->bits_per_component + 1;

	return drm_dsc_compute_rc_parameters(dsc);
}

static int lpm026m648c_init_dsc_config(struct lpm026m648c_ctx *ctx)
{
	ctx->dsc_cfg = (struct drm_dsc_config) {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x2,
		.slice_height = 48,
		.slice_width = 540,
		.slice_count = 4,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,

		.pic_width = 2160,
		.pic_height = 2160,
	};

	return dsi_populate_dsc_params(ctx);
}

// should be called after get_panel_info
static int update_vcom(struct lpm026m648c_ctx *ctx)
{
	u16 vcom_left, vcom_right;
	u8 payload[3] = {0xd5, 0, 0};
	u8 wpc_unlock[2] = {0xe5, 0x07};
	u8 wpc_lock[2] = {0xe5, 0x00};
	int ret = 0;
	// TODO interpolate for other modes?
	if ( ctx->current_mode == MODE_108 || ctx->current_mode == MODE_120 || ctx->current_mode == MODE_144 ) {
		vcom_left = ctx->vcom120[0];
		vcom_right = ctx->vcom120[1];
	} else {
		// 90/80/72 use same settings
		vcom_left = ctx->vcom90[0];
		vcom_right = ctx->vcom90[1];
	}

	if ( vcom_left ) {
		payload[1] = ( vcom_left & 0xFF00 ) >> 8;
		payload[2] = vcom_left & 0x00FF;
		ret |= mipi_dsi_generic_write(ctx->dsi[0], wpc_unlock, 2);
		ret |= mipi_dsi_generic_write(ctx->dsi[0], payload, 3);
		ret |= mipi_dsi_generic_write(ctx->dsi[0], wpc_lock, 2);
	}
	if ( vcom_right ) {
		payload[1] = ( vcom_right & 0xFF00 ) >> 8;
		payload[2] = vcom_right & 0x00FF;
		ret |= mipi_dsi_generic_write(ctx->dsi[1], wpc_unlock, 2);
		ret |= mipi_dsi_generic_write(ctx->dsi[1], payload, 3);
		ret |= mipi_dsi_generic_write(ctx->dsi[1], wpc_lock, 2);
	}
	if ( ret < 0 ) {
		DRM_DEV_ERROR(ctx->dev, "Failed to set vcom ( %d )\n", ret);
	}
	return ret;
}

static int get_gamma_code(const u8 *gamma, int node)
{
	return get_unaligned_be16(&gamma[2 * node]);
}

static void set_gamma_code(u8 *gamma, int node, int code)
{
	put_unaligned_be16(code, &gamma[2 * node]);
}

// Datasheet gray level of each node, and the offset that makes gamma voltage linear in (code + offset).
static const int gamma_node_gray[19] = {
	0, 4, 15, 27, 43, 67, 91, 111, 119, 128, 152, 176, 192, 212, 228, 243, 250, 254, 255
};
static const int gamma_node_offset[19] = {
	0, 0, 0, 0, 256, 256, 256, 512, 512, 512, 512, 769, 769, 769, 1025, 1025, 1025, 1025, 1025
};

// Voltage must fall at every node from 0 to 18, which means (code + offset) must rise.
static bool gamma_curve_is_valid(const u8 *gamma, int base)
{
	int node;

	for (node = 0; node < 18; node++) {
		if (get_gamma_code(gamma, base + node) + gamma_node_offset[node] >=
		    get_gamma_code(gamma, base + node + 1) + gamma_node_offset[node + 1])
			return false;
	}
	return true;
}

// The fit needs 128-bit integers; without them the panels stay at factory gamma.
#ifdef __SIZEOF_INT128__
struct gamma_black_mode_fit {
	s64 A, C;
	s128 sse, slope;
};

static u64 mul_q62(u64 a, u64 b)
{
	return ((u128)a * b + (1ULL << (GAMMA_BLACK_MODE_Q_FRAC - 1))) >> GAMMA_BLACK_MODE_Q_FRAC;
}

// Every t is an integer, so exp(-B * t) = q^t with q = exp(-B), and no exp() is needed.
static u64 pow_q62(u64 q, int t)
{
	u64 r = 1ULL << GAMMA_BLACK_MODE_Q_FRAC;

	for (; t; t >>= 1) {
		if (t & 1)
			r = mul_q62(r, q);
		q = mul_q62(q, q);
	}
	return r;
}

// q^t for a node, in Q40.
static s64 gamma_black_mode_z(u64 q, int node)
{
	return pow_q62(q, 255 - gamma_node_gray[node]) >> (GAMMA_BLACK_MODE_Q_FRAC - GAMMA_BLACK_MODE_Z_FRAC);
}

// a * b / d, truncated toward zero, for d > 0.
static s64 gamma_black_mode_mul_div(s64 a, s64 b, u64 d)
{
	u64 r = mul_u64_u64_div_u64(abs(a), abs(b), d);

	return (a < 0) != (b < 0) ? -(s64)r : (s64)r;
}

// Reference float math (gamma_alg4.c, fit_eval):
//	for (i = 0; i < 5; i++) z[i] = exp(-B * t[i]);
//	mz = mean(z); my = mean(y);
//	szz = sum((z - mz)^2); szy = sum((z - mz) * (y - my));
//	A = szy / szz;
//	C = my - A * mz;
//	for (i = 0; i < 5; i++) { e = C + A * z[i] - y[i]; sse += e * e; }
// y is nodes 10-14 in effective codes. Returns false if the fit is unusable.
static bool gamma_black_mode_fit_eval(const int *y, u64 q, struct gamma_black_mode_fit *fit)
{
	s64 z[5], dz[5], sz = 0, sy = 0, szy = 0, e;
	s128 szz = 0;
	int i;

	for (i = 0; i < 5; i++) {
		z[i] = gamma_black_mode_z(q, 10 + i);
		sz += z[i];
		sy += y[i];
	}
	// Deviations are scaled by 5 so the means stay exact; the 5s cancel in A.
	for (i = 0; i < 5; i++) {
		dz[i] = 5 * z[i] - sz;
		szz += (s128)dz[i] * dz[i];
		szy += dz[i] * (5 * (s64)y[i] - sy);
	}
	// Also rejects |A| >= 2^20 codes, which keeps the fixed point in range (real panels are ~900).
	if ((szz >> GAMMA_BLACK_MODE_Z_FRAC) == 0 || abs(szy) >= (szz >> 20))
		return false;

	fit->A = gamma_black_mode_mul_div(szy, 1LL << GAMMA_BLACK_MODE_FRAC,
					       szz >> GAMMA_BLACK_MODE_Z_FRAC);
	fit->C = ((sy << GAMMA_BLACK_MODE_FRAC) - (s64)(((s128)fit->A * sz) >> GAMMA_BLACK_MODE_Z_FRAC)) / 5;
	fit->sse = 0;
	fit->slope = 0;
	for (i = 0; i < 5; i++) {
		e = fit->C + (s64)(((s128)fit->A * z[i]) >> GAMMA_BLACK_MODE_Z_FRAC) -
		    ((s64)y[i] << GAMMA_BLACK_MODE_FRAC);
		fit->sse += (s128)e * e;
		// dSSE/dq = 2A/q * sum(e * t * z), since A and C are already least squares.
		fit->slope += (s128)e * (255 - gamma_node_gray[10 + i]) * z[i];
	}
	return true;
}

// Reference float math (gamma_alg4.c, fit_exp_from_white):
//	for (i = 0; i <= 2000; i++) {
//		Bi = 1e-4 + (0.2 - 1e-4) * i / 2000;
//		r = fit_eval(t, y, Bi);
//		if (r.ok && r.A > 0 && (!have || r.sse < best_sse)) { best_sse = r.sse; best_B = Bi; have = 1; }
//	}
//	if (!have || best_B <= 1e-4 + step || best_B >= 0.2 - step) return -1;
//	40 golden section steps on SSE over [best_B - step, best_B + step], then B = midpoint;
//	if (r.A <= 0) return -1;
// The refinement here bisects on the sign of the SSE slope instead. It finds the same minimum, but
// golden section compares nearly equal SSEs, which in fixed point is off by a code on ~0.1% of panels.
static bool gamma_black_mode_fit(const int *y, u64 *q_fit, struct gamma_black_mode_fit *fit)
{
	u64 q = GAMMA_BLACK_MODE_Q_FIRST, best_q = 0, lo, hi, mid;
	struct gamma_black_mode_fit f;
	s128 best_sse = 0;
	int i, best_i = -1;

	for (i = 0; i <= GAMMA_BLACK_MODE_GRID_STEPS; i++) {
		if (gamma_black_mode_fit_eval(y, q, &f) && f.A > 0 && (best_i < 0 || f.sse < best_sse)) {
			best_sse = f.sse;
			best_i = i;
			best_q = q;
		}
		q = mul_q62(q, GAMMA_BLACK_MODE_Q_RATIO);
	}
	// Best B at either end of the range means the nodes don't curve the right way.
	if (best_i <= 1 || best_i >= GAMMA_BLACK_MODE_GRID_STEPS - 1)
		return false;

	// q falls as B rises, so [B + step, B - step] is [q * ratio, q / ratio].
	lo = mul_q62(best_q, GAMMA_BLACK_MODE_Q_RATIO);
	hi = mul_u64_u64_div_u64(best_q, 1ULL << GAMMA_BLACK_MODE_Q_FRAC, GAMMA_BLACK_MODE_Q_RATIO);
	while (hi - lo > 1) {
		mid = lo + (hi - lo) / 2;
		if (gamma_black_mode_fit_eval(y, mid, &f) && f.slope > 0)
			hi = mid;
		else
			lo = mid;
	}
	*q_fit = lo + (hi - lo) / 2;
	return gamma_black_mode_fit_eval(y, *q_fit, fit) && fit->A > 0;
}

// Reference float math (gamma_alg4.c, alg4_bank):
//	e14 = code_14 + 1025;
//	rise17 = C + A * exp(-B * 1) - e14;
//	g = gain + (rise17 > 0 ? bias / rise17 : 0.0);
//	for (k = 15; k <= 17; k++) {
//		pred = C + A * exp(-B * (255 - gray_k));
//		eff = e14 + g * (pred - e14);
//		code_k = clamp(floor(eff + 0.5) - 1025, 0, 0x3ff);
//	}
//	if (!(c14 < c15 && c15 < c16 && c16 < c17 && c17 < c18)) return ALG4_ERR_ORDER;
// Gain is fixed at 1.00, so eff = e14 + rise * (rise17 + bias) / rise17.
// The order check here covers the whole final curve, not just nodes 14-18.
// Replaces nodes 15-17 with an exponential fitted to the panel's own nodes 10-14, shifted by bias at node 17.
static int gamma_black_mode_bank(u8 *gamma, int base, int bias)
{
	s64 e14, rise, rise17, eff, pred[3];
	struct gamma_black_mode_fit fit;
	int y[5], i;
	u64 q;

	for (i = 0; i < 5; i++)
		y[i] = get_gamma_code(gamma, base + 10 + i) + gamma_node_offset[10 + i];
	if (!gamma_black_mode_fit(y, &q, &fit))
		return -EINVAL;

	e14 = (s64)(get_gamma_code(gamma, base + 14) + gamma_node_offset[14]) << GAMMA_BLACK_MODE_FRAC;
	for (i = 0; i < 3; i++)
		pred[i] = fit.C + (s64)(((s128)fit.A * gamma_black_mode_z(q, 15 + i)) >> GAMMA_BLACK_MODE_Z_FRAC);
	rise17 = pred[2] - e14;
	// Less than one code of rise would scale the bias out of range; the reference accepts it, we don't.
	if (rise17 < (1LL << GAMMA_BLACK_MODE_FRAC))
		return -ERANGE;

	for (i = 0; i < 3; i++) {
		rise = pred[i] - e14;
		eff = e14 + gamma_black_mode_mul_div(rise, rise17 + (s64)bias * (1LL << GAMMA_BLACK_MODE_FRAC), rise17);
		eff = (eff + (1LL << (GAMMA_BLACK_MODE_FRAC - 1))) >> GAMMA_BLACK_MODE_FRAC;
		set_gamma_code(gamma, base + 15 + i, clamp_t(s64, eff - gamma_node_offset[15 + i], 0, GAMMA_CODE_MAX));
	}
	if (!gamma_curve_is_valid(gamma, base))
		return -ERANGE;
	return 0;
}
#else
static int gamma_black_mode_bank(u8 *gamma, int base, int bias)
{
	return -EOPNOTSUPP;
}
#endif

// Both the VGMP and VGMN banks are changed, or neither is. id 0 is the left eye.
static int gamma_black_mode_calc(u8 *gamma, int id)
{
	int bias = id == 0 ? GAMMA_BLACK_MODE_BIAS_LEFT : GAMMA_BLACK_MODE_BIAS_RIGHT;
	u8 adjusted[GAMMA_SET_NUM_BYTES];
	int ret;

	memcpy(adjusted, gamma, sizeof(adjusted));
	ret = gamma_black_mode_bank(adjusted, 0, bias);
	if (!ret)
		ret = gamma_black_mode_bank(adjusted, GAMMA_VGMN_BASE, bias);
	if (!ret)
		memcpy(gamma, adjusted, sizeof(adjusted));
	return ret;
}

static int read_gamma_set(struct lpm026m648c_ctx *ctx, int id, u8 *gamma)
{
	u8 reg = LPM02648C_GAMMA_SET;

	return mipi_dsi_generic_read(ctx->dsi[id], &reg, 1, gamma, GAMMA_SET_NUM_BYTES);
}

static int write_gamma_set(struct lpm026m648c_ctx *ctx, int id, const u8 *gamma)
{
	u8 payload[1 + GAMMA_SET_NUM_BYTES] = { LPM02648C_GAMMA_SET };

	memcpy(&payload[1], gamma, GAMMA_SET_NUM_BYTES);
	return mipi_dsi_generic_write(ctx->dsi[id], payload, sizeof(payload));
}

// Panel reset reloads the factory gamma from OTP, so one read per boot is enough.
static void read_factory_gamma(struct lpm026m648c_ctx *ctx)
{
	unsigned int id, panel_num = ctx->is_dual_panel ? 2 : 1;
	int ret;

	for (id = 0; id < panel_num; id++) {
		if (ctx->factory_gamma_valid[id])
			continue;

		// The msm DSI host returns 0 rather than an error for some failed reads.
		ret = read_gamma_set(ctx, id, ctx->factory_gamma[id]);
		if (ret != GAMMA_SET_NUM_BYTES) {
			DRM_DEV_ERROR(ctx->dev, "failed to read panel %d gamma ( %d )\n", id, ret);
			continue;
		}
		// The full register, so any later result can be recomputed offline.
		DRM_DEV_INFO(ctx->dev, "panel %d factory gamma VGMP %*ph\n", id, GAMMA_VGMN_BASE * 2, ctx->factory_gamma[id]);
		DRM_DEV_INFO(ctx->dev, "panel %d factory gamma VGMN %*ph\n", id, GAMMA_VGMN_BASE * 2,
			     &ctx->factory_gamma[id][GAMMA_VGMN_BASE * 2]);
		// The host can misreport a read, and a bad copy would later be written to the panel; retried next prepare.
		if (!gamma_curve_is_valid(ctx->factory_gamma[id], 0) ||
		    !gamma_curve_is_valid(ctx->factory_gamma[id], GAMMA_VGMN_BASE)) {
			DRM_DEV_ERROR(ctx->dev, "panel %d factory gamma read isn't a valid curve, ignoring it\n", id);
			continue;
		}
		ctx->factory_gamma_valid[id] = true;
	}
}

// Writes the gamma black mode or the factory gamma, depending on ctx->gamma_black_mode.
static void update_gamma_black_mode(struct lpm026m648c_ctx *ctx)
{
	unsigned int id, mode, panel_num = ctx->is_dual_panel ? 2 : 1;
	u8 gamma[GAMMA_SET_NUM_BYTES];
	int ret;

	if (ctx->panel_vendor != VENDOR_BOE)
		return;

	for (id = 0; id < panel_num; id++) {
		if (!ctx->factory_gamma_valid[id])
			continue;

		memcpy(gamma, ctx->factory_gamma[id], sizeof(gamma));
		mode = ctx->gamma_black_mode;
		if (mode) {
			// On failure gamma is left at factory, which is written instead.
			ret = gamma_black_mode_calc(gamma, id);
			if (ret) {
				DRM_DEV_ERROR(ctx->dev, "panel %d gamma black mode %u rejected, %s ( %d ), using factory gamma\n",
					      id, mode, ret == -EINVAL ? "nodes 10-14 don't fit" :
					      ret == -ERANGE ? "curve check failed" : "not supported", ret);
				mode = 0;
			}
		}

		ret = write_gamma_set(ctx, id, gamma);
		if (ret < 0) {
			DRM_DEV_ERROR(ctx->dev, "failed to write panel %d gamma ( %d )\n", id, ret);
			continue;
		}
		DRM_DEV_INFO(ctx->dev, "panel %d gamma black mode %u, VGMP15-17 %d %d %d, VGMN15-17 %d %d %d\n", id, mode,
			     get_gamma_code(gamma, 15), get_gamma_code(gamma, 16), get_gamma_code(gamma, 17),
			     get_gamma_code(gamma, GAMMA_VGMN_BASE + 15), get_gamma_code(gamma, GAMMA_VGMN_BASE + 16),
			     get_gamma_code(gamma, GAMMA_VGMN_BASE + 17));
	}
}

static void dsi_write_panel_init_jdi(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	u8 pps_cmd[1 + sizeof(struct drm_dsc_picture_parameter_set)];

	drm_dsc_pps_payload_pack((void *)&pps_cmd[1], &ctx->dsc_cfg);
	pps_cmd[0] = LPM02648C_PPS_SET;

	lpm026m648c_dsi_write_seq(ctx, LPM02648C_MF_CMD_ACCESS_PROTECT, 0x04);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_SEQ_CTL, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DSI_CTL,
				  0x20, 0x6b, 0x80, 0x06, 0x33, 0x9a, 0x00,
				  0x1a, 0x7a);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_MODE, 0x54, 0x00, 0x00,
				  0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_GEN_OUTPIN_SET,
				  BE16(0), BE16(0), 0x00, 0x00,
				  0x00, 0x00,
				  BE16(GPO1_TES1_120_JDI-CAMERA_TRIGGER_OFFSET),
				  BE16(CAMERA_TRIGGER_OFFSET * 2 + GPO1_TEW1));
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET2, 0x1e);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3, 0x70, 0x08, 0xd0,
				  0x02, 0x21, 0x6f, 0x08, 0x5a, 0x00, 0x00,
				  0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET1, RTN, 0x86,
				  LE16(VBP), 0x08, 0x70, BE16(VFP));
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3_2, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET4, 0x8b, 0x00, 0x80,
				  0x46, 0x61, 0x00, 0x8b);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET5, BE16(VID_VS_DELAY),
				  0x00, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_TE_GPIO_CTL, 0x00, 0x6A, 0x02);
	lpm026m648c_dsi_write_buffer(ctx, pps_cmd);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x01);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_SCANLINE, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_PIXEL_FORMAT, 0x77);
	//lpm026m648c_dsi_write_seq(ctx, LPM02648C_SEQ_CTL, 0x80);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_MF_CMD_ACCESS_PROTECT, 0x03);
}

static void dsi_write_panel_init_sharp(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	u8 pps_cmd[1 + sizeof(struct drm_dsc_picture_parameter_set)];

	drm_dsc_pps_payload_pack((void *)&pps_cmd[1], &ctx->dsc_cfg);
	pps_cmd[0] = LPM02648C_PPS_SET;

	lpm026m648c_dsi_write_seq(ctx, LPM02648C_MF_CMD_ACCESS_PROTECT, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_SEQ_CTL, 0x00);
	// update vcom per starting mode
	update_vcom(ctx);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DSI_CTL,
				  0x20, 0x6b, 0x80, 0x06, 0x33, 0x9a, 0x00,
				  0x1a, 0x7a);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_MODE, 0x54, 0x00, 0x00,
				  0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_GEN_OUTPIN_SET,
				  BE16(0), BE16(0), 0x00, 0x00,
				  0x00, 0x00,
				  BE16(GPO1_TES1_120_SHARP-CAMERA_TRIGGER_OFFSET),
				  BE16(CAMERA_TRIGGER_OFFSET * 2 + GPO1_TEW1));
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3, 0x08, 0x70, 0x28,
				  0x48, 0x00, 0x00, 0x13, 0x21, 0xff, 0x00, 0x0f, 0x01,
				  0x14, 0x17, 0x00, 0x00, 0x00, 0x02, 0x40, 0x0C, 0x00, 0x00,
				  0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x70, 0x08,
				  0xD0, 0x02, 0x21, 0x6F, 0x08, 0x5A, 0x00, 0x00, 0x00, 0x00,
				  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				  0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET1, RTN, 0x86, LE16(VBP), 0x08, 0x70,
				  BE16(VFP), 0x00, 0x00, 0x08, 0x3B, 0x00, 0x00, 0x19, 0x01, 0x22);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3_2, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET4, 0x8b, 0x00, 0x80,
				  0x46, 0x61, 0x00, 0x8b);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET5, BE16(VID_VS_DELAY),
				  0x00, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET6, 0x00, 0x24, 0x00, 0x00,
				  0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
				  0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x0A,
				  0x00, 0x00, 0x00, 0x03, 0x1D, 0x01, 0x00, 0x00, 0x00, 0x01,
				  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
				  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);				  
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_TE_GPIO_CTL, 0x00, 0x6A, 0x02);
	lpm026m648c_dsi_write_buffer(ctx, pps_cmd);
	//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x01);
	//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_SCANLINE, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x40);
	//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_PIXEL_FORMAT, 0x77);
}

static void dsi_write_panel_init_boe(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	u8 pps_cmd[1 + sizeof(struct drm_dsc_picture_parameter_set)];

	drm_dsc_pps_payload_pack((void *)&pps_cmd[1], &ctx->dsc_cfg);
	pps_cmd[0] = LPM02648C_PPS_SET;

	if ( !ctx->dphy ) {
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_MF_CMD_ACCESS_PROTECT, 0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_SEQ_CTL, 0x00);
		// update vcom per starting mode
		update_vcom(ctx);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DSI_CTL,
					  0x20, 0x6b, 0x80, 0x06, 0x33, 0x9A, 0x00,
					  0x1a, 0x7a);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_MODE, 0x54, 0x00, 0x00,
					  0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_GEN_OUTPIN_SET,
					  BE16(0), BE16(0), 0x00, 0x00,
					  0x00, 0x00,
					  BE16(GPO1_TES1_120_BOE-CAMERA_TRIGGER_OFFSET),
					  BE16(CAMERA_TRIGGER_OFFSET * 2 + GPO1_TEW1));
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3, 0x0B, 0x70, 0x08,
					  0x48, 0x00, 0x00, 0x13, 0x01, 0xFF, 0x00, 0x01, 0x09,
					  0x16, 0x16, 0x00, 0x00, 0x00, 0x01, 0x40, 0x0C, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
					  0x70, 0x08, 0xD0, 0x02, 0x21, 0x6F, 0x08, 0x5A, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET1, RTN, 0x86, LE16(VBP),
					  0x08, 0x70, BE16(VFP), 0x00, 0x00, 0x08, 0x3A, 0x00, 0x00,
					  0x17, 0x00, 0x17);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3_2, 0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET4, 0x8b, 0x00, 0x80,
					  0x46, 0x61, 0x00, 0x8b);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET5, BE16(VID_VS_DELAY),
					  0x00, 0x00, 0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_TE_GPIO_CTL, 0x00, 0x6A, 0x02);
		lpm026m648c_dsi_write_buffer(ctx, pps_cmd);
		//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x01);
		//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_SCANLINE, 0x00, 0x00);
		lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
		//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x00);
		//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_PIXEL_FORMAT, 0x77);
	} else {
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_MF_CMD_ACCESS_PROTECT, 0x04);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_SEQ_CTL, 0x00);
		// update vcom per starting mode
		update_vcom(ctx);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DSI_CTL,
					0x30, 0x6b, 0x00, 0x06, 0x33, 0x8E, 0x00,
					0x1a, 0x7a);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_MODE, 0x54, 0x00, 0x00,
					0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_GEN_OUTPIN_SET,
					BE16(0), BE16(0), 0x00, 0x00,
					0x00, 0x00,
					BE16(GPO1_TES1_120_BOE-CAMERA_TRIGGER_OFFSET),
					BE16(CAMERA_TRIGGER_OFFSET * 2 + GPO1_TEW1));
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET1, RTN, 0x86, LE16(VBP),
					0x08, 0x70, BE16(VFP), 0x00, 0x00, 0x08, 0x3A, 0x00, 0x00,
					0x17, 0x00, 0x17);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET2, 0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3, 0x0B);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3_2, 0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET4, 0x8b, 0x00, 0x80, 0x46, 0x61, 0x00, 0x8b);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET5, BE16(VID_VS_DELAY), 0x00, 0x00, 0x00);
		lpm026m648c_dsi_write_seq(ctx, LPM02648C_TE_GPIO_CTL, 0x00, 0x6A, 0x02);
		lpm026m648c_dsi_write_buffer(ctx, pps_cmd);
		//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x01);
		//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_SCANLINE, 0x00, 0x00);
		lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
		//lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x00);
		lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_PIXEL_FORMAT, 0x77);		
	}
}

static void dsi_write_panel_on(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(170);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_ON);
	msleep(200);
}

// verify each panel is in the correct power state
static void dsi_check_panel_power(struct work_struct *work)
{
	unsigned int panel_num, id;
	uint8_t mode;
	int ret;
	struct lpm026m648c_ctx *ctx = container_of((struct delayed_work *)work,struct lpm026m648c_ctx,check_display_power);
	panel_num = ctx->is_dual_panel ? 2 : 1;
	for (id = 0; id < panel_num; id++)
	{
		mode = 0;
		ret = mipi_dsi_dcs_get_power_mode(ctx->dsi[id], &mode);
		if (ret < 0)
		{
			DRM_DEV_ERROR(ctx->dev, "failed to read panel %d power status\n", id);
		}
		else
		{
			if (mode != (MIPI_DSI_DCS_POWER_MODE_DISPLAY | MIPI_DSI_DCS_POWER_MODE_NORMAL
				 | MIPI_DSI_DCS_POWER_MODE_SLEEP))
			{
				DRM_DEV_ERROR(ctx->dev, "Panel %d power on failure ( %x )\n", id, mode);
			}
		}
	}
}

static int get_panel_info(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret;
	unsigned int id, offset, panel_num;
	u8 panel_info_regs[] = { LPM02648C_MFG_INFO };
	u8 *info = ctx->panel_mfg_info;

	panel_num = ctx->is_dual_panel ? 2 : 1;
	for (id = 0; id < panel_num; id++) {
		offset =  id*PANEL_MFG_INFO_NUM_BYTES;
		ret = mipi_dsi_generic_read(ctx->dsi[id], panel_info_regs, 1,
				&info[offset], PANEL_MFG_INFO_NUM_BYTES);
		if (ret < 0) {
			DRM_DEV_ERROR(ctx->dev, "failed to read panel%d info\n", id);
        }
	}
	// save left vcom values
	ctx->vcom120[0] = ctx->panel_mfg_info[VCOM120_OFFSET] << 8 | ctx->panel_mfg_info[VCOM120_OFFSET + 1];
	ctx->vcom90[0] = ctx->panel_mfg_info[VCOM90_OFFSET] << 8 | ctx->panel_mfg_info[VCOM90_OFFSET + 1];
	// save right vcom values
	ctx->vcom120[1] = ctx->panel_mfg_info[VCOM120_OFFSET + PANEL_MFG_INFO_NUM_BYTES] << 8 | ctx->panel_mfg_info[VCOM120_OFFSET + PANEL_MFG_INFO_NUM_BYTES + 1];
	ctx->vcom90[1] = ctx->panel_mfg_info[VCOM90_OFFSET + PANEL_MFG_INFO_NUM_BYTES] << 8 | ctx->panel_mfg_info[VCOM90_OFFSET + PANEL_MFG_INFO_NUM_BYTES + 1];
    return 0;
}

static void dsi_write_panel_init(struct drm_panel *panel)
{
	int ret;
	char panel_left, panel_right;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (!ctx->dphy) {
		// read panel info to determine panel type
		ret = get_panel_info(panel);
		if ( ret ) {
			DRM_DEV_ERROR(ctx->dev, "Could not get panel info for init sequence, defaulting to Sharp\n" );
			ctx->panel_vendor = VENDOR_SHARP;
		} else {
			// need to support old format JDI displays
			if ( ctx->panel_mfg_info[VENDOR_OFFSET] == 1 ) {
				panel_left = ctx->panel_mfg_info[VENDOR_OFFSET + 1];
			} else {
				panel_left = ctx->panel_mfg_info[VENDOR_OFFSET];
			}
			if ( ctx->panel_mfg_info[VENDOR_OFFSET + PANEL_MFG_INFO_NUM_BYTES] == 1 ) {
				panel_right = ctx->panel_mfg_info[VENDOR_OFFSET + PANEL_MFG_INFO_NUM_BYTES + 1];
			} else {
				panel_right = ctx->panel_mfg_info[VENDOR_OFFSET + PANEL_MFG_INFO_NUM_BYTES];
			}
			if (panel_left && panel_right && panel_left != panel_right) {
				DRM_DEV_ERROR(ctx->dev, "Displays don't match ( %c != %c ), using %c\n", panel_left, panel_right, panel_left );
				ctx->panel_vendor = panel_left;
			} else if (panel_left && panel_right) {
				DRM_DEV_INFO(ctx->dev, "Displays match ( %c == %c ), using %c\n", panel_left, panel_right, panel_left );
				ctx->panel_vendor = panel_left;
			} else if ( panel_left && !panel_right ) {
				DRM_DEV_INFO(ctx->dev, "Only left panel, using %c\n", panel_left);
				ctx->panel_vendor = panel_left;
			} else if ( !panel_left && panel_right ) {
				DRM_DEV_INFO(ctx->dev, "Only right panel, using %c\n", panel_right );
				ctx->panel_vendor = panel_right;
			} else {
				DRM_DEV_ERROR(ctx->dev, "no panel mfg info, defaulting to sharp ( %c %c )\n", panel_left, panel_right );
				ctx->panel_vendor = VENDOR_SHARP;
			}
		}
	} else {
		// Dphy units default to BOE
		DRM_DEV_INFO(ctx->dev, "DPHY board, default to BOE\n" );
		ctx->panel_vendor = VENDOR_BOE;
	}

	if ( ctx->panel_vendor == VENDOR_SHARP ) {
		dsi_write_panel_init_sharp(panel);
	} else if ( ctx->panel_vendor == VENDOR_BOE ) {
		dsi_write_panel_init_boe(panel);
	} else if ( ctx->panel_vendor == VENDOR_JDI ) {
		dsi_write_panel_init_jdi(panel);
	} else {
		DRM_DEV_ERROR(ctx->dev, "unrecognized panel vendor ( %c ), defualt to sharp\n", ctx->panel_mfg_info[VENDOR_OFFSET] );
		dsi_write_panel_init_sharp(panel);
		ctx->panel_vendor = VENDOR_SHARP;
	}
}

static void dsi_write_panel_off(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_OFF);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(200);
}

// Enable/disable backlights if they are not in the desired state
// This prevents the driver from voting multiple times to enable/disable
// the backlight regulators and get out of sync
static int enable_backlight_regulators(struct lpm026m648c_ctx *ctx, bool enable)
{
	int i, ret = 0;
	bool preboost_turned_on = false;
	if( enable ) {
		for ( i = 0; i < ARRAY_SIZE(ctx->preboost_drivers); i++ ) {
			if ( !regulator_is_enabled(ctx->preboost_drivers[i].consumer) ) {
				ret = regulator_enable(ctx->preboost_drivers[i].consumer);
				if ( ret ) {
					dev_err(ctx->dev, "Could not enable preboost driver\n");
				}
				preboost_turned_on = true;
			}
		}

		// Preboost Startup time, only needed if a preboost was just turned on
		if ( preboost_turned_on )
			usleep_range(40000, 50000);

		for ( i = 0; i < ARRAY_SIZE(ctx->backlight_drivers); i++ ) {
			if ( !regulator_is_enabled(ctx->backlight_drivers[i].consumer) ) {
				ret = regulator_enable(ctx->backlight_drivers[i].consumer);
				if ( ret ) {
					dev_err(ctx->dev, "Could not enable backlight driver\n");
				}
			}
		}
	} else {
		for ( i = 0; i < ARRAY_SIZE(ctx->backlight_drivers); i++ ) {
			if ( regulator_is_enabled(ctx->backlight_drivers[i].consumer) ) {
				ret |= regulator_disable(ctx->backlight_drivers[i].consumer);
			}
		}

		for ( i = 0; i < ARRAY_SIZE(ctx->preboost_drivers); i++ ) {
			if ( regulator_is_enabled(ctx->preboost_drivers[i].consumer) ) {
				ret |= regulator_disable(ctx->preboost_drivers[i].consumer);
			}
		}
	}

	return ret;
}

static int set_brightness(struct lpm026m648c_ctx *ctx, u16 brightness,
			  u16 pulse_offset_rows)
{
	u16 gpo1_tes1 = pulse_offset_rows;
	u16 gpo1_tew1;
	u16 cam_trig_start = 0, cam_trig_width = 0;

	gpo1_tew1 = brightness > BRIGHTNESS_MAX_120 ?
		    BRIGHTNESS_MAX_120 : brightness;
	// Scale brightness based on current mode
	gpo1_tew1 = gpo1_tew1 * modes[ctx->current_mode].vtotal / modes[MODE_120].vtotal;

	// set offset based on frame rate to optimize settling time vs latency
	if ( modes[ctx->current_mode].vtotal == modes[MODE_144].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_144_SHARP : GPO1_TES1_144_BOE;
	} else if ( modes[ctx->current_mode].vtotal == modes[MODE_120].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_120_SHARP : GPO1_TES1_120_BOE;
	} else if ( modes[ctx->current_mode].vtotal == modes[MODE_108].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_108_SHARP : GPO1_TES1_108_BOE;
	} else if ( modes[ctx->current_mode].vtotal == modes[MODE_96].vtotal ) {
		gpo1_tes1 = GPO1_TES1_96;
	} else {
		gpo1_tes1 = GPO1_TES1_90;
	}

	if (!ctx->dphy) {
		ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
		if (ctx->is_dual_panel)
			ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	}

	if (brightness) {
		cam_trig_start = (gpo1_tes1 > CAMERA_TRIGGER_OFFSET) ?
				 (gpo1_tes1 - CAMERA_TRIGGER_OFFSET) : 0;
		cam_trig_width = gpo1_tew1 + CAMERA_TRIGGER_OFFSET*2;
	}

	// make sure regulators are enabled before starting pwm pulse
	// from the display  if we are turning on the backlight

	if (brightness > 0)
		enable_backlight_regulators(ctx, true);

	lpm026m648c_dsi_write_seq(ctx, LPM02648C_GEN_OUTPIN_SET,
				  BE16(gpo1_tes1), BE16(gpo1_tew1), 0x00, 0x00,
				  0x00, 0x00, BE16(cam_trig_start),
				  BE16(cam_trig_width));

	if (!ctx->dphy) {
		ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
		if (ctx->is_dual_panel)
			ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;
	}

	// Turn off backlight regulators to save power if
	// we are setting the brightness to 0
	if (brightness == 0)
		enable_backlight_regulators(ctx, false);

	return 0;
}

static int lpm026m648c_power_on(struct lpm026m648c_ctx *ctx)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->power_supplies),
				    ctx->power_supplies);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	// turn on AVDD
	ret = regulator_enable(ctx->lcd_bias_regulators[LEFT_POS].consumer);
	ret |= regulator_enable(ctx->lcd_bias_regulators[RIGHT_POS].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not enable AVDD\n");
		return ret;
	}
	usleep_range(1000, 2000);
	// turn on AVEE
	ret = regulator_enable(ctx->lcd_bias_regulators[LEFT_NEG].consumer);
	ret |= regulator_enable(ctx->lcd_bias_regulators[RIGHT_NEG].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not enable AVEE\n");
		return ret;
	}
	
	msleep(20);
	gpiod_set_value(ctx->reset_gpios->desc[0], 1);
	gpiod_set_value(ctx->reset_gpios->desc[1], 1);
	return 0;
}

static int lpm026m648c_power_off(struct lpm026m648c_ctx *ctx)
{
	int ret;
	gpiod_set_value(ctx->reset_gpios->desc[0], 0);
	gpiod_set_value(ctx->reset_gpios->desc[1], 0);
	msleep(25);

	// turn off AVEE
	ret = regulator_disable(ctx->lcd_bias_regulators[LEFT_NEG].consumer);
	ret |= regulator_disable(ctx->lcd_bias_regulators[RIGHT_NEG].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not disable AVEE\n");
	}
	usleep_range(1000, 2000);
	// turn off AVDD
	ret = regulator_disable(ctx->lcd_bias_regulators[LEFT_POS].consumer);
	ret |= regulator_disable(ctx->lcd_bias_regulators[RIGHT_POS].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not disable AVDD\n");
	}

	msleep(20);

	return regulator_bulk_disable(ARRAY_SIZE(ctx->power_supplies),
				      ctx->power_supplies);
}

static int lpm026m648c_disable(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (!ctx->enabled)
		return 0;

	backlight_disable(ctx->backlight);

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	dsi_write_panel_off(panel);

	ctx->enabled = false;
	return 0;
}

static int lpm026m648c_unprepare(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret = 0;

	if (!ctx->prepared)
		return 0;

	mutex_lock(&ctx->gamma_lock);
	ctx->gamma_safe_to_write = false;
	mutex_unlock(&ctx->gamma_lock);

	ret = lpm026m648c_power_off(ctx);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev, "power_off failed ret = %d\n", ret);

	// disable preboost and backlight drivers
	enable_backlight_regulators(ctx, false);
	
	ctx->prepared = false;
	return ret;
}

static int lpm026m648c_prepare(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	// turn on preboost and backlight drivers
	ret = enable_backlight_regulators(ctx, true);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "failed to turn on backlight drivers\n");
		goto power_off;
	}

	// clear out panel manufacturing info
	memset(ctx->panel_mfg_info, 0, ARRAY_SIZE(ctx->panel_mfg_info));

	ret = lpm026m648c_power_on(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "power_on failed\n");
		goto power_off;
	}

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	msleep(20);

	dsi_write_panel_init(panel);

	// Init leaves sleep-out OTP reload disabled (D6h), so this gamma survives enable.
	mutex_lock(&ctx->gamma_lock);
	if (ctx->panel_vendor == VENDOR_BOE) {
		read_factory_gamma(ctx);
		if (ctx->gamma_black_mode)
			update_gamma_black_mode(ctx);
	}
	ctx->gamma_safe_to_write = true;
	mutex_unlock(&ctx->gamma_lock);

	ctx->prepared = true;

	return 0;

power_off:
	if (lpm026m648c_power_off(ctx))
		DRM_DEV_ERROR(ctx->dev, "power_off failed\n");
	return ret;
}

static int lpm026m648c_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int lpm026m648c_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct lpm026m648c_ctx *lpm026m648c_ctx = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!lpm026m648c_ctx->prepared)
		return 0;

	ret = set_brightness(lpm026m648c_ctx, bl->props.brightness,
			     bl->props.pulse_offset_rows);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops lpm026m648c_bl_ops = {
	.update_status = lpm026m648c_bl_update_status,
	.get_brightness = lpm026m648c_bl_get_brightness,
};

static int lpm026m648c_enable(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret;

	if (ctx->enabled)
		return 0;

	dsi_write_panel_on(panel);

    // >=6 frames of video before TE starts to output
	msleep(300);

	// default to 120, but we will change this based on frame rate
	ctx->backlight->props.pulse_offset_rows = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_120_SHARP : GPO1_TES1_120_BOE;
	ret = backlight_enable(ctx->backlight);

	INIT_DELAYED_WORK(&ctx->check_display_power, dsi_check_panel_power);
	schedule_delayed_work(&ctx->check_display_power, HZ);

	ctx->enabled = true;

	return ret;
}

static int lpm026m648c_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	struct drm_display_mode *mode;
	int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		// only add 72 Hz if the backlight driver supports it
		if ( i == MODE_72 ) {
			if ( !ctx->low_fps_support ) {
				continue;
			}
		}

		mode = drm_mode_create(connector->dev);
		if (!mode) {
			DRM_DEV_ERROR(ctx->dev,
				"failed to create a new display mode\n");
			return i;
		}

		drm_mode_copy(mode, &modes[i]);
		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		if (ctx->is_dual_panel) {
			mode->name[0] = '2';
			mode->name[1] = '*';
			mode->clock *= 2;
			mode->hdisplay *= 2;
			mode->hsync_start *= 2;
			mode->hsync_end *= 2;
			mode->htotal *= 2;
			mode->width_mm *= 2;
		}
		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static int lpm026m648c_mode_set(struct drm_panel *panel,
			       const struct drm_display_mode *mode,
			       const struct drm_display_mode *adjusted_mode)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	lpm026m648c_modes_t new_mode;

	// use the vtotal to determine the mode
	if (mode->vtotal == modes[MODE_120].vtotal){
			new_mode = MODE_120;
	}else if (mode->vtotal == modes[MODE_96].vtotal){
			new_mode = MODE_96;
	}else if (mode->vtotal == modes[MODE_90].vtotal){
			new_mode = MODE_90;
	}else if (mode->vtotal == modes[MODE_80].vtotal){
			new_mode = MODE_80;
	}else if (mode->vtotal == modes[MODE_72].vtotal){
			new_mode = MODE_72;
	}else if (mode->vtotal == modes[MODE_108].vtotal){
			new_mode = MODE_108;
	}else if (mode->vtotal == modes[MODE_144].vtotal){
			new_mode = MODE_144;
	}else{
			DRM_DEV_ERROR(ctx->dev, "Invalid vtotal in mode set %d", mode->vtotal);
			return -EINVAL;
	}

	// check if we need to update settings
	if (ctx->current_mode != new_mode)
	{
		ctx->current_mode = new_mode;
		// only reset the panel parematers if we already initialized the panels
		if (ctx->enabled)
		{
			// update vcom
			update_vcom(ctx);
			// update brightness
			set_brightness(ctx, ctx->backlight->props.brightness, ctx->backlight->props.pulse_offset_rows);
		}
	}

	return 0;
}

static int panel_info_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	unsigned int id, offset, panel_num;
	u8 *info = ctx->panel_mfg_info;

	panel_num = ctx->is_dual_panel ? 2 : 1;
	for (id = 0; id < panel_num; id++) {
		offset =  id*PANEL_MFG_INFO_NUM_BYTES;

		seq_printf(s, "****** JDI Panel info for %s panel:\n",
			   id ? "right" : "left");
		seq_printf(s, " SupplierID  %x\n",
			   info[1+offset] << 8 | info[0+offset]);
		seq_printf(s, " DriverICProductCode  %x\n",
			   info[3+offset] << 8 | info[2+offset]);
		seq_printf(s, " brightnessDataW255  %x\n", info[5+offset]);
		seq_printf(s, " colorPointofWhiteX  %x\n",  info[6+offset]);
		seq_printf(s, " colorPointofWhiteY  %x\n",  info[7+offset]);
		seq_printf(s, " contrast  %x\n",  info[8+offset]);
		seq_printf(s, " gamma  %x\n",  info[9+offset]);
		seq_printf(s, " brightnessDataW32  %x\n",  info[12+offset]);
		seq_printf(s, " 120 vcom1  %x\n",  info[13+offset]);
		seq_printf(s, " 120 vcom2  %x\n",  info[14+offset]);
		seq_printf(s, " 90 vcom1  %x\n",  info[15+offset]);
		seq_printf(s, " 90 vcom2  %x\n",  info[16+offset]);
		seq_printf(s, " colorPointofRedX  %x\n",  info[21+offset]);
		seq_printf(s, " colorPointofRedY  %x\n",  info[22+offset]);
		seq_printf(s, " colorPointofGreenX  %x\n",  info[23+offset]);
		seq_printf(s, " colorPointofGreenY  %x\n",  info[24+offset]);
		seq_printf(s, " colorPointofBlueX  %x\n",  info[25+offset]);
		seq_printf(s, " colorPointofBlueY  %x\n",  info[26+offset]);
		seq_printf(s, " format  %x\n",  info[27+offset]);
		seq_printf(s, " vendor1  %x\n",  info[28+offset]);
		seq_printf(s, " vendor2  %x\n",  info[29+offset]);
		seq_printf(s, " mdlBaseCode1  %x\n",  info[30+offset]);
		seq_printf(s, " mdlBaseCode2  %x\n",  info[31+offset]);
		seq_printf(s, " mdlBaseCode3  %x\n",  info[32+offset]);
		seq_printf(s, " id  %x\n",  info[33+offset]);
		seq_printf(s, " serialization3  %x\n",  info[34+offset]);
		seq_printf(s, " serialization2  %x\n",  info[35+offset]);
		seq_printf(s, " serialization1  %x\n",  info[36+offset]);
		seq_printf(s, " year  %x\n",  info[37+offset]);
		seq_printf(s, " week  %x\n",  info[38+offset]);
		seq_printf(s, " day  %x\n",  info[39+offset]);
	}
	return 0;
}

static int panel_mfg_info_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctx->panel_mfg_info); i++) {
		seq_printf(s, "%u ",  ctx->panel_mfg_info[i]);
	}
	seq_printf(s, "\n");
	return 0;
}

static int panel_read_show(struct seq_file *s, void *data)
{
	int ret;
	unsigned int i;
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (ctx->panel_read_reg_addr >= 0xb0) {
		ret = mipi_dsi_generic_read(ctx->dsi[ctx->panel_read_id], &ctx->panel_read_reg_addr, 1, ctx->panel_read_data, ctx->panel_read_num_bytes);
	} else {
		ret = mipi_dsi_dcs_read(ctx->dsi[ctx->panel_read_id], ctx->panel_read_reg_addr, ctx->panel_read_data, ctx->panel_read_num_bytes);
	}
	
	if ( ret < 0 )
	{
		DRM_DEV_ERROR(ctx->dev, "failed to read panel %d reg %u\n", ctx->panel_read_id, ctx->panel_read_reg_addr);
		return ret;
	}
	else
	{
		for (i = 0; i < ctx->panel_read_num_bytes; i++) {
			seq_printf(s, "%02x ",  ctx->panel_read_data[i]);
		}
		seq_printf(s, "\n");
		return 0;
	}
}

static ssize_t panel_read_write(struct file *file,
					  const char __user *userbuf,
					  size_t count, loff_t *ppos)
{
	int ret;
	struct seq_file *s = file->private_data;
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	u8 buf[32];
	int id, reg_addr, num_bytes;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EOVERFLOW;
	buf[count] = '\0';

	ret = sscanf(buf, "%d %2x %d", &id, &reg_addr, &num_bytes);
	if(ret != 3)
	{
		DRM_DEV_ERROR(ctx->dev, "panel read got wrong number of input parameters from sscanf");
		return -EINVAL;
	}
	if( (id != 0 && id != 1) || !ctx->dsi[id] )
	{
		// invalid panel id
		DRM_DEV_ERROR(ctx->dev, "panel read found invalid id value %d", id);
		ctx->panel_read_id = 0;
		return -EINVAL;
	}
	if (num_bytes < 1 || num_bytes > (int)sizeof(ctx->panel_read_data))
	{
		DRM_DEV_ERROR(ctx->dev, "panel read found invalid length %d", num_bytes);
		return -EINVAL;
	}
	ctx->panel_read_id = id;
	ctx->panel_read_reg_addr = reg_addr;
	ctx->panel_read_num_bytes = num_bytes;
	return count;
}

static ssize_t panel_write_write(struct file *file,
					  const char __user *userbuf,
					  size_t count, loff_t *ppos)
{
	int ret;
	int id, addr, len, i, v;
	char char_data[256];
	u8 payload[256];
	u8 buf[256];
	struct seq_file *s = file->private_data;
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EOVERFLOW;
	buf[count] = '\0';

	// get the base parameters
	ret = sscanf(buf, "%d %2x %d %255s", &id, &addr, &len, char_data);
	if(ret != 4)
	{
		DRM_DEV_ERROR(ctx->dev, "panel write got wrong number of input parameters (%d) from sscanf", ret);
		return -EINVAL;
	}
	if((id != 0 && id != 1) || !ctx->dsi[id])
	{
		// invalid panel id
		DRM_DEV_ERROR(ctx->dev, "panel read found invalid id value %u", ctx->panel_read_id);
		ctx->panel_read_id = 0;
		return -EINVAL;
	}
	// we limit writes to a total of 256 bytes
	if(len < 0 || len > (int)sizeof(payload) - 1)
	{
		DRM_DEV_ERROR(ctx->dev, "jdi4k panel write exceed max length %d\n", len);
		return -EINVAL;
	}
	payload[0] = addr;
	// get the data payload
	for(i=0;i<len && i * 2 < strlen(char_data);i++)
	{
		if(sscanf(char_data + i * 2, "%2x", &v) != 1) break;
		payload[i+1] = (unsigned char)v;
	}

	if (addr >= 0xb0)
		ret = mipi_dsi_generic_write(ctx->dsi[id], payload, len+1);
	else if (len == 0)
		ret = mipi_dsi_dcs_write(ctx->dsi[id], addr, NULL, 0);
	else
		ret = mipi_dsi_dcs_write_buffer(ctx->dsi[id], payload, len+1);
	if (ret < 0)
	{
		DRM_DEV_ERROR(ctx->dev, "failed to tx cmd [%.2x], err: %d\n", payload[0], ret);
	}

	return count;
}

static int panel_write_show(struct seq_file *s, void *data)
{
	return 0;
}

static int panel_available_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	seq_printf(s, "%d\n",  ctx->enabled );
	return 0;
}

// RCLK = 32.5 MHz
#define RCLK_PERIOD_PSEC (uint64_t)( 1000000000/32500 )
#define INTERNAL_LINE_TIME_PSEC  (uint64_t)( RTN * RCLK_PERIOD_PSEC )
#define EXTERNAL_LINE_TIME_PSEC (uint64_t)( ( 1000000000 * (uint64_t)modes[ctx->current_mode].htotal ) / ( (uint64_t)modes[ctx->current_mode].clock ) )

static int panel_vsync_to_photons_usec_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	uint64_t vsync_to_photons_psec, gpo1_tes1, frame_period_psec;

	// set offset based on frame rate to optimize settling time vs latency
	if ( modes[ctx->current_mode].vtotal == modes[MODE_144].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_144_SHARP : GPO1_TES1_144_BOE;
	} else if ( modes[ctx->current_mode].vtotal == modes[MODE_120].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_120_SHARP : GPO1_TES1_120_BOE;
	} else if ( modes[ctx->current_mode].vtotal == modes[MODE_108].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_108_SHARP : GPO1_TES1_108_BOE;
	} else if ( modes[ctx->current_mode].vtotal == modes[MODE_96].vtotal ) {
		gpo1_tes1 = GPO1_TES1_96;
	} else {
		gpo1_tes1 = GPO1_TES1_90;
	}

	if ( ctx->panel_vendor == VENDOR_JDI 
		|| ctx->panel_vendor == VENDOR_BOE
		|| ctx->current_mode == MODE_72
		|| ctx->current_mode == MODE_80
		|| ctx->current_mode == MODE_90
		|| ctx->current_mode == MODE_96 ) {
		// VID_VS_DELAY + TES
		vsync_to_photons_psec = VID_VS_DELAY * EXTERNAL_LINE_TIME_PSEC + gpo1_tes1 * INTERNAL_LINE_TIME_PSEC;
	} else { // SHARP 108/120/144
		switch ( ctx->current_mode )
		{
			case MODE_144:
				frame_period_psec = 1000000000000 / 144;
				break;
			case MODE_120:
				frame_period_psec = 1000000000000 / 120;
				break;
			case MODE_108:
				frame_period_psec = 1000000000000 / 108;
				break;
			default:
				frame_period_psec = 1000000000000 / 120;
		}
		// 1/FPS + TES
		vsync_to_photons_psec = VID_VS_DELAY * EXTERNAL_LINE_TIME_PSEC + frame_period_psec + gpo1_tes1 * INTERNAL_LINE_TIME_PSEC;
	}
	seq_printf(s, "%llu\n", vsync_to_photons_psec / 1000000);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(panel_info);
DEFINE_SHOW_ATTRIBUTE(panel_mfg_info);
DEFINE_SHOW_ATTRIBUTE(panel_available);
DEFINE_SHOW_ATTRIBUTE(panel_vsync_to_photons_usec);
DEFINE_SHOW_STORE_ATTRIBUTE(panel_read);
DEFINE_SHOW_STORE_ATTRIBUTE(panel_write);

static void lpm026m648c_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	debugfs_create_file("panel_info", 0600, root, panel, &panel_info_fops);
	debugfs_create_file("panel_mfg_info", 0600, root, panel, &panel_mfg_info_fops);
	debugfs_create_file("panel_available", 0600, root, panel, &panel_available_fops);
	debugfs_create_file("panel_vsync_to_photons_usec", 0600, root, panel, &panel_vsync_to_photons_usec_fops);
	debugfs_create_file("panel_write", 0600, root, panel, &panel_write_fops);
	debugfs_create_file("panel_read", 0600, root, panel, &panel_read_fops);
}

static const struct drm_panel_funcs lpm026m648c_drm_funcs = {
	.disable = lpm026m648c_disable,
	.enable = lpm026m648c_enable,
	.unprepare = lpm026m648c_unprepare,
	.prepare = lpm026m648c_prepare,
	.get_modes = lpm026m648c_get_modes,
    .mode_set = lpm026m648c_mode_set,
	.debugfs_init = lpm026m648c_debugfs_init,
};

static int lpm026m648c_panel_add(struct lpm026m648c_ctx *ctx)
{
	struct device *dev = ctx->dev;
	struct backlight_properties bl_props;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(ctx->power_supplies); i++)
		ctx->power_supplies[i].supply = regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->power_supplies),
				      ctx->power_supplies);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "Failed to get power supplies\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ctx->preboost_drivers); i++)
						ctx->preboost_drivers[i].supply = preboost_driver_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->preboost_drivers),
						ctx->preboost_drivers);
	if (ret < 0){
		DRM_DEV_ERROR(ctx->dev, "Failed to get preboost supplies\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ctx->backlight_drivers); i++)
		ctx->backlight_drivers[i].supply = backlight_driver_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->backlight_drivers),
				      ctx->backlight_drivers);
	if (ret < 0){
		DRM_DEV_ERROR(ctx->dev, "Failed to get backlight supplies\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ctx->lcd_bias_regulators); i++)
		ctx->lcd_bias_regulators[i].supply = lcd_bias_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->lcd_bias_regulators),
		ctx->lcd_bias_regulators);
	if (ret < 0){
		DRM_DEV_ERROR(ctx->dev, "Failed to get lcd bias supplies\n");
		return ret;
	}

	ctx->reset_gpios = devm_gpiod_get_array(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpios)){
		DRM_DEV_ERROR(ctx->dev, "Failed to get reset gpio\n");
		return PTR_ERR(ctx->reset_gpios);
	}

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = BRIGHTNESS_DEFAULT;
	bl_props.max_brightness = BRIGHTNESS_MAX_120;

	ctx->backlight = devm_backlight_device_register(dev, dev_name(dev),
							dev, ctx->dsi[0],
							&lpm026m648c_bl_ops,
							&bl_props);
	if (IS_ERR(ctx->backlight)) {
		ret = PTR_ERR(ctx->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_add(&ctx->panel);

	return 0;
}

static int lpm026m648c_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lpm026m648c_ctx *ctx;
	struct mipi_dsi_device *dsi1_device;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct mipi_dsi_device *dsi_dev;
	int ret = 0, dsi_num = 1;
	int i;

	const struct mipi_dsi_device_info info = {
		.type = "deckard4k",
		.channel = 0,
		.node = NULL,
	};

	ctx = devm_drm_panel_alloc(dev, struct lpm026m648c_ctx, panel,
			&lpm026m648c_drm_funcs,
			DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	mutex_init(&ctx->gamma_lock);

	ctx->panel.prepare_prev_first = true;

	ctx->dphy = of_property_read_bool(dev->of_node, "dphy");

	ctx->is_dual_panel = of_property_read_bool(dev->of_node, "dual-panel");
	if (!ctx->is_dual_panel)
		goto skip_panel2;

	dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
	if (!dsi1) {
		DRM_DEV_ERROR(dev, "failed to get secondary dsi\n");
		return -ENODEV;
	}

	dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
	of_node_put(dsi1);
	if (!dsi1_host) {
		DRM_DEV_ERROR(dev, "failed to find secondary dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi1_device = mipi_dsi_device_register_full(dsi1_host, &info);
	if (IS_ERR(dsi1_device)) {
		DRM_DEV_ERROR(dev, "failed to create dsi device\n");
		return PTR_ERR(dsi1_device);
	}
	ctx->dsi[1] = dsi1_device;
	dsi_num = 2;

skip_panel2:
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->dsi[0] = dsi;

#ifndef NO_DSC
	ret = lpm026m648c_init_dsc_config(ctx);
	if (ret)
		goto err_panel_add;

	ctx->dsi[0]->dsc = &ctx->dsc_cfg;
	ctx->dsi[0]->dsc_slice_per_pkt = 4;
	if (ctx->is_dual_panel) {
		ctx->dsi[1]->dsc = &ctx->dsc_cfg;
		ctx->dsi[1]->dsc_slice_per_pkt = 4;
	}
#endif

	ctx->low_fps_support = of_property_read_bool(dev->of_node, "low-fps-support");

	ret = lpm026m648c_panel_add(ctx);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to add panel %d\n", ret);
		goto err_panel_add;
	}

	for (i = 0; i < dsi_num; i++) {
		dsi_dev = ctx->dsi[i];
		if (!ctx->dphy ) {
			dsi_dev->lanes = 3;
		} else {
			dsi_dev->lanes = 4;
		}
		dsi_dev->format = MIPI_DSI_FMT_RGB888;
		dsi_dev->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;
		ret = mipi_dsi_attach(dsi_dev);
		if (ret < 0) {
			DRM_DEV_ERROR(dev, "dsi attach failed i = %d\n", i);
			goto err_dsi_attach;
		}
	}

	return 0;

err_dsi_attach:
	drm_panel_remove(&ctx->panel);
err_panel_add:
	mipi_dsi_device_unregister(dsi1_device);
	return -EPROBE_DEFER;;
}

static void lpm026m648c_remove(struct mipi_dsi_device *dsi)
{
	struct lpm026m648c_ctx *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->dsi[0])
		mipi_dsi_detach(ctx->dsi[0]);
	if (ctx->dsi[1]) {
		mipi_dsi_detach(ctx->dsi[1]);
		mipi_dsi_device_unregister(ctx->dsi[1]);
	}

	drm_panel_remove(&ctx->panel);
}

static ssize_t gamma_black_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct lpm026m648c_ctx *ctx = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ctx->gamma_black_mode);
}

static ssize_t gamma_black_mode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct lpm026m648c_ctx *ctx = dev_get_drvdata(dev);
	unsigned int mode;
	int ret;

	ret = kstrtouint(buf, 0, &mode);
	if (ret)
		return ret;
	if (mode > GAMMA_BLACK_MODE_MAX)
		return -EINVAL;

	mutex_lock(&ctx->gamma_lock);
	ctx->gamma_black_mode = mode;
	if (ctx->gamma_safe_to_write)
		update_gamma_black_mode(ctx);
	mutex_unlock(&ctx->gamma_lock);

	return count;
}
static DEVICE_ATTR_RW(gamma_black_mode);

// Same on every panel; non-BOE panels store the mode but ignore it.
static ssize_t gamma_black_mode_max_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", GAMMA_BLACK_MODE_MAX);
}
static DEVICE_ATTR_RO(gamma_black_mode_max);

static struct attribute *lpm026m648c_attrs[] = {
	&dev_attr_gamma_black_mode.attr,
	&dev_attr_gamma_black_mode_max.attr,
	NULL,
};
ATTRIBUTE_GROUPS(lpm026m648c);

static const struct of_device_id lpm026m648c_of_match[] = {
	{ .compatible = "deckard-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpm026m648c_of_match);

static struct mipi_dsi_driver lpm026m648c_driver = {
	.driver = {
		.name = "deckard-panel",
		.of_match_table = lpm026m648c_of_match,
		.dev_groups = lpm026m648c_groups,
	},
	.probe = lpm026m648c_probe,
	.remove = lpm026m648c_remove,
};
module_mipi_dsi_driver(lpm026m648c_driver);

MODULE_DESCRIPTION("Sharp LS0DAS7232 DSI Panel Driver with BOE support");
MODULE_LICENSE("GPL v2");
