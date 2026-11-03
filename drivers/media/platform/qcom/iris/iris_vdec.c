// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/soc/qcom/ubwc.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>

#include "iris_buffer.h"
#include "iris_common.h"
#include "iris_ctrls.h"
#include "iris_instance.h"
#include "iris_power.h"
#include "iris_vdec.h"
#include "iris_vpu_buffer.h"

#define DEFAULT_CODEC_ALIGNMENT 16

int iris_vdec_inst_init(struct iris_inst *inst)
{
	struct iris_core *core = inst->core;
	struct v4l2_format *f;

	inst->fmt_src = kzalloc_obj(*inst->fmt_src);
	inst->fmt_dst = kzalloc_obj(*inst->fmt_dst);

	inst->fw_min_count = 0;

	f = inst->fmt_src;
	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f->fmt.pix_mp.width = DEFAULT_WIDTH;
	f->fmt.pix_mp.height = DEFAULT_HEIGHT;
	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	inst->codec = f->fmt.pix_mp.pixelformat;
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = 0;
	f->fmt.pix_mp.plane_fmt[0].sizeimage = iris_get_buffer_size(inst, BUF_INPUT);
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	inst->buffers[BUF_INPUT].min_count = iris_vpu_buf_count(inst, BUF_INPUT);
	inst->buffers[BUF_INPUT].size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	f = inst->fmt_dst;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	f->fmt.pix_mp.width = ALIGN(DEFAULT_WIDTH, 128);
	f->fmt.pix_mp.height = ALIGN(DEFAULT_HEIGHT, 32);
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = ALIGN(DEFAULT_WIDTH, 128);
	f->fmt.pix_mp.plane_fmt[0].sizeimage = iris_get_buffer_size(inst, BUF_OUTPUT);
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
	f->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	f->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	inst->buffers[BUF_OUTPUT].min_count = iris_vpu_buf_count(inst, BUF_OUTPUT);
	inst->buffers[BUF_OUTPUT].size = f->fmt.pix_mp.plane_fmt[0].sizeimage;
	inst->frame_rate = MAXIMUM_FPS;

	memcpy(&inst->fw_caps[0], &core->inst_fw_caps_dec[0],
	       INST_FW_CAP_MAX * sizeof(struct platform_inst_fw_cap));

	return iris_ctrls_init(inst);
}

/*
 * Publish the driver's current minimum CAPTURE buffer count to userspace:
 * both V4L2_CID_MIN_BUFFERS_FOR_CAPTURE and the queue's minimum allocation.
 * Call whenever inst->buffers[BUF_OUTPUT].min_count is recalculated.
 */
void iris_vdec_update_min_capture_bufs(struct iris_inst *inst)
{
	u32 count = inst->buffers[BUF_OUTPUT].min_count;
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_find(&inst->ctrl_handler, V4L2_CID_MIN_BUFFERS_FOR_CAPTURE);
	if (ctrl) {
		if (count > ctrl->maximum)
			dev_warn_ratelimited(inst->core->dev,
					     "min capture buffers %u exceeds control maximum %lld\n",
					     count, ctrl->maximum);
		v4l2_ctrl_s_ctrl(ctrl, count);
	}

	v4l2_m2m_get_dst_vq(inst->m2m_ctx)->min_reqbufs_allocation = count;
}

static const u32 iris_vdec_formats_cap[] = {
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_QC08C,
	V4L2_PIX_FMT_P010,
	V4L2_PIX_FMT_QC10C,
};

static const u32 iris_vdec_formats_noubwc_cap[] = {
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_P010,
};

static bool check_format(struct iris_inst *inst, u32 pixfmt, u32 type)
{
	const struct qcom_ubwc_cfg_data *ubwc = inst->core->ubwc_cfg;
	unsigned int size, i;
	const u32 *fmt;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		fmt = inst->core->iris_platform_data->inst_iris_fmts;
		size = inst->core->iris_platform_data->inst_iris_fmts_size;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		if (ubwc->ubwc_enc_version) {
			fmt = iris_vdec_formats_cap;
			size = ARRAY_SIZE(iris_vdec_formats_cap);
		} else {
			fmt = iris_vdec_formats_noubwc_cap;
			size = ARRAY_SIZE(iris_vdec_formats_noubwc_cap);
		}
		break;
	default:
		return false;
	}

	for (i = 0; i < size; i++) {
		if (fmt[i] == pixfmt)
			break;
	}

	if (i == size)
		return false;

	return true;
}

static u32 find_format_by_index(struct iris_inst *inst, u32 index, u32 type)
{
	const struct qcom_ubwc_cfg_data *ubwc = inst->core->ubwc_cfg;
	unsigned int size;
	const u32 *fmt;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		fmt = inst->core->iris_platform_data->inst_iris_fmts;
		size = inst->core->iris_platform_data->inst_iris_fmts_size;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		if (ubwc->ubwc_enc_version) {
			fmt = iris_vdec_formats_cap;
			size = ARRAY_SIZE(iris_vdec_formats_cap);
		} else {
			fmt = iris_vdec_formats_noubwc_cap;
			size = ARRAY_SIZE(iris_vdec_formats_noubwc_cap);
		}
		break;
	default:
		return 0;
	}

	if (index >= size)
		return 0;

	return fmt[index];
}

int iris_vdec_enum_fmt(struct iris_inst *inst, struct v4l2_fmtdesc *f)
{
	u32 fmt;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		fmt = find_format_by_index(inst, f->index, f->type);
		if (!fmt)
			return -EINVAL;

		f->pixelformat = fmt;
		f->flags = V4L2_FMT_FLAG_COMPRESSED | V4L2_FMT_FLAG_DYN_RESOLUTION;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		fmt = find_format_by_index(inst, f->index, f->type);
		if (!fmt)
			return -EINVAL;
		f->pixelformat = fmt;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int iris_vdec_try_fmt(struct iris_inst *inst, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_format *f_inst;
	struct vb2_queue *src_q;
	bool supported;

	memset(pixmp->reserved, 0, sizeof(pixmp->reserved));
	supported = check_format(inst, pixmp->pixelformat, f->type);
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		if (!supported) {
			f_inst = inst->fmt_src;
			f->fmt.pix_mp.width = f_inst->fmt.pix_mp.width;
			f->fmt.pix_mp.height = f_inst->fmt.pix_mp.height;
			f->fmt.pix_mp.pixelformat = f_inst->fmt.pix_mp.pixelformat;
		}
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		if (!supported) {
			f_inst = inst->fmt_dst;
			f->fmt.pix_mp.pixelformat = f_inst->fmt.pix_mp.pixelformat;
			f->fmt.pix_mp.width = f_inst->fmt.pix_mp.width;
			f->fmt.pix_mp.height = f_inst->fmt.pix_mp.height;
		}

		src_q = v4l2_m2m_get_src_vq(m2m_ctx);
		if (vb2_is_streaming(src_q)) {
			f_inst = inst->fmt_src;
			f->fmt.pix_mp.height = f_inst->fmt.pix_mp.height;
			f->fmt.pix_mp.width = f_inst->fmt.pix_mp.width;
		}
		break;
	default:
		return -EINVAL;
	}

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	pixmp->num_planes = 1;

	return 0;
}

static void iris_vdec_set_capture_size(struct iris_inst *inst, u32 width,
				       u32 height)
{
	struct v4l2_pix_format_mplane *pix_mp = &inst->fmt_dst->fmt.pix_mp;
	struct iris_buffers *bufs = &inst->buffers[BUF_OUTPUT];

	switch (pix_mp->pixelformat) {
	case V4L2_PIX_FMT_P010:
		pix_mp->width = ALIGN(width, 128);
		pix_mp->height = ALIGN(height, 32);
		pix_mp->plane_fmt[0].bytesperline = ALIGN(width * 2, 256);
		break;
	case V4L2_PIX_FMT_QC10C:
		pix_mp->width = roundup(width, 192);
		pix_mp->height = ALIGN(height, 16);
		pix_mp->plane_fmt[0].bytesperline = ALIGN(width * 4 / 3, 256);
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_QC08C:
		pix_mp->width = ALIGN(width, 128);
		pix_mp->height = ALIGN(height, 32);
		pix_mp->plane_fmt[0].bytesperline = ALIGN(width, 128);
		break;
	}
	pix_mp->plane_fmt[0].sizeimage = iris_get_buffer_size(inst, BUF_OUTPUT);
	bufs->min_count = iris_vpu_buf_count(inst, BUF_OUTPUT);
	bufs->size = pix_mp->plane_fmt[0].sizeimage;
}

int iris_vdec_s_fmt(struct iris_inst *inst, struct v4l2_format *f)
{
	struct v4l2_format *fmt, *output_fmt;
	struct vb2_queue *q;
	u32 codec_align;

	q = v4l2_m2m_get_vq(inst->m2m_ctx, f->type);

	if (vb2_is_busy(q))
		return -EBUSY;

	/* Width and height are optional, so fall back to a valid placeholder
	 * resolution until the real one is decoded from the bitstream.
	 */
	if (f->fmt.pix_mp.width == 0 && f->fmt.pix_mp.height == 0) {
		f->fmt.pix_mp.width = DEFAULT_WIDTH;
		f->fmt.pix_mp.height = DEFAULT_HEIGHT;
	}

	iris_vdec_try_fmt(inst, f);

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		if (!check_format(inst, f->fmt.pix_mp.pixelformat, f->type))
			return -EINVAL;

		fmt = inst->fmt_src;
		fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		fmt->fmt.pix_mp.pixelformat = f->fmt.pix_mp.pixelformat;
		inst->codec = fmt->fmt.pix_mp.pixelformat;
		/* The source queue is idle: any firmware count is from an old stream. */
		inst->fw_min_count = 0;
		codec_align = inst->codec == V4L2_PIX_FMT_HEVC ? 32 : 16;
		fmt->fmt.pix_mp.width = ALIGN(f->fmt.pix_mp.width, codec_align);
		fmt->fmt.pix_mp.height = ALIGN(f->fmt.pix_mp.height, codec_align);
		fmt->fmt.pix_mp.num_planes = 1;
		fmt->fmt.pix_mp.plane_fmt[0].bytesperline = 0;
		fmt->fmt.pix_mp.plane_fmt[0].sizeimage = iris_get_buffer_size(inst, BUF_INPUT);
		inst->buffers[BUF_INPUT].min_count = iris_vpu_buf_count(inst, BUF_INPUT);
		inst->buffers[BUF_INPUT].size = fmt->fmt.pix_mp.plane_fmt[0].sizeimage;

		fmt->fmt.pix_mp.colorspace = f->fmt.pix_mp.colorspace;
		fmt->fmt.pix_mp.xfer_func = f->fmt.pix_mp.xfer_func;
		fmt->fmt.pix_mp.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
		fmt->fmt.pix_mp.quantization = f->fmt.pix_mp.quantization;

		output_fmt = inst->fmt_dst;
		output_fmt->fmt.pix_mp.colorspace = f->fmt.pix_mp.colorspace;
		output_fmt->fmt.pix_mp.xfer_func = f->fmt.pix_mp.xfer_func;
		output_fmt->fmt.pix_mp.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
		output_fmt->fmt.pix_mp.quantization = f->fmt.pix_mp.quantization;

		/* Update capture format based on new ip w/h */
		iris_vdec_set_capture_size(inst, f->fmt.pix_mp.width,
					   f->fmt.pix_mp.height);
		iris_vdec_update_min_capture_bufs(inst);

		inst->crop.left = 0;
		inst->crop.top = 0;
		inst->crop.width = f->fmt.pix_mp.width;
		inst->crop.height = f->fmt.pix_mp.height;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		if (!check_format(inst, f->fmt.pix_mp.pixelformat, f->type))
			return -EINVAL;

		fmt = inst->fmt_dst;
		fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		fmt->fmt.pix_mp.pixelformat = f->fmt.pix_mp.pixelformat;
		fmt->fmt.pix_mp.num_planes = 1;
		iris_vdec_set_capture_size(inst, f->fmt.pix_mp.width,
					   f->fmt.pix_mp.height);
		iris_vdec_update_min_capture_bufs(inst);

		inst->crop.top = 0;
		inst->crop.left = 0;
		inst->crop.width = f->fmt.pix_mp.width;
		inst->crop.height = f->fmt.pix_mp.height;
		break;
	default:
		return -EINVAL;
	}
	memcpy(f, fmt, sizeof(*fmt));

	return 0;
}

int iris_vdec_validate_format(struct iris_inst *inst, u32 pixelformat)
{
	bool supported;

	supported = check_format(inst, pixelformat, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!supported)
		supported = check_format(inst, pixelformat, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	return supported ? 0 : -EINVAL;
}

int iris_vdec_subscribe_event(struct iris_inst *inst, const struct v4l2_event_subscription *sub)
{
	int ret = 0;

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		ret = v4l2_event_subscribe(&inst->fh, sub, 0, NULL);
		break;
	case V4L2_EVENT_SOURCE_CHANGE:
		ret = v4l2_src_change_event_subscribe(&inst->fh, sub);
		break;
	case V4L2_EVENT_CTRL:
		ret = v4l2_ctrl_subscribe_event(&inst->fh, sub);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

void iris_vdec_update_drc_format(struct iris_inst *inst, const struct v4l2_format *old_fmt)
{
	const struct v4l2_pix_format_mplane *old = &old_fmt->fmt.pix_mp;
	const struct v4l2_pix_format_mplane *cap = &inst->fmt_dst->fmt.pix_mp;

	inst->drc_cap_fmt_unchanged = cap->pixelformat == old->pixelformat &&
		cap->width == old->width && cap->height == old->height &&
		cap->plane_fmt[0].bytesperline == old->plane_fmt[0].bytesperline;
	inst->drc_start_pending = false;
}

void iris_vdec_src_change(struct iris_inst *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_event event = {0};
	struct vb2_queue *src_q;

	src_q = v4l2_m2m_get_src_vq(m2m_ctx);
	if (!vb2_is_streaming(src_q))
		return;

	event.type = V4L2_EVENT_SOURCE_CHANGE;
	event.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION;
	v4l2_event_queue_fh(&inst->fh, &event);
}

int iris_vdec_streamon_input(struct iris_inst *inst)
{
	int ret;

	ret = iris_set_properties(inst, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (ret)
		return ret;

	ret = iris_alloc_and_queue_persist_bufs(inst, BUF_PERSIST);
	if (ret)
		return ret;

	iris_get_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	ret = iris_destroy_dequeued_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (ret)
		return ret;

	ret = iris_create_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (ret)
		return ret;

	ret = iris_queue_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (ret)
		return ret;

	inst->frame_counter = 0;

	return iris_process_streamon_input(inst);
}

static bool iris_vdec_input_with_fw(struct iris_inst *inst)
{
	struct v4l2_m2m_buffer *m2m_buf, *n;

	v4l2_m2m_for_each_src_buf_safe(inst->m2m_ctx, m2m_buf, n) {
		if (to_iris_buffer(&m2m_buf->vb)->attr & BUF_ATTR_QUEUED)
			return true;
	}

	return false;
}

#define IRIS_FIRST_IPSC_WAIT_MS	1000

bool iris_vdec_wait_first_ipsc(struct iris_inst *inst)
{
	unsigned long start = jiffies;
	unsigned long timeout = start + msecs_to_jiffies(IRIS_FIRST_IPSC_WAIT_MS);
	bool waited = false;

	while (inst->state == IRIS_INST_INPUT_STREAMING &&
	       !(inst->sub_state & IRIS_INST_SUB_FIRST_IPSC) &&
	       iris_vdec_input_with_fw(inst) && time_before(jiffies, timeout)) {
		waited = true;
		mutex_unlock(&inst->lock);
		usleep_range(500, 1000);
		mutex_lock(&inst->lock);
	}

	if (waited && !(inst->sub_state & IRIS_INST_SUB_FIRST_IPSC))
		dev_warn(inst->core->dev,
			 "first source change not handled within %u ms, starting capture anyway (state %d)\n",
			 IRIS_FIRST_IPSC_WAIT_MS, inst->state);
	else if (waited)
		dev_dbg(inst->core->dev, "first source change handled after %u ms\n",
			 jiffies_to_msecs(jiffies - start));

	return waited && (inst->sub_state & IRIS_INST_SUB_FIRST_IPSC);
}

bool iris_vdec_capture_fits_stream(struct iris_inst *inst)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(inst->m2m_ctx);
	u32 size = inst->fmt_dst->fmt.pix_mp.plane_fmt[0].sizeimage;
	struct vb2_buffer *vb;
	unsigned int i;

	if (vb2_get_num_buffers(q) < inst->buffers[BUF_OUTPUT].min_count)
		return false;

	for (i = 0; i < q->max_num_buffers; i++) {
		vb = vb2_get_buffer(q, i);
		if (vb && vb2_plane_size(vb, 0) < size)
			return false;
	}

	return true;
}

int iris_vdec_streamon_output(struct iris_inst *inst)
{
	const struct iris_hfi_session_ops *hfi_ops = inst->hfi_session_ops;
	int ret;

	ret = hfi_ops->session_set_config_params(inst, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (ret)
		return ret;

	iris_get_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	ret = iris_destroy_dequeued_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (ret)
		return ret;

	ret = iris_create_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (ret)
		return ret;

	ret = iris_process_streamon_output(inst);
	if (ret)
		goto error;

	ret = iris_queue_internal_buffers(inst, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (ret)
		goto error;

	return ret;

error:
	iris_session_streamoff(inst, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	return ret;
}

int iris_vdec_qbuf(struct iris_inst *inst, struct vb2_v4l2_buffer *vbuf)
{
	struct iris_buffer *buf = to_iris_buffer(vbuf);
	struct vb2_buffer *vb2 = &vbuf->vb2_buf;
	u64 cur_buf_ns, delta_ns;
	struct vb2_queue *q;
	int ret;

	ret = iris_vb2_buffer_to_driver(vb2, buf);
	if (ret)
		return ret;

	if (buf->type == BUF_INPUT)
		iris_set_ts_metadata(inst, vbuf);

	q = v4l2_m2m_get_vq(inst->m2m_ctx, vb2->type);
	if (!vb2_is_streaming(q)) {
		buf->attr |= BUF_ATTR_DEFERRED;
		return 0;
	}

	if (buf->type == BUF_INPUT) {
		cur_buf_ns = ktime_get_ns();

		if (!inst->frame_counter)
			inst->last_buf_ns = cur_buf_ns;

		inst->frame_counter++;
		delta_ns = cur_buf_ns - inst->last_buf_ns;

		if (delta_ns >= NSEC_PER_SEC) {
			inst->frame_rate = clamp_t(u32, inst->frame_counter, DEFAULT_FPS,
						   MAXIMUM_FPS);
			inst->frame_counter = 0;
		}
	}

	iris_scale_power(inst);

	return iris_queue_buffer(inst, buf);
}

/* Check actual allocations: the firmware's new size is not the allocated size. */
static bool iris_vdec_drc_capture_sufficient(struct iris_inst *inst)
{
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(inst->m2m_ctx);
	u32 size = inst->fmt_dst->fmt.pix_mp.plane_fmt[0].sizeimage;
	struct vb2_buffer *vb;
	unsigned int i;

	if (!inst->drc_cap_fmt_unchanged ||
	    vb2_get_num_buffers(q) < inst->buffers[BUF_OUTPUT].min_count)
		return false;

	for (i = 0; i < q->max_num_buffers; i++) {
		vb = vb2_get_buffer(q, i);
		if (vb && vb2_plane_size(vb, 0) < size)
			return false;
	}

	return true;
}

static bool iris_vdec_last_buf_in_done_list(struct vb2_queue *q)
{
	struct vb2_buffer *vb;
	unsigned long flags;
	bool found = false;

	spin_lock_irqsave(&q->done_lock, flags);
	list_for_each_entry(vb, &q->done_list, done_entry) {
		if (to_vb2_v4l2_buffer(vb)->flags & V4L2_BUF_FLAG_LAST) {
			found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&q->done_lock, flags);

	return found;
}

/*
 * A source change that leaves the capture format unchanged and needs no more
 * capture buffers than are already allocated (typically the first one, after
 * userspace configured and started CAPTURE from the bitstream headers) needs
 * nothing from userspace. Resume in place as if V4L2_DEC_CMD_START had been
 * sent: the empty PSC_LAST buffer is held back and decoding continues, instead
 * of signalling an event and ending the capture queue with a LAST buffer that
 * clients which see no format change treat as end of stream.
 */
bool iris_vdec_drc_in_place(struct iris_inst *inst)
{
	/* Unsupported content moves the instance to ERROR: let userspace see the event. */
	if (inst->state != IRIS_INST_STREAMING ||
	    !vb2_is_streaming(v4l2_m2m_get_dst_vq(inst->m2m_ctx)) ||
	    !iris_vdec_drc_capture_sufficient(inst))
		return false;

	inst->drc_start_pending = true;

	return true;
}

int iris_vdec_start_cmd(struct iris_inst *inst)
{
	const struct iris_hfi_session_ops *hfi_ops = inst->hfi_session_ops;
	enum iris_inst_sub_state clear_sub_state = 0;
	struct vb2_queue *dst_vq;
	bool drc;
	int ret;

	dst_vq = v4l2_m2m_get_dst_vq(inst->m2m_ctx);

	if ((inst->sub_state & IRIS_INST_SUB_DRC) &&
	    !(inst->sub_state & IRIS_INST_SUB_DRC_LAST)) {
		if (!iris_vdec_drc_capture_sufficient(inst))
			return -EBUSY;
		inst->drc_start_pending = true;
		return 0;
	}

	drc = iris_drc_pending(inst);
	if (drc) {
		ret = iris_alloc_and_queue_input_int_bufs(inst);
		if (ret)
			return ret;
		ret = iris_set_stage(inst, STAGE);
		if (ret)
			return ret;
		ret = iris_set_pipe(inst, PIPE);
		if (ret)
			return ret;
	}

	if (inst->sub_state & IRIS_INST_SUB_DRC &&
	    inst->sub_state & IRIS_INST_SUB_DRC_LAST) {
		vb2_clear_last_buffer_dequeued(dst_vq);
		clear_sub_state = IRIS_INST_SUB_DRC | IRIS_INST_SUB_DRC_LAST;

		if (inst->sub_state & IRIS_INST_SUB_INPUT_PAUSE) {
			ret = hfi_ops->session_resume_drc(inst,
							  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
			if (ret)
				return ret;
			clear_sub_state |= IRIS_INST_SUB_INPUT_PAUSE;
		}
		if (inst->sub_state & IRIS_INST_SUB_OUTPUT_PAUSE) {
			ret = hfi_ops->session_resume_drc(inst,
							  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
			if (ret)
				return ret;
			clear_sub_state |= IRIS_INST_SUB_OUTPUT_PAUSE;
		}
	} else if (inst->sub_state & IRIS_INST_SUB_DRAIN &&
		   inst->sub_state & IRIS_INST_SUB_DRAIN_LAST) {
		vb2_clear_last_buffer_dequeued(dst_vq);
		clear_sub_state = IRIS_INST_SUB_DRAIN | IRIS_INST_SUB_DRAIN_LAST;
		if (inst->sub_state & IRIS_INST_SUB_INPUT_PAUSE) {
			if (hfi_ops->session_resume_drain) {
				ret =
				hfi_ops->session_resume_drain(inst,
							      V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
				if (ret)
					return ret;
			}

			clear_sub_state |= IRIS_INST_SUB_INPUT_PAUSE;
		}
		if (inst->sub_state & IRIS_INST_SUB_OUTPUT_PAUSE) {
			if (hfi_ops->session_resume_drain) {
				ret =
				hfi_ops->session_resume_drain(inst,
							      V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
				if (ret)
					return ret;
			}

			clear_sub_state |= IRIS_INST_SUB_OUTPUT_PAUSE;
		}
	} else {
		dev_err(inst->core->dev, "start called before receiving last_flag\n");
		iris_inst_change_state(inst, IRIS_INST_ERROR);
		return -EBUSY;
	}

	ret = iris_inst_change_sub_state(inst, clear_sub_state, 0);
	if (ret)
		return ret;

	inst->last_buffer_dequeued = false;
	v4l2_m2m_clear_state(inst->m2m_ctx);
	if (drc && iris_vdec_last_buf_in_done_list(dst_vq))
		inst->drc_unlatch_last = true;

	return iris_queue_deferred_buffers(inst, BUF_OUTPUT);
}

/* Called after the response batch, with inst->lock held. */
int iris_vdec_drc_deferred_start(struct iris_inst *inst)
{
	if (!inst->drc_start_pending || !iris_drc_pending(inst))
		return 0;

	inst->drc_start_pending = false;
	if (inst->state != IRIS_INST_STREAMING)
		return 0;

	return iris_vdec_start_cmd(inst);
}

int iris_vdec_stop_cmd(struct iris_inst *inst)
{
	const struct iris_hfi_session_ops *hfi_ops = inst->hfi_session_ops;
	int ret;

	ret = hfi_ops->session_drain(inst, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (ret)
		return ret;

	return iris_inst_change_sub_state(inst, 0, IRIS_INST_SUB_DRAIN);
}
