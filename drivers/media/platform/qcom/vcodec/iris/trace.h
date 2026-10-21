/* SPDX-License-Identifier: GPL-2.0 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM qcom_iris

#if !defined(QCOM_IRIS_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define QCOM_IRIS_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(iris_hfi_queue_buffer,
	TP_PROTO(unsigned int arg1),
	TP_ARGS(arg1),
	TP_STRUCT__entry(
		__field(unsigned int, arg1)
	),
	TP_fast_assign(
		__entry->arg1 = arg1;
	),
	TP_printk("arg1 = %u", __entry->arg1)
);

TRACE_EVENT(iris_vdec_streamon_input,
	TP_PROTO(unsigned int arg1),
	TP_ARGS(arg1),
	TP_STRUCT__entry(
		__field(unsigned int, arg1)
	),
	TP_fast_assign(
		__entry->arg1 = arg1;
	),
	TP_printk("arg1 = %u", __entry->arg1)
);

#endif /* QCOM_IRIS_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/media/platform/qcom/vcodec/iris
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>
