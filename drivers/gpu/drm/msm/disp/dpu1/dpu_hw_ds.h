/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _DPU_HW_DS_H
#define _DPU_HW_DS_H

#include "dpu_hw_catalog.h"
#include "dpu_hw_util.h"

struct dpu_hw_ds;

/**
 * struct dpu_hw_ds_ops - interface to the dest scaler
 */
struct dpu_hw_ds_ops {
	/**
	 * @setup_scaler: program the QSEED scaler, including its detail enhancer
	 * @ctx: Pointer to dest scaler context
	 * @cfg: Pointer to scaler configuration
	 */
	void (*setup_scaler)(struct dpu_hw_ds *ctx, struct dpu_hw_scaler3_cfg *cfg);

	/**
	 * @disable: turn the scaler off, so the LM output passes through unchanged
	 * @ctx: Pointer to dest scaler context
	 */
	void (*disable)(struct dpu_hw_ds *ctx);
};

/**
 * struct dpu_hw_ds - dest scaler description
 * @hw: block register map object
 * @idx: dest scaler index
 * @cap: pointer to the catalog entry
 * @ops: dest scaler operations
 */
struct dpu_hw_ds {
	struct dpu_hw_blk_reg_map hw;
	enum dpu_ds idx;
	const struct dpu_ds_cfg *cap;
	struct dpu_hw_ds_ops ops;
};

struct dpu_hw_ds *dpu_hw_ds_init(struct drm_device *dev,
				 const struct dpu_ds_cfg *cfg,
				 void __iomem *addr);

#endif /* _DPU_HW_DS_H */
