// SPDX-License-Identifier: GPL-2.0-only

#include <drm/drm_managed.h>

#include "dpu_hwio.h"
#include "dpu_hw_ds.h"

#define DEST_SCALER_OP_MODE	0x04

static void dpu_hw_ds_setup_scaler(struct dpu_hw_ds *ctx, struct dpu_hw_scaler3_cfg *cfg)
{
	dpu_hw_setup_scaler3(&ctx->hw, cfg, ctx->cap->scaler_blk.base,
			     ctx->cap->scaler_blk.version, NULL);
}

static void dpu_hw_ds_disable(struct dpu_hw_ds *ctx)
{
	DPU_REG_WRITE(&ctx->hw, ctx->cap->scaler_blk.base + DEST_SCALER_OP_MODE, 0);
}

/**
 * dpu_hw_ds_init() - Initializes the dest scaler hw driver object.
 * @dev:  Corresponding device for devres management
 * @cfg:  dest scaler catalog entry for which driver object is required
 * @addr: Mapped register io address of MDP
 * Return: pointer to structure or ERR_PTR
 */
struct dpu_hw_ds *dpu_hw_ds_init(struct drm_device *dev,
				 const struct dpu_ds_cfg *cfg,
				 void __iomem *addr)
{
	struct dpu_hw_ds *c;

	if (!addr)
		return ERR_PTR(-EINVAL);

	c = drmm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->hw.blk_addr = addr + cfg->base;
	c->hw.log_mask = DPU_DBG_MASK_DS;

	c->idx = cfg->id;
	c->cap = cfg;
	c->ops.setup_scaler = dpu_hw_ds_setup_scaler;
	c->ops.disable = dpu_hw_ds_disable;

	return c;
}
