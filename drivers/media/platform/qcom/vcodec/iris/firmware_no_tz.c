// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Linaro Ltd.
 */

#include <linux/iommu.h>
#include <linux/of_device.h>
#include "iris_core.h"
#include "firmware_no_tz.h"

#define VENUS_FW_MEM_SIZE		(8 * SZ_1M)
#define VENUS_FW_START_ADDR		0x0

#define WRAPPER_TZ_XTSS_SW_RESET 	0xc1000
#define WRAPPER_CPA_START_ADDR		0xc1020
#define WRAPPER_CPA_END_ADDR		0xc1024
#define WRAPPER_FW_START_ADDR		0xc1028
#define WRAPPER_FW_END_ADDR		0xc102c
#define WRAPPER_NONPIX_START_ADDR	0xc1030
#define WRAPPER_NONPIX_END_ADDR		0xc1034

#define WRAPPER_XTSS_SW_RESET_BIT	BIT(0)

int init_fw_no_tz(struct iris_core *core)
{
	struct platform_device_info info;
	struct iommu_domain *iommu_dom;
	struct platform_device *pdev;
	struct device_node *np;
	int ret;

	np = of_get_child_by_name(core->dev->of_node, "video-firmware");

	memset(&info, 0, sizeof(info));
	info.fwnode = &np->fwnode;
	info.parent = core->dev;
	info.name = np->name;
	info.dma_mask = DMA_BIT_MASK(32);

	pdev = platform_device_register_full(&info);
	if (IS_ERR(pdev)) {
		of_node_put(np);
		return PTR_ERR(pdev);
	}

	pdev->dev.of_node = np;

	ret = of_dma_configure(&pdev->dev, np, true);
	if (ret) {
		dev_err(core->dev, "dma configure fail\n");
		goto err_unregister;
	}

	core->fw.dev = &pdev->dev;

	iommu_dom = iommu_paging_domain_alloc(core->fw.dev);
	if (!iommu_dom) {
		dev_err(core->fw.dev, "Failed to allocate iommu domain\n");
		ret = -ENOMEM;
		goto err_unregister;
	}

	ret = iommu_attach_device(iommu_dom, core->fw.dev);
	if (ret) {
		dev_err(core->fw.dev, "could not attach device\n");
		goto err_iommu_free;
	}

	core->fw.iommu_domain = iommu_dom;

	of_node_put(np);

	return 0;

err_iommu_free:
	iommu_domain_free(iommu_dom);
err_unregister:
	platform_device_unregister(pdev);
	of_node_put(np);
	return ret;
}

void deinit_fw_no_tz(struct iris_core *core)
{
	struct iommu_domain *iommu;

	if (!core->fw.dev)
		return;

	iommu = core->fw.iommu_domain;

	iommu_detach_device(iommu, core->fw.dev);

	if (iommu) {
		iommu_domain_free(iommu);
		iommu = NULL;
	}

	platform_device_unregister(to_platform_device(core->fw.dev));
}

static void reset_cpu_no_tz(struct iris_core *core)
{
	u32 fw_size = core->fw.mapped_mem_size;
	void __iomem *wrapper_base = core->reg_base;

	writel(0, wrapper_base + WRAPPER_FW_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_FW_END_ADDR);
	writel(0, wrapper_base + WRAPPER_CPA_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_CPA_END_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_NONPIX_START_ADDR);
	writel(fw_size, wrapper_base + WRAPPER_NONPIX_END_ADDR);

	/* Bring XTSS out of reset */
	writel(0, wrapper_base + WRAPPER_TZ_XTSS_SW_RESET);

	writel(0, wrapper_base + 0xB0088); // WRAPPER_CORE_CLOCK_CONFIG (CORE_AXI_CLK_HALT=0, CORE_CLK_HALT=0)
}

int set_hw_state_no_tz(struct iris_core *core, bool resume)
{
	if (resume) {
		reset_cpu_no_tz(core);
	} else {
		writel(WRAPPER_XTSS_SW_RESET_BIT,
		       core->reg_base + WRAPPER_TZ_XTSS_SW_RESET);
	}
	return 0;
}

int auth_reset_fw_no_tz(struct iris_core *core, phys_addr_t mem_phys,
			size_t mem_size)
{
	struct iommu_domain *iommu;
	struct device *dev;
	int ret;

	dev = core->fw.dev;
	if (!dev)
		return -EPROBE_DEFER;

	iommu = core->fw.iommu_domain;
	core->fw.mapped_mem_size = mem_size;

	ret = iommu_map(iommu, VENUS_FW_START_ADDR, mem_phys, mem_size,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "could not map video firmware region\n");
		return ret;
	}

	reset_cpu_no_tz(core);

	return 0;
}

void unload_fw_no_tz(struct iris_core *core)
{
	const size_t mapped = core->fw.mapped_mem_size;
	struct iommu_domain *iommu;
	size_t unmapped;
	u32 reg;
	struct device *dev = core->fw.dev;

	/* Assert the reset to XTSS */
	reg = readl(core->reg_base + WRAPPER_TZ_XTSS_SW_RESET);
	reg |= WRAPPER_XTSS_SW_RESET_BIT;
	writel(reg, core->reg_base + WRAPPER_TZ_XTSS_SW_RESET);

	iommu = core->fw.iommu_domain;

	if (core->fw.mapped_mem_size && iommu) {
		unmapped = iommu_unmap(iommu, VENUS_FW_START_ADDR, mapped);

		if (unmapped != mapped)
			dev_err(dev, "failed to unmap firmware\n");
		else
			core->fw.mapped_mem_size = 0;
	}
}
