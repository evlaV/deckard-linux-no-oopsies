// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "firmware.h"

bool use_tz(struct device *core_dev)
{
	struct device_node *np;

	np = of_get_child_by_name(core_dev->of_node, "video-firmware");
	if (!np)
		return true;

	return false;
}

int protect_secure_region(u32 cp_start, u32 cp_size, u32 cp_nonpixel_start,
			  u32 cp_nonpixel_size, u32 pas_id)
{
	WARN_ON(1);
	return -ENOENT;

	int ret;
	/*
	 * Clues for porting using downstream data:
	 * cp_start = 0
	 * cp_size = venus_ns/virtual-addr-pool[0] - yes, address and not size!
	 *   This works, as the non-secure context bank is placed
	 *   contiguously right after the Content Protection region.
	 *
	 * cp_nonpixel_start = venus_sec_non_pixel/virtual-addr-pool[0]
	 * cp_nonpixel_size = venus_sec_non_pixel/virtual-addr-pool[1]
	 */
	ret = qcom_scm_mem_protect_video_var(cp_start,
					     cp_size,
					     cp_nonpixel_start,
					     cp_nonpixel_size);
	if (ret)
		qcom_scm_pas_shutdown(pas_id);

	return ret;
}

int load_fw(struct device *dev, const char *fw_name, phys_addr_t *mem_phys,
	    size_t *mem_size, u32 pas_id, bool use_tz)
{
	const struct firmware *firmware = NULL;
	struct reserved_mem *rmem;
	struct device_node *node;
	void *mem_virt = NULL;
	ssize_t fw_size = 0;
	int ret;

	if (!IS_ENABLED(CONFIG_QCOM_MDT_LOADER))
		return -EPROBE_DEFER;

	if (!fw_name || !(*fw_name))
		return -EINVAL;

	*mem_phys = 0;
	*mem_size = 0;

	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(dev, "no memory-region specified\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!rmem) {
		dev_err(dev, "failed to lookup reserved memory-region\n");
		return -EINVAL;
	}

	ret = request_firmware(&firmware, fw_name, dev);
	if (ret) {
		dev_err(dev, "%s: failed to request fw \"%s\", error %d\n",
			__func__, fw_name, ret);
		return ret;
	}

	fw_size = qcom_mdt_get_size(firmware);
	if (fw_size < 0) {
		ret = fw_size;
		dev_err(dev, "%s: out of bound fw image fw size: %ld\n",
			__func__, fw_size);
		goto err_release_fw;
	}

	*mem_phys = rmem->base;
	*mem_size = rmem->size;

	if (*mem_size < fw_size) {
		ret = -EINVAL;
		goto err_release_fw;
	}

	mem_virt = memremap(*mem_phys, *mem_size, MEMREMAP_WC);
	if (!mem_virt) {
		dev_err(dev, "unable to remap fw memory region %pa size %#zx\n",
			mem_phys, *mem_size);
		goto err_release_fw;
	}

	if (0)
		ret = qcom_mdt_load(dev, firmware, fw_name, pas_id, mem_virt,
				    *mem_phys, *mem_size, NULL);
	else
		ret = qcom_mdt_load_no_init(dev, firmware, fw_name, mem_virt,
					    *mem_phys, *mem_size, NULL);
	if (ret) {
		dev_err(dev, "%s: error %d loading fw \"%s\"\n",
			__func__, ret, fw_name);
	}

	memunmap(mem_virt);
err_release_fw:
	release_firmware(firmware);
	return ret;
}

int auth_reset_fw(u32 pas_id)
{
	WARN_ON(1);
	return -ENOENT; // qcom_scm_pas_auth_and_reset(pas_id);
}

void unload_fw(u32 pas_id)
{
	WARN_ON(1);
	//qcom_scm_pas_shutdown(pas_id);
}

int set_hw_state(bool resume)
{
	WARN_ON(1);
	return -ENOENT; //return qcom_scm_set_remote_state(resume, 0);
}
