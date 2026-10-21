// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/interconnect.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sort.h>

#include "iris_core.h"
#include "platform_common.h"
#include "resources.h"

#define BW_THRESHOLD 50000

int opp_set_rate(struct iris_core *core, u64 freq)
{
	unsigned long opp_freq = 0;
	struct dev_pm_opp *opp;
	int ret;

	opp_freq = freq;

	opp = dev_pm_opp_find_freq_ceil(core->dev, &opp_freq);
	if (IS_ERR(opp)) {
		opp = dev_pm_opp_find_freq_floor(core->dev, &opp_freq);
		if (IS_ERR(opp)) {
			dev_err(core->dev,
				"unable to find freq %lld in opp table\n", freq);
			return -EINVAL;
		}
	}
	dev_pm_opp_put(opp);

	ret = dev_pm_opp_set_rate(core->dev, opp_freq);
	if (ret) {
		dev_err(core->dev, "failed to set rate\n");
		return ret;
	}

	return ret;
}

static int init_bus(struct iris_core *core)
{
	const struct bus_info *bus_tbl;
	struct bus_info *binfo = NULL;
	u32 i = 0;

	bus_tbl = core->platform_data->bus_tbl;

	core->bus_count = core->platform_data->bus_tbl_size;
	core->bus_tbl = devm_kzalloc(core->dev,
				     sizeof(struct bus_info) * core->bus_count,
				     GFP_KERNEL);
	if (!core->bus_tbl)
		return -ENOMEM;

	for (i = 0; i < core->bus_count; i++) {
		binfo = &core->bus_tbl[i];
		binfo->name = bus_tbl[i].name;
		binfo->bw_min_kbps = bus_tbl[i].bw_min_kbps;
		binfo->bw_max_kbps = bus_tbl[i].bw_max_kbps;
		binfo->icc = devm_of_icc_get(core->dev, binfo->name);
		if (IS_ERR(binfo->icc)) {
			dev_err(core->dev,
				"%s: failed to get bus: %s\n", __func__, binfo->name);
			return PTR_ERR(binfo->icc);
		}
	}

	return 0;
}

static int init_power_domains(struct iris_core *core)
{

	const struct platform_clk_data *clk_tbl;
	u32 clk_cnt;
	int ret;

	struct dev_pm_domain_attach_data iris_pd_data = {
		.pd_names = core->platform_data->pd_tbl,
		.num_pd_names = core->platform_data->pd_tbl_size,
		.pd_flags = PD_FLAG_NO_DEV_LINK,
	};

	struct dev_pm_domain_attach_data iris_opp_pd_data = {
		.pd_names = core->platform_data->opp_pd_tbl,
		.num_pd_names = core->platform_data->opp_pd_tbl_size,
		.pd_flags = PD_FLAG_DEV_LINK_ON,
	};

	ret = devm_pm_domain_attach_list(core->dev, &iris_pd_data, &core->pmdomain_tbl);
	if (ret < 0)
		return ret;

	ret = devm_pm_domain_attach_list(core->dev, &iris_opp_pd_data, &core->opp_pmdomain_tbl);
	if (ret < 0)
		return ret;

	ret = devm_pm_opp_set_clkname(core->dev, "vcodec_core");
	if (ret)
		return ret;

	ret = devm_pm_opp_of_add_table(core->dev);
	if (ret) {
		dev_err(core->dev, "%s: failed to add opp table\n", __func__);
		return ret;
	}

	return ret;
}

int enable_power_domains(struct iris_core *core, const char *name)
{
	struct power_domain_info *pdinfo = NULL;
	int ret;
	u32 i;

	ret = opp_set_rate(core, ULONG_MAX);
	if (ret)
		return ret;

	core->pd_count = core->platform_data->pd_tbl_size;
	for (i = 0; i < (core->pd_count - 1); i++) {
		pdinfo = &core->power_domain_tbl[i];
		if (strcmp(pdinfo->name, name))
			continue;
		ret = pm_runtime_get_sync(pdinfo->genpd_dev);
		if (ret < 0)
			return ret;
	}

	ret = opp_set_rate(core, ULONG_MAX);
	if (ret)
		return ret;

	return ret;
}

int disable_power_domains(struct iris_core *core, const char *name)
{
	struct power_domain_info *pdinfo = NULL;
	int ret;
	u32 i;

	ret = opp_set_rate(core, 0);
	if (ret)
		return ret;

	core->pd_count = core->platform_data->pd_tbl_size;
	for (i = 0; i < (core->pd_count - 1); i++) {
		pdinfo = &core->power_domain_tbl[i];
		if (strcmp(pdinfo->name, name))
			continue;
		ret = pm_runtime_put_sync(pdinfo->genpd_dev);
		if (ret)
			return ret;
	}

	return ret;
}

static int init_clocks(struct iris_core *core)
{
	const struct clock_info *clk_tbl;
	struct clock_info *cinfo = NULL;
	u32 i;

	clk_tbl = core->platform_data->clk_tbl;

	core->clk_count = core->platform_data->clk_tbl_size;
	core->clock_tbl = devm_kzalloc(core->dev,
				       sizeof(struct clock_info) * core->clk_count,
				       GFP_KERNEL);
	if (!core->clock_tbl)
		return -ENOMEM;

	for (i = 0; i < core->clk_count; i++) {
		cinfo = &core->clock_tbl[i];
		cinfo->name = clk_tbl[i].name;
		cinfo->clk_id = clk_tbl[i].clk_id;
		cinfo->has_scaling = clk_tbl[i].has_scaling;
		cinfo->clk = devm_clk_get(core->dev, cinfo->name);
		if (IS_ERR(cinfo->clk)) {
			dev_err(core->dev,
				"%s: failed to get clock: %s\n", __func__, cinfo->name);
			return PTR_ERR(cinfo->clk);
		}
	}

	return 0;
}

static int init_reset_clocks(struct iris_core *core)
{
	struct reset_info *rinfo = NULL;
	const char * const *rst_tbl;
	u32 i = 0;

	rst_tbl = core->platform_data->clk_rst_tbl;

	core->reset_count = core->platform_data->clk_rst_tbl_size;
	core->reset_tbl = devm_kzalloc(core->dev,
				       sizeof(struct reset_info) * core->reset_count,
				       GFP_KERNEL);
	if (!core->reset_tbl)
		return -ENOMEM;

	for (i = 0; i < (core->reset_count - 1); i++) {
		rinfo = &core->reset_tbl[i];
		rinfo->name = rst_tbl[i];
		rinfo->rst = devm_reset_control_get(core->dev, rinfo->name);
		if (IS_ERR(rinfo->rst)) {
			dev_err(core->dev,
				"%s: failed to get reset clock: %s\n", __func__, rinfo->name);
			return PTR_ERR(rinfo->rst);
		}
	}

	return 0;
}

int unvote_buses(struct iris_core *core)
{
	struct bus_info *bus = NULL;
	int ret = 0;
	u32 i;

	core->power.bus_bw = 0;
	core->bus_count = core->platform_data->bus_tbl_size;

	for (i = 0; i < core->bus_count; i++) {
		bus = &core->bus_tbl[i];
		if (!bus->icc)
			return -EINVAL;

		ret = icc_set_bw(bus->icc, 0, 0);
		if (ret)
			return ret;
	}

	return ret;
}

int vote_buses(struct iris_core *core, unsigned long bus_bw)
{
	unsigned long bw_kbps = 0, bw_prev = 0;
	struct bus_info *bus = NULL;
	int ret = 0;
	u32 i;

	core->bus_count = core->platform_data->bus_tbl_size;

	for (i = 0; i < core->bus_count; i++) {
		bus = &core->bus_tbl[i];
		if (bus && bus->icc) {
			if (!strcmp(bus->name, "iris-ddr")) {
				bw_kbps = bus_bw;
				bw_prev = core->power.bus_bw;
			} else {
				bw_kbps = bus->bw_max_kbps;
				bw_prev = core->power.bus_bw ?
						bw_kbps : 0;
			}

			bw_kbps = clamp_t(typeof(bw_kbps), bw_kbps,
					  bus->bw_min_kbps, bus->bw_max_kbps);

			if (abs(bw_kbps - bw_prev) < BW_THRESHOLD && bw_prev)
				continue;

			ret = icc_set_bw(bus->icc, bw_kbps, 0);
			if (ret)
				return ret;

			if (!strcmp(bus->name, "iris-ddr"))
				core->power.bus_bw = bw_kbps;
		}
	}

	return ret;
}

static int deassert_reset_control(struct iris_core *core)
{
	struct reset_info *rcinfo = NULL;
	int ret = 0;
	u32 i;

	core->reset_count = core->platform_data->clk_rst_tbl_size;

	for (i = 0; i < (core->reset_count - 1); i++) {
		rcinfo = &core->reset_tbl[i];
		ret = reset_control_deassert(rcinfo->rst);
		if (ret) {
			dev_err(core->dev, "deassert reset control failed. ret = %d\n", ret);
			continue;
		}
	}

	return ret;
}

static int assert_reset_control(struct iris_core *core)
{
	struct reset_info *rcinfo = NULL;
	int ret = 0, cnt = 0;
	u32 i;

	core->reset_count = core->platform_data->clk_rst_tbl_size;

	for (i = 0; i < (core->reset_count - 1); i++) {
		rcinfo = &core->reset_tbl[i];
		if (!rcinfo->rst)
			return -EINVAL;

		ret = reset_control_assert(rcinfo->rst);
		if (ret) {
			dev_err(core->dev, "failed to assert reset control %s, ret = %d\n",
				rcinfo->name, ret);
			goto deassert_reset_control;
		}
		cnt++;

		usleep_range(1000, 1100);
	}

	return ret;
deassert_reset_control:
	for (i = 0; i < cnt; i++) {
		rcinfo = &core->reset_tbl[i];
		reset_control_deassert(rcinfo->rst);
	}

	return ret;
}

int reset_ahb2axi_bridge(struct iris_core *core)
{
	int ret;

	ret = assert_reset_control(core);
	if (ret)
		return ret;

	ret = deassert_reset_control(core);

	return ret;
}

int disable_unprepare_clock(struct iris_core *core, const char *clk_name)
{
	struct clock_info *cl;
	bool found = false;
	u32 i;

	core->clk_count = core->platform_data->clk_tbl_size;

	for (i = 0; i < core->clk_count; i++) {
		cl = &core->clock_tbl[i];
		if (!cl->clk)
			return -EINVAL;

		if (strcmp(cl->name, clk_name))
			continue;

		found = true;
		clk_disable_unprepare(cl->clk);
		cl->prev = 0;
		break;
	}

	if (!found)
		return -EINVAL;

	return 0;
}

int prepare_enable_clock(struct iris_core *core, const char *clk_name)
{
	struct clock_info *cl;
	bool found = false;
	int ret = 0;
	u32 i;

	core->clk_count = core->platform_data->clk_tbl_size;

	for (i = 0; i < core->clk_count; i++) {
		cl = &core->clock_tbl[i];
		if (!cl->clk)
			return -EINVAL;

		if (strcmp(cl->name, clk_name))
			continue;

		found = true;

		ret = clk_prepare_enable(cl->clk);
		if (ret) {
			dev_err(core->dev, "failed to enable clock %s\n", cl->name);
			return ret;
		}

		if (!__clk_is_enabled(cl->clk)) {
			clk_disable_unprepare(cl->clk);
			return -EINVAL;
		}
		break;
	}

	if (!found)
		return -EINVAL;

	return ret;
}

int init_resources(struct iris_core *core)
{
	int ret;

	ret = init_bus(core);
	if (ret)
		return ret;

	ret = init_power_domains(core);
	if (ret)
		return ret;

	ret = init_clocks(core);
	if (ret)
		return ret;

	ret = init_reset_clocks(core);

	return ret;
}
