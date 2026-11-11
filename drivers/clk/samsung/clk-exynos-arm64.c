// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Linaro Ltd.
 * Copyright (C) 2021 Dávid Virág <virag.david003@gmail.com>
 * Author: Sam Protsenko <semen.protsenko@linaro.org>
 * Author: Dávid Virág <virag.david003@gmail.com>
 *
 * This file contains shared functions used by some arm64 Exynos SoCs,
 * such as Exynos7885 or Exynos850 to register and init CMUs.
 */
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "clk-exynos-arm64.h"

/* Gate register bits */
#define GATE_MANUAL		BIT(20)
#define GATE_ENABLE_HWACG	BIT(28)

/* Gate register offsets range */
#define GATE_OFF_START		0x2000
#define GATE_OFF_END		0x2fff

struct exynos_arm64_cmu_data {
	struct samsung_clk_reg_dump *clk_save;
	unsigned int nr_clk_save;
	const struct samsung_clk_reg_dump *clk_suspend;
	unsigned int nr_clk_suspend;

	struct clk *clk;
	struct clk **pclks;
	int nr_pclks;

	/* see samsung_cmu_info.pd_off_gates */
	const struct samsung_clk_reg_dump *pd_off_gates;
	unsigned int nr_pd_off_gates;
	struct notifier_block pd_nb;
	bool pd_clks_on;

	struct samsung_clk_provider *ctx;
};

/**
 * exynos_arm64_init_clocks - Set clocks initial configuration
 * @np:			CMU device tree node with "reg" property (CMU addr)
 * @reg_offs:		Register offsets array for clocks to init
 * @reg_offs_len:	Number of register offsets in reg_offs array
 *
 * Set manual control mode for all gate clocks.
 */
static void __init exynos_arm64_init_clocks(struct device_node *np,
		const unsigned long *reg_offs, size_t reg_offs_len)
{
	void __iomem *reg_base;
	size_t i;

	reg_base = of_iomap(np, 0);
	if (!reg_base)
		panic("%s: failed to map registers\n", __func__);

	for (i = 0; i < reg_offs_len; ++i) {
		void __iomem *reg = reg_base + reg_offs[i];
		u32 val;

		/* Modify only gate clock registers */
		if (reg_offs[i] < GATE_OFF_START || reg_offs[i] > GATE_OFF_END)
			continue;

		val = readl(reg);
		val |= GATE_MANUAL;
		val &= ~GATE_ENABLE_HWACG;
		writel(val, reg);
	}

	iounmap(reg_base);
}

/**
 * exynos_arm64_enable_bus_clk - Enable parent clock of specified CMU
 *
 * @dev:	Device object; may be NULL if this function is not being
 *		called from platform driver probe function
 * @np:		CMU device tree node
 * @cmu:	CMU data
 *
 * Keep CMU parent clock running (needed for CMU registers access).
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int __init exynos_arm64_enable_bus_clk(struct device *dev,
		struct device_node *np, const struct samsung_cmu_info *cmu)
{
	struct clk *parent_clk;

	if (!cmu->clk_name)
		return 0;

	if (dev) {
		struct exynos_arm64_cmu_data *data;

		parent_clk = clk_get(dev, cmu->clk_name);
		data = dev_get_drvdata(dev);
		if (data)
			data->clk = parent_clk;
	} else {
		parent_clk = of_clk_get_by_name(np, cmu->clk_name);
	}

	if (IS_ERR(parent_clk))
		return PTR_ERR(parent_clk);

	return clk_prepare_enable(parent_clk);
}

static int __init exynos_arm64_cmu_prepare_pm(struct device *dev,
		const struct samsung_cmu_info *cmu)
{
	struct exynos_arm64_cmu_data *data = dev_get_drvdata(dev);
	int i;

	data->clk_save = samsung_clk_alloc_reg_dump(cmu->clk_regs,
						    cmu->nr_clk_regs);
	if (!data->clk_save)
		return -ENOMEM;

	data->nr_clk_save = cmu->nr_clk_regs;
	data->clk_suspend = cmu->suspend_regs;
	data->nr_clk_suspend = cmu->nr_suspend_regs;
	data->nr_pclks = of_clk_get_parent_count(dev->of_node);
	if (!data->nr_pclks)
		return 0;

	data->pclks = devm_kcalloc(dev, sizeof(struct clk *), data->nr_pclks,
				   GFP_KERNEL);
	if (!data->pclks) {
		kfree(data->clk_save);
		return -ENOMEM;
	}

	for (i = 0; i < data->nr_pclks; i++) {
		struct clk *clk = of_clk_get(dev->of_node, i);

		if (IS_ERR(clk)) {
			kfree(data->clk_save);
			while (--i >= 0)
				clk_put(data->pclks[i]);
			return PTR_ERR(clk);
		}
		data->pclks[i] = clk;
	}

	return 0;
}

/**
 * exynos_arm64_register_cmu - Register specified Exynos CMU domain
 * @dev:	Device object; may be NULL if this function is not being
 *		called from platform driver probe function
 * @np:		CMU device tree node
 * @cmu:	CMU data
 *
 * Register specified CMU domain, which includes next steps:
 *
 * 1. Enable parent clock of @cmu CMU
 * 2. Set initial registers configuration for @cmu CMU clocks
 * 3. Register @cmu CMU clocks using Samsung clock framework API
 */
void __init exynos_arm64_register_cmu(struct device *dev,
		struct device_node *np, const struct samsung_cmu_info *cmu)
{
	int err;

	/*
	 * Try to boot even if the parent clock enablement fails, as it might be
	 * already enabled by bootloader.
	 */
	err = exynos_arm64_enable_bus_clk(dev, np, cmu);
	if (err)
		pr_err("%s: could not enable bus clock %s; err = %d\n",
		       __func__, cmu->clk_name, err);

	exynos_arm64_init_clocks(np, cmu->clk_regs, cmu->nr_clk_regs);
	samsung_cmu_register_one(np, cmu);
}

/* Bus and parent clocks: on exactly while the power domain is on */
static void exynos_arm64_cmu_pd_clks(struct exynos_arm64_cmu_data *data,
				     bool enable)
{
	int i;

	if (data->pd_clks_on == enable)
		return;

	if (enable)
		clk_enable(data->clk);
	for (i = 0; i < data->nr_pclks; i++) {
		if (enable)
			clk_enable(data->pclks[i]);
		else
			clk_disable(data->pclks[i]);
	}
	if (!enable)
		clk_disable(data->clk);
	data->pd_clks_on = enable;
}

/*
 * Some blocks (Exynos7420 CAM0) only finish their power-down handshake with
 * the bus clocks running and the local gates open; otherwise the domain
 * times out half off. Like the vendor code, keep the CMU bus and parent
 * clocks on for as long as the domain is on (also while the CMU is
 * runtime suspended), open the gates just before it goes down, and turn
 * the clocks back on before it powers up. Runtime resume then restores the
 * saved gate state; the restored muxes select these parent clocks.
 *
 * Runs under the genpd lock, so it only uses clk_enable()/clk_disable()
 * (spinlock); the clocks stay prepared from registration on.
 */
static int exynos_arm64_cmu_pd_notify(struct notifier_block *nb,
				      unsigned long action, void *unused)
{
	struct exynos_arm64_cmu_data *data =
		container_of(nb, struct exynos_arm64_cmu_data, pd_nb);
	void __iomem *base = data->ctx->reg_base;
	unsigned int i;

	switch (action) {
	case GENPD_NOTIFY_PRE_OFF:
		for (i = 0; i < data->nr_pd_off_gates; i++) {
			const struct samsung_clk_reg_dump *g = &data->pd_off_gates[i];

			writel(readl(base + g->offset) | g->value,
			       base + g->offset);
		}
		break;
	case GENPD_NOTIFY_OFF:
		exynos_arm64_cmu_pd_clks(data, false);
		break;
	case GENPD_NOTIFY_PRE_ON:
		exynos_arm64_cmu_pd_clks(data, true);
		break;
	}

	return NOTIFY_OK;
}

static int __init exynos_arm64_cmu_pd_init(struct device *dev,
		struct exynos_arm64_cmu_data *data,
		const struct samsung_cmu_info *cmu)
{
	int i, ret;

	if (!cmu->nr_pd_off_gates)
		return 0;
	if (IS_ERR_OR_NULL(data->clk)) {
		dev_err(dev, "pd_off_gates needs the bus clock\n");
		return -EINVAL;
	}

	/* The domain is on here; the bus clock is already enabled */
	for (i = 0; i < data->nr_pclks; i++) {
		ret = clk_prepare_enable(data->pclks[i]);
		if (ret) {
			while (--i >= 0)
				clk_disable_unprepare(data->pclks[i]);
			return ret;
		}
	}

	data->pd_off_gates = cmu->pd_off_gates;
	data->nr_pd_off_gates = cmu->nr_pd_off_gates;
	data->pd_clks_on = true;
	data->pd_nb.notifier_call = exynos_arm64_cmu_pd_notify;
	ret = dev_pm_genpd_add_notifier(dev, &data->pd_nb);
	if (ret) {
		for (i = 0; i < data->nr_pclks; i++)
			clk_disable_unprepare(data->pclks[i]);
		data->nr_pd_off_gates = 0;
	}

	return ret;
}

/**
 * exynos_arm64_register_cmu_pm - Register Exynos CMU domain with PM support
 *
 * @pdev:	Platform device object
 * @set_manual:	If true, set gate clocks to manual mode
 *
 * It's a version of exynos_arm64_register_cmu() with PM support. Should be
 * called from probe function of platform driver.
 *
 * Return: 0 on success, or negative error code on error.
 */
int __init exynos_arm64_register_cmu_pm(struct platform_device *pdev,
					bool set_manual)
{
	const struct samsung_cmu_info *cmu;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct exynos_arm64_cmu_data *data;
	void __iomem *reg_base;
	int ret;

	cmu = of_device_get_match_data(dev);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	ret = exynos_arm64_cmu_prepare_pm(dev, cmu);
	if (ret)
		return ret;

	/*
	 * Try to boot even if the parent clock enablement fails, as it might be
	 * already enabled by bootloader.
	 */
	ret = exynos_arm64_enable_bus_clk(dev, NULL, cmu);
	if (ret)
		dev_err(dev, "%s: could not enable bus clock %s; err = %d\n",
		       __func__, cmu->clk_name, ret);

	if (set_manual)
		exynos_arm64_init_clocks(np, cmu->clk_regs, cmu->nr_clk_regs);

	reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg_base))
		return PTR_ERR(reg_base);

	data->ctx = samsung_clk_init(dev, reg_base, cmu->nr_clk_ids);

	ret = exynos_arm64_cmu_pd_init(dev, data, cmu);
	if (ret) {
		dev_err(dev, "power domain notifier failed: %d\n", ret);
		return ret;
	}

	/*
	 * Enable runtime PM here to allow the clock core using runtime PM
	 * for the registered clocks. Additionally, we increase the runtime
	 * PM usage count before registering the clocks, to prevent the
	 * clock core from runtime suspending the device.
	 */
	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	samsung_cmu_register_clocks(data->ctx, cmu);
	samsung_clk_of_add_provider(dev->of_node, data->ctx);
	pm_runtime_put_sync(dev);

	return 0;
}

int exynos_arm64_cmu_suspend(struct device *dev)
{
	struct exynos_arm64_cmu_data *data = dev_get_drvdata(dev);
	int i;

	samsung_clk_save(data->ctx->reg_base, data->clk_save,
			 data->nr_clk_save);

	for (i = 0; i < data->nr_pclks; i++)
		clk_prepare_enable(data->pclks[i]);

	/* For suspend some registers have to be set to certain values */
	samsung_clk_restore(data->ctx->reg_base, data->clk_suspend,
			    data->nr_clk_suspend);

	for (i = 0; i < data->nr_pclks; i++)
		clk_disable_unprepare(data->pclks[i]);

	/* With pd_off_gates the bus clock goes off with the domain */
	if (!data->nr_pd_off_gates)
		clk_disable_unprepare(data->clk);

	return 0;
}

int exynos_arm64_cmu_resume(struct device *dev)
{
	struct exynos_arm64_cmu_data *data = dev_get_drvdata(dev);
	int i;

	if (!data->nr_pd_off_gates)
		clk_prepare_enable(data->clk);

	for (i = 0; i < data->nr_pclks; i++)
		clk_prepare_enable(data->pclks[i]);

	samsung_clk_restore(data->ctx->reg_base, data->clk_save,
			    data->nr_clk_save);

	for (i = 0; i < data->nr_pclks; i++)
		clk_disable_unprepare(data->pclks[i]);

	return 0;
}
