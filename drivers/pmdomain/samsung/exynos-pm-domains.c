// SPDX-License-Identifier: GPL-2.0
//
// Exynos Generic power domain support.
//
// Copyright (c) 2012 Samsung Electronics Co., Ltd.
//		http://www.samsung.com
//
// Implementation of Exynos specific power domain control which is used in
// conjunction with runtime-pm. Support for both device-tree and non-device-tree
// based power domain support is included.

#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-pmu.h>

/* A PMU register and the bits to clear in it before each transition */
struct exynos_pd_reg_bits {
	u32 offset;
	u32 mask;
};

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	/* Value for the OPTION register, written before each transition */
	u32 pd_option;
	/* Domain must never be powered off by genpd */
	bool always_on;
	/* PMU bits cleared before every power transition */
	const struct exynos_pd_reg_bits *sys_pwr_clr;
	unsigned int nr_sys_pwr_clr;
	/*
	 * A second reg entry holds the block's LPI busmaster mask, set when a
	 * power-off times out before trying once more.
	 */
	bool lpi_mask;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	u32 pd_option;
	const struct exynos_pm_domain_config *cfg;
	void __iomem *lpi_mask;
	struct regmap *pmu;
};

static void exynos_pd_clear_sys_pwr(struct exynos_pm_domain *pd)
{
	const struct exynos_pm_domain_config *cfg = pd->cfg;
	unsigned int i;

	if (!cfg->nr_sys_pwr_clr)
		return;

	if (IS_ERR_OR_NULL(pd->pmu)) {
		pd->pmu = exynos_get_pmu_regmap();
		if (IS_ERR(pd->pmu)) {
			pr_warn_once("Power domain %s: no PMU regmap: %ld\n",
				     pd->pd.name, PTR_ERR(pd->pmu));
			return;
		}
	}

	for (i = 0; i < cfg->nr_sys_pwr_clr; i++)
		regmap_update_bits(pd->pmu, cfg->sys_pwr_clr[i].offset,
				   cfg->sys_pwr_clr[i].mask, 0);
}

/* Write the request and wait max 1ms for the status to follow */
static int exynos_pd_set(struct exynos_pm_domain *pd, u32 pwr)
{
	void __iomem *base = pd->base;
	u32 timeout = 10;

	if (pd->pd_option)
		writel_relaxed(pd->pd_option, base + 0x8);

	writel_relaxed(pwr, base);

	while ((readl_relaxed(base + 0x4) & pd->local_pwr_cfg) != pwr) {
		if (!timeout)
			return -ETIMEDOUT;
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	return 0;
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	void __iomem *base;
	u32 pwr;
	int ret;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	exynos_pd_clear_sys_pwr(pd);

	pwr = power_on ? pd->local_pwr_cfg : 0;
	ret = exynos_pd_set(pd, pwr);
	if (!ret)
		return 0;

	pr_err("Power domain %s %s failed: config %08x status %08x option %08x\n",
	       domain->name, power_on ? "enable" : "disable",
	       readl_relaxed(base), readl_relaxed(base + 0x4),
	       readl_relaxed(base + 0x8));
	if (power_on)
		return ret;

	if (pd->lpi_mask) {
		/* Stop waiting for the bus masters and try once more */
		writel_relaxed(0xffffffff, pd->lpi_mask);
		ret = exynos_pd_set(pd, 0);
		if (!ret) {
			pr_warn("Power domain %s off with the LPI mask set\n",
				domain->name);
			return 0;
		}
	}

	/*
	 * genpd keeps a domain whose power-off failed marked on, but the
	 * block may finish going down later and the next access then faults.
	 * Ask for it back on so the hardware matches genpd.
	 */
	if (exynos_pd_set(pd, pd->local_pwr_cfg))
		pr_err("Power domain %s did not come back on\n", domain->name);

	return -ETIMEDOUT;
}

static int exynos_pd_power_on(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, true);
}

static int exynos_pd_power_off(struct generic_pm_domain *domain)
{
	return exynos_pd_power(domain, false);
}

static const struct exynos_pm_domain_config exynos4210_cfg = {
	.local_pwr_cfg		= 0x7,
};

static const struct exynos_pm_domain_config exynos5433_cfg = {
	.local_pwr_cfg		= 0xf,
};

/*
 * Exynos7420 CAM0 follows the vendor power sequence: OPTION 0x0102, the
 * block's PMU clock/reset/memory controls cleared before each transition,
 * and the LPI busmaster mask on a power-off timeout. CMU_CAM0 keeps its
 * bus clocks on and opens its local gates around the power-off.
 */
static const struct exynos_pd_reg_bits exynos7420_cam0_sys_pwr[] = {
	{ 0x1448, BIT(0) },	/* CLKRUN_CMU_CAM0_SYS_PWR_REG */
	{ 0x1488, BIT(0) },	/* CLKSTOP_CMU_CAM0_SYS_PWR_REG */
	{ 0x14c8, BIT(0) },	/* DISABLE_PLL_CMU_CAM0_SYS_PWR_REG */
	{ 0x1508, BIT(0) },	/* RESET_LOGIC_CAM0_SYS_PWR_REG */
	{ 0x1548, 0x33 },	/* MEMORY_CAM0_SYS_PWR_REG */
	{ 0x1588, BIT(0) },	/* RESET_CMU_CAM0_SYS_PWR_REG */
};

static const struct exynos_pm_domain_config exynos7420_cam0_cfg = {
	.local_pwr_cfg		= 0xf,
	.pd_option		= 0x0102,
	.sys_pwr_clr		= exynos7420_cam0_sys_pwr,
	.nr_sys_pwr_clr		= ARRAY_SIZE(exynos7420_cam0_sys_pwr),
	.lpi_mask		= true,
};

/*
 * CAM1 needs its own vendor sequence (CMU_CAM1 clocks, A7IS option, LPI
 * masks), none of which is ported; keep it on so it never goes down with
 * the generic one once CAM0 can power off.
 */
static const struct exynos_pm_domain_config exynos7420_cam1_cfg = {
	.local_pwr_cfg		= 0xf,
	.pd_option		= 0x0102,
	.always_on		= true,
};

static const struct exynos_pm_domain_config exynos7420_g3d_cfg = {
	.local_pwr_cfg		= 0xf,
	.pd_option		= 0x1101,
	.always_on		= true,
};

static const struct of_device_id exynos_pm_domain_of_match[] = {
	{
		.compatible = "samsung,exynos4210-pd",
		.data = &exynos4210_cfg,
	}, {
		/* must precede the generic exynos5433-pd entry */
		.compatible = "samsung,exynos7420-g3d-pd",
		.data = &exynos7420_g3d_cfg,
	}, {
		.compatible = "samsung,exynos7420-cam0-pd",
		.data = &exynos7420_cam0_cfg,
	}, {
		.compatible = "samsung,exynos7420-cam1-pd",
		.data = &exynos7420_cam1_cfg,
	}, {
		.compatible = "samsung,exynos5433-pd",
		.data = &exynos5433_cfg,
	},
	{ },
};

static const char *exynos_get_domain_name(struct device *dev,
					  struct device_node *node)
{
	const char *name;

	if (of_property_read_string(node, "label", &name) < 0)
		name = kbasename(node->full_name);
	return devm_kstrdup_const(dev, name, GFP_KERNEL);
}

static int exynos_pd_probe(struct platform_device *pdev)
{
	const struct exynos_pm_domain_config *pm_domain_cfg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_args child, parent;
	struct exynos_pm_domain *pd;
	int on, ret;

	pm_domain_cfg = of_device_get_match_data(dev);
	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->pd.name = exynos_get_domain_name(dev, np);
	if (!pd->pd.name)
		return -ENOMEM;

	pd->base = of_iomap(np, 0);
	if (!pd->base)
		return -ENODEV;

	pd->pd.power_off = exynos_pd_power_off;
	pd->pd.power_on = exynos_pd_power_on;
	pd->local_pwr_cfg = pm_domain_cfg->local_pwr_cfg;
	pd->pd_option = pm_domain_cfg->pd_option;
	pd->cfg = pm_domain_cfg;

	if (pm_domain_cfg->lpi_mask) {
		pd->lpi_mask = of_iomap(np, 1);
		if (!pd->lpi_mask)
			dev_warn(dev, "%s: no LPI mask register\n", pd->pd.name);
	}

	on = readl_relaxed(pd->base + 0x4) & pd->local_pwr_cfg;

	pm_genpd_init(&pd->pd, NULL, !on);

	/*
	 * genpd refuses to initialise an always-on domain that starts off,
	 * so set the flag only after pm_genpd_init(). A domain found off is
	 * powered on at its first consumer resume and then kept on.
	 */
	if (pm_domain_cfg->always_on) {
		if (!on)
			dev_warn(dev, "%s is off, kept on once powered\n",
				 pd->pd.name);
		pd->pd.flags |= GENPD_FLAG_ALWAYS_ON;
	}

	ret = of_genpd_add_provider_simple(np, &pd->pd);

	if (ret == 0 && of_parse_phandle_with_args(np, "power-domains",
				      "#power-domain-cells", 0, &parent) == 0) {
		child.np = np;
		child.args_count = 0;

		if (of_genpd_add_subdomain(&parent, &child))
			pr_warn("%pOF failed to add subdomain: %pOF\n",
				parent.np, child.np);
		else
			pr_info("%pOF has as child subdomain: %pOF.\n",
				parent.np, child.np);
	}

	pm_runtime_enable(dev);
	return ret;
}

static struct platform_driver exynos_pd_driver = {
	.probe	= exynos_pd_probe,
	.driver	= {
		.name		= "exynos-pd",
		.of_match_table	= exynos_pm_domain_of_match,
		.suppress_bind_attrs = true,
	}
};

static __init int exynos4_pm_init_power_domain(void)
{
	return platform_driver_register(&exynos_pd_driver);
}
core_initcall(exynos4_pm_init_power_domain);
