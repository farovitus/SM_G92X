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
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

/* A read-modify-write of one register around the power switch */
struct exynos_pd_reg {
	u32 off;
	u32 set;
	u32 clr;
};

/* A register window and the writes made to it around the power switch */
struct exynos_pd_regs {
	phys_addr_t base;
	resource_size_t size;
	const struct exynos_pd_reg *regs;
	unsigned int n_regs;
};

struct exynos_pm_domain_config {
	/* Value for LOCAL_PWR_CFG and STATUS fields for each domain */
	u32 local_pwr_cfg;
	/* Value for the OPTION register, written before each transition */
	u32 pd_option;
	/* Value for the DURATION0 register (base + 0x10); 0 = skip */
	u32 duration0;
	/* Delay after a power-on, once the post writes ran; 0 = none */
	u32 on_delay_us;
	/* Domain must never be powered off by genpd */
	bool always_on;
	/*
	 * Power off only when the g3d_off parameter allows it; otherwise the
	 * domain stays on, as if a device in it were still busy.
	 */
	bool gated_off;
	/* Written before the switch in both directions */
	struct exynos_pd_regs pre;
	/* Written after a power-on */
	struct exynos_pd_regs post;
};

/*
 * Exynos specific wrapper around the generic power domain
 */
struct exynos_pm_domain {
	void __iomem *base;
	struct generic_pm_domain pd;
	u32 local_pwr_cfg;
	u32 pd_option;
	u32 duration0;
	u32 on_delay_us;
	bool gated_off;
	const struct exynos_pd_regs *pre;
	const struct exynos_pd_regs *post;
	void __iomem *pre_base;
	void __iomem *post_base;
};

/*
 * Exynos7420 G3D: 0 keeps the domain powered when the GPU idles, 1 lets
 * genpd switch it off. Writable at run time for an A/B of the two.
 */
static bool g3d_off;
module_param(g3d_off, bool, 0644);
MODULE_PARM_DESC(g3d_off, "Power the Exynos7420 G3D domain off when idle");

/* Log every power switch with its PMU status and duration */
static bool trace;
module_param(trace, bool, 0644);
MODULE_PARM_DESC(trace, "Log power domain switches");

static void exynos_pd_write_regs(void __iomem *base,
				 const struct exynos_pd_regs *r)
{
	unsigned int i;
	u32 v;

	if (!base)
		return;

	for (i = 0; i < r->n_regs; i++) {
		v = readl_relaxed(base + r->regs[i].off);
		v &= ~r->regs[i].clr;
		v |= r->regs[i].set;
		writel_relaxed(v, base + r->regs[i].off);
	}
}

static int exynos_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct exynos_pm_domain *pd;
	const char *op = power_on ? "on" : "off";
	void __iomem *base;
	u32 timeout, pwr, status;
	ktime_t t0 = 0;

	pd = container_of(domain, struct exynos_pm_domain, pd);
	base = pd->base;

	if (!power_on && pd->gated_off && !READ_ONCE(g3d_off))
		return -EBUSY;

	if (READ_ONCE(trace)) {
		t0 = ktime_get();
		pr_info("%s: power %s, status 0x%x\n", domain->name, op,
			readl_relaxed(base + 0x4));
	}

	if (pd->pre)
		exynos_pd_write_regs(pd->pre_base, pd->pre);

	if (pd->pd_option)
		writel_relaxed(pd->pd_option, base + 0x8);
	if (pd->duration0)
		writel_relaxed(pd->duration0, base + 0x10);

	pwr = power_on ? pd->local_pwr_cfg : 0;
	writel_relaxed(pwr, base);

	/* Wait max 1ms, like the vendor driver (pd-timeout 1: 100 x 10 us) */
	timeout = 10;

	while (((status = readl_relaxed(base + 0x4)) & pd->local_pwr_cfg) != pwr) {
		if (!timeout) {
			pr_err("Power domain %s %s failed: CONFIGURATION 0x%x STATUS 0x%x OPTION 0x%x\n",
			       domain->name, op, readl_relaxed(base), status,
			       readl_relaxed(base + 0x8));
			return -ETIMEDOUT;
		}
		timeout--;
		cpu_relax();
		usleep_range(80, 100);
	}

	if (power_on) {
		if (pd->post)
			exynos_pd_write_regs(pd->post_base, pd->post);
		if (pd->on_delay_us)
			udelay(pd->on_delay_us);
	}

	if (t0)
		pr_info("%s: power %s done, status 0x%x, %lld us\n",
			domain->name, op, status,
			ktime_us_delta(ktime_get(), t0));

	return 0;
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
 * Exynos7420 G3D power domain, after the vendor sequence
 * (pm_domains-exynos7420-cal.h): the PMU sys_pwr registers of CMU_G3D
 * (CLKRUN, CLKSTOP, DISABLE_PLL, RESET_LOGIC, MEMORY[1:0], RESET_CMU) are
 * cleared before the switch in both directions, the OPTION register gets
 * 0x1101 (vendor DT pd-option) and DURATION0 0xffff00c0. The top-to-G3D
 * clock and the CMU_G3D registers are handled by the clock controller,
 * which runs in this domain with runtime PM and keeps the block clocks
 * on through the power-off handshake. Power-off is gated by the g3d_off
 * parameter until it is verified on the board.
 */
static const struct exynos_pd_reg exynos7420_g3d_sys_pwr[] = {
	{ 0x1460, 0, BIT(0) },
	{ 0x14a0, 0, BIT(0) },
	{ 0x14e0, 0, BIT(0) },
	{ 0x1520, 0, BIT(0) },
	{ 0x1560, 0, BIT(1) | BIT(0) },
	{ 0x15a0, 0, BIT(0) },
};

/*
 * After power-on the vendor GPU driver (exynos_gpu_init_hw) sets
 * G3D_CFGREG0[16] in the top SYSREG, "the blkg3d clock path from aclk-g3d
 * to the aclk-g3d feed-back path", and waits 100 us "to provide the
 * stable SRAM power" before it resets the GPU. The GPU driver here resets
 * the GPU right after the domain is on, so both happen here.
 */
static const struct exynos_pd_reg exynos7420_g3d_cfgreg[] = {
	{ 0x1234, BIT(16), 0 },
};

static const struct exynos_pm_domain_config exynos7420_g3d_cfg = {
	.local_pwr_cfg		= 0xf,
	.pd_option		= 0x1101,
	.duration0		= 0xffff00c0,
	.on_delay_us		= 100,
	.gated_off		= true,
	/* PMU; the sys_pwr registers sit in its second 4 KiB page */
	.pre = {
		.base		= 0x105c0000,
		.size		= 0x2000,
		.regs		= exynos7420_g3d_sys_pwr,
		.n_regs		= ARRAY_SIZE(exynos7420_g3d_sys_pwr),
	},
	/* top SYSREG; G3D_CFGREG0 sits in its second 4 KiB page */
	.post = {
		.base		= 0x10050000,
		.size		= 0x2000,
		.regs		= exynos7420_g3d_cfgreg,
		.n_regs		= ARRAY_SIZE(exynos7420_g3d_cfgreg),
	},
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
	pd->duration0 = pm_domain_cfg->duration0;
	pd->on_delay_us = pm_domain_cfg->on_delay_us;
	pd->gated_off = pm_domain_cfg->gated_off;

	/* Exynos7420: map the register windows written around the switch */
	if (pm_domain_cfg->pre.n_regs) {
		pd->pre = &pm_domain_cfg->pre;
		pd->pre_base = devm_ioremap(dev, pd->pre->base, pd->pre->size);
		if (!pd->pre_base)
			dev_warn(dev, "failed to map %pa\n", &pd->pre->base);
	}
	if (pm_domain_cfg->post.n_regs) {
		pd->post = &pm_domain_cfg->post;
		pd->post_base = devm_ioremap(dev, pd->post->base, pd->post->size);
		if (!pd->post_base)
			dev_warn(dev, "failed to map %pa\n", &pd->post->base);
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
