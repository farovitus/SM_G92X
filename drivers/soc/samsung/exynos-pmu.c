// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2011-2014 Samsung Electronics Co., Ltd.
//		http://www.samsung.com/
//
// Exynos - CPU PMU(Power Management Unit) support

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <asm/cputype.h>

#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/soc/samsung/exynos-pmu.h>

#include "exynos-pmu.h"

struct exynos_pmu_context {
	struct device *dev;
	const struct exynos_pmu_data *pmu_data;
};

void __iomem *pmu_base_addr;
static struct exynos_pmu_context *pmu_context;

void pmu_raw_writel(u32 val, u32 offset)
{
	writel_relaxed(val, pmu_base_addr + offset);
}

u32 pmu_raw_readl(u32 offset)
{
	return readl_relaxed(pmu_base_addr + offset);
}

void exynos_sys_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int i;
	const struct exynos_pmu_data *pmu_data;

	if (!pmu_context || !pmu_context->pmu_data)
		return;

	pmu_data = pmu_context->pmu_data;

	if (pmu_data->powerdown_conf)
		pmu_data->powerdown_conf(mode);

	if (pmu_data->pmu_config) {
		for (i = 0; (pmu_data->pmu_config[i].offset != PMU_TABLE_END); i++)
			pmu_raw_writel(pmu_data->pmu_config[i].val[mode],
					pmu_data->pmu_config[i].offset);
	}

	if (pmu_data->powerdown_conf_extra)
		pmu_data->powerdown_conf_extra(mode);

	if (pmu_data->pmu_config_extra) {
		for (i = 0; pmu_data->pmu_config_extra[i].offset != PMU_TABLE_END; i++)
			pmu_raw_writel(pmu_data->pmu_config_extra[i].val[mode],
				       pmu_data->pmu_config_extra[i].offset);
	}
}

#ifdef CONFIG_ARM64
/*
 * Exynos7 core power-down for cpuidle.
 *
 * The PMU powers a core down when it enters WFI only if that core's
 * LOCAL_PWR_CFG field is clear; the PSCI firmware does not touch the
 * field. Clear it right before a power-down idle state is entered (the
 * PSCI suspend call follows cpu_pm_enter()) and set it again on exit,
 * so a plain WFI never powers a core down without its context saved.
 * Each core only writes its own configuration register, indexed by
 * physical core: Aff0 + 4 * Aff1.
 */
#define EXYNOS7_CPU_CONFIGURATION(n)	(0x2000 + (n) * 0x80)
#define EXYNOS7_LOCAL_PWR_CFG		0xf

static int exynos7_cpu_pm_notify(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	u64 mpidr = read_cpuid_mpidr();
	u32 core = MPIDR_AFFINITY_LEVEL(mpidr, 0) +
		   4 * MPIDR_AFFINITY_LEVEL(mpidr, 1);
	u32 offset = EXYNOS7_CPU_CONFIGURATION(core);
	u32 val = pmu_raw_readl(offset);

	switch (action) {
	case CPU_PM_ENTER:
		pmu_raw_writel(val & ~EXYNOS7_LOCAL_PWR_CFG, offset);
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		pmu_raw_writel(val | EXYNOS7_LOCAL_PWR_CFG, offset);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos7_cpu_pm_nb = {
	.notifier_call = exynos7_cpu_pm_notify,
};

/*
 * Power-off with a charger attached.
 *
 * Samsung's boot loader shows the off-charging screen only when it is
 * started with a charger present and INFORM2 clear (INFORM2 = 0x12345678
 * tells it to skip that screen; see reboot-mode-no-lpm in the board DT).
 * Dropping PS_HOLD while external power is present does not power the
 * phone off, so the stock kernels restart instead. Do the same: note at
 * shutdown, while drivers can still be queried, whether external power is
 * present, and if so restart from the power-off handler with INFORM2
 * cleared. Without external power the syscon-poweroff handler drops
 * PS_HOLD as usual.
 */
#define EXYNOS7_INFORM2			0x0808

static bool exynos7_poweroff_to_charger;

/*
 * A restart must not land in the off-charging screen, whatever reason
 * userspace gives ("userrequested", the charger's own, ...). Like the
 * stock kernels, set INFORM2 to skip it and INFORM3 to a normal boot on
 * every restart. The syscon-reboot-mode nodes run after this notifier
 * (lower priority) and replace INFORM3 for the modes they know, such as
 * recovery and download.
 */
#define EXYNOS7_INFORM3			0x080c
#define EXYNOS7_INFORM2_NO_LPM		0x12345678
#define EXYNOS7_INFORM3_NORMAL		0x12345670

static int exynos7_pmu_reboot_notify(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	switch (action) {
	case SYS_RESTART:
		pmu_raw_writel(EXYNOS7_INFORM2_NO_LPM, EXYNOS7_INFORM2);
		pmu_raw_writel(EXYNOS7_INFORM3_NORMAL, EXYNOS7_INFORM3);
		break;
	case SYS_POWER_OFF:
		exynos7_poweroff_to_charger =
			power_supply_is_system_supplied() > 0;
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block exynos7_pmu_reboot_nb = {
	.notifier_call = exynos7_pmu_reboot_notify,
	.priority = 1,
};

static int exynos7_pmu_power_off(struct sys_off_data *data)
{
	if (!exynos7_poweroff_to_charger)
		return NOTIFY_DONE;

	pr_emerg("exynos-pmu: external power present, restarting to the charger screen\n");
	pmu_raw_writel(0, EXYNOS7_INFORM2);
	do_kernel_restart(NULL);

	return NOTIFY_DONE;
}

/*
 * Boot-time PMU setup from Samsung's Exynos 7420 CAL (pmu_cal_sys_init).
 * The boot loader leaves these at reset values. Without them, taking all
 * four A57 (ATLAS) cores offline powers the cluster's non-CPU block down,
 * so any CMU_ATLAS access aborts, and the cores do not come back online.
 */
#define EXYNOS7_UP_SCHEDULER			0x0120
#define EXYNOS7_CPU_OPTION(n)			(0x2008 + (n) * 0x80)
#define EXYNOS7_CPU_DURATION0(n)		(0x2010 + (n) * 0x80)
#define EXYNOS7_ATLAS_DBG_CONFIGURATION		0x2440
#define EXYNOS7_ATLAS_CPUSEQUENCER_OPTION	0x2488
#define EXYNOS7_L2_OPTION(n)			(0x2608 + (n) * 0x20)
#define EXYNOS7_XXTI_DURATION3			0x343c
#define EXYNOS7_EXT_REGULATOR_DURATION3		0x361c

/* CPU_OPTION */
#define EXYNOS7_USE_SMPEN			BIT(28)
#define EXYNOS7_USE_STANDBYWFE			BIT(24)
#define EXYNOS7_USE_STANDBYWFI			BIT(16)
#define EXYNOS7_USE_MEMPWRDOWN			BIT(2)
#define EXYNOS7_USE_SC_FEEDBACK			BIT(1)
#define EXYNOS7_USE_SC_COUNTER			BIT(0)
/* CPU_DURATION0 */
#define EXYNOS7_DUR_WAIT_RESET			(0xf << 20)
#define EXYNOS7_DUR_SCALL			(0xf << 4)
#define EXYNOS7_DUR_SCALL_VALUE			(0x1 << 4)
/* L2_OPTION */
#define EXYNOS7_USE_AUTOMATIC_L2FLUSHREQ	BIT(17)
#define EXYNOS7_USE_STANDBYWFIL2		BIT(16)
#define EXYNOS7_USE_RETENTION			BIT(4)
/* CPUSEQUENCER_OPTION */
#define EXYNOS7_SKIP_BLK_PWR_DOWN		BIT(8)
/* UP_SCHEDULER */
#define EXYNOS7_ENABLE_ATLAS_CPU		BIT(0)
#define EXYNOS7_ENABLE_APOLLO_CPU		BIT(1)
/* ATLAS_DBG_CONFIGURATION */
#define EXYNOS7_DBG_INITIATE_WAKEUP		(0x3 << 16)
/* DURATION3 */
#define EXYNOS7_DUR_STABLE_MASK			0xfffff
#define EXYNOS7_XXTI_DUR_STABLE			0x1d4c	/* 5 ms */
#define EXYNOS7_EXTREG_DUR_STABLE		0x3fff	/* 10 ms */

/* Block option registers that use the sequencer feedback, not a counter. */
static const u32 exynos7_feedback_options[] = {
	0x2408,	/* ATLAS_NONCPU */
	0x2428,	/* APOLLO_NONCPU */
	0x2c48,	/* TOP_PWR */
	0x2cc8,	/* TOP_PWR_MIF */
	0x4008,	/* AUD */
	0x4048,	/* CAM0 */
	0x4088,	/* DISP */
	0x4108,	/* G3D */
	0x41c8,	/* MSCL */
	0x41a8,	/* MFC */
	0x4028,	/* BUS0 */
	0x40a8,	/* FSYS0 */
	0x40c8,	/* FSYS1 */
	0x4148,	/* ISP0 */
	0x4188,	/* ISP1 */
	0x41e8,	/* VPP */
};

static void exynos7_pmu_update(u32 offset, u32 clear, u32 set)
{
	pmu_raw_writel((pmu_raw_readl(offset) & ~clear) | set, offset);
}

static void exynos7_pmu_sys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(exynos7_feedback_options); i++)
		exynos7_pmu_update(exynos7_feedback_options[i],
				   EXYNOS7_USE_SC_COUNTER,
				   EXYNOS7_USE_SC_FEEDBACK);

	/* No automatic L2 flush or retention; STANDBYWFIL2 handshake. */
	for (i = 0; i < 2; i++)
		exynos7_pmu_update(EXYNOS7_L2_OPTION(i),
				   EXYNOS7_USE_AUTOMATIC_L2FLUSHREQ |
				   EXYNOS7_USE_RETENTION,
				   EXYNOS7_USE_STANDBYWFIL2);

	/* Cores 0-3 are ATLAS, 4-7 APOLLO. */
	for (i = 0; i < 8; i++) {
		exynos7_pmu_update(EXYNOS7_CPU_OPTION(i),
				   EXYNOS7_USE_STANDBYWFE,
				   (i < 4 ? EXYNOS7_USE_MEMPWRDOWN : 0) |
				   EXYNOS7_USE_SC_FEEDBACK |
				   EXYNOS7_USE_SC_COUNTER |
				   EXYNOS7_USE_SMPEN |
				   EXYNOS7_USE_STANDBYWFI);
		exynos7_pmu_update(EXYNOS7_CPU_DURATION0(i),
				   EXYNOS7_DUR_SCALL,
				   EXYNOS7_DUR_WAIT_RESET |
				   EXYNOS7_DUR_SCALL_VALUE);
	}

	/* Keep the ATLAS block powered when its cores power down. */
	exynos7_pmu_update(EXYNOS7_ATLAS_CPUSEQUENCER_OPTION, 0,
			   EXYNOS7_SKIP_BLK_PWR_DOWN);

	/* Limit in-rush current when a cluster powers up. */
	exynos7_pmu_update(EXYNOS7_UP_SCHEDULER, 0,
			   EXYNOS7_ENABLE_ATLAS_CPU | EXYNOS7_ENABLE_APOLLO_CPU);

	exynos7_pmu_update(EXYNOS7_XXTI_DURATION3, EXYNOS7_DUR_STABLE_MASK,
			   EXYNOS7_XXTI_DUR_STABLE);
	exynos7_pmu_update(EXYNOS7_EXT_REGULATOR_DURATION3,
			   EXYNOS7_DUR_STABLE_MASK, EXYNOS7_EXTREG_DUR_STABLE);

	exynos7_pmu_update(EXYNOS7_ATLAS_DBG_CONFIGURATION, 0,
			   EXYNOS7_DBG_INITIATE_WAKEUP);
}

static void exynos7_pmu_init(void)
{
	exynos7_pmu_sys_init();
	cpu_pm_register_notifier(&exynos7_cpu_pm_nb);
	register_reboot_notifier(&exynos7_pmu_reboot_nb);
	register_sys_off_handler(SYS_OFF_MODE_POWER_OFF, SYS_OFF_PRIO_HIGH,
				 exynos7_pmu_power_off, NULL);
	exynos7_pm_init();
}

static const struct exynos_pmu_data exynos7_pmu_data = {
	.pmu_init = exynos7_pmu_init,
};
#define exynos7_pmu_data_ptr	(&exynos7_pmu_data)
#else
#define exynos7_pmu_data_ptr	NULL
#endif

/*
 * Split the data between ARM architectures because it is relatively big
 * and useless on other arch.
 */
#ifdef CONFIG_EXYNOS_PMU_ARM_DRIVERS
#define exynos_pmu_data_arm_ptr(data)	(&data)
#else
#define exynos_pmu_data_arm_ptr(data)	NULL
#endif

/*
 * PMU platform driver and devicetree bindings.
 */
static const struct of_device_id exynos_pmu_of_device_ids[] = {
	{
		.compatible = "samsung,exynos3250-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos3250_pmu_data),
	}, {
		.compatible = "samsung,exynos4210-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4210_pmu_data),
	}, {
		.compatible = "samsung,exynos4212-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4212_pmu_data),
	}, {
		.compatible = "samsung,exynos4412-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos4412_pmu_data),
	}, {
		.compatible = "samsung,exynos5250-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos5250_pmu_data),
	}, {
		.compatible = "samsung,exynos5410-pmu",
	}, {
		.compatible = "samsung,exynos5420-pmu",
		.data = exynos_pmu_data_arm_ptr(exynos5420_pmu_data),
	}, {
		.compatible = "samsung,exynos5433-pmu",
	}, {
		.compatible = "samsung,exynos7-pmu",
		.data = exynos7_pmu_data_ptr,
	}, {
		.compatible = "samsung,exynos850-pmu",
	},
	{ /*sentinel*/ },
};

static const struct mfd_cell exynos_pmu_devs[] = {
	{ .name = "exynos-clkout", },
};

struct regmap *exynos_get_pmu_regmap(void)
{
	struct device_node *np = of_find_matching_node(NULL,
						      exynos_pmu_of_device_ids);
	if (np)
		return syscon_node_to_regmap(np);
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(exynos_get_pmu_regmap);

static int exynos_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	pmu_base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pmu_base_addr))
		return PTR_ERR(pmu_base_addr);

	pmu_context = devm_kzalloc(&pdev->dev,
			sizeof(struct exynos_pmu_context),
			GFP_KERNEL);
	if (!pmu_context)
		return -ENOMEM;
	pmu_context->dev = dev;
	pmu_context->pmu_data = of_device_get_match_data(dev);

	if (pmu_context->pmu_data && pmu_context->pmu_data->pmu_init)
		pmu_context->pmu_data->pmu_init();

	platform_set_drvdata(pdev, pmu_context);

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, exynos_pmu_devs,
				   ARRAY_SIZE(exynos_pmu_devs), NULL, 0, NULL);
	if (ret)
		return ret;

	if (devm_of_platform_populate(dev))
		dev_err(dev, "Error populating children, reboot and poweroff might not work properly\n");

	dev_dbg(dev, "Exynos PMU Driver probe done\n");
	return 0;
}

static struct platform_driver exynos_pmu_driver = {
	.driver  = {
		.name   = "exynos-pmu",
		.of_match_table = exynos_pmu_of_device_ids,
	},
	.probe = exynos_pmu_probe,
};

static int __init exynos_pmu_init(void)
{
	return platform_driver_register(&exynos_pmu_driver);

}
postcore_initcall(exynos_pmu_init);
