// SPDX-License-Identifier: GPL-2.0
/*
 * Bring-up console for SM-G920F: write the kernel log into the exynos-
 * snapshot kernel log ring that Samsung's 3.10 kernels use (2 MiB at
 * 0x46800000), and keep its physical write pointer in the snapshot header
 * (0x46000200). The 3.10 TWRP kernel then shows this boot as
 * /proc/last_kmsg after a forced restart. S-Boot passes console=ram, so
 * the console takes that name and is enabled by it.
 */
#include <linux/bits.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kmsg_dump.h>
#include <linux/kstrtox.h>
#include <linux/minmax.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/panic_notifier.h>
#include <linux/pid_namespace.h>
#include <linux/reboot.h>
#include <linux/sched/debug.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#define ESS_HEADER_PA		0x46000000UL
#define ESS_LAST_LOGBUF		0x200
#define ESS_KLOG_PA		0x46800000UL
#define ESS_KLOG_SIZE		0x200000UL

static void __iomem *ess_header;
static u8 __iomem *ess_klog;
static unsigned long ess_pos;

static void ess_console_write(struct console *con, const char *s,
			      unsigned int n)
{
	if (!ess_klog)		/* kept for the hung boot, see below */
		return;

	while (n) {
		unsigned long chunk = min_t(unsigned long, n,
					    ESS_KLOG_SIZE - ess_pos);

		memcpy_toio(ess_klog + ess_pos, s, chunk);
		ess_pos = (ess_pos + chunk) % ESS_KLOG_SIZE;
		s += chunk;
		n -= chunk;
	}
	writel(ESS_KLOG_PA + ess_pos, ess_header + ESS_LAST_LOGBUF);
}

static struct console ess_console = {
	.name	= "ram",
	.write	= ess_console_write,
	.flags	= CON_PRINTBUFFER | CON_ANYTIME,
	.index	= -1,
};

static bool ess_wdt_redirect(void);

static int __init ess_console_init(void)
{
	/*
	 * After a watchdog reset the ring still holds the boot that hung.
	 * This boot only restarts into recovery (ess_rescue_init), so leave
	 * the ring alone and TWRP shows the hung boot as last_kmsg.
	 */
	if (ess_wdt_redirect())
		return 0;

	/* Both ranges are no-map reserved in the board DT. */
	ess_header = ioremap(ESS_HEADER_PA, PAGE_SIZE);
	ess_klog = ioremap(ESS_KLOG_PA, ESS_KLOG_SIZE);
	if (!ess_header || !ess_klog)
		return -ENOMEM;

	/* A zeroed ring tells the 3.10 reader that the log did not wrap. */
	memset_io(ess_klog, 0, ESS_KLOG_SIZE);
	writel(ESS_KLOG_PA, ess_header + ESS_LAST_LOGBUF);
	register_console(&ess_console);
	return 0;
}
console_initcall(ess_console_init);

/*
 * Bring-up heartbeat: every 30 s, say the kernel is alive and show init's
 * stack and every blocked task. A log that keeps beating after userspace
 * goes quiet means init is waiting on something; a log that stops means
 * the kernel itself hung.
 *
 * dsh_autorecovery=<seconds> restarts into recovery at the first beat after
 * that uptime, so a test boot returns to TWRP with its log without anyone
 * holding the keys. Writing 0 to
 * /sys/module/dsh_esslog/parameters/autorecovery cancels it and stops the
 * beat, so a boot that completes stays up without dumping stacks every
 * 30 s; writing a non-zero value starts the beat again. An off-mode charging boot
 * (androidboot.mode=charger) never completes and has no adb to cancel it,
 * so it is left alone.
 */
static unsigned int ess_autorecovery;

/*
 * S-Boot reads the restart target from PMU INFORM3 on every reset, not
 * only on a kernel restart, and skips the charger screen when INFORM2
 * holds 0x12345678 (the same values as the board's syscon-reboot-mode
 * nodes). A test boot arms recovery there as soon as it can, so a panic,
 * a watchdog reset or a hang that ends in a reset all land in TWRP,
 * where adb works and a new kernel can be flashed. S-Boot ignores INFORM3
 * after a watchdog reset, so the boot that follows one restarts into
 * recovery itself through PMU SWRESET. Writing 0 to the
 * autorecovery parameter restores the normal target and any other value
 * arms recovery again; a panic always goes to recovery.
 */
#define ESS_PMU_SWRESET		0x400
#define ESS_PMU_RST_STAT	0x404
#define ESS_RST_A57_WDT		BIT(23)
#define ESS_PMU_INFORM2		0x808
#define ESS_PMU_INFORM3		0x80c
#define ESS_TARGET_NORMAL	0x12345670
#define ESS_TARGET_RECOVERY	0x12345674
#define ESS_NO_LPM		0x12345678

static void __iomem *ess_pmu;
static bool ess_charger_mode;

static void ess_set_target(u32 target)
{
	if (!ess_pmu)
		return;
	writel(ESS_NO_LPM, ess_pmu + ESS_PMU_INFORM2);
	writel(target, ess_pmu + ESS_PMU_INFORM3);
}

static void ess_beat(struct work_struct *work);
static DECLARE_DELAYED_WORK(ess_beat_work, ess_beat);

static int ess_autorecovery_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_uint(val, kp);

	if (ret)
		return ret;
	if (!ess_charger_mode)
		ess_set_target(ess_autorecovery ? ESS_TARGET_RECOVERY :
				  ESS_TARGET_NORMAL);
	/* Before the late initcall the beat is not set up yet. */
	if (ess_autorecovery && system_state == SYSTEM_RUNNING)
		schedule_delayed_work(&ess_beat_work, 30 * HZ);
	return 0;
}

static const struct kernel_param_ops ess_autorecovery_ops = {
	.set	= ess_autorecovery_set,
	.get	= param_get_uint,
};
module_param_cb(autorecovery, &ess_autorecovery_ops, &ess_autorecovery, 0644);

static int __init ess_autorecovery_setup(char *s)
{
	return !kstrtouint(s, 0, &ess_autorecovery);
}
__setup("dsh_autorecovery=", ess_autorecovery_setup);

static int __init ess_bootmode_setup(char *s)
{
	ess_charger_mode = !strcmp(s, "charger");
	return 1;	/* init still reads it from /proc/cmdline */
}
__setup("androidboot.mode=", ess_bootmode_setup);

/*
 * Android writes so much through /dev/kmsg that the kernel's own boot
 * messages leave the 512 KiB TWRP shows. Before the restart, copy every
 * kernel-facility line (syslog prefix 0-7) from the printk buffer to the
 * end of the ring again.
 */
static void ess_replay_kernel_log(void)
{
	static char line[1024];
	static const char head[] = "\n==== dsh: kernel messages since boot ====\n";
	static const char tail[] = "==== dsh: end of kernel messages ====\n";
	struct kmsg_dump_iter iter;
	size_t len;

	/* Hold the console so printk does not write the ring meanwhile. */
	console_lock();
	ess_console_write(NULL, head, sizeof(head) - 1);
	kmsg_dump_rewind(&iter);
	while (kmsg_dump_get_line(&iter, true, line, sizeof(line), &len)) {
		if (len < 3 || line[0] != '<' || line[1] < '0' ||
		    line[1] > '7' || line[2] != '>')
			continue;
		ess_console_write(NULL, line + 3, len - 3);
	}
	ess_console_write(NULL, tail, sizeof(tail) - 1);
	console_unlock();
}


static void ess_beat(struct work_struct *work)
{
	struct task_struct *init_task_p;

	/* Cancelled: the boot completed, so stop the beat. */
	if (!ess_autorecovery)
		return;

	pr_emerg("dsh: alive %llu s\n", ktime_get_boottime_seconds());
	rcu_read_lock();
	init_task_p = find_task_by_pid_ns(1, &init_pid_ns);
	if (init_task_p)
		sched_show_task(init_task_p);
	rcu_read_unlock();
	show_state_filter(TASK_UNINTERRUPTIBLE);
	if (ess_autorecovery && !ess_charger_mode &&
	    ktime_get_boottime_seconds() >= ess_autorecovery) {
		/*
		 * A boot that never completed may be stuck on a lock that
		 * device_shutdown() needs too (test-93/94: a devfreq/PM QoS
		 * lock cycle held kernel_restart() there): sync in the
		 * background, give it 3 s, then reset without the shutdown
		 * callbacks.
		 */
		ess_set_target(ESS_TARGET_RECOVERY);
		emergency_sync();
		mdelay(3000);
		ess_replay_kernel_log();
		pr_emerg("dsh: restarting to recovery\n");
		emergency_restart();
	}
	schedule_delayed_work(&ess_beat_work, 30 * HZ);
}

static int ess_panic(struct notifier_block *nb, unsigned long ev, void *p)
{
	ess_set_target(ESS_TARGET_RECOVERY);
	return NOTIFY_DONE;
}

static struct notifier_block ess_panic_nb = {
	.notifier_call	= ess_panic,
	.priority	= INT_MAX,
};

static int __init ess_map_pmu(void)
{
	struct device_node *np;

	if (ess_pmu)
		return 0;
	np = of_find_compatible_node(NULL, NULL, "samsung,exynos7-pmu");
	if (!np)
		return -ENODEV;
	ess_pmu = of_iomap(np, 0);
	of_node_put(np);
	return ess_pmu ? 0 : -ENOMEM;
}

/* A test boot that follows a watchdog reset; runs before the console. */
static bool __init ess_wdt_redirect(void)
{
	if (!ess_autorecovery || ess_charger_mode || ess_map_pmu())
		return false;
	return readl(ess_pmu + ESS_PMU_RST_STAT) & ESS_RST_A57_WDT;
}

static int __init ess_rescue_init(void)
{
	u32 rst;
	int ret;

	/* Only test boots, which set dsh_autorecovery, are rescued. */
	if (!ess_autorecovery)
		return 0;
	ret = ess_map_pmu();
	if (ret)
		return ret;
	if (ess_charger_mode)
		goto out;
	ess_set_target(ESS_TARGET_RECOVERY);
	rst = readl(ess_pmu + ESS_PMU_RST_STAT);
	pr_info("dsh: last reset 0x%08x\n", rst);
	if (rst & ESS_RST_A57_WDT) {
		pr_emerg("dsh: watchdog reset, restarting to recovery\n");
		writel(1, ess_pmu + ESS_PMU_SWRESET);
		mdelay(1000);
	}
out:
	atomic_notifier_chain_register(&panic_notifier_list, &ess_panic_nb);
	pr_info("dsh: resets go to recovery until the boot completes\n");
	return 0;
}
early_initcall(ess_rescue_init);

static int __init ess_beat_init(void)
{
	schedule_delayed_work(&ess_beat_work, 30 * HZ);
	return 0;
}
late_initcall(ess_beat_init);
