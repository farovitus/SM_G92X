// SPDX-License-Identifier: GPL-2.0
/*
 * Bring-up console for SM-G920F: write the kernel log into the exynos-
 * snapshot kernel log ring that Samsung's 3.10 kernels use (2 MiB at
 * 0x46800000), and keep its physical write pointer in the snapshot header
 * (0x46000200). The 3.10 TWRP kernel then shows this boot as
 * /proc/last_kmsg after a forced restart. S-Boot passes console=ram, so
 * the console takes that name and is enabled by it.
 */
#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kmsg_dump.h>
#include <linux/kstrtox.h>
#include <linux/minmax.h>
#include <linux/moduleparam.h>
#include <linux/pid_namespace.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/sched/debug.h>
#include <linux/syscalls.h>
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

static int __init ess_console_init(void)
{
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
 * /sys/module/dsh_esslog/parameters/autorecovery cancels it, so a boot
 * that completes stays up. An off-mode charging boot
 * (androidboot.mode=charger) never completes and has no adb to cancel it,
 * so it is left alone.
 */
static unsigned int ess_autorecovery;
module_param_named(autorecovery, ess_autorecovery, uint, 0644);

static int __init ess_autorecovery_setup(char *s)
{
	return !kstrtouint(s, 0, &ess_autorecovery);
}
__setup("dsh_autorecovery=", ess_autorecovery_setup);

static bool ess_charger_mode;

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

static void ess_beat(struct work_struct *work);
static DECLARE_DELAYED_WORK(ess_beat_work, ess_beat);

static void ess_beat(struct work_struct *work)
{
	struct task_struct *init_task_p;

	pr_emerg("dsh: alive %llu s\n", ktime_get_boottime_seconds());
	rcu_read_lock();
	init_task_p = find_task_by_pid_ns(1, &init_pid_ns);
	if (init_task_p)
		sched_show_task(init_task_p);
	rcu_read_unlock();
	show_state_filter(TASK_UNINTERRUPTIBLE);
	if (ess_autorecovery && !ess_charger_mode &&
	    ktime_get_boottime_seconds() >= ess_autorecovery) {
		ksys_sync();
		ess_replay_kernel_log();
		pr_emerg("dsh: restarting to recovery\n");
		kernel_restart("recovery");
	}
	schedule_delayed_work(&ess_beat_work, 30 * HZ);
}

static int __init ess_beat_init(void)
{
	schedule_delayed_work(&ess_beat_work, 30 * HZ);
	return 0;
}
late_initcall(ess_beat_init);
