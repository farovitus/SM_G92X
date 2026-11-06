// SPDX-License-Identifier: GPL-2.0
/*
 * SS333 main link over MIPI-LLI shared memory.
 *
 * Copyright (C) 2010 Samsung Electronics.
 * 6.6 port: FSM link PM only, gpiod, no GPIO signal sideband, no debugfs.
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/mipi-lli.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"

static int pm_enable = 1;
module_param(pm_enable, int, 0444);
MODULE_PARM_DESC(pm_enable, "LLI link PM (0: keep the link locked on)");

/* AP-to-CP doorbell: an LLI signal; the GPIO sideband is not wired here */
static void send_ap2cp_irq(struct mem_link_device *mld, u16 mask)
{
	mipi_lli_send_interrupt(mask);
}

/*
 * Wake the CP if it may sleep and set @flag in pm->ref_cnt.
 * permit_cp_sleep() must follow a successful forbid_cp_sleep().
 */
static void forbid_cp_sleep(struct mem_link_device *mld, int flag)
{
	struct modem_link_pm *pm = &mld->link_dev.pm;
	int cnt = atomic_fetch_or(flag, &pm->ref_cnt) | flag;

	if (cnt > 1)
		return;

	if (pm->request_hold)
		pm->request_hold(pm);
}

static void permit_cp_sleep(struct mem_link_device *mld, int flag)
{
	struct modem_link_pm *pm = &mld->link_dev.pm;
	int cnt = atomic_fetch_andnot(flag, &pm->ref_cnt) & ~flag;

	if (cnt > 0)
		return;

	if (pm->release_hold)
		pm->release_hold(pm);
}

static bool check_link_status(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_link_pm *pm = &ld->pm;

	if (!gpiod_get_value(mld->gpio_cp_status))
		return false;

	if (mipi_lli_get_link_status() != LLI_MOUNTED)
		return false;

	if (cp_online(ld->mc))
		return pm->link_active ? pm->link_active(pm) : true;

	return true;
}

static void pm_fail_cb(struct modem_link_pm *pm)
{
	mipi_lli_debug_info();
	modemctl_notify_event(MDM_CRASH_PM_FAIL);
}

static void stop_pm(struct mem_link_device *mld);

static void pm_cp_fail_cb(struct modem_link_pm *pm)
{
	struct link_device *ld = pm_to_link_device(pm);
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	struct modem_ctl *mc = ld->mc;
	struct io_device *iod = mc->iod;
	bool online, booting;
	unsigned long flags;

	mipi_lli_debug_info();

	spin_lock_irqsave(&mc->lock, flags);
	online = cp_online(mc);
	booting = cp_booting(mc);
	spin_unlock_irqrestore(&mc->lock, flags);

	if (online) {
		stop_pm(mld);
		modemctl_notify_event(MDM_CRASH_PM_CP_FAIL);
		return;
	}

	if (booting) {
		if (iod)
			iod->modem_state_changed(iod, STATE_OFFLINE);
		/* stock reset under mc->lock; outside it is enough */
		ld->reset(ld);
	}
}

static void start_pm(struct mem_link_device *mld)
{
	struct modem_link_pm *pm = &mld->link_dev.pm;

	if (!pm->start)
		return;

	pm->start(pm, pm_enable ? PM_EVENT_CP_BOOTING : PM_EVENT_LOCK_ON);
}

static void stop_pm(struct mem_link_device *mld)
{
	struct modem_link_pm *pm = &mld->link_dev.pm;

	if (pm->stop)
		pm->stop(pm);
}

static int init_pm(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_link_pm *pm = &ld->pm;

	spin_lock_init(&mld->sig_lock);
	atomic_set(&mld->ref_cnt, 0);
	atomic_set(&pm->ref_cnt, 0);

	return init_link_device_pm(ld, pm, pm_fail_cb, pm_cp_fail_cb);
}

static void lli_link_reset(struct link_device *ld)
{
	mif_err("%s: PM %s <%pS>\n", ld->name, FUNC, CALLER);
	mipi_lli_intr_enable();
	mipi_lli_reset();
}

static void lli_link_reload(struct link_device *ld)
{
	mif_err("%s: PM %s <%pS>\n", ld->name, FUNC, CALLER);
	mipi_lli_reload();
}

static void lli_link_off(struct link_device *ld)
{
	mif_err("%s: PM %s <%pS>\n", ld->name, FUNC, CALLER);
	mipi_lli_intr_disable();
	stop_pm(ld_to_mem_link_device(ld));
}

static bool lli_link_unmounted(struct link_device *ld)
{
	return mipi_lli_get_link_status() == LLI_UNMOUNTED;
}

static bool lli_link_suspended(struct link_device *ld)
{
	return mipi_lli_suspended() != 0;
}

static void lli_disable_irq(struct link_device *ld)
{
	mipi_lli_mask_sb_intr(true);
}

static void lli_enable_irq(struct link_device *ld)
{
	mipi_lli_mask_sb_intr(false);
}

/*
 * LLI event callback (from the LLI driver's threaded IRQ).
 * A signal event carries the CP-to-AP interrupt value: take a snapshot of
 * the shared memory and run the common memory-link IRQ path. Every other
 * event drives the link PM FSM.
 */
static void lli_irq_handler(void *data, enum mipi_lli_event event, u32 intr)
{
	struct mem_link_device *mld = data;
	struct mst_buff *msb;

	if (event != LLI_EVENT_SIG) {
		check_lli_irq(&mld->link_dev.pm, event);
		return;
	}

	msb = mem_take_snapshot(mld, RX);
	if (!msb)
		return;
	msb->snapshot.int2ap = (u16)intr;

	mem_irq_handler(mld, msb);
}

static void lli_release(void *data)
{
	struct mem_link_device *mld = data;

	mipi_lli_unregister_handler(lli_irq_handler);
	stop_pm(mld);
}

struct link_device *lli_create_link_device(struct platform_device *pdev,
					   struct modem_data *modem)
{
	struct mem_link_device *mld;
	struct link_device *ld;
	void __iomem *base;
	unsigned long size;
	phys_addr_t start;
	int err;

	if (!mipi_lli_ready())
		return ERR_PTR(-EPROBE_DEFER);

	if (!modem->gpio_ap_wakeup || !modem->gpio_cp_status) {
		mif_err("ERR! link PM GPIOs missing\n");
		return ERR_PTR(-ENODEV);
	}

	mif_err("MODEM:%s LINK:%s\n", modem->name, modem->link_name);

	mld = mem_create_link_device(MEM_LLI_SHMEM, modem);
	if (!mld) {
		mif_err("%s: ERR! create_link_device fail\n", modem->link_name);
		return ERR_PTR(-ENOMEM);
	}

	ld = &mld->link_dev;
	ld->dev = &pdev->dev;

	ld->reset = lli_link_reset;
	ld->reload = lli_link_reload;
	ld->off = lli_link_off;
	ld->unmounted = lli_link_unmounted;
	ld->suspended = lli_link_suspended;
	ld->enable_irq = lli_enable_irq;
	ld->disable_irq = lli_disable_irq;

	mld->send_ap2cp_irq = send_ap2cp_irq;
	mld->start_pm = start_pm;
	mld->stop_pm = stop_pm;
	mld->forbid_cp_sleep = forbid_cp_sleep;
	mld->permit_cp_sleep = permit_cp_sleep;
	mld->link_active = check_link_status;
	mld->debug_info = mipi_lli_debug_info;

	mld->gpio_ap_wakeup = modem->gpio_ap_wakeup;
	mld->gpio_cp_wakeup = modem->gpio_cp_wakeup;
	mld->gpio_cp_status = modem->gpio_cp_status;
	mld->gpio_ap_status = modem->gpio_ap_status;
	mld->gpio_ipc_int2cp = modem->gpio_ipc_int2cp;
	init_waitqueue_head(&mld->wq);

	/* IPC region: the LLI shared memory, mapped by the LLI driver */
	start = mipi_lli_get_phys_base();
	size = mipi_lli_get_phys_size();
	base = mipi_lli_request_sh_region(start, size);
	err = mem_register_ipc_rgn(mld, start, base, size);
	if (err < 0) {
		mif_err("%s: ERR! register_ipc_rgn fail (%d)\n", ld->name, err);
		goto error;
	}

	err = mem_setup_ipc_map(mld);
	if (err < 0) {
		mif_err("%s: ERR! setup_ipc_map fail (%d)\n", ld->name, err);
		goto error_rgn;
	}

	if (ld->sbd_ipc) {
		err = create_sbd_link_device(ld, &mld->sbd_link_dev,
					     mld->base, mld->size);
		if (err < 0)
			goto error_rgn;
	}

	err = init_pm(mld);
	if (err) {
		mif_err("%s: ERR! init_pm fail (%d)\n", ld->name, err);
		goto error_rgn;
	}

	/* Last: events may arrive as soon as the handler is registered */
	err = mipi_lli_register_handler(lli_irq_handler, mld);
	if (err) {
		mif_err("%s: ERR! register_handler fail (%d)\n", ld->name, err);
		goto error_rgn;
	}

	err = devm_add_action_or_reset(&pdev->dev, lli_release, mld);
	if (err)
		goto error_rgn;

	return ld;

error_rgn:
	mem_unregister_ipc_rgn(mld);
error:
	mif_err("xxx\n");
	/* mld holds work items and a tasklet that are idle here; see PORT.md */
	return ERR_PTR(err);
}
