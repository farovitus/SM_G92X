// SPDX-License-Identifier: GPL-2.0
/*
 * Link power management FSM for the SS333 LLI link.
 *
 * Copyright (C) 2014 Samsung Electronics.
 * 6.6 port: gpiod, timer_setup, wakeup sources, edge-both IRQs.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"

static unsigned long cp_hold_time = CP_HOLD_TIME;
module_param(cp_hold_time, ulong, 0644);
MODULE_PARM_DESC(cp_hold_time, "modem_v1 pm cp_hold_time");

static inline void print_pm_event(struct modem_link_pm *pm, enum pm_event event)
{
	int cp2ap_wakeup;
	int ap2cp_wakeup;
	int cp2ap_status;
	int ap2cp_status;

	cp2ap_wakeup = gpiod_get_value(pm->gpio_cp2ap_wakeup);
	ap2cp_wakeup = gpiod_get_value(pm->gpio_ap2cp_wakeup);
	cp2ap_status = gpiod_get_value(pm->gpio_cp2ap_status);
	ap2cp_status = gpiod_get_value(pm->gpio_ap2cp_status);

	/*
	PM {cp2ap_wakeup:ap2cp_wakeup:cp2ap_status:ap2cp_status}{event:state}
	   <CALLER>
	*/
	mif_info("%s: PM {%d:%d:%d:%d}{%s:%s}\n", pm->link_name,
		cp2ap_wakeup, ap2cp_wakeup, cp2ap_status, ap2cp_status,
		pm_event2str(event), pm_state2str(pm->fsm.state));
}

static inline void print_pm_fsm(struct modem_link_pm *pm)
{
	struct pm_wdog *wdog = &pm->wdog;
	int cp2ap_wakeup;
	int ap2cp_wakeup;
	int cp2ap_status;
	int ap2cp_status;

	cp2ap_wakeup = gpiod_get_value(pm->gpio_cp2ap_wakeup);
	ap2cp_wakeup = gpiod_get_value(pm->gpio_ap2cp_wakeup);
	cp2ap_status = gpiod_get_value(pm->gpio_cp2ap_status);
	ap2cp_status = gpiod_get_value(pm->gpio_ap2cp_status);

	/*
	PM {cp2ap_wakeup:ap2cp_wakeup:cp2ap_status:ap2cp_status}\
	   {event:current_state->next_state} <CALLER>
	*/
	mif_info("%s: PM {%d:%d:%d:%d}{%s:%s->%s}\n", pm->link_name,
		cp2ap_wakeup, ap2cp_wakeup, cp2ap_status, ap2cp_status,
		pm_event2str(pm->fsm.event), pm_state2str(pm->fsm.prev_state),
		pm_state2str(pm->fsm.state));

	if (wdog->msg[0]) {
		pr_err("%s\n", wdog->msg);
		wdog->msg[0] = 0;
	}
}


static inline void release_ap2cp_wakeup(struct modem_link_pm *pm)
{
	gpiod_set_value(pm->gpio_ap2cp_wakeup, 0);
}

static inline void assert_ap2cp_wakeup(struct modem_link_pm *pm)
{
	gpiod_set_value(pm->gpio_ap2cp_wakeup, 1);
}

static inline void release_ap2cp_status(struct modem_link_pm *pm)
{
	gpiod_set_value(pm->gpio_ap2cp_status, 0);
}

static inline void assert_ap2cp_status(struct modem_link_pm *pm)
{
	gpiod_set_value(pm->gpio_ap2cp_status, 1);
}

static inline void schedule_cp_free(struct modem_link_pm *pm)
{
	/* Hold gpio_ap2cp_wakeup for CP_HOLD_TIME */
	if (work_pending(&pm->cp_free_dwork.work))
		return;

	queue_delayed_work(pm->wq, &pm->cp_free_dwork,
			   msecs_to_jiffies(cp_hold_time));
}

static inline void cancel_cp_free(struct modem_link_pm *pm)
{
	if (work_pending(&pm->cp_free_dwork.work))
		cancel_delayed_work(&pm->cp_free_dwork);
}

static inline void start_pm_wdog(struct modem_link_pm *pm, enum pm_state state,
				 enum pm_state w_state, enum pm_event w_event,
				 unsigned long ms)
{
	struct pm_wdog *wdog = &pm->wdog;
	struct timer_list *timer = &wdog->timer;
	unsigned long expire = msecs_to_jiffies(ms);

	wdog->state = state;
	wdog->w_state = w_state;
	wdog->w_event = w_event;


	mod_timer(timer, jiffies + expire);
}

static inline void stop_pm_wdog(struct modem_link_pm *pm, enum pm_state state,
				enum pm_event event)
{
	struct pm_wdog *wdog = &pm->wdog;

	if (state == wdog->state) {
		struct timer_list *timer = &wdog->timer;

		del_timer(timer);

		mif_debug("%s: PM WDOG kicked by {%s@%s}\n",
			pm->link_name, pm_event2str(event),
			pm_state2str(state));
	} else {
		mif_err("%s: ERR! PM WDOG illegal state {%s@%s}\n",
			pm->link_name, pm_event2str(event),
			pm_state2str(state));
	}
}

static inline bool link_suspended(struct modem_link_pm *pm)
{
	struct link_device *ld = pm_to_link_device(pm);
	return ld->suspended ? ld->suspended(ld) : false;
}

static inline void reload_link(struct modem_link_pm *pm)
{
	struct link_device *ld = pm_to_link_device(pm);
	if (ld->reload)
		ld->reload(ld);
}

static inline void handle_pm_fail(struct modem_link_pm *pm)
{
	if (pm->fail_handler)
		pm->fail_handler(pm);
}

static inline void handle_cp_fail(struct modem_link_pm *pm)
{
	if (pm->cp_fail_handler)
		pm->cp_fail_handler(pm);
}

static inline void handle_wdog_timeout(struct modem_link_pm *pm)
{
	if (gpiod_get_value(pm->gpio_cp2ap_status))
		handle_pm_fail(pm);
	else
		handle_cp_fail(pm);
}

static inline void lock_pm_wake(struct modem_link_pm *pm)
{
	mif_stay_awake(pm->ws);
}

static inline void unlock_pm_wake(struct modem_link_pm *pm)
{
	mif_relax(pm->ws);
}

static inline void prepare_mount(struct modem_link_pm *pm)
{
	lock_pm_wake(pm);
}

static inline void unmounted_to_resetting(struct modem_link_pm *pm)
{
	reload_link(pm);
	assert_ap2cp_wakeup(pm);
	start_pm_wdog(pm, PM_STATE_RESETTING, PM_STATE_MOUNTING,
		      PM_EVENT_LINK_RESET, LINKPM_WATCHDOG_TIMEOUT);
}

static inline void unmounted_to_holding(struct modem_link_pm *pm)
{
	//reload_link(pm);
	assert_ap2cp_wakeup(pm);
	start_pm_wdog(pm, PM_STATE_HOLDING, PM_STATE_RESETTING,
		      PM_EVENT_CP2AP_WAKEUP_HIGH, LINKPM_WATCHDOG_TIMEOUT);
}

static inline void holding_to_resetting(struct modem_link_pm *pm)
{
	stop_pm_wdog(pm, PM_STATE_HOLDING, PM_EVENT_CP2AP_WAKEUP_HIGH);
	reload_link(pm);
	start_pm_wdog(pm, PM_STATE_RESETTING, PM_STATE_MOUNTING,
		      PM_EVENT_LINK_RESET, LINKPM_WATCHDOG_TIMEOUT);
}

static inline enum pm_state next_state_from_resume(struct modem_link_pm *pm)
{
	if (gpiod_get_value(pm->gpio_cp2ap_wakeup)) {
		pm->hold_requested = false;
		return PM_STATE_RESETTING;
	}

	if (pm->hold_requested) {
		pm->hold_requested = false;
		return PM_STATE_HOLDING;
	}

	return PM_STATE_UNMOUNTED;
}

static inline bool fsm_locked(enum pm_state state)
{
	if (state == PM_STATE_LOCKED_ON
	    || state == PM_STATE_WDOG_TIMEOUT
	    || state == PM_STATE_CP_FAIL)
		return true;
	else
		return false;
}

static inline bool hold_possible(enum pm_state state)
{
	if (state == PM_STATE_UNMOUNTED
	    || state == PM_STATE_AP_FREE
	    || state == PM_STATE_CP_FREE)
		return true;
	else
		return false;
}

static inline void set_pm_fsm(struct modem_link_pm *pm, enum pm_state c_state,
			      enum pm_state n_state, enum pm_event event)
{
	struct pm_fsm *fsm = &pm->fsm;

	fsm->prev_state = c_state;
	fsm->event = event;
	fsm->state = n_state;
}

static inline void decide_pm_wake(struct modem_link_pm *pm,
				  enum pm_state c_state,
				  enum pm_state n_state)
{
	if (n_state == c_state)
		return;

	switch (n_state) {
	case PM_STATE_UNMOUNTED:
		unlock_pm_wake(pm);
		break;

	case PM_STATE_ACTIVE:
		lock_pm_wake(pm);
		break;

	default:
		break;
	}
}

static inline void check_pm_fail(struct modem_link_pm *pm,
				 enum pm_state c_state,
				 enum pm_state n_state)
{
	if (n_state == c_state)
		return;

	switch (n_state) {
	case PM_STATE_WDOG_TIMEOUT:
		handle_wdog_timeout(pm);
		break;

	case PM_STATE_CP_FAIL:
		handle_cp_fail(pm);
		break;

	case PM_STATE_AP_FAIL:
		/* stock panicked here on debug builds; never panic */
		handle_cp_fail(pm);
		break;

	default:
		break;
	}
}

static void run_pm_fsm(struct modem_link_pm *pm, enum pm_event event)
{
	struct link_device *ld = pm_to_link_device(pm);
	struct modem_ctl *mc = ld->mc;
	struct pm_fsm *fsm = &pm->fsm;
	enum pm_state c_state;
	enum pm_state n_state;
	unsigned long flags;

	spin_lock_irqsave(&pm->lock, flags);

	c_state = fsm->state;
	n_state = fsm->state;

	if (!pm->active) {
		release_ap2cp_wakeup(pm);
		if (event == PM_EVENT_LINK_ERROR)
			n_state = PM_STATE_CP_FAIL;
		goto exit;
	}

	if (fsm_locked(c_state))
		goto exit;

	if (event == PM_EVENT_CP_HOLD_REQUEST) {
		if (!cp_online(mc))
			goto exit;

		pm->hold_requested = true;

		if (!hold_possible(c_state))
			goto exit;
	}

	switch (c_state) {
	case PM_STATE_UNMOUNTED:
		if (event == PM_EVENT_LINK_SUSPENDED) {
			n_state = PM_STATE_SUSPENDED;
		} else if (event == PM_EVENT_CP2AP_WAKEUP_HIGH
			   || event == PM_EVENT_CP_HOLD_REQUEST) {
			if (link_suspended(pm)) {
				n_state = PM_STATE_SUSPENDED;
				prepare_mount(pm);
			} else {
				n_state = next_state_from_resume(pm);
				if (n_state == PM_STATE_RESETTING) {
					prepare_mount(pm);
					unmounted_to_resetting(pm);
				} else if (n_state == PM_STATE_HOLDING) {
					prepare_mount(pm);
					unmounted_to_holding(pm);
				}
			}
		}
		break;

	case PM_STATE_SUSPENDED:
		if (event == PM_EVENT_LINK_RESUMED) {
			n_state = next_state_from_resume(pm);
			if (n_state == PM_STATE_RESETTING) {
				prepare_mount(pm);
				unmounted_to_resetting(pm);
			} else if (n_state == PM_STATE_HOLDING) {
				prepare_mount(pm);
				unmounted_to_holding(pm);
			}
		} else if (event == PM_EVENT_CP2AP_WAKEUP_HIGH
			   || event == PM_EVENT_CP_HOLD_REQUEST) {
			n_state = PM_STATE_SUSPENDED;
			prepare_mount(pm);
		}
		break;

	case PM_STATE_HOLDING:
		if (event == PM_EVENT_CP2AP_WAKEUP_HIGH) {
			n_state = PM_STATE_RESETTING;
			holding_to_resetting(pm);
		} else if (event == PM_EVENT_WDOG_TIMEOUT) {
			/*
			It is not guaranteed for FSM to succeed in getting GPIO
			interrupt events or for stop_pm_wdog() to succeed in
			deleting the WDOG timer always.
			So, gpio_cp2ap_wakeup and gpio_cp2ap_status must always
			be checked before state transition.
			*/
			if (gpiod_get_value(pm->gpio_cp2ap_wakeup)) {
				n_state = PM_STATE_RESETTING;
				holding_to_resetting(pm);
			} else {
				n_state = PM_STATE_WDOG_TIMEOUT;
			}
		}
		break;

	case PM_STATE_RESETTING:
		if (event == PM_EVENT_LINK_RESET) {
			n_state = PM_STATE_MOUNTING;
			stop_pm_wdog(pm, c_state, event);
			assert_ap2cp_status(pm);
			start_pm_wdog(pm, n_state, PM_STATE_ACTIVE,
				      PM_EVENT_LINK_MOUNTED, LINKPM_WATCHDOG_TIMEOUT);
		} else if (event == PM_EVENT_WDOG_TIMEOUT) {
			n_state = PM_STATE_AP_FAIL;
		} else if (event == PM_EVENT_CP2AP_WAKEUP_LOW) {
			n_state = PM_STATE_CP_FAIL;
		}
		break;

	case PM_STATE_MOUNTING:
		if (event == PM_EVENT_LINK_MOUNTED
			|| event == PM_EVENT_CP2AP_STATUS_HIGH) {
			n_state = PM_STATE_ACTIVE;
			stop_pm_wdog(pm, c_state, event);
		} else if (event == PM_EVENT_WDOG_TIMEOUT) {
			n_state = PM_STATE_WDOG_TIMEOUT;
		} else if (event == PM_EVENT_CP2AP_WAKEUP_LOW) {
			n_state = PM_STATE_AP_FAIL;
		}
		break;

	case PM_STATE_ACTIVE:
		if (event == PM_EVENT_CP2AP_WAKEUP_LOW) {
			n_state = PM_STATE_AP_FREE;
			schedule_cp_free(pm);
		} else if (event == PM_EVENT_CP2AP_STATUS_LOW) {
			n_state = PM_STATE_CP_FAIL;
		} else if (event == PM_EVENT_CP_HOLD_TIMEOUT) {
			if (!gpiod_get_value(pm->gpio_cp2ap_wakeup)) {
				cancel_cp_free(pm);
				n_state = PM_STATE_AP_FREE;
				schedule_cp_free(pm);
			}
		}
		break;

	case PM_STATE_AP_FREE:
		if (event == PM_EVENT_CP2AP_WAKEUP_HIGH) {
			n_state = PM_STATE_ACTIVE;
			cancel_cp_free(pm);
			assert_ap2cp_wakeup(pm);
		} else if (event == PM_EVENT_CP_HOLD_REQUEST) {
			n_state = PM_STATE_ACTIVE;
			cancel_cp_free(pm);
			assert_ap2cp_wakeup(pm);
			schedule_cp_free(pm);
		} else if (event == PM_EVENT_CP_HOLD_TIMEOUT) {
			/*
			It is not guaranteed for cancel_cp_free() to succeed
			in canceling the cp_free_dwork always.
			So, cp2ap_wakeup must always be checked before state
			transition.
			*/
			if (!gpiod_get_value(pm->gpio_cp2ap_wakeup) &&
					!atomic_read(&pm->ref_cnt)) {
				n_state = PM_STATE_CP_FREE;
				pm->hold_requested = false;
				release_ap2cp_wakeup(pm);
			} else {
				n_state = PM_STATE_ACTIVE;
				cancel_cp_free(pm);
				assert_ap2cp_wakeup(pm);
			}
		} else if (event == PM_EVENT_CP2AP_STATUS_LOW) {
			n_state = PM_STATE_CP_FAIL;
		}
		break;

	case PM_STATE_CP_FREE:
		if (event == PM_EVENT_CP2AP_STATUS_LOW) {
			n_state = PM_STATE_UNMOUNTING;
			start_pm_wdog(pm, n_state, PM_STATE_UNMOUNTED,
				      PM_EVENT_LINK_UNMOUNTED, LINKPM_WATCHDOG_TIMEOUT);
		} else if (event == PM_EVENT_CP2AP_WAKEUP_HIGH) {
			n_state = PM_STATE_ACTIVE;
			assert_ap2cp_wakeup(pm);
		} else if (event == PM_EVENT_CP_HOLD_REQUEST) {
			n_state = PM_STATE_ACTIVATING;
			assert_ap2cp_wakeup(pm);
			start_pm_wdog(pm, n_state, PM_STATE_ACTIVE,
				      PM_EVENT_CP2AP_WAKEUP_HIGH, LINKPM_WATCHDOG_TIMEOUT);
		}
		break;

	case PM_STATE_ACTIVATING:
		if (event == PM_EVENT_CP2AP_WAKEUP_HIGH) {
			n_state = PM_STATE_ACTIVE;
			stop_pm_wdog(pm, c_state, event);
			assert_ap2cp_wakeup(pm);
		} else if (event == PM_EVENT_CP2AP_STATUS_LOW) {
			n_state = PM_STATE_UNMOUNTING;
			stop_pm_wdog(pm, c_state, event);
			release_ap2cp_wakeup(pm);
		} else if (event == PM_EVENT_WDOG_TIMEOUT) {
			/*
			It is not guaranteed for FSM to succeed in getting GPIO
			interrupt events or for stop_pm_wdog() to succeed in
			deleting the WDOG timer always.
			So, gpio_cp2ap_wakeup and gpio_cp2ap_status must always
			be checked before state transition.
			*/
			if (gpiod_get_value(pm->gpio_cp2ap_wakeup))
				n_state = PM_STATE_ACTIVE;
			else if (!gpiod_get_value(pm->gpio_cp2ap_status))
				n_state = PM_STATE_UNMOUNTING;
			else
				n_state = PM_STATE_WDOG_TIMEOUT;
		}
		break;

	case PM_STATE_UNMOUNTING:
		if (event == PM_EVENT_LINK_UNMOUNTED) {
			if (pm->hold_requested) {
				if (cp_online(mc))
					n_state = PM_STATE_HOLDING;
				else
					n_state = PM_STATE_UNMOUNTED;
				pm->hold_requested = false;
			} else {
				n_state = PM_STATE_UNMOUNTED;
			}
			stop_pm_wdog(pm, c_state, event);
			release_ap2cp_status(pm);
			if (n_state == PM_STATE_HOLDING) {
				prepare_mount(pm);
				unmounted_to_holding(pm);
			}
		} else if (event == PM_EVENT_WDOG_TIMEOUT) {
			n_state = PM_STATE_WDOG_TIMEOUT;
		}
		break;

	case PM_STATE_CP_BOOTING:
		if (event == PM_EVENT_CP2AP_WAKEUP_HIGH) {
			n_state = PM_STATE_ACTIVE;
			assert_ap2cp_wakeup(pm);
		} else if (event == PM_EVENT_LINK_ERROR) {
			n_state = PM_STATE_CP_FAIL;
		}
		break;

	default:
		break;
	}

	set_pm_fsm(pm, c_state, n_state, event);

	print_pm_fsm(pm);

	decide_pm_wake(pm, c_state, n_state);

exit:
	spin_unlock_irqrestore(&pm->lock, flags);

	check_pm_fail(pm, c_state, n_state);
}

/**
@brief	interrupt handler for a wakeup interrupt

1) Reads the interrupt value\n
2) Performs interrupt handling\n

@param irq	the IRQ number
@param data	the pointer to a data
*/
static irqreturn_t cp2ap_wakeup_handler(int irq, void *data)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)data;
	int cp2ap_wakeup = gpiod_get_value(pm->gpio_cp2ap_wakeup);

	if (cp2ap_wakeup)
		run_pm_fsm(pm, PM_EVENT_CP2AP_WAKEUP_HIGH);
	else
		run_pm_fsm(pm, PM_EVENT_CP2AP_WAKEUP_LOW);

	return IRQ_HANDLED;
}

static irqreturn_t cp2ap_status_handler(int irq, void *data)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)data;
	int cp2ap_status = gpiod_get_value(pm->gpio_cp2ap_status);
	struct link_device *ld = pm_to_link_device(pm);
	struct mem_link_device *mld = ld_to_mem_link_device(ld);

	if (cp2ap_status) {
		run_pm_fsm(pm, PM_EVENT_CP2AP_STATUS_HIGH);
		wake_up_all(&mld->wq);
	} else {
		run_pm_fsm(pm, PM_EVENT_CP2AP_STATUS_LOW);
	}

	return IRQ_HANDLED;
}

static void cp_free_work_func(struct work_struct *ws)
{
	struct modem_link_pm *pm;

	pm = container_of(ws, struct modem_link_pm, cp_free_dwork.work);

	run_pm_fsm(pm, PM_EVENT_CP_HOLD_TIMEOUT);
}

static void pm_wdog_bark(struct timer_list *t)
{
	struct pm_wdog *wdog = from_timer(wdog, t, timer);
	struct modem_link_pm *pm = wdog_to_pm(wdog);
	struct pm_fsm *fsm = &pm->fsm;
	enum pm_state c_state;
	unsigned long flags;

	spin_lock_irqsave(&pm->lock, flags);
	c_state = fsm->state;
	spin_unlock_irqrestore(&pm->lock, flags);

	if (wdog->w_state == c_state) {
		mif_err("%s: PM WDOG lost event {%s@%s}\n", pm->link_name,
			pm_event2str(wdog->w_event), pm_state2str(wdog->state));
		return;
	}

	run_pm_fsm(pm, PM_EVENT_WDOG_TIMEOUT);
}

static void request_hold(struct modem_link_pm *pm)
{
	run_pm_fsm(pm, PM_EVENT_CP_HOLD_REQUEST);
}

static void release_hold(struct modem_link_pm *pm)
{
	/* stock left this empty: the hold ends on the cp_free timeout */
}

static inline void link_reset_cb(void *owner)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)owner;
	run_pm_fsm(pm, PM_EVENT_LINK_RESET);
}

static inline void link_unmount_cb(void *owner)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)owner;
	run_pm_fsm(pm, PM_EVENT_LINK_UNMOUNTED);
}

static inline void link_suspend_cb(void *owner)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)owner;
	run_pm_fsm(pm, PM_EVENT_LINK_SUSPENDED);
}

static inline void link_resume_cb(void *owner)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)owner;
	run_pm_fsm(pm, PM_EVENT_LINK_RESUMED);
}

static inline void link_mount_cb(void *owner)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)owner;
	struct link_device *ld = pm_to_link_device(pm);
	struct mem_link_device *mld = ld_to_mem_link_device(ld);

	run_pm_fsm(pm, PM_EVENT_LINK_MOUNTED);

	wake_up_all(&mld->wq);
}

static inline void link_error_cb(void *owner)
{
	struct modem_link_pm *pm = (struct modem_link_pm *)owner;
	run_pm_fsm(pm, PM_EVENT_LINK_ERROR);
}

void check_lli_irq(struct modem_link_pm *pm, enum mipi_lli_event event)
{
	switch (event) {
	case LLI_EVENT_RESET:
		link_reset_cb(pm);
		break;
	case LLI_EVENT_WAITFORMOUNT:
		break;
	case LLI_EVENT_MOUNTED:
		link_mount_cb(pm);
		break;
	case LLI_EVENT_UNMOUNTED:
		link_unmount_cb(pm);
		break;
	case LLI_EVENT_SUSPEND:
		link_suspend_cb(pm);
		break;
	case LLI_EVENT_RESUME:
		link_resume_cb(pm);
		break;
	default:
		break;
	}
}

static bool link_active(struct modem_link_pm *pm)
{
	bool ret;
	struct pm_fsm *fsm = &pm->fsm;
	unsigned long flags;

	spin_lock_irqsave(&pm->lock, flags);

	if (fsm->state == PM_STATE_LOCKED_ON
	    || fsm->state == PM_STATE_ACTIVE
	    || fsm->state == PM_STATE_AP_FREE)
		ret = true;
	else
		ret = false;

	spin_unlock_irqrestore(&pm->lock, flags);

	return ret;
}

static void start_link_pm(struct modem_link_pm *pm, enum pm_event event)
{
	enum pm_state state;
	unsigned long flags;

	spin_lock_irqsave(&pm->lock, flags);

	if (pm->active) {
		mif_err("%s: PM is already ACTIVE\n", pm->link_name);
		goto exit;
	}

	lock_pm_wake(pm);

	if (event == PM_EVENT_LOCK_ON) {
		state = PM_STATE_LOCKED_ON;
		assert_ap2cp_wakeup(pm);
		assert_ap2cp_status(pm);
	} else if (event == PM_EVENT_CP_BOOTING) {
		state = PM_STATE_CP_BOOTING;
		assert_ap2cp_wakeup(pm);
		assert_ap2cp_status(pm);
	} else {
		state = PM_STATE_UNMOUNTED;
		release_ap2cp_wakeup(pm);
		release_ap2cp_status(pm);
	}

	/* Enable every CP-to-AP IRQ (both edges, wake-up capable) */
	mif_enable_irq(&pm->cp2ap_wakeup_irq);
	mif_enable_irq(&pm->cp2ap_status_irq);

	set_pm_fsm(pm, PM_STATE_UNMOUNTED, state, event);

	pm->hold_requested = false;

	pm->active = true;

	print_pm_fsm(pm);

exit:
	spin_unlock_irqrestore(&pm->lock, flags);
}

static void stop_link_pm(struct modem_link_pm *pm)
{
	unsigned long flags;
	enum pm_state state;
	enum pm_event event;

	spin_lock_irqsave(&pm->lock, flags);

	if (!pm->active)
		goto exit;

	pm->active = false;

	mif_disable_irq(&pm->cp2ap_wakeup_irq);
	mif_err("%s: PM %s_irq#%d handler disabled\n", pm->link_name,
		pm->cp2ap_wakeup_irq.name, pm->cp2ap_wakeup_irq.num);

	mif_disable_irq(&pm->cp2ap_status_irq);
	mif_err("%s: PM %s_irq#%d handler disabled\n", pm->link_name,
		pm->cp2ap_status_irq.name, pm->cp2ap_status_irq.num);

	state = pm->fsm.state;
	event = PM_EVENT_STOP_PM;
	set_pm_fsm(pm, state, PM_STATE_UNMOUNTED, event);
	print_pm_event(pm, event);

exit:
	spin_unlock_irqrestore(&pm->lock, flags);
}

static inline void init_pm_fsm(struct modem_link_pm *pm)
{
	set_pm_fsm(pm, PM_STATE_UNMOUNTED, PM_STATE_UNMOUNTED,
		   PM_EVENT_NO_EVENT);
}

int init_link_device_pm(struct link_device *ld,
			struct modem_link_pm *pm,
			void (*fail_fn)(struct modem_link_pm *),
			void (*cp_fail_fn)(struct modem_link_pm *))
{
	struct modem_data *mdata = ld->mdm_data;
	struct device *dev = ld->dev;
	unsigned long flags;
	char name[MAX_NAME_LEN];
	int num;
	int err;

	pm->link_name = ld->name;
	pm->fail_handler = fail_fn;
	pm->cp_fail_handler = cp_fail_fn;

	pm->gpio_cp2ap_wakeup = mdata->gpio_ap_wakeup;
	pm->gpio_ap2cp_wakeup = mdata->gpio_cp_wakeup;
	pm->gpio_cp2ap_status = mdata->gpio_cp_status;
	pm->gpio_ap2cp_status = mdata->gpio_ap_status;

	if (!pm->gpio_cp2ap_wakeup || !pm->gpio_ap2cp_wakeup ||
	    !pm->gpio_cp2ap_status || !pm->gpio_ap2cp_status) {
		mif_err("%s: ERR! link PM GPIOs missing\n", ld->name);
		return -ENODEV;
	}

	spin_lock_init(&pm->lock);
	timer_setup(&pm->wdog.timer, pm_wdog_bark, 0);
	INIT_DELAYED_WORK(&pm->cp_free_dwork, cp_free_work_func);
	init_pm_fsm(pm);

	snprintf(pm->wlock_name, MAX_NAME_LEN, "%s_pm_wlock", pm->link_name);
	pm->ws = wakeup_source_register(dev, pm->wlock_name);
	if (!pm->ws)
		return -ENOMEM;

	snprintf(pm->wq_name, MAX_NAME_LEN, "%s_pm_wq", pm->link_name);
	pm->wq = alloc_ordered_workqueue("%s", WQ_HIGHPRI, pm->wq_name);
	if (!pm->wq) {
		err = -ENOMEM;
		goto err_ws;
	}

	/*
	 * Stock requested level-triggered IRQs and flipped the level in the
	 * handler. Both edges with the level read in the handler give the
	 * same events without irq_set_irq_type() from hard-IRQ context.
	 *
	 * The lines are wake IRQs (mif_request_irq() arms them), not
	 * IRQF_NO_SUSPEND: the core then wakes the system on an edge and
	 * replays it after resume_noirq, instead of running the link state
	 * machine, and its link reload, while devices are suspended.
	 */
	flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	num = gpiod_to_irq(pm->gpio_cp2ap_wakeup);
	if (num < 0) {
		err = num;
		goto err_wq;
	}
	snprintf(name, MAX_NAME_LEN, "%s_cp2ap_wakeup", pm->link_name);
	mif_init_irq(&pm->cp2ap_wakeup_irq, num, name, flags);
	err = mif_request_irq(dev, &pm->cp2ap_wakeup_irq, cp2ap_wakeup_handler,
			      pm);
	if (err)
		goto err_wq;
	mif_disable_irq(&pm->cp2ap_wakeup_irq);

	num = gpiod_to_irq(pm->gpio_cp2ap_status);
	if (num < 0) {
		err = num;
		goto err_wq;
	}
	snprintf(name, MAX_NAME_LEN, "%s_cp2ap_status", pm->link_name);
	mif_init_irq(&pm->cp2ap_status_irq, num, name, flags);
	err = mif_request_irq(dev, &pm->cp2ap_status_irq, cp2ap_status_handler,
			      pm);
	if (err)
		goto err_wq;
	mif_disable_irq(&pm->cp2ap_status_irq);

	pm->start = start_link_pm;
	pm->stop = stop_link_pm;
	pm->request_hold = request_hold;
	pm->release_hold = release_hold;
	pm->link_active = link_active;

	return 0;

err_wq:
	destroy_workqueue(pm->wq);
	pm->wq = NULL;
err_ws:
	wakeup_source_unregister(pm->ws);
	pm->ws = NULL;
	return err;
}

/*
 * Parse the "link-pm" child of the modem node. The stock property names
 * "gpio_cp2ap_wakeup" etc. become standard "<name>-gpios" properties.
 * cp2ap_status is the same pin as the parent's "cp-status-gpios" and is
 * taken from there.
 */
int link_pm_parse_dt_gpio_pdata(struct device *dev, struct device_node *np,
				struct modem_data *mdm)
{
	struct fwnode_handle *fw = of_fwnode_handle(np);

	mdm->gpio_ap_wakeup = devm_fwnode_gpiod_get(dev, fw, "cp2ap-wakeup",
						    GPIOD_IN, "cp2ap_wakeup");
	if (IS_ERR(mdm->gpio_ap_wakeup))
		return dev_err_probe(dev, PTR_ERR(mdm->gpio_ap_wakeup),
				     "cp2ap-wakeup\n");

	mdm->gpio_cp_wakeup = devm_fwnode_gpiod_get(dev, fw, "ap2cp-wakeup",
						    GPIOD_OUT_LOW,
						    "ap2cp_wakeup");
	if (IS_ERR(mdm->gpio_cp_wakeup))
		return dev_err_probe(dev, PTR_ERR(mdm->gpio_cp_wakeup),
				     "ap2cp-wakeup\n");

	mdm->gpio_ap_status = devm_fwnode_gpiod_get(dev, fw, "ap2cp-status",
						    GPIOD_OUT_LOW,
						    "ap2cp_status");
	if (IS_ERR(mdm->gpio_ap_status))
		return dev_err_probe(dev, PTR_ERR(mdm->gpio_ap_status),
				     "ap2cp-status\n");

	return 0;
}
