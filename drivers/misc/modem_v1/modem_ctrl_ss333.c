// SPDX-License-Identifier: GPL-2.0
/*
 * Power and reset control for the Shannon 333 (SS333) CP.
 *
 * Copyright (C) 2010 Samsung Electronics.
 * 6.6 port: gpiod, timer_setup, a wakeup source, killable boot wait.
 * The GPIO sequences and delays are stock.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/sched/signal.h>

#include "modem_prj.h"
#include "modem_utils.h"

/*
 * Stock waited 30 s for CP_START. Keep it below the 60 s hung-task panic
 * and make the wait killable so cbd can always be stopped.
 */
#define MIF_INIT_TIMEOUT	(30 * HZ)

enum crash_reason_event {
	CRASH_REASON_RESET = 0,
	CRASH_REASON_EXIT,
	CRASH_REASON_AT_RX,
	CRASH_REASON_AT_TX,
	CRASH_REASON_AP_PM,
	CRASH_REASON_CP_PM,
	CRASH_REASON_MAX_CNT
};

static const char * const cp_crash_reason[] = {
	[CRASH_REASON_RESET] = "CP_CRASH_RESET\n",
	[CRASH_REASON_EXIT] = "CP_CRASH_EXIT\n",
	[CRASH_REASON_AT_RX] = "abnormal RX. contact to CP team\n",
	[CRASH_REASON_AT_TX] = "abnormal TX. contack to AP team\n",
	[CRASH_REASON_AP_PM] = "PM TIMEOUT. contact to AP team\n",
	[CRASH_REASON_CP_PM] = "PM failed. CP team needs to investigate on their improper PM operations\n"
};

static void print_mc_state(struct modem_ctl *mc)
{
	mif_info("%s: %pS: MC state:%s on:%d reset:%d active:%d status:%d cp_dump_int:%d reason:%d\n",
		 mc->name, CALLER, mc_state(mc),
		 gpiod_get_value(mc->gpio_cp_on),
		 gpiod_get_value(mc->gpio_cp_reset),
		 gpiod_get_value(mc->gpio_phone_active),
		 gpiod_get_value(mc->gpio_cp_status),
		 gpiod_get_value(mc->gpio_cp_dump_int),
		 mc->crash_info);
}

void modem_state_change(struct modem_ctl *mc, enum modem_state new_state)
{
	if (mc->iod && mc->iod->modem_state_changed)
		mc->iod->modem_state_changed(mc->iod, new_state);

	if (mc->iod_ds && mc->iod_ds->modem_state_changed)
		mc->iod_ds->modem_state_changed(mc->iod_ds, new_state);

	if (mc->bootd && mc->bootd->modem_state_changed)
		mc->bootd->modem_state_changed(mc->bootd, new_state);

	modem_notify_event((enum modem_event)new_state);
}

/*
 * PHONE_ACTIVE rising edge. While the CP is online, phone_active going
 * high means it stopped: cp_on low = power-off, cp_dump_int low = crash
 * reset, otherwise crash exit (stock decision table).
 */
static irqreturn_t cp_active_handler(int irq, void *arg)
{
	struct modem_ctl *mc = arg;
	struct link_device *ld;
	enum modem_state old_state;
	enum modem_state new_state;
	unsigned long flags;

	if (!cp_online(mc))
		return IRQ_HANDLED;

	print_mc_state(mc);

	if (gpiod_get_value(mc->gpio_cp_reset) == 0)
		return IRQ_HANDLED;

	ld = get_current_link(mc->iod);

	spin_lock_irqsave(&mc->lock, flags);

	old_state = mc->phone_state;
	new_state = mc->phone_state;

	if (gpiod_get_value(mc->gpio_phone_active)) {
		if (gpiod_get_value(mc->gpio_cp_on) == 0)
			new_state = STATE_OFFLINE;
		else if (gpiod_get_value(mc->gpio_cp_dump_int) == 0)
			new_state = STATE_CRASH_RESET;
		else if (old_state == STATE_ONLINE)
			new_state = STATE_CRASH_EXIT;
		else
			mif_err("%s: %s: don't care!!!\n", mc->name, FUNC);
	}

	if (new_state != old_state)
		modem_state_change(mc, new_state);

	spin_unlock_irqrestore(&mc->lock, flags);

	if (old_state == STATE_ONLINE &&
	    (new_state == STATE_CRASH_EXIT || new_state == STATE_CRASH_RESET)) {
		del_timer(&mc->crash_ack_timer);
		if (ld && ld->close_tx)
			ld->close_tx(ld);
	}

	return IRQ_HANDLED;
}

static int ss333_on(struct modem_ctl *mc)
{
	struct io_device *iod = mc->iod;
	struct link_device *ld = get_current_link(iod);
	unsigned long flags;

	mif_disable_irq(&mc->irq_cp_active);

	mif_info("%s: %s: +++\n", mc->name, FUNC);
	print_mc_state(mc);

	spin_lock_irqsave(&mc->lock, flags);
	iod->modem_state_changed(iod, STATE_OFFLINE);
	gpiod_set_value(mc->gpio_pda_active, 1);
	mif_stay_awake(mc->ws);
	if (ld->off)
		ld->off(ld);
	spin_unlock_irqrestore(&mc->lock, flags);

	gpiod_set_value(mc->gpio_cp_on, 0);
	msleep(100);

	gpiod_set_value(mc->gpio_cp_reset, 0);
	print_mc_state(mc);
	msleep(500);

	gpiod_set_value(mc->gpio_cp_on, 1);
	msleep(100);

	if (ld->reset)
		ld->reset(ld);

	gpiod_set_value(mc->gpio_ap_dump_int, 0);

	gpiod_set_value(mc->gpio_cp_reset, 1);
	print_mc_state(mc);
	msleep(300);

	mif_info("%s: %s: ---\n", mc->name, FUNC);
	return 0;
}

static int ss333_off(struct modem_ctl *mc)
{
	struct io_device *iod = mc->iod;
	struct link_device *ld = get_current_link(iod);
	unsigned long flags;
	int i;

	mif_disable_irq(&mc->irq_cp_active);

	mif_info("%s: %s: +++\n", mc->name, FUNC);
	print_mc_state(mc);

	spin_lock_irqsave(&mc->lock, flags);
	if (cp_offline(mc)) {
		spin_unlock_irqrestore(&mc->lock, flags);
		mif_err("%s: %s: OFFLINE already!!!\n", mc->name, FUNC);
		goto exit;
	}
	iod->modem_state_changed(iod, STATE_OFFLINE);
	spin_unlock_irqrestore(&mc->lock, flags);

	del_timer_sync(&mc->crash_ack_timer);

	if (ld->close_tx)
		ld->close_tx(ld);

	if (gpiod_get_value(mc->gpio_cp_on) == 0) {
		mif_err("%s: cp_on == 0\n", mc->name);
		goto exit;
	}

	/* wait up to 3 s for phone_active */
	for (i = 0; i < 150; i++) {
		if (gpiod_get_value(mc->gpio_phone_active))
			break;
		msleep(20);
	}

	print_mc_state(mc);

	if (ld->off)
		ld->off(ld);

	if (gpiod_get_value(mc->gpio_cp_reset)) {
		mif_err("%s: %s: cp_reset -> 0\n", mc->name, FUNC);
		gpiod_set_value(mc->gpio_cp_reset, 0);
		print_mc_state(mc);
	}

exit:
	mif_info("%s: %s: ---\n", mc->name, FUNC);
	return 0;
}

static int ss333_reset(struct modem_ctl *mc)
{
	mif_err("+++\n");

	if (mc->phone_state == STATE_ONLINE)
		modem_notify_event(MODEM_EVENT_RESET);

	if (ss333_off(mc))
		return -EIO;

	msleep(100);

	if (ss333_on(mc))
		return -EIO;

	mif_err("---\n");
	return 0;
}

static void handle_no_response_cp_crash(struct timer_list *t)
{
	struct modem_ctl *mc = from_timer(mc, t, crash_ack_timer);
	unsigned long flags;

	if (cp_crashed(mc)) {
		mif_info("%s: STATE_CRASH_EXIT without response from CP\n",
			 mc->name);
		return;
	}

	mif_err("%s: ERR! No response from CP\n", mc->name);

	spin_lock_irqsave(&mc->lock, flags);
	modem_state_change(mc, STATE_CRASH_EXIT);
	spin_unlock_irqrestore(&mc->lock, flags);
}

static int ss333_force_crash_exit(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);

	mif_err("+++\n");

	mod_timer(&mc->crash_ack_timer, jiffies + FORCE_CRASH_ACK_TIMEOUT);

	mif_stay_awake(mc->ws);

	if (ld && ld->off)
		ld->off(ld);

	gpiod_set_value(mc->gpio_ap_dump_int, 1);
	mif_info("ap_dump_int -> %d\n", gpiod_get_value(mc->gpio_ap_dump_int));

	mif_err("---\n");
	return 0;
}

static int ss333_dump_reset(struct modem_ctl *mc)
{
	struct io_device *iod = mc->iod;
	struct link_device *ld = get_current_link(iod);
	unsigned long flags;

	mif_disable_irq(&mc->irq_cp_active);

	mif_info("%s: %s: +++\n", mc->name, FUNC);
	print_mc_state(mc);

	spin_lock_irqsave(&mc->lock, flags);
	iod->modem_state_changed(iod, STATE_CRASH_EXIT);
	mif_stay_awake(mc->ws);
	if (ld->off)
		ld->off(ld);
	spin_unlock_irqrestore(&mc->lock, flags);

	gpiod_set_value(mc->gpio_cp_reset, 0);
	print_mc_state(mc);
	udelay(200);

	if (ld->reset)
		ld->reset(ld);

	gpiod_set_value(mc->gpio_cp_reset, 1);
	print_mc_state(mc);
	msleep(300);

	gpiod_set_value(mc->gpio_ap_status, 1);

	mif_info("%s: %s: ---\n", mc->name, FUNC);
	return 0;
}

static int ss333_boot_on(struct modem_ctl *mc)
{
	struct link_device *ld = get_current_link(mc->iod);
	unsigned long flags;

	mif_err("+++\n");

	if (ld->boot_on)
		ld->boot_on(ld, mc->bootd);

	gpiod_set_value(mc->gpio_ap_status, 1);

	reinit_completion(&mc->init_cmpl);
	reinit_completion(&mc->off_cmpl);

	spin_lock_irqsave(&mc->lock, flags);
	mc->bootd->modem_state_changed(mc->bootd, STATE_BOOTING);
	mc->iod->modem_state_changed(mc->iod, STATE_BOOTING);
	spin_unlock_irqrestore(&mc->lock, flags);

	mif_err("---\n");
	return 0;
}

static int ss333_boot_off(struct modem_ctl *mc)
{
	unsigned long flags;
	long remain;

	mif_err("+++\n");

	/* CP_START from the CP completes init_cmpl (mem_cmd_handler) */
	remain = wait_for_completion_killable_timeout(&mc->init_cmpl,
						      MIF_INIT_TIMEOUT);
	if (remain <= 0) {
		mif_err("%s: CP_START %s\n", mc->name,
			remain ? "wait killed" : "T-I-M-E-O-U-T");
		return remain ? remain : -EAGAIN;
	}

	mif_enable_irq(&mc->irq_cp_active);

	spin_lock_irqsave(&mc->lock, flags);
	mc->bootd->modem_state_changed(mc->bootd, STATE_ONLINE);
	mc->iod->modem_state_changed(mc->iod, STATE_ONLINE);
	spin_unlock_irqrestore(&mc->lock, flags);

	atomic_set(&mc->forced_cp_crash, 0);

	modem_notify_event(MODEM_EVENT_ONLINE);

	mif_err("---\n");
	return 0;
}

static int ss333_boot_done(struct modem_ctl *mc)
{
	mif_err("+++\n");
	mif_relax(mc->ws);
	mif_err("---\n");
	return 0;
}

static int ss333_cp_crash_reason(struct modem_ctl *mc, char *buf)
{
	int event;

	switch (mc->crash_info) {
	case MDM_EVENT_CP_FORCE_RESET:
	case MDM_CRASH_CMD_RESET:
		event = CRASH_REASON_RESET;
		break;
	case MDM_EVENT_CP_FORCE_CRASH:
	case MDM_CRASH_CMD_EXIT:
		event = CRASH_REASON_EXIT;
		break;
	case MDM_CRASH_INVALID_RB:
	case MDM_CRASH_NO_MEM:
		event = CRASH_REASON_AT_TX;
		break;
	case MDM_CRASH_INVALID_IOD:
	case MDM_CRASH_INVALID_SKBCB:
	case MDM_CRASH_INVALID_SKBIOD:
	case MDM_CRASH_PACKET_CRACKED:
	case MDM_EVENT_CP_ABNORMAL_RX:
		event = CRASH_REASON_AT_RX;
		break;
	case MDM_CRASH_PM_FAIL:
		event = CRASH_REASON_AP_PM;
		break;
	case MDM_CRASH_PM_CP_FAIL:
		event = CRASH_REASON_CP_PM;
		break;
	default:
		return 0;
	}

	/* the caller's buffer has room after the crash tag (cp_crash_info) */
	strcpy(buf, cp_crash_reason[event]);
	return strlen(cp_crash_reason[event]);
}

static void ss333_get_ops(struct modem_ctl *mc)
{
	mc->ops.modem_on = ss333_on;
	mc->ops.modem_off = ss333_off;
	mc->ops.modem_reset = ss333_reset;
	mc->ops.modem_boot_on = ss333_boot_on;
	mc->ops.modem_boot_off = ss333_boot_off;
	mc->ops.modem_boot_done = ss333_boot_done;
	mc->ops.modem_force_crash_exit = ss333_force_crash_exit;
	mc->ops.modem_dump_reset = ss333_dump_reset;
	mc->ops.modem_cp_upload = ss333_cp_crash_reason;
}

static int modemctl_notify_call(struct notifier_block *nfb,
				unsigned long event, void *arg)
{
	struct modem_ctl *mc = container_of(nfb, struct modem_ctl, event_nfb);
	static int abnormal_rx_cnt;

	mif_info("got event: %ld\n", event);
	mc->crash_info = (enum modemctl_event)event;

	switch (event) {
	case MDM_EVENT_CP_FORCE_RESET:
		modem_state_change(mc, STATE_CRASH_RESET);
		break;
	case MDM_CRASH_PM_FAIL:
	case MDM_CRASH_PM_CP_FAIL:
	case MDM_CRASH_INVALID_RB:
	case MDM_CRASH_INVALID_IOD:
	case MDM_CRASH_INVALID_SKBCB:
	case MDM_CRASH_INVALID_SKBIOD:
	case MDM_CRASH_NO_MEM:
	case MDM_CRASH_CMD_RESET:
	case MDM_CRASH_CMD_EXIT:
	case MDM_EVENT_CP_FORCE_CRASH:
		ss333_force_crash_exit(mc);
		break;
	case MDM_EVENT_CP_ABNORMAL_RX:
		if (abnormal_rx_cnt < 5) {
			abnormal_rx_cnt++;
		} else {
			mif_err("abnormal rx count was overflowed.\n");
			abnormal_rx_cnt = 0;
		}
		break;
	}

	modem_notify_event(event);
	return 0;
}

static void ss333_release(void *data)
{
	struct modem_ctl *mc = data;

	unregister_cp_crash_notifier(&mc->event_nfb);
	del_timer_sync(&mc->crash_ack_timer);
	wakeup_source_unregister(mc->ws);
	mc->ws = NULL;
}

int ss333_init_modemctl_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	char name[MAX_NAME_LEN];
	int irq;
	int ret;

	mif_err("+++\n");

	if (!pdata->gpio_cp_on || !pdata->gpio_cp_reset ||
	    !pdata->gpio_pda_active || !pdata->gpio_phone_active ||
	    !pdata->gpio_ap_wakeup || !pdata->gpio_ap_status ||
	    !pdata->gpio_cp_wakeup || !pdata->gpio_cp_status ||
	    !pdata->gpio_ap_dump_int || !pdata->gpio_cp_dump_int) {
		mif_err("ERR! no GPIO data\n");
		return -ENXIO;
	}

	mc->gpio_cp_on = pdata->gpio_cp_on;
	mc->gpio_cp_reset = pdata->gpio_cp_reset;
	mc->gpio_pda_active = pdata->gpio_pda_active;
	mc->gpio_phone_active = pdata->gpio_phone_active;
	mc->gpio_ap_wakeup = pdata->gpio_ap_wakeup;
	mc->gpio_ap_status = pdata->gpio_ap_status;
	mc->gpio_cp_wakeup = pdata->gpio_cp_wakeup;
	mc->gpio_cp_status = pdata->gpio_cp_status;
	mc->gpio_ap_dump_int = pdata->gpio_ap_dump_int;
	mc->gpio_cp_dump_int = pdata->gpio_cp_dump_int;

	/* The CP stays off until cbd asks for it (IOCTL_MODEM_ON) */
	gpiod_set_value(mc->gpio_cp_reset, 0);
	gpiod_set_value(mc->gpio_cp_on, 0);
	print_mc_state(mc);

	ss333_get_ops(mc);
	timer_setup(&mc->crash_ack_timer, handle_no_response_cp_crash, 0);

	/* Held from power-on until IOCTL_MODEM_BOOT_DONE, and on a crash */
	mc->ws = wakeup_source_register(mc->dev, "ss333_wake_lock");
	if (!mc->ws)
		return -ENOMEM;

	irq = gpiod_to_irq(mc->gpio_phone_active);
	if (irq < 0) {
		mif_err("ERR! no irq_cp_active (%d)\n", irq);
		ret = irq;
		goto err_ws;
	}
	mif_err("PHONE_ACTIVE IRQ# = %d\n", irq);

	snprintf(name, MAX_NAME_LEN, "%s_active", mc->name);
	mif_init_irq(&mc->irq_cp_active, irq, name,
		     IRQF_TRIGGER_RISING | IRQF_NO_THREAD);
	ret = mif_request_irq(mc->dev, &mc->irq_cp_active, cp_active_handler,
			      mc);
	if (ret) {
		mif_err("%s: ERR! request_irq(%s#%d) fail (%d)\n", mc->name,
			mc->irq_cp_active.name, mc->irq_cp_active.num, ret);
		goto err_ws;
	}
	/* Armed by ss333_boot_off() once the CP is online */
	mif_disable_irq(&mc->irq_cp_active);

	mc->event_nfb.notifier_call = modemctl_notify_call;
	register_cp_crash_notifier(&mc->event_nfb);

	/* Undone if a later probe step fails (deferral included) */
	ret = devm_add_action_or_reset(mc->dev, ss333_release, mc);
	if (ret)
		return ret;

	mif_err("---\n");
	return 0;

err_ws:
	wakeup_source_unregister(mc->ws);
	mc->ws = NULL;
	return ret;
}
