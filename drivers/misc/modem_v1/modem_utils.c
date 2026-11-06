// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 * 6.6 port: the IPC logger, the set_fs() file helpers, the trace events,
 * the /proc/baseband entry and gpio sysfs exports are gone. IPC packets
 * are logged with pr_info/pr_debug only, gated by the dflags parameter.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/sizes.h>
#include <linux/inet.h>

#include "modem_prj.h"
#include "modem_utils.h"

#define CMD_SUSPEND	((u16)(0x00CA))
#define CMD_RESUME	((u16)(0x00CB))

enum bit_debug_flags {
	DEBUG_FLAG_FMT,
	DEBUG_FLAG_RFS,
	DEBUG_FLAG_PS,
	DEBUG_FLAG_IOD,
	DEBUG_FLAG_CSVT,
	DEBUG_FLAG_BOOT,
	DEBUG_FLAG_DUMP,
	DEBUG_FLAG_LOG,
	DEBUG_FLAG_UNKNOWN
};

/* Stock logged every FMT (RIL) frame by default; keep that off here. */
static unsigned long dflags;
module_param(dflags, ulong, 0644);
MODULE_PARM_DESC(dflags, "modem_v1 IPC log flags (bit per channel class)");


static void dump2hex(char *buff, size_t buff_size, const char *data,
		     size_t data_len)
{
	size_t len = min(data_len, buff_size / 3);
	size_t i;
	char *d = buff;

	for (i = 0; i < len; i++)
		d += sprintf(d, "%02x ", (u8)data[i]);
	if (len)
		d--;
	*d = 0;
}

void log_ipc_pkt(enum ipc_layer layer, u8 ch, struct sk_buff *skb)
{
	char str[16 * 3 + 1];

	if (unlikely(!skb) || likely(!dflags))
		return;

	if (sipc_ps_ch(ch) && !test_bit(DEBUG_FLAG_PS, &dflags))
		return;
	if ((layer == IOD_RX || layer == IOD_TX) &&
	    !test_bit(DEBUG_FLAG_IOD, &dflags))
		return;

	if (!((sipc5_fmt_ch(ch) && test_bit(DEBUG_FLAG_FMT, &dflags)) ||
	      (sipc5_rfs_ch(ch) && test_bit(DEBUG_FLAG_RFS, &dflags)) ||
	      (sipc_csd_ch(ch) && test_bit(DEBUG_FLAG_CSVT, &dflags)) ||
	      (sipc_log_ch(ch) && test_bit(DEBUG_FLAG_LOG, &dflags)) ||
	      (sipc5_boot_ch(ch) && test_bit(DEBUG_FLAG_BOOT, &dflags)) ||
	      (sipc5_dump_ch(ch) && test_bit(DEBUG_FLAG_DUMP, &dflags)) ||
	      sipc_ps_ch(ch) || test_bit(DEBUG_FLAG_UNKNOWN, &dflags)))
		return;

	dump2hex(str, sizeof(str), skb->data, min_t(u32, skb->len, 16));
	pr_info("%s: %s(%u:%d): %s\n", MIF_TAG, layer_str(layer), ch,
		skb->len, str);
}

int pr_buffer(const char *tag, const char *data, size_t data_len,
	      size_t max_len)
{
	char str[32 * 3 + 1];
	size_t len = min3(data_len, max_len, (size_t)32);

	dump2hex(str, sizeof(str), data, len);
	pr_info("%s: %s(%zu): %s%s\n", MIF_TAG, tag, data_len, str,
		(len == data_len) ? "" : " ...");
	return 0;
}

/* flow control CMD from CP */
int link_rx_flowctl_cmd(struct link_device *ld, const char *data, size_t len)
{
	struct modem_shared *msd = ld->msd;
	const u16 *cmd, *end = (const u16 *)(data + len);

	for (cmd = (const u16 *)data; cmd < end; cmd++) {
		switch (*cmd) {
		case CMD_SUSPEND:
			iodevs_for_each(msd, iodev_netif_stop, NULL);
			ld->raw_tx_suspended = true;
			mif_info("flowctl CMD_SUSPEND(%04X)\n", *cmd);
			break;

		case CMD_RESUME:
			iodevs_for_each(msd, iodev_netif_wake, NULL);
			ld->raw_tx_suspended = false;
			complete_all(&ld->raw_tx_resumed_by_cp);
			mif_info("flowctl CMD_RESUME(%04X)\n", *cmd);
			break;

		default:
			mif_err("flowctl BAD CMD: %04X\n", *cmd);
			break;
		}
	}

	return 0;
}

struct io_device *get_iod_with_format(struct modem_shared *msd,
				      enum dev_format format)
{
	struct rb_node *n = msd->iodevs_tree_fmt.rb_node;

	while (n) {
		struct io_device *iodev;

		iodev = rb_entry(n, struct io_device, node_fmt);
		if (format < iodev->format)
			n = n->rb_left;
		else if (format > iodev->format)
			n = n->rb_right;
		else
			return iodev;
	}

	return NULL;
}

void insert_iod_with_channel(struct modem_shared *msd, unsigned int channel,
			     struct io_device *iod)
{
	unsigned int idx = msd->num_channels;

	msd->ch2iod[channel] = iod;
	msd->ch[idx] = channel;
	msd->num_channels++;
}

struct io_device *insert_iod_with_format(struct modem_shared *msd,
		enum dev_format format, struct io_device *iod)
{
	struct rb_node **p = &msd->iodevs_tree_fmt.rb_node;
	struct rb_node *parent = NULL;

	while (*p) {
		struct io_device *iodev;

		parent = *p;
		iodev = rb_entry(parent, struct io_device, node_fmt);
		if (format < iodev->format)
			p = &(*p)->rb_left;
		else if (format > iodev->format)
			p = &(*p)->rb_right;
		else
			return iodev;
	}

	rb_link_node(&iod->node_fmt, parent, p);
	rb_insert_color(&iod->node_fmt, &msd->iodevs_tree_fmt);
	return NULL;
}

void iodevs_for_each(struct modem_shared *msd, action_fn action, void *args)
{
	unsigned int i;

	for (i = 0; i < msd->num_channels; i++) {
		u8 ch = msd->ch[i];
		struct io_device *iod = msd->ch2iod[ch];

		if (iod)
			action(iod, args);
	}
}

void iodev_netif_wake(struct io_device *iod, void *args)
{
	if (iod && iod->io_typ == IODEV_NET && iod->ndev) {
		netif_wake_queue(iod->ndev);
		mif_debug("%s\n", iod->name);
	}
}

void iodev_netif_stop(struct io_device *iod, void *args)
{
	if (iod && iod->io_typ == IODEV_NET && iod->ndev) {
		netif_stop_queue(iod->ndev);
		mif_debug("%s\n", iod->name);
	}
}

static void iodev_set_tx_link(struct io_device *iod, void *args)
{
	struct link_device *ld = args;

	if (iod->format == IPC_RAW && IS_CONNECTED(iod, ld)) {
		set_current_link(iod, ld);
		mif_err("%s -> %s\n", iod->name, ld->name);
	}
}

void rawdevs_set_tx_link(struct modem_shared *msd, enum modem_link link_type)
{
	struct link_device *ld = find_linkdev(msd, link_type);

	if (ld)
		iodevs_for_each(msd, iodev_set_tx_link, ld);
}

void stop_net_iface(struct link_device *ld, unsigned int channel)
{
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (test_bit(channel, &ld->netif_stop_mask)) {
		mif_err("channel %d was already stopped!\n", channel);
		goto exit;
	}

	iodev_netif_stop(link_get_iod_with_channel(ld, channel), NULL);
	set_bit(channel, &ld->netif_stop_mask);

exit:
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void stop_net_ifaces(struct link_device *ld)
{
	struct io_device *iod;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (atomic_read(&ld->netif_stopped) > 0)
		goto exit;

	iod = link_get_iod_with_channel(ld, SIPC_CH_ID_PDP_0);
	if (iod)
		iodevs_for_each(iod->msd, iodev_netif_stop, NULL);

	atomic_set(&ld->netif_stopped, 1);

exit:
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void resume_net_iface(struct link_device *ld, unsigned int channel)
{
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (!test_bit(channel, &ld->netif_stop_mask)) {
		mif_err("channel %d was already resumed!\n", channel);
		goto exit;
	}

	iodev_netif_wake(link_get_iod_with_channel(ld, channel), NULL);
	clear_bit(channel, &ld->netif_stop_mask);

exit:
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void resume_net_ifaces(struct link_device *ld)
{
	struct io_device *iod;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (atomic_read(&ld->netif_stopped) == 0)
		goto exit;

	iod = link_get_iod_with_channel(ld, SIPC_CH_ID_PDP_0);
	if (iod)
		iodevs_for_each(iod->msd, iodev_netif_wake, NULL);

	atomic_set(&ld->netif_stopped, 0);

exit:
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

/* "a.b.c.d" to a big-endian address; 0 (0.0.0.0, disabled) on error */
__be32 ipv4str_to_be32(const char *ipv4str, size_t count)
{
	__be32 addr;

	if (!ipv4str || !count)
		return 0;

	if (!in4_pton(ipv4str, count, (u8 *)&addr, -1, NULL))
		return 0;

	return addr;
}

void mif_init_irq(struct modem_irq *irq, unsigned int num, const char *name,
		  unsigned long flags)
{
	spin_lock_init(&irq->lock);
	irq->num = num;
	strscpy(irq->name, name, sizeof(irq->name));
	irq->flags = flags;
	irq->active = false;
}

/*
 * Request a CP-to-AP interrupt, devm-managed and marked as a wakeup
 * source. The IRQ is left enabled; callers that want it masked until
 * the link starts call mif_disable_irq() right after.
 */
int mif_request_irq(struct device *dev, struct modem_irq *irq,
		    irq_handler_t isr, void *data)
{
	int ret;

	ret = devm_request_irq(dev, irq->num, isr, irq->flags, irq->name,
			       data);
	if (ret) {
		mif_err("%s: ERR! request_irq fail (%d)\n", irq->name, ret);
		return ret;
	}

	irq->active = true;

	ret = enable_irq_wake(irq->num);
	if (ret)
		mif_err("%s: enable_irq_wake fail (%d)\n", irq->name, ret);

	mif_info("%s(#%d) handler registered (flags:0x%08lX)\n",
		 irq->name, irq->num, irq->flags);
	return 0;
}

void mif_enable_irq(struct modem_irq *irq)
{
	unsigned long flags;

	spin_lock_irqsave(&irq->lock, flags);
	if (!irq->active) {
		enable_irq(irq->num);
		irq->active = true;
	}
	spin_unlock_irqrestore(&irq->lock, flags);
}

void mif_disable_irq(struct modem_irq *irq)
{
	unsigned long flags;

	spin_lock_irqsave(&irq->lock, flags);
	if (irq->active) {
		disable_irq_nosync(irq->num);
		irq->active = false;
	}
	spin_unlock_irqrestore(&irq->lock, flags);
}

/*
 * CP crash and modem event notifiers. Atomic chains: events are raised
 * from IRQ context (LLI events, PHONE_ACTIVE) as well as process context.
 */
static ATOMIC_NOTIFIER_HEAD(cp_crash_notifier);

int register_cp_crash_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&cp_crash_notifier, nb);
}

int unregister_cp_crash_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&cp_crash_notifier, nb);
}

void modemctl_notify_event(enum modemctl_event evt)
{
	atomic_notifier_call_chain(&cp_crash_notifier, evt, NULL);
}

static ATOMIC_NOTIFIER_HEAD(modem_event_notifier);

int register_modem_event_notifier(struct notifier_block *nb)
{
	if (!nb)
		return -ENOENT;
	return atomic_notifier_chain_register(&modem_event_notifier, nb);
}

void modem_notify_event(enum modem_event evt)
{
	mif_info("event notify (%d)\n", evt);
	atomic_notifier_call_chain(&modem_event_notifier, evt, NULL);
}

