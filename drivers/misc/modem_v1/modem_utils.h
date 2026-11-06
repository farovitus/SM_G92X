/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2011 Samsung Electronics.
 */

#ifndef __MODEM_UTILS_H__
#define __MODEM_UTILS_H__

#include <linux/rbtree.h>
#include "modem_prj.h"

#define MIF_TAG	"mif"

#define IS_CONNECTED(iod, ld) ((iod)->link_types & LINKTYPE((ld)->link_type))

#define MAX_IPC_SKB_SIZE 4096

enum ipc_layer {
	LNK_RX,
	LNK_TX,
	IOD_RX,
	IOD_TX,
	PS_RX,
	PS_TX,
	MAX_SIPC_LAYER
};

static const char * const sipc_layer_str[] = {
	[LNK_RX] = "LNK-RX",
	[LNK_TX] = "LNK-TX",
	[IOD_RX] = "IOD-RX",
	[IOD_TX] = "IOD-TX",
	[PS_RX] = "PS-RX",
	[PS_TX] = "PS-TX",
	[MAX_SIPC_LAYER] = "INVALID"
};

static inline const char *layer_str(enum ipc_layer layer)
{
	if (unlikely(layer >= MAX_SIPC_LAYER))
		return "INVALID";
	return sipc_layer_str[layer];
}

static const char * const dev_format_string[] = {
	[IPC_FMT]	= "FMT",
	[IPC_RAW]	= "RAW",
	[IPC_RFS]	= "RFS",
	[IPC_MULTI_RAW]	= "MULTI_RAW",
	[IPC_BOOT]	= "BOOT",
	[IPC_DUMP]	= "DUMP",
	[IPC_CMD]	= "CMD",
	[IPC_DEBUG]	= "DEBUG",
};

static inline const char *dev_str(enum dev_format dev)
{
	if (unlikely(dev >= MAX_DEV_FORMAT))
		return "INVALID";
	return dev_format_string[dev];
}

static inline enum direction opposite(enum direction dir)
{
	return (dir == TX) ? RX : TX;
}

static const char * const direction_string[] = {
	[TX] = "TX",
	[RX] = "RX"
};

static inline const char *dir_str(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "INVALID";
	return direction_string[dir];
}

static const char * const udl_string[] = {
	[UL] = "UL",
	[DL] = "DL"
};

static inline const char *udl_str(enum direction dir)
{
	if (unlikely(dir >= ULDL))
		return "INVALID";
	return udl_string[dir];
}

static const char * const q_direction_string[] = {
	[TX] = "TXQ",
	[RX] = "RXQ"
};

static inline const char *q_dir(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "INVALID";
	return q_direction_string[dir];
}

static const char * const ipc_direction_string[] = {
	[TX] = "AP->CP",
	[RX] = "AP<-CP"
};

static inline const char *ipc_dir(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "INVALID";
	return ipc_direction_string[dir];
}

static const char * const arrow_direction[] = {
	[TX] = "->",
	[RX] = "<-"
};

static inline const char *arrow(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "><";
	return arrow_direction[dir];
}

static const char * const modem_state_string[] = {
	[STATE_OFFLINE]		= "OFFLINE",
	[STATE_CRASH_RESET]	= "CRASH_RESET",
	[STATE_CRASH_EXIT]	= "CRASH_EXIT",
	[STATE_BOOTING]		= "BOOTING",
	[STATE_ONLINE]		= "ONLINE",
	[STATE_NV_REBUILDING]	= "NV_REBUILDING",
	[STATE_LOADER_DONE]	= "LOADER_DONE",
	[STATE_SIM_ATTACH]	= "SIM_ATTACH",
	[STATE_SIM_DETACH]	= "SIM_DETACH",
};

static inline const char *cp_state_str(enum modem_state state)
{
	if (unlikely(state > STATE_SIM_DETACH))
		return "INVALID";
	return modem_state_string[state];
}

static inline const char *mc_state(struct modem_ctl *mc)
{
	return cp_state_str(mc->phone_state);
}

static inline unsigned int calc_offset(void *target, void *base)
{
	return (unsigned long)target - (unsigned long)base;
}

static inline struct link_device *find_linkdev(struct modem_shared *msd,
					       enum modem_link link_type)
{
	struct link_device *ld;

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (ld->link_type == link_type)
			return ld;
	}
	return NULL;
}

static inline unsigned int count_bits(unsigned int n)
{
	return hweight32(n);
}

/* true when @cnt reaches a multiple of (@mask + 1); @mask is 2^n - 1 */
static inline bool count_flood(int cnt, int mask)
{
	return cnt > 0 && (cnt & mask) == 0;
}

void log_ipc_pkt(enum ipc_layer layer, u8 ch, struct sk_buff *skb);

/* print buffer as hex string */
int pr_buffer(const char *tag, const char *data, size_t data_len,
	      size_t max_len);

#define pr_skb(tag, skb) \
	pr_buffer(tag, (char *)((skb)->data), (size_t)((skb)->len), (size_t)16)

/* Stop/wake all TX queues in network interfaces */
void stop_net_iface(struct link_device *ld, unsigned int channel);
void resume_net_iface(struct link_device *ld, unsigned int channel);
void stop_net_ifaces(struct link_device *ld);
void resume_net_ifaces(struct link_device *ld);

/* flow control CMD from CP */
int link_rx_flowctl_cmd(struct link_device *ld, const char *data, size_t len);

struct io_device *get_iod_with_format(struct modem_shared *msd,
				      enum dev_format format);

static inline struct io_device *link_get_iod_with_format(
			struct link_device *ld, enum dev_format format)
{
	struct io_device *iod = get_iod_with_format(ld->msd, format);

	return (iod && IS_CONNECTED(iod, ld)) ? iod : NULL;
}

static inline struct io_device *get_iod_with_channel(
			struct modem_shared *msd, unsigned int channel)
{
	return msd->ch2iod[channel];
}

static inline struct io_device *link_get_iod_with_channel(
			struct link_device *ld, unsigned int channel)
{
	struct io_device *iod = get_iod_with_channel(ld->msd, channel);

	return (iod && IS_CONNECTED(iod, ld)) ? iod : NULL;
}

struct io_device *insert_iod_with_format(struct modem_shared *msd,
			enum dev_format format, struct io_device *iod);
void insert_iod_with_channel(struct modem_shared *msd, unsigned int channel,
			     struct io_device *iod);

typedef void (*action_fn)(struct io_device *iod, void *args);
void iodevs_for_each(struct modem_shared *msd, action_fn action, void *args);

void iodev_netif_wake(struct io_device *iod, void *args);
void iodev_netif_stop(struct io_device *iod, void *args);

void rawdevs_set_tx_link(struct modem_shared *msd, enum modem_link link_type);

__be32 ipv4str_to_be32(const char *ipv4str, size_t count);


#define IPV4_HDR_SIZE	20
#define TCP_HDR_SIZE	20
#define UDP_HDR_SIZE	8

void print_ipv4_packet(const u8 *ip_pkt, enum ipc_layer layer);
bool is_dns_packet(const u8 *ip_pkt);
bool is_syn_packet(const u8 *ip_pkt);

void mif_init_irq(struct modem_irq *irq, unsigned int num, const char *name,
		  unsigned long flags);
int mif_request_irq(struct device *dev, struct modem_irq *irq,
		    irq_handler_t isr, void *data);
void mif_enable_irq(struct modem_irq *irq);
void mif_disable_irq(struct modem_irq *irq);

static inline void mif_stay_awake(struct wakeup_source *ws)
{
	if (ws && !ws->active)
		__pm_stay_awake(ws);
}

static inline void mif_relax(struct wakeup_source *ws)
{
	if (ws && ws->active)
		__pm_relax(ws);
}

#endif /* __MODEM_UTILS_H__ */
