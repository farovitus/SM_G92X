// SPDX-License-Identifier: GPL-2.0-only
/**
@file		link_device_memory_debug.c
@brief		functions for debugging memory-type interfaces
@date		2014/02/18
@author		Hankook Jang (hankook.jang@samsung.com)
*/

/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/irq.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/netdevice.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device_memory.h"

#ifdef GROUP_MEM_LINK_DEBUG
/**
@weakgroup group_mem_link_debug
@{
*/

void print_pm_status(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	unsigned int magic;
	int ap_wakeup;
	int ap_status;
	int cp_wakeup;
	int cp_status;

	magic = get_magic(mld);
	ap_wakeup = gpiod_get_value(mld->gpio_ap_wakeup);
	ap_status = gpiod_get_value(mld->gpio_ap_status);
	cp_wakeup = gpiod_get_value(mld->gpio_cp_wakeup);
	cp_status = gpiod_get_value(mld->gpio_cp_status);

	/*
	** PM {ap_wakeup:cp_wakeup:cp_status:ap_status:magic} <CALLER>
	*/
	mif_info("%s: PM {%d:%d:%d:%d:%X} %d <%pf>\n", ld->name,
		ap_wakeup, cp_wakeup, cp_status, ap_status, magic,
		atomic_read(&mld->ref_cnt), CALLER);
}

/**
@brief		print a REQ_ACK

Prints a snapshot of the status of the @b @@dev circular queue when AP sends or
receives an REQ_ACK.

@param mld	the pointer to a mem_link_device instance
@param mst	the pointer to a mem_snapshot instance
@param dev	the pointer to a mem_ipc_device instance (IPC_FMT, etc.)
@param dir	the direction of communication (TX or RX)
*/
void print_req_ack(struct mem_link_device *mld, struct mem_snapshot *mst,
		   struct mem_ipc_device *dev, enum direction dir)
{
}

/**
@brief		print a RES_ACK

Prints a snapshot of the status of the @b @@dev circular queue when AP sends or
receives an RES_ACK.

@param mld	the pointer to a mem_link_device instance
@param mst	the pointer to a mem_snapshot instance
@param dev	the pointer to a mem_ipc_device instance (IPC_FMT, etc.)
@param dir	the direction of communication (TX or RX)

@remark		When a RES_ACK is sent, the status of the RXQ in @b @@dev must
		be shown. On the contrary, the status of the TXQ in @b @@dev
		must be shown When a RES_ACK is received.
*/
void print_res_ack(struct mem_link_device *mld, struct mem_snapshot *mst,
		   struct mem_ipc_device *dev, enum direction dir)
{
}

/**
@brief		print a snapshot of the status of a memory interface

@param mld	the pointer to a mem_link_device instance
@param mst	the pointer to a mem_snapshot instance
*/
void print_mem_snapshot(struct mem_link_device *mld, struct mem_snapshot *mst)
{
}

/**
@brief		print a snapshot of the status of a circular queue

@param mld	the pointer to a mem_link_device instance
@param mst	the pointer to a mem_snapshot instance
@param dev	the pointer to a mem_ipc_device instance (IPC_FMT, etc.)
*/
void print_dev_snapshot(struct mem_link_device *mld, struct mem_snapshot *mst,
			struct mem_ipc_device *dev)
{
}

#endif
