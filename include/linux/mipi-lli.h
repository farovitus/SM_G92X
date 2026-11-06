/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MIPI Low Latency Interface (LLI) consumer API.
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Author: Yulgon Kim <yulgon.kim@samsung.com>
 *
 * The SoC has one LLI controller, so the API keeps the stock form of
 * global calls. A consumer must check mipi_lli_ready() first and defer
 * its probe until it returns true.
 *
 * Every call may run in atomic context except mipi_lli_unregister_handler()
 * and mipi_lli_suspend(), which sleep. The handler runs in hard IRQ context
 * for LLI_EVENT_SIG and in the controller's IRQ thread or workqueue for the
 * other events.
 */

#ifndef __MIPI_LLI_H
#define __MIPI_LLI_H

#include <linux/types.h>

enum mipi_lli_event {
	LLI_EVENT_SIG		= 0x01,
	LLI_EVENT_RESET		= 0x02,
	LLI_EVENT_WAITFORMOUNT	= 0x04,
	LLI_EVENT_MOUNTED	= 0x08,
	LLI_EVENT_UNMOUNTED	= 0x10,
	LLI_EVENT_SUSPEND	= 0x20,
	LLI_EVENT_RESUME	= 0x40,
};

enum mipi_lli_link_status {
	LLI_UNMOUNTED,
	LLI_MOUNTED,
	LLI_WAITFORMOUNT,
};

typedef void (*mipi_lli_handler_t)(void *data, enum mipi_lli_event event,
				   u32 intr);

#if IS_ENABLED(CONFIG_MIPI_LLI)
bool mipi_lli_ready(void);

/* Shared memory carved out for the CP (8 MB, from DT reserved-memory) */
void __iomem *mipi_lli_request_sh_region(unsigned long sh_addr,
					 unsigned long size);
void mipi_lli_release_sh_region(void __iomem *rgn);
phys_addr_t mipi_lli_get_phys_base(void);
unsigned long mipi_lli_get_phys_size(void);

int mipi_lli_get_link_status(void);
int mipi_lli_set_link_status(int status);
int mipi_lli_suspended(void);

int mipi_lli_register_handler(mipi_lli_handler_t handler, void *data);
int mipi_lli_unregister_handler(mipi_lli_handler_t handler);

void mipi_lli_send_interrupt(u32 cmd);
void mipi_lli_reset_interrupt(void);
u32 mipi_lli_read_interrupt(void);
void mipi_lli_debug_info(void);
void mipi_lli_intr_enable(void);
void mipi_lli_intr_disable(void);
void mipi_lli_reset(void);
void mipi_lli_reload(void);
void mipi_lli_suspend(void);
void mipi_lli_resume(void);
void mipi_lli_mask_sb_intr(bool mask);
#else
static inline bool mipi_lli_ready(void) { return false; }
#endif

#endif /* __MIPI_LLI_H */
