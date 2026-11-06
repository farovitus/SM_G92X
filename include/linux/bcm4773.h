/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Broadcom BCM4773 bridge: interface for the sensor hub driver.
 *
 * @hub is the BCM4773 SPI device, the parent of the sensor hub node.
 * The hub carries a byte stream of SSP messages; it is usable only between
 * ready(true) and ready(false), which follow lhd's ESW:READY/NOTREADY.
 */
#ifndef _LINUX_BCM4773_H
#define _LINUX_BCM4773_H

#include <linux/types.h>

struct device;

struct bcm4773_client_ops {
	/* Both run in atomic context: queue work, do not sleep. */
	void (*rx)(void *priv);			/* new bytes to bcm4773_recv() */
	void (*ready)(void *priv, bool ready);	/* hub firmware up or down */
};

int bcm4773_register_client(struct device *hub,
			    const struct bcm4773_client_ops *ops, void *priv);
void bcm4773_unregister_client(struct device *hub);
bool bcm4773_ready(struct device *hub);
int bcm4773_send(struct device *hub, const void *buf, size_t len);
/* Waits for exactly @len bytes; returns @len or a negative errno. */
ssize_t bcm4773_recv(struct device *hub, void *buf, size_t len,
		     unsigned int timeout_ms);
size_t bcm4773_rx_avail(struct device *hub);
void bcm4773_flush(struct device *hub);
/* Asks lhd to power-cycle the chip and reload its firmware. */
int bcm4773_request_reset(struct device *hub);

#endif /* _LINUX_BCM4773_H */
