/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Broadcom BCM4773 bridge driver: shared state and the glue that the
 * imported Broadcom transport code (transport/, utils/) expects.
 */
#ifndef BCM4773_BBD_INTERNAL_H
#define BCM4773_BBD_INTERNAL_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "bbd_ifc.h"
#include "transport/bbd_engine.h"

/* Character devices, in the order and with the names lhd expects. */
enum {
	BBD_MINOR_SENSOR,
	BBD_MINOR_CONTROL,
	BBD_MINOR_PACKET,
	BBD_MINOR_RELIABLE,
	BBD_MINORS,
};

#define BBD_MAX_DATA_SIZE	4096	/* largest write accepted from lhd */
#define BBD_QUEUE_MAX		1024	/* items per read queue, as stock */
#define BBD_SENSOR_RX_SIZE	SZ_16K	/* hub -> SSP byte stream */

struct bbd_qitem {
	struct list_head node;
	size_t len;
	u8 data[];
};

/* One read queue per character device: BBD -> lhd. */
struct bbd_queue {
	spinlock_t lock;
	struct list_head items;
	unsigned int count;
	unsigned long dropped;
	wait_queue_head_t wait;
};

struct bcm4773_client_ops;
struct bbd_ssi;
struct platform_device;

struct bbd_device {
	/* Keep this name: the imported bridge reaches it through gpbbd_dev. */
	struct BbdEngine bbd_engine;
	struct delayed_work tick_work;

	struct device *dev;		/* the SPI device */
	struct bbd_ssi *ssi;
	struct platform_device *pdev;	/* bbd.0, where lhd looks */

	dev_t devt;
	struct cdev cdev;
	struct device *chr[BBD_MINORS];
	struct bbd_queue q[BBD_MINORS];

	/* Hub -> SSP byte stream (IRpcSensorResponse::Data payloads). */
	spinlock_t rx_lock;
	DECLARE_KFIFO(rx_fifo, u8, BBD_SENSOR_RX_SIZE);
	wait_queue_head_t rx_wait;
	unsigned long rx_dropped;

	spinlock_t client_lock;
	const struct bcm4773_client_ops *client_ops;
	void *client_priv;

	bool esw_ready;
	bool shmd;		/* sensor data stays in the kernel */
	bool debug;
};

extern struct bbd_device *gpbbd_dev;

/* core.c */
int bbd_core_init(struct bbd_device *bbd);
void bbd_core_exit(struct bbd_device *bbd);
ssize_t bbd_on_read(int minor, const unsigned char *buf, size_t size);
void bbd_sensor_rx(struct bbd_device *bbd, const u8 *buf, size_t len);
void bbd_control_str(struct bbd_device *bbd, const char *str);

/* engine.c */
void bbd_engine_init(struct bbd_device *bbd);
void bbd_engine_exit(struct bbd_device *bbd);
void bbd_engine_open(struct bbd_device *bbd);
void bbd_engine_close(struct bbd_device *bbd);
void bbd_engine_setup(struct bbd_device *bbd);
int bbd_engine_send_sensor(struct bbd_device *bbd, const void *buf, size_t len);
void bbd_engine_rx(struct bbd_device *bbd, const u8 *buf, size_t len);

/* ssi.c */
int bbd_ssi_send(struct bbd_ssi *ssi, const u8 *buf, size_t len);
void bbd_ssi_flush(struct bbd_ssi *ssi);
int bbd_ssi_get_power(struct bbd_ssi *ssi);
int bbd_ssi_set_power(struct bbd_ssi *ssi, bool on);
void bbd_ssi_set_debug(struct bbd_ssi *ssi, bool on);

/* What the imported transport code calls. */
static inline void *bbd_alloc(size_t size)
{
	return kmalloc(size, GFP_KERNEL);
}

static inline void bbd_free(void *p)
{
	kfree(p);
}

#define dprint(fmt, ...)	pr_debug("bcm4773: " fmt, ##__VA_ARGS__)

#endif /* BCM4773_BBD_INTERNAL_H */
