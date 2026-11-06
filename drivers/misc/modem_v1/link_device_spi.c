// SPDX-License-Identifier: GPL-2.0
/*
 * SS333 boot link: the CP bootloader is pushed over SPI before the LLI
 * main link comes up.
 *
 * Copyright (C) 2011 Samsung Electronics.
 * 6.6 port: the SPI driver is registered once from modem_main, the device
 * comes from DT ("samsung,ss333-spi-boot", stock "spi_boot_link"), and the
 * bus is spi-gpio because the hardware SPI controllers' clocks sit in
 * CMU_PERIC1.
 *
 * cbd writes each bootloader chunk to /dev/umts_boot0 after switching that
 * node's TX link to SPI (IOCTL_MODEM_SET_TX_LINK). The payload goes out
 * without its SIPC5 header, as in stock.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "modem_prj.h"
#include "modem_utils.h"

struct spi_boot_link_device {
	struct link_device ld;
};

/* One CP per board; the spi_driver and the link device meet here */
static struct spi_device *spi_boot_dev;
static DEFINE_MUTEX(spi_boot_lock);	/* guards spi_boot_dev, serialises TX */

static int spi_boot_init(struct link_device *ld, struct io_device *iod)
{
	return 0;
}

static void spi_boot_terminate(struct link_device *ld, struct io_device *iod)
{
}

/*
 * The CP boot ROM takes one byte per chip-select frame. Stock got that from
 * the SPI2 controller's auto chip-select mode, which raises nCS after every
 * 8-bit packet; Samsung's other SPI boot driver (modem_boot_device_spi.c)
 * sends one byte per spi_sync for the same reason. spi-gpio holds CS for a
 * whole transfer, so frame each byte as its own message.
 */
static int spi_transmit_data(struct spi_device *spi, const u8 *buf,
			     unsigned int len)
{
	ktime_t start = ktime_get();
	unsigned int i;
	int ret = 0;

	for (i = 0; i < len; i++) {
		struct spi_transfer t = {
			.tx_buf = &buf[i],
			.len = 1,
		};

		ret = spi_sync_transfer(spi, &t, 1);
		if (ret < 0)
			break;
	}

	/* stock: delay_usecs = 10 after the transfer */
	udelay(10);

	mif_info("SPI boot: %u/%u bytes in %lld us (%d)\n", i, len,
		 ktime_us_delta(ktime_get(), start), ret);
	return ret;
}

static int spi_boot_send(struct link_device *ld, struct io_device *iod,
			 struct sk_buff *skb)
{
	unsigned int hdr_len, len;
	int ret;

	if (iod->format != IPC_BOOT) {
		mif_err("%s: not a boot device (format %d)\n", iod->name,
			iod->format);
		return -EINVAL;
	}

	hdr_len = skbpriv(skb)->lnk_hdr ? sipc5_get_hdr_len(skb->data) : 0;
	if (skb->len <= hdr_len)
		return -EINVAL;
	len = skb->len - hdr_len;

	mutex_lock(&spi_boot_lock);
	if (!spi_boot_dev) {
		mutex_unlock(&spi_boot_lock);
		mif_err("%s: SPI boot device not bound\n", ld->name);
		return -ENODEV;
	}
	ret = spi_transmit_data(spi_boot_dev, skb->data + hdr_len, len);
	mutex_unlock(&spi_boot_lock);

	if (ret < 0) {
		mif_err("%s: spi_sync fail (%d)\n", ld->name, ret);
		return ret;
	}

	/* stock returned without freeing the skb on success */
	ret = skb->len;
	dev_kfree_skb_any(skb);
	return ret;
}

static int spi_boot_probe(struct spi_device *spi)
{
	int ret;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret, "spi_setup\n");

	mutex_lock(&spi_boot_lock);
	spi_boot_dev = spi;
	mutex_unlock(&spi_boot_lock);

	dev_info(&spi->dev, "SS333 SPI boot link, %u Hz\n", spi->max_speed_hz);
	return 0;
}

static void spi_boot_remove(struct spi_device *spi)
{
	mutex_lock(&spi_boot_lock);
	if (spi_boot_dev == spi)
		spi_boot_dev = NULL;
	mutex_unlock(&spi_boot_lock);
}

static const struct of_device_id spi_boot_of_match[] = {
	{ .compatible = "samsung,ss333-spi-boot" },
	{ .compatible = "spi_boot_link" },	/* stock name */
	{ }
};
MODULE_DEVICE_TABLE(of, spi_boot_of_match);

static const struct spi_device_id spi_boot_ids[] = {
	{ "ss333-spi-boot" },
	{ "spi_boot_link" },
	{ }
};
MODULE_DEVICE_TABLE(spi, spi_boot_ids);

static struct spi_driver spi_boot_driver = {
	.probe = spi_boot_probe,
	.remove = spi_boot_remove,
	.id_table = spi_boot_ids,
	.driver = {
		.name = "spi_modem_boot",
		.of_match_table = spi_boot_of_match,
	},
};

int spi_boot_link_register(void)
{
	return spi_register_driver(&spi_boot_driver);
}

void spi_boot_link_unregister(void)
{
	spi_unregister_driver(&spi_boot_driver);
}

struct link_device *spi_create_link_device(struct platform_device *pdev,
					   struct modem_data *modem)
{
	struct spi_boot_link_device *sbld;
	struct link_device *ld;

	sbld = devm_kzalloc(&pdev->dev, sizeof(*sbld), GFP_KERNEL);
	if (!sbld)
		return ERR_PTR(-ENOMEM);

	ld = &sbld->ld;
	INIT_LIST_HEAD(&ld->list);
	ld->name = "spi_boot";
	ld->dev = &pdev->dev;
	ld->mdm_data = modem;
	ld->ipc_version = modem->ipc_version;
	ld->init_comm = spi_boot_init;
	ld->terminate_comm = spi_boot_terminate;
	ld->send = spi_boot_send;
	mif_info("%s link created\n", ld->name);
	return ld;
}
