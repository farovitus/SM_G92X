// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCM4773 SSI transport over SPI.
 *
 * Each SPI transfer starts with a command byte; the chip answers with a
 * status byte and a count:
 *   read status:  [0x20 0]             -> [stat count]
 *   read data:    [0x20 0 x count]     -> [stat count data...]
 *   write:        [0x40 n data x n]    -> [stat count rxdata...]
 * Full duplex: a write also returns up to n bytes the chip had queued.
 * A burst is framed by MCU_REQ (AP wakes the chip) and MCU_RESP (chip
 * awake). HOST_REQ is high while the chip has data for the AP.
 *
 * Bytes read are handed to the bridge from a work item, never under the
 * transfer lock, so a sender holding the bridge lock can always wait for
 * TX room.
 */

#include <linux/bcm4773.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_wakeup.h>
#include <linux/spi/spi.h>

#include "bbd_internal.h"

#define SSI_MODE_FULL_DUPLEX	0x40
#define SSI_READ_TRANS		0x20
#define SSI_WRITE_TRANS		0x00
#define SSI_MAX_RW		255
#define SSI_MAX_READ		254	/* stock: 255 + header broke DMA */
#define SSI_BUF_SIZE		(SSI_MAX_RW + 2)
#define SSI_MAX_LOOPS		300	/* then yield the lock; the run resumes */
#define SSI_FIFO_SIZE		SZ_32K
#define SSI_TX_WAIT_MS		1000

struct bbd_ssi {
	struct spi_device *spi;
	struct bbd_device *bbd;
	struct gpio_desc *host_req;
	struct gpio_desc *mcu_req;
	struct gpio_desc *mcu_resp;
	struct gpio_desc *enable;

	struct mutex xfer_lock;		/* SPI bursts and the power state */
	u8 *tx;
	u8 *rx;
	bool powered;

	spinlock_t fifo_lock;
	DECLARE_KFIFO(tx_fifo, u8, SSI_FIFO_SIZE);
	DECLARE_KFIFO(rx_fifo, u8, SSI_FIFO_SIZE);
	wait_queue_head_t tx_wait;

	struct workqueue_struct *tx_wq;
	struct workqueue_struct *rx_wq;
	struct work_struct tx_work;
	struct work_struct rx_work;

	bool debug;
	unsigned long tx_bytes, rx_bytes;
	unsigned long tx_dropped, rx_dropped, errors;
};

static int ssi_xfer(struct bbd_ssi *ssi, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = ssi->tx,
		.rx_buf = ssi->rx,
		.len = len,
	};
	int ret = spi_sync_transfer(ssi->spi, &t, 1);

	if (ssi->debug)
		dev_info(&ssi->spi->dev, "ssi %02x/%zu -> %02x %02x %*ph\n",
			 ssi->tx[0], len, ssi->rx[0], ssi->rx[1],
			 (int)min_t(size_t, len - 2, 16), ssi->rx + 2);
	return ret;
}

/* Called with xfer_lock held. */
static void ssi_queue_rx(struct bbd_ssi *ssi, const u8 *buf, size_t len)
{
	unsigned long flags;

	spin_lock_irqsave(&ssi->fifo_lock, flags);
	if (kfifo_avail(&ssi->rx_fifo) >= len)
		kfifo_in(&ssi->rx_fifo, buf, len);
	else
		ssi->rx_dropped += len;	/* the transport layer recovers */
	spin_unlock_irqrestore(&ssi->fifo_lock, flags);

	ssi->rx_bytes += len;
	queue_work(ssi->rx_wq, &ssi->rx_work);
}

static int ssi_read(struct bbd_ssi *ssi)
{
	unsigned int len, got;
	int ret;

	memset(ssi->tx, 0, 2);
	ssi->tx[0] = SSI_READ_TRANS;
	ret = ssi_xfer(ssi, 2);
	if (ret)
		return ret;
	if (ssi->rx[0])
		return -EIO;

	len = min_t(unsigned int, ssi->rx[1], SSI_MAX_READ);
	if (!len)
		return 0;

	memset(ssi->tx, 0, len + 2);
	ssi->tx[0] = SSI_READ_TRANS;
	ret = ssi_xfer(ssi, len + 2);
	if (ret)
		return ret;

	got = min_t(unsigned int, len, ssi->rx[1]);
	if (got < len)
		dev_dbg(&ssi->spi->dev, "short read %u of %u\n", got, len);
	ssi_queue_rx(ssi, ssi->rx + 2, got);
	return got;
}

static int ssi_write(struct bbd_ssi *ssi)
{
	unsigned int n, got;
	int ret;

	n = kfifo_out_spinlocked(&ssi->tx_fifo, ssi->tx + 2, SSI_MAX_RW,
				 &ssi->fifo_lock);
	if (!n)
		return 0;
	wake_up(&ssi->tx_wait);

	ssi->tx[0] = SSI_WRITE_TRANS | SSI_MODE_FULL_DUPLEX;
	ssi->tx[1] = n;
	ret = ssi_xfer(ssi, n + 2);
	if (ret)
		return ret;
	if (ssi->rx[0])
		return -EIO;	/* written bytes are lost: the transport resends */

	ssi->tx_bytes += n;
	got = min_t(unsigned int, n, ssi->rx[1]);
	if (got)
		ssi_queue_rx(ssi, ssi->rx + 2, got);
	return n;
}

static bool ssi_hello(struct bbd_ssi *ssi)
{
	int count, retries = 0;

	gpiod_set_value_cansleep(ssi->mcu_req, 1);
	for (count = 0; count <= 100; count++) {
		if (gpiod_get_value_cansleep(ssi->mcu_resp))
			return true;
		usleep_range(1000, 1200);
		if (count % 20 == 19 && retries++ < 3) {
			gpiod_set_value_cansleep(ssi->mcu_req, 0);
			usleep_range(1000, 1200);
			gpiod_set_value_cansleep(ssi->mcu_req, 1);
			usleep_range(1000, 1200);
		}
	}
	gpiod_set_value_cansleep(ssi->mcu_req, 0);
	return false;
}

static bool ssi_tx_pending(struct bbd_ssi *ssi)
{
	return !kfifo_is_empty(&ssi->tx_fifo);
}

/*
 * Drain both directions. Returns 0 or a negative errno.
 *
 * A burst stops after SSI_MAX_LOOPS so power changes and the other caller
 * get the lock. Bytes still queued then must not wait for the next send:
 * lhd's patch download keeps a window of packets in flight and sends no
 * more until the chip acknowledges them, so a stranded tail stalls the
 * download until lhd times out and reports the hub crashed. The run is
 * queued again instead.
 */
static int ssi_run(struct bbd_ssi *ssi)
{
	struct device *dev = &ssi->spi->dev;
	bool req;
	int loops, ret = 0;

	mutex_lock(&ssi->xfer_lock);
	if (!ssi->powered)
		goto unlock;

	req = gpiod_get_value_cansleep(ssi->host_req);
	if (!req && !ssi_tx_pending(ssi))
		goto unlock;

	pm_stay_awake(dev);
	if (!ssi_hello(ssi)) {
		dev_err_ratelimited(dev, "no MCU_RESP\n");
		ret = -ETIMEDOUT;
		goto relax;
	}

	for (loops = 0; req || ssi_tx_pending(ssi); loops++) {
		if (loops >= SSI_MAX_LOOPS) {
			dev_dbg(dev, "burst yields after %d loops\n", loops);
			break;
		}
		if (req) {
			ret = ssi_read(ssi);
			if (ret < 0)
				break;
		}
		if (ssi_tx_pending(ssi)) {
			ret = ssi_write(ssi);
			if (ret < 0)
				break;
		}
		req = gpiod_get_value_cansleep(ssi->host_req);
	}
	gpiod_set_value_cansleep(ssi->mcu_req, 0);
	if (ret > 0)
		ret = 0;
relax:
	if (ret) {
		ssi->errors++;
		dev_err_ratelimited(dev, "transfer failed: %d\n", ret);
	}
	pm_relax(dev);
unlock:
	mutex_unlock(&ssi->xfer_lock);
	if (!ret && READ_ONCE(ssi->powered) && ssi_tx_pending(ssi))
		queue_work(ssi->tx_wq, &ssi->tx_work);
	return ret;
}

static irqreturn_t ssi_irq_thread(int irq, void *data)
{
	struct bbd_ssi *ssi = data;

	/*
	 * HOST_REQ is level-triggered: back off when it cannot be served
	 * (chip off, transfer error) so a stuck line cannot spin.
	 */
	if (ssi_run(ssi) || !READ_ONCE(ssi->powered)) {
		/* The TX work retries what this run could not send. */
		if (ssi_tx_pending(ssi))
			queue_work(ssi->tx_wq, &ssi->tx_work);
		msleep(20);
	}
	return IRQ_HANDLED;
}

static void ssi_tx_work(struct work_struct *work)
{
	struct bbd_ssi *ssi = container_of(work, struct bbd_ssi, tx_work);

	if (ssi_run(ssi) && ssi_tx_pending(ssi)) {
		msleep(20);
		queue_work(ssi->tx_wq, &ssi->tx_work);
	}
}

static void ssi_rx_work(struct work_struct *work)
{
	struct bbd_ssi *ssi = container_of(work, struct bbd_ssi, rx_work);
	u8 buf[SSI_MAX_RW];
	unsigned int n;

	while ((n = kfifo_out_spinlocked(&ssi->rx_fifo, buf, sizeof(buf),
					&ssi->fifo_lock)))
		bbd_engine_rx(ssi->bbd, buf, n);
}

/* Called from the bridge, in process context, with the bridge lock held. */
int bbd_ssi_send(struct bbd_ssi *ssi, const u8 *buf, size_t len)
{
	unsigned long flags;
	long left;

	if (len > SSI_FIFO_SIZE)
		return -EMSGSIZE;

	left = wait_event_timeout(ssi->tx_wait,
				  kfifo_avail(&ssi->tx_fifo) >= len ||
				  !READ_ONCE(ssi->powered),
				  msecs_to_jiffies(SSI_TX_WAIT_MS));

	spin_lock_irqsave(&ssi->fifo_lock, flags);
	if (!READ_ONCE(ssi->powered) || kfifo_avail(&ssi->tx_fifo) < len) {
		ssi->tx_dropped += len;
		spin_unlock_irqrestore(&ssi->fifo_lock, flags);
		dev_warn_ratelimited(&ssi->spi->dev, "tx dropped %zu (%s)\n",
				     len, left ? "off" : "timeout");
		return -EIO;
	}
	kfifo_in(&ssi->tx_fifo, buf, len);
	spin_unlock_irqrestore(&ssi->fifo_lock, flags);

	queue_work(ssi->tx_wq, &ssi->tx_work);
	return 0;
}

void bbd_ssi_flush(struct bbd_ssi *ssi)
{
	unsigned long flags;

	spin_lock_irqsave(&ssi->fifo_lock, flags);
	kfifo_reset(&ssi->tx_fifo);
	kfifo_reset(&ssi->rx_fifo);
	spin_unlock_irqrestore(&ssi->fifo_lock, flags);
	wake_up(&ssi->tx_wait);
}

int bbd_ssi_get_power(struct bbd_ssi *ssi)
{
	return READ_ONCE(ssi->powered);
}

/* nSTANDBY, driven by lhd through bbd.0/nstandby. */
int bbd_ssi_set_power(struct bbd_ssi *ssi, bool on)
{
	mutex_lock(&ssi->xfer_lock);
	if (on != ssi->powered) {
		dev_info(&ssi->spi->dev, "power %s\n", on ? "on" : "off");
		gpiod_set_value_cansleep(ssi->mcu_req, 0);
		gpiod_set_value_cansleep(ssi->enable, on);
		WRITE_ONCE(ssi->powered, on);
		bbd_ssi_flush(ssi);
	}
	mutex_unlock(&ssi->xfer_lock);
	return 0;
}

void bbd_ssi_set_debug(struct bbd_ssi *ssi, bool on)
{
	ssi->debug = on;
}

static void ssi_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

static void ssi_core_exit(void *bbd)
{
	bbd_core_exit(bbd);
}

static int bbd_ssi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct bbd_device *bbd;
	struct bbd_ssi *ssi;
	int ret;

	bbd = devm_kzalloc(dev, sizeof(*bbd), GFP_KERNEL);
	ssi = devm_kzalloc(dev, sizeof(*ssi), GFP_KERNEL);
	if (!bbd || !ssi)
		return -ENOMEM;
	ssi->tx = devm_kzalloc(dev, SSI_BUF_SIZE, GFP_KERNEL);
	ssi->rx = devm_kzalloc(dev, SSI_BUF_SIZE, GFP_KERNEL);
	if (!ssi->tx || !ssi->rx)
		return -ENOMEM;

	ssi->spi = spi;
	ssi->bbd = bbd;
	bbd->dev = dev;
	bbd->ssi = ssi;
	mutex_init(&ssi->xfer_lock);
	spin_lock_init(&ssi->fifo_lock);
	INIT_KFIFO(ssi->tx_fifo);
	INIT_KFIFO(ssi->rx_fifo);
	init_waitqueue_head(&ssi->tx_wait);
	INIT_WORK(&ssi->tx_work, ssi_tx_work);
	INIT_WORK(&ssi->rx_work, ssi_rx_work);

	/* Held off until lhd raises nSTANDBY and loads the patch. */
	ssi->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ssi->enable))
		return dev_err_probe(dev, PTR_ERR(ssi->enable), "enable gpio\n");
	ssi->mcu_req = devm_gpiod_get(dev, "mcu-req", GPIOD_OUT_LOW);
	if (IS_ERR(ssi->mcu_req))
		return dev_err_probe(dev, PTR_ERR(ssi->mcu_req), "mcu-req gpio\n");
	ssi->mcu_resp = devm_gpiod_get(dev, "mcu-resp", GPIOD_IN);
	if (IS_ERR(ssi->mcu_resp))
		return dev_err_probe(dev, PTR_ERR(ssi->mcu_resp), "mcu-resp gpio\n");
	ssi->host_req = devm_gpiod_get(dev, "host-req", GPIOD_IN);
	if (IS_ERR(ssi->host_req))
		return dev_err_probe(dev, PTR_ERR(ssi->host_req), "host-req gpio\n");
	if (spi->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi setup\n");

	spi_set_drvdata(spi, bbd);
	ret = bbd_core_init(bbd);
	if (ret)
		return dev_err_probe(dev, ret, "bridge init\n");
	ret = devm_add_action_or_reset(dev, ssi_core_exit, bbd);
	if (ret)
		return ret;

	/*
	 * Not freezable: the sensor hub tells the chip about suspend from its
	 * own suspend callback, and that must go out before the AP sleeps.
	 */
	ssi->rx_wq = alloc_ordered_workqueue("bcm4773-rx", WQ_HIGHPRI);
	if (!ssi->rx_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, ssi_destroy_wq, ssi->rx_wq);
	if (ret)
		return ret;
	ssi->tx_wq = alloc_ordered_workqueue("bcm4773-tx", WQ_HIGHPRI);
	if (!ssi->tx_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, ssi_destroy_wq, ssi->tx_wq);
	if (ret)
		return ret;

	device_init_wakeup(dev, true);
	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ssi_irq_thread,
					IRQF_ONESHOT, "bcm4773", ssi);
	if (ret)
		return dev_err_probe(dev, ret, "irq\n");

	/* The sensor hub node. */
	ret = devm_of_platform_populate(dev);
	if (ret)
		return dev_err_probe(dev, ret, "children\n");

	dev_info(dev, "BCM4773 bridge ready, waiting for lhd\n");
	return 0;
}

static int bbd_ssi_suspend(struct device *dev)
{
	struct bbd_device *bbd = dev_get_drvdata(dev);

	/* The sensor hub child suspended first: send what it queued. */
	flush_work(&bbd->ssi->tx_work);

	/* The hub keeps sampling in suspend; HOST_REQ wakes the AP. */
	if (device_may_wakeup(dev))
		enable_irq_wake(to_spi_device(dev)->irq);
	return 0;
}

static int bbd_ssi_resume(struct device *dev)
{
	struct bbd_device *bbd = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(to_spi_device(dev)->irq);
	queue_work(bbd->ssi->tx_wq, &bbd->ssi->tx_work);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bbd_ssi_pm, bbd_ssi_suspend, bbd_ssi_resume);

static const struct of_device_id bbd_ssi_of_match[] = {
	{ .compatible = "brcm,bcm4773" },
	{ }
};
MODULE_DEVICE_TABLE(of, bbd_ssi_of_match);

static const struct spi_device_id bbd_ssi_id[] = {
	{ "bcm4773" },
	{ }
};
MODULE_DEVICE_TABLE(spi, bbd_ssi_id);

static struct spi_driver bbd_ssi_driver = {
	.driver = {
		.name = "bcm4773",
		.of_match_table = bbd_ssi_of_match,
		.pm = pm_sleep_ptr(&bbd_ssi_pm),
		/* lhd holds the character devices; no manual unbind. */
		.suppress_bind_attrs = true,
	},
	.probe = bbd_ssi_probe,
	.id_table = bbd_ssi_id,
};
module_spi_driver(bbd_ssi_driver);

MODULE_DESCRIPTION("Broadcom BCM4773 GNSS and sensor hub bridge");
MODULE_LICENSE("GPL");
