// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCM4773 bridge driver (BBD): the userspace and in-kernel faces.
 *
 * The BCM4773 GNSS chip carries the sensor hub of the Galaxy S6. It has no
 * flash: Broadcom's lhd daemon powers it, loads its RAM patch and runs the
 * transport sync through the character devices below, then writes
 * "ESW:READY". From then on the sensor hub driver exchanges SSP messages
 * with the hub through bcm4773_send()/bcm4773_recv(), carried in-kernel as
 * transport RPCs, while lhd keeps the GNSS traffic.
 *
 * Userspace ABI kept from Broadcom's driver, because lhd is a binary:
 *   /dev/bbd_sensor, /dev/bbd_control, /dev/bbd_packet, /dev/bbd_reliable
 *   /sys/devices/platform/bbd.0/{BBD,ESW,DEV,debug,shmd,pl,buf,passthru}
 * Added: /sys/devices/platform/bbd.0/nstandby, the chip's power line, for
 * lhd.conf's GpioNStdbyPath.
 */

#include <linux/bcm4773.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "bbd_internal.h"
#include "transport/bbd_packet_layer_c.h"

struct bbd_device *gpbbd_dev;

static const char * const bbd_names[BBD_MINORS] = {
	[BBD_MINOR_SENSOR]	= "bbd_sensor",
	[BBD_MINOR_CONTROL]	= "bbd_control",
	[BBD_MINOR_PACKET]	= "bbd_packet",
	[BBD_MINOR_RELIABLE]	= "bbd_reliable",
};

static const struct class bbd_class = {
	.name = "bbd",
};

/* Read queues: BBD -> lhd */

static void bbd_queue_init(struct bbd_queue *q)
{
	spin_lock_init(&q->lock);
	INIT_LIST_HEAD(&q->items);
	init_waitqueue_head(&q->wait);
}

static void bbd_queue_flush(struct bbd_queue *q)
{
	struct bbd_qitem *item, *tmp;
	LIST_HEAD(list);

	spin_lock(&q->lock);
	list_splice_init(&q->items, &list);
	q->count = 0;
	spin_unlock(&q->lock);

	list_for_each_entry_safe(item, tmp, &list, node)
		kfree(item);
}

ssize_t bbd_on_read(int minor, const unsigned char *buf, size_t size)
{
	struct bbd_device *bbd = gpbbd_dev;
	struct bbd_qitem *item;
	struct bbd_queue *q;

	if (!bbd || minor < 0 || minor >= BBD_MINORS || !size)
		return -EINVAL;
	q = &bbd->q[minor];

	item = kmalloc(struct_size(item, data, size), GFP_KERNEL);
	if (!item)
		return -ENOMEM;
	item->len = size;
	memcpy(item->data, buf, size);

	spin_lock(&q->lock);
	if (q->count >= BBD_QUEUE_MAX) {
		q->dropped++;
		spin_unlock(&q->lock);
		kfree(item);
		dev_warn_ratelimited(bbd->dev, "%s queue full, dropped\n",
				     bbd_names[minor]);
		return -ENOSPC;
	}
	list_add_tail(&item->node, &q->items);
	q->count++;
	spin_unlock(&q->lock);

	wake_up_interruptible(&q->wait);
	return size;
}

/* One item per read(). An empty queue reads 0, as with Broadcom's driver. */
static ssize_t bbd_queue_read(struct bbd_queue *q, char __user *ubuf,
			      size_t size)
{
	struct bbd_qitem *item;
	ssize_t ret;

	spin_lock(&q->lock);
	item = list_first_entry_or_null(&q->items, struct bbd_qitem, node);
	if (!item) {
		spin_unlock(&q->lock);
		return 0;
	}
	if (item->len > size) {
		spin_unlock(&q->lock);
		return -EMSGSIZE;
	}
	list_del(&item->node);
	q->count--;
	spin_unlock(&q->lock);

	ret = copy_to_user(ubuf, item->data, item->len) ? -EFAULT : item->len;
	kfree(item);
	return ret;
}

/* Hub -> sensor hub client */

static void bbd_set_ready(struct bbd_device *bbd, bool ready)
{
	unsigned long flags;

	if (!ready) {
		spin_lock_irqsave(&bbd->rx_lock, flags);
		kfifo_reset(&bbd->rx_fifo);
		spin_unlock_irqrestore(&bbd->rx_lock, flags);
	}

	spin_lock_irqsave(&bbd->client_lock, flags);
	bbd->esw_ready = ready;
	if (bbd->client_ops && bbd->client_ops->ready)
		bbd->client_ops->ready(bbd->client_priv, ready);
	spin_unlock_irqrestore(&bbd->client_lock, flags);

	wake_up(&bbd->rx_wait);
}

void bbd_sensor_rx(struct bbd_device *bbd, const u8 *buf, size_t len)
{
	unsigned long flags;
	bool queued = false;

	spin_lock_irqsave(&bbd->rx_lock, flags);
	/* Never queue part of a chunk: that would split an SSP frame. */
	if (kfifo_avail(&bbd->rx_fifo) >= len) {
		kfifo_in(&bbd->rx_fifo, buf, len);
		queued = true;
	} else {
		bbd->rx_dropped += len;
	}
	spin_unlock_irqrestore(&bbd->rx_lock, flags);

	if (!queued) {
		dev_warn_ratelimited(bbd->dev, "sensor rx full, %zu dropped\n",
				     len);
		return;
	}

	spin_lock_irqsave(&bbd->client_lock, flags);
	if (bbd->client_ops && bbd->client_ops->rx)
		bbd->client_ops->rx(bbd->client_priv);
	spin_unlock_irqrestore(&bbd->client_lock, flags);

	wake_up(&bbd->rx_wait);
}

/* Control strings from lhd (bbd_control, ESW and DEV attributes). */
void bbd_control_str(struct bbd_device *bbd, const char *str)
{
	struct BbdEngine *eng = &bbd->bbd_engine;
	bool setup = false;

	dev_dbg(bbd->dev, "control: %s\n", str);

	if (strstr(str, ESW_CTRL_READY)) {
		dev_info(bbd->dev, "hub firmware ready\n");
		bbd_set_ready(bbd, true);
	} else if (strstr(str, ESW_CTRL_NOTREADY)) {
		dev_info(bbd->dev, "hub firmware not ready\n");
		bbd_set_ready(bbd, false);
	} else if (strstr(str, ESW_CTRL_CRASHED)) {
		dev_err(bbd->dev, "hub firmware crashed\n");
		bbd_set_ready(bbd, false);
	} else if (strstr(str, BBD_CTRL_DEBUG_ON)) {
		bbd->debug = true;
	} else if (strstr(str, BBD_CTRL_DEBUG_OFF)) {
		bbd->debug = false;
	} else if (strstr(str, SSI_DEBUG_ON)) {
		bbd_ssi_set_debug(bbd->ssi, true);
	} else if (strstr(str, SSI_DEBUG_OFF)) {
		bbd_ssi_set_debug(bbd->ssi, false);
	} else if (strstr(str, SSP_DEBUG_ON) || strstr(str, SSP_DEBUG_OFF)) {
		/* The sensor hub driver uses dynamic debug instead. */
	} else if (strstr(str, SSI_PATCH_BEGIN) || strstr(str, SSI_PATCH_END)) {
		/*
		 * lhd brackets the patch download with these. Stock only
		 * batched its SSI debug-port writes on them; the download
		 * itself runs over the transport like any other packet.
		 */
	} else if (strstr(str, BBD_CTRL_SHMD_ON)) {
		bbd->shmd = true;
		setup = true;
	} else if (strstr(str, BBD_CTRL_SHMD_OFF)) {
		bbd->shmd = false;
		setup = true;
	} else if (strstr(str, BBD_CTRL_SPI) || strstr(str, BBD_CTRL_TTY) ||
		   strstr(str, BBD_CTRL_SIO)) {
		/* One serial backend here: SSI over SPI. */
	} else if (eng->open &&
		   BbdBridge_SetControlMessage(&eng->bridge, (char *)str)) {
		/* PassThru, StartRemoteSync */
	} else {
		dev_dbg(bbd->dev, "unhandled control: %s\n", str);
	}

	if (setup)
		bbd_engine_setup(bbd);
}

/* Character devices */

static int bbd_open(struct inode *inode, struct file *filp)
{
	struct bbd_device *bbd = container_of(inode->i_cdev,
					      struct bbd_device, cdev);
	int minor = iminor(inode);

	if (minor >= BBD_MINORS)
		return -ENODEV;
	filp->private_data = bbd;

	if (minor == BBD_MINOR_PACKET)
		bbd_engine_open(bbd);
	return stream_open(inode, filp);
}

static int bbd_release(struct inode *inode, struct file *filp)
{
	struct bbd_device *bbd = filp->private_data;
	int minor = iminor(inode);

	if (minor == BBD_MINOR_PACKET) {
		/* lhd is gone: so is the transport, and with it the hub. */
		bbd_engine_close(bbd);
		if (bbd->esw_ready)
			bbd_set_ready(bbd, false);
	}
	bbd_queue_flush(&bbd->q[minor]);
	return 0;
}

static ssize_t bbd_read(struct file *filp, char __user *buf, size_t size,
			loff_t *ppos)
{
	struct bbd_device *bbd = filp->private_data;

	return bbd_queue_read(&bbd->q[iminor(file_inode(filp))], buf, size);
}

static __poll_t bbd_poll(struct file *filp, poll_table *wait)
{
	struct bbd_device *bbd = filp->private_data;
	struct bbd_queue *q = &bbd->q[iminor(file_inode(filp))];
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(filp, &q->wait, wait);
	spin_lock(&q->lock);
	if (q->count)
		mask |= EPOLLIN | EPOLLRDNORM;
	spin_unlock(&q->lock);
	return mask;
}

static ssize_t bbd_write_reliable(struct bbd_device *bbd,
				  const char __user *buf, size_t size)
{
	struct sBbdReliableTransaction trans;

	if (size != sizeof(trans))
		return -EINVAL;
	if (copy_from_user(&trans, buf, size))
		return -EFAULT;
	/* The transport copies trans.pucData from userspace itself. */
	if (!BbdBridge_SendReliablePacket(&bbd->bbd_engine.bridge, &trans))
		return -EINVAL;
	return size;
}

static ssize_t bbd_write(struct file *filp, const char __user *buf,
			 size_t size, loff_t *ppos)
{
	struct bbd_device *bbd = filp->private_data;
	int minor = iminor(file_inode(filp));
	char *kbuf;

	if (minor == BBD_MINOR_RELIABLE)
		return bbd_write_reliable(bbd, buf, size);

	if (!size || size > BBD_MAX_DATA_SIZE)
		return -EINVAL;
	/* NUL-terminated for the control parser. */
	kbuf = memdup_user_nul(buf, size);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	switch (minor) {
	case BBD_MINOR_SENSOR:
		bbd_sensor_rx(bbd, kbuf, size);
		break;
	case BBD_MINOR_CONTROL:
		bbd_control_str(bbd, kbuf);
		break;
	case BBD_MINOR_PACKET:
		BbdBridge_SendPacket(&bbd->bbd_engine.bridge, kbuf, size);
		break;
	}
	kfree(kbuf);
	return size;
}

static const struct file_operations bbd_fops = {
	.owner		= THIS_MODULE,
	.open		= bbd_open,
	.release	= bbd_release,
	.read		= bbd_read,
	.write		= bbd_write,
	.poll		= bbd_poll,
	.llseek		= no_llseek,
};

/* bbd.0 attributes */

static ssize_t BBD_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	ssize_t ret = bbd_on_read(BBD_MINOR_CONTROL, buf, len);

	return ret < 0 ? ret : len;
}
/* Modes as Broadcom's driver: init hands the group to lhd. */
static DEVICE_ATTR(BBD, 0220, NULL, BBD_store);

static ssize_t ESW_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	char str[BBD_ESW_CTRL_MAX_STR_LEN];

	strscpy(str, buf, sizeof(str));
	bbd_control_str(gpbbd_dev, str);
	return len;
}
static DEVICE_ATTR(ESW, 0220, NULL, ESW_store);

/* Broadcom's driver also had DEV: the same as ESW. */
static struct device_attribute dev_attr_DEV = __ATTR(DEV, 0220, NULL,
						     ESW_store);

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sysfs_emit(buf, "%d\n", gpbbd_dev->debug);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t len)
{
	bool on;
	int ret = kstrtobool(buf, &on);

	if (ret)
		return ret;
	gpbbd_dev->debug = on;
	return len;
}
static DEVICE_ATTR(debug, 0660, debug_show, debug_store);

static ssize_t shmd_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct bbd_device *bbd = gpbbd_dev;

	return sysfs_emit(buf, "shmd=%d ready=%d rx=%u dropped=%lu\n",
			  bbd->shmd, bbd->esw_ready,
			  kfifo_len(&bbd->rx_fifo), bbd->rx_dropped);
}
static DEVICE_ATTR_RO(shmd);

static ssize_t pl_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct BbdEngine *eng = &gpbbd_dev->bbd_engine;
	struct stTransportLayerStats st = { };
	bool open;

	mutex_lock(&eng->lock);
	open = eng->open;
	if (open)
		st = eng->bridge.m_otTL.m_otCurrentStats;
	mutex_unlock(&eng->lock);

	if (!open)
		return sysfs_emit(buf, "closed\n");
	return sysfs_emit(buf,
		"tx=%lu rx=%lu ack=%lu rel_tx=%lu rel_retx=%lu rel_rx=%lu lost=%lu garbage=%lu remote_lost=%lu remote_garbage=%lu\n",
		st.ulPacketSent, st.ulPacketReceived, st.ulAckReceived,
		st.ulReliablePacketSent, st.ulReliableRetransmit,
		st.ulReliablePacketReceived, st.ulRxPacketLost,
		st.ulRxGarbageBytes, st.ulRemotePacketLost,
		st.ulRemoteGarbage);
}
static DEVICE_ATTR_RO(pl);

static ssize_t buf_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct bbd_device *bbd = gpbbd_dev;
	int len = 0, i;

	for (i = 0; i < BBD_MINORS; i++)
		len += sysfs_emit_at(buf, len, "%s: queued=%u dropped=%lu\n",
				     bbd_names[i], bbd->q[i].count,
				     bbd->q[i].dropped);
	return len;
}
static DEVICE_ATTR_RO(buf);

static ssize_t passthru_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct BbdEngine *eng = &gpbbd_dev->bbd_engine;
	bool on;

	mutex_lock(&eng->lock);
	on = eng->open && BbdTransportLayer_IsPassThrough(&eng->bridge.m_otTL);
	mutex_unlock(&eng->lock);
	return sysfs_emit(buf, "%d\n", on);
}
static DEVICE_ATTR_RO(passthru);

static ssize_t nstandby_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int on = bbd_ssi_get_power(gpbbd_dev->ssi);

	return on < 0 ? on : sysfs_emit(buf, "%d\n", on);
}

static ssize_t nstandby_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	bool on;
	int ret = kstrtobool(buf, &on);

	if (!ret)
		ret = bbd_ssi_set_power(gpbbd_dev->ssi, on);
	return ret ? ret : len;
}
static DEVICE_ATTR(nstandby, 0660, nstandby_show, nstandby_store);

static struct attribute *bbd_attrs[] = {
	&dev_attr_BBD.attr,
	&dev_attr_ESW.attr,
	&dev_attr_DEV.attr,
	&dev_attr_debug.attr,
	&dev_attr_shmd.attr,
	&dev_attr_pl.attr,
	&dev_attr_buf.attr,
	&dev_attr_passthru.attr,
	&dev_attr_nstandby.attr,
	NULL
};
ATTRIBUTE_GROUPS(bbd);

/* Sensor hub client API */

static struct bbd_device *hub_to_bbd(struct device *hub)
{
	struct bbd_device *bbd = hub ? dev_get_drvdata(hub) : NULL;

	return bbd == gpbbd_dev ? bbd : NULL;
}

int bcm4773_register_client(struct device *hub,
			    const struct bcm4773_client_ops *ops, void *priv)
{
	struct bbd_device *bbd = hub_to_bbd(hub);
	unsigned long flags;
	int ret = 0;

	if (!bbd)
		return -EPROBE_DEFER;

	spin_lock_irqsave(&bbd->client_lock, flags);
	if (bbd->client_ops) {
		ret = -EBUSY;
	} else {
		bbd->client_ops = ops;
		bbd->client_priv = priv;
		if (bbd->esw_ready && ops->ready)
			ops->ready(priv, true);
	}
	spin_unlock_irqrestore(&bbd->client_lock, flags);
	return ret;
}
EXPORT_SYMBOL_NS_GPL(bcm4773_register_client, BCM4773);

void bcm4773_unregister_client(struct device *hub)
{
	struct bbd_device *bbd = hub_to_bbd(hub);
	unsigned long flags;

	if (!bbd)
		return;
	spin_lock_irqsave(&bbd->client_lock, flags);
	bbd->client_ops = NULL;
	bbd->client_priv = NULL;
	spin_unlock_irqrestore(&bbd->client_lock, flags);
	wake_up(&bbd->rx_wait);
}
EXPORT_SYMBOL_NS_GPL(bcm4773_unregister_client, BCM4773);

bool bcm4773_ready(struct device *hub)
{
	struct bbd_device *bbd = hub_to_bbd(hub);

	return bbd && READ_ONCE(bbd->esw_ready);
}
EXPORT_SYMBOL_NS_GPL(bcm4773_ready, BCM4773);

int bcm4773_send(struct device *hub, const void *buf, size_t len)
{
	struct bbd_device *bbd = hub_to_bbd(hub);
	ssize_t ret;

	if (!bbd)
		return -ENODEV;
	if (!READ_ONCE(bbd->esw_ready))
		return -ENOTCONN;

	if (READ_ONCE(bbd->shmd))
		return bbd_engine_send_sensor(bbd, buf, len);

	/* SHMD off: lhd carries the sensor stream through /dev/bbd_sensor. */
	ret = bbd_on_read(BBD_MINOR_SENSOR, buf, len);
	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_NS_GPL(bcm4773_send, BCM4773);

static bool bbd_rx_avail(struct bbd_device *bbd, size_t len)
{
	return kfifo_len(&bbd->rx_fifo) >= len || !READ_ONCE(bbd->esw_ready);
}

ssize_t bcm4773_recv(struct device *hub, void *buf, size_t len,
		     unsigned int timeout_ms)
{
	struct bbd_device *bbd = hub_to_bbd(hub);
	unsigned long flags;
	ssize_t ret;

	if (!bbd)
		return -ENODEV;
	if (len > BBD_SENSOR_RX_SIZE)
		return -EINVAL;

	if (!wait_event_timeout(bbd->rx_wait, bbd_rx_avail(bbd, len),
				msecs_to_jiffies(timeout_ms)))
		return -ETIMEDOUT;

	spin_lock_irqsave(&bbd->rx_lock, flags);
	if (!bbd->esw_ready)
		ret = -ENOTCONN;
	else if (kfifo_len(&bbd->rx_fifo) < len)
		ret = -EAGAIN;
	else
		ret = kfifo_out(&bbd->rx_fifo, buf, len);
	spin_unlock_irqrestore(&bbd->rx_lock, flags);
	return ret;
}
EXPORT_SYMBOL_NS_GPL(bcm4773_recv, BCM4773);

size_t bcm4773_rx_avail(struct device *hub)
{
	struct bbd_device *bbd = hub_to_bbd(hub);

	return bbd ? kfifo_len(&bbd->rx_fifo) : 0;
}
EXPORT_SYMBOL_NS_GPL(bcm4773_rx_avail, BCM4773);

void bcm4773_flush(struct device *hub)
{
	struct bbd_device *bbd = hub_to_bbd(hub);
	unsigned long flags;

	if (!bbd)
		return;
	spin_lock_irqsave(&bbd->rx_lock, flags);
	kfifo_reset(&bbd->rx_fifo);
	spin_unlock_irqrestore(&bbd->rx_lock, flags);
}
EXPORT_SYMBOL_NS_GPL(bcm4773_flush, BCM4773);

int bcm4773_request_reset(struct device *hub)
{
	ssize_t ret;

	if (!hub_to_bbd(hub))
		return -ENODEV;
	/* lhd answers by cycling nSTANDBY and reloading the patch. */
	ret = bbd_on_read(BBD_MINOR_CONTROL, BBD_CTRL_RESET_REQ,
			  sizeof(BBD_CTRL_RESET_REQ));
	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_NS_GPL(bcm4773_request_reset, BCM4773);

/* Setup and teardown, called by the SSI driver's probe and remove. */

int bbd_core_init(struct bbd_device *bbd)
{
	struct platform_device *pdev;
	int i, ret;

	if (gpbbd_dev)
		return -EBUSY;

	for (i = 0; i < BBD_MINORS; i++)
		bbd_queue_init(&bbd->q[i]);
	spin_lock_init(&bbd->rx_lock);
	INIT_KFIFO(bbd->rx_fifo);
	init_waitqueue_head(&bbd->rx_wait);
	spin_lock_init(&bbd->client_lock);
	bbd->shmd = true;
	bbd_engine_init(bbd);
	gpbbd_dev = bbd;

	ret = class_register(&bbd_class);
	if (ret)
		goto err_gp;

	ret = alloc_chrdev_region(&bbd->devt, 0, BBD_MINORS, "bbd");
	if (ret)
		goto err_class;

	cdev_init(&bbd->cdev, &bbd_fops);
	ret = cdev_add(&bbd->cdev, bbd->devt, BBD_MINORS);
	if (ret)
		goto err_region;

	for (i = 0; i < BBD_MINORS; i++) {
		bbd->chr[i] = device_create(&bbd_class, bbd->dev,
					    MKDEV(MAJOR(bbd->devt), i), bbd,
					    "%s", bbd_names[i]);
		if (IS_ERR(bbd->chr[i])) {
			ret = PTR_ERR(bbd->chr[i]);
			goto err_chr;
		}
	}

	/* lhd and its init script expect /sys/devices/platform/bbd.0. */
	pdev = platform_device_alloc("bbd", 0);
	if (!pdev) {
		ret = -ENOMEM;
		goto err_chr;
	}
	pdev->dev.groups = bbd_groups;
	ret = platform_device_add(pdev);
	if (ret) {
		platform_device_put(pdev);
		goto err_chr;
	}
	bbd->pdev = pdev;
	return 0;

err_chr:
	while (--i >= 0)
		device_destroy(&bbd_class, MKDEV(MAJOR(bbd->devt), i));
	cdev_del(&bbd->cdev);
err_region:
	unregister_chrdev_region(bbd->devt, BBD_MINORS);
err_class:
	class_unregister(&bbd_class);
err_gp:
	gpbbd_dev = NULL;
	return ret;
}

void bbd_core_exit(struct bbd_device *bbd)
{
	int i;

	platform_device_unregister(bbd->pdev);
	for (i = 0; i < BBD_MINORS; i++)
		device_destroy(&bbd_class, MKDEV(MAJOR(bbd->devt), i));
	cdev_del(&bbd->cdev);
	unregister_chrdev_region(bbd->devt, BBD_MINORS);
	class_unregister(&bbd_class);
	bbd_engine_exit(bbd);
	for (i = 0; i < BBD_MINORS; i++)
		bbd_queue_flush(&bbd->q[i]);
	gpbbd_dev = NULL;
}
