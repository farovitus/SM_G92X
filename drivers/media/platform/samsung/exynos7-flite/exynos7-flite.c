// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos7420 FIMC-LITE (FIMC-BNS) raw capture driver
 *
 * The FIMC-LITE block takes the pixel stream of a MIPI CSIS receiver and
 * writes it to memory through its own DMA engine. On Exynos7420 the block
 * also carries a binning scaler and a direct path to the 3AA ISP; neither
 * is used here, the driver only captures raw Bayer frames to memory.
 *
 * The output DMA has up to 32 address slots enabled by a bit mask. The
 * driver uses two of them: at each frame start the hardware reports the
 * slot it writes to, at frame end that slot's buffer is complete and the
 * slot is refilled from the queue, or disabled when the queue is empty.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#define FLITE_DRV_NAME			"exynos7-flite"

#define FLITE_REG_CISRCSIZE		0x00
#define FLITE_REG_CIGCTRL		0x04
#define FLITE_CIGCTRL_FMT_SHIFT		24
#define FLITE_CIGCTRL_OLOCAL_DISABLE	BIT(22)
#define FLITE_CIGCTRL_ODMA_DISABLE	BIT(20)
#define FLITE_CIGCTRL_SWRST_REQ		BIT(19)
#define FLITE_CIGCTRL_SWRST_RDY		BIT(18)
#define FLITE_CIGCTRL_SWRST		BIT(17)
#define FLITE_CIGCTRL_IRQ_LASTEN_DIS	BIT(8)
#define FLITE_CIGCTRL_IRQ_ENDEN_DIS	BIT(7)
#define FLITE_CIGCTRL_IRQ_STARTEN_DIS	BIT(6)
#define FLITE_CIGCTRL_IRQ_OVFEN_DIS	BIT(5)
#define FLITE_CIGCTRL_IRQ_ALL_DIS	(FLITE_CIGCTRL_IRQ_LASTEN_DIS | \
					 FLITE_CIGCTRL_IRQ_ENDEN_DIS | \
					 FLITE_CIGCTRL_IRQ_STARTEN_DIS | \
					 FLITE_CIGCTRL_IRQ_OVFEN_DIS)
#define FLITE_CIGCTRL_SELCAM_MIPI	BIT(3)
#define FLITE_REG_CIIMGCPT		0x08
#define FLITE_CIIMGCPT_IMGCPTEN		BIT(31)
#define FLITE_REG_CIWDOFST		0x10
#define FLITE_CIWDOFST_WINOFSEN		BIT(31)
#define FLITE_CIWDOFST_CLROVIY		BIT(30)
#define FLITE_CIWDOFST_CLROVFICB	BIT(15)
#define FLITE_CIWDOFST_CLROVFICR	BIT(14)
#define FLITE_REG_CIWDOFST2		0x14
#define FLITE_REG_CIODMAFMT		0x18
#define FLITE_REG_CIOCAN		0x20
#define FLITE_REG_CIOOFF		0x24
#define FLITE_REG_CISTATUS		0x40
#define FLITE_CISTATUS_OVFIY		BIT(10)
#define FLITE_CISTATUS_OVFICB		BIT(9)
#define FLITE_CISTATUS_OVFICR		BIT(8)
#define FLITE_CISTATUS_OVF_ALL		(FLITE_CISTATUS_OVFIY | \
					 FLITE_CISTATUS_OVFICB | \
					 FLITE_CISTATUS_OVFICR)
#define FLITE_CISTATUS_IRQ_LASTCAPEND	BIT(6)
#define FLITE_CISTATUS_IRQ_FRMSTART	BIT(5)
#define FLITE_CISTATUS_IRQ_FRMEND	BIT(4)
#define FLITE_REG_CISTATUS2		0x44
#define FLITE_CISTATUS2_LASTCAPEND	BIT(1)
#define FLITE_REG_CISTATUS3		0x48
#define FLITE_CISTATUS3_PRESENT_MASK	0x3f
#define FLITE_REG_CIFCNTSEQ		0x100

#define FLITE_NUM_SLOTS_HW		32
#define FLITE_NUM_SLOTS			2
#define FLITE_HW_FMT_RAW8		0x2a
#define FLITE_HW_FMT_RAW10		0x2b

#define FLITE_MIN_WIDTH			16
#define FLITE_MIN_HEIGHT		8
#define FLITE_MAX_SIZE			8192
#define FLITE_STOP_TIMEOUT_MS		200

static inline u32 flite_slot_reg(unsigned int slot)
{
	return slot ? 0x200 + 4 * (slot - 1) : 0x30;
}

struct flite_fmt {
	u32 code;
	u32 fourcc;
	u8 hw_fmt;
	u8 bpp;		/* bytes per pixel in memory */
};

/*
 * RAW10 goes to memory unpacked, one 16-bit word per pixel; the block's
 * PACK12 mode is not used.
 */
static const struct flite_fmt flite_formats[] = {
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10, FLITE_HW_FMT_RAW10, 2 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10, FLITE_HW_FMT_RAW10, 2 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10, FLITE_HW_FMT_RAW10, 2 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10, FLITE_HW_FMT_RAW10, 2 },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_PIX_FMT_SBGGR8, FLITE_HW_FMT_RAW8, 1 },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_PIX_FMT_SGBRG8, FLITE_HW_FMT_RAW8, 1 },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_PIX_FMT_SGRBG8, FLITE_HW_FMT_RAW8, 1 },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_PIX_FMT_SRGGB8, FLITE_HW_FMT_RAW8, 1 },
};

struct flite_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct flite {
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data *clks;
	int num_clks;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *src_sd;
	struct video_device vdev;
	struct media_pad vdev_pad;
	struct media_pipeline pipe;
	struct vb2_queue queue;
	struct mutex lock;		/* video device and queue */

	struct v4l2_pix_format pix;
	const struct flite_fmt *fmt;

	spinlock_t slock;		/* everything below */
	struct list_head pending;
	struct flite_buffer *slot[FLITE_NUM_SLOTS];
	int writing;			/* slot of the frame in flight, or -1 */
	bool odma_on;
	bool stopped;
	u32 sequence;
	u32 frames;
	u32 drops;
	u32 overflows;
	wait_queue_head_t stop_wq;
};

static const struct flite_fmt *flite_find_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(flite_formats); i++)
		if (flite_formats[i].fourcc == fourcc)
			return &flite_formats[i];
	return NULL;
}

static inline u32 flite_read(struct flite *fl, u32 reg)
{
	return readl(fl->regs + reg);
}

static inline void flite_write(struct flite *fl, u32 reg, u32 val)
{
	writel(val, fl->regs + reg);
}

static void flite_update(struct flite *fl, u32 reg, u32 mask, u32 val)
{
	flite_write(fl, reg, (flite_read(fl, reg) & ~mask) | val);
}

/* -------------------------------------------------------------------------
 * Hardware
 */

static void flite_hw_reset(struct flite *fl)
{
	u32 cfg;
	int ret;

	flite_update(fl, FLITE_REG_CIGCTRL, 0, FLITE_CIGCTRL_SWRST_REQ);
	ret = readl_poll_timeout_atomic(fl->regs + FLITE_REG_CIGCTRL, cfg,
					cfg & FLITE_CIGCTRL_SWRST_RDY, 1, 1000);
	if (ret)
		dev_warn(fl->dev, "reset not ready, forcing it\n");
	flite_update(fl, FLITE_REG_CIGCTRL, 0, FLITE_CIGCTRL_SWRST);
}

static void flite_hw_init_slots(struct flite *fl)
{
	unsigned int i;

	flite_write(fl, FLITE_REG_CIFCNTSEQ, 0);
	for (i = 0; i < FLITE_NUM_SLOTS_HW; i++)
		flite_write(fl, flite_slot_reg(i), 0xffffffff);
}

static void flite_hw_set_odma(struct flite *fl, bool on)
{
	flite_update(fl, FLITE_REG_CIGCTRL, FLITE_CIGCTRL_ODMA_DISABLE,
		     on ? 0 : FLITE_CIGCTRL_ODMA_DISABLE);
	fl->odma_on = on;
}

/* Called with slock held. */
static void flite_arm_slot(struct flite *fl, unsigned int slot,
			   struct flite_buffer *buf)
{
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);

	fl->slot[slot] = buf;
	flite_write(fl, flite_slot_reg(slot), lower_32_bits(addr));
	flite_update(fl, FLITE_REG_CIFCNTSEQ, 0, BIT(slot));
	if (!fl->odma_on)
		flite_hw_set_odma(fl, true);
}

/* Called with slock held. */
static void flite_disarm_slot(struct flite *fl, unsigned int slot)
{
	fl->slot[slot] = NULL;
	flite_update(fl, FLITE_REG_CIFCNTSEQ, BIT(slot), 0);
	if (!(flite_read(fl, FLITE_REG_CIFCNTSEQ) & GENMASK(FLITE_NUM_SLOTS - 1, 0)))
		flite_hw_set_odma(fl, false);
}

/* Called with slock held. */
static void flite_refill_slot(struct flite *fl, unsigned int slot)
{
	struct flite_buffer *buf;

	buf = list_first_entry_or_null(&fl->pending, struct flite_buffer, list);
	if (buf) {
		list_del(&buf->list);
		flite_arm_slot(fl, slot, buf);
	} else {
		flite_disarm_slot(fl, slot);
	}
}

static void flite_hw_start(struct flite *fl)
{
	const struct v4l2_pix_format *pix = &fl->pix;
	u32 cfg;

	flite_write(fl, FLITE_REG_CISRCSIZE, pix->width << 16 | pix->height);
	flite_write(fl, FLITE_REG_CIOCAN, pix->height << 16 | pix->bytesperline);
	flite_write(fl, FLITE_REG_CIOOFF, 0);
	/* 2D DMA, no 12-bit packing */
	flite_write(fl, FLITE_REG_CIODMAFMT, 0);
	flite_write(fl, FLITE_REG_CIWDOFST, FLITE_CIWDOFST_WINOFSEN);
	flite_write(fl, FLITE_REG_CIWDOFST2, 0);

	/*
	 * MIPI input, DMA output only, normal polarities, all four
	 * interrupts enabled. The ODMA bit was set by flite_arm_slot().
	 */
	cfg = fl->fmt->hw_fmt << FLITE_CIGCTRL_FMT_SHIFT |
	      FLITE_CIGCTRL_OLOCAL_DISABLE | FLITE_CIGCTRL_SELCAM_MIPI;
	if (!fl->odma_on)
		cfg |= FLITE_CIGCTRL_ODMA_DISABLE;
	flite_write(fl, FLITE_REG_CIGCTRL, cfg);

	flite_update(fl, FLITE_REG_CISTATUS2, FLITE_CISTATUS2_LASTCAPEND, 0);
	flite_update(fl, FLITE_REG_CIIMGCPT, 0, FLITE_CIIMGCPT_IMGCPTEN);
}

static void flite_hw_stop(struct flite *fl)
{
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&fl->slock, flags);
	fl->stopped = false;
	flite_update(fl, FLITE_REG_CIIMGCPT, FLITE_CIIMGCPT_IMGCPTEN, 0);
	spin_unlock_irqrestore(&fl->slock, flags);

	idle = wait_event_timeout(fl->stop_wq, READ_ONCE(fl->stopped),
				  msecs_to_jiffies(FLITE_STOP_TIMEOUT_MS));
	if (!idle)
		dev_warn(fl->dev, "no last-capture interrupt, resetting\n");

	spin_lock_irqsave(&fl->slock, flags);
	flite_write(fl, FLITE_REG_CIFCNTSEQ, 0);
	flite_hw_set_odma(fl, false);
	flite_update(fl, FLITE_REG_CIGCTRL, 0, FLITE_CIGCTRL_IRQ_ALL_DIS);
	spin_unlock_irqrestore(&fl->slock, flags);

	flite_hw_reset(fl);
}

static void flite_frame_start(struct flite *fl)
{
	u32 present;

	fl->frames++;
	present = flite_read(fl, FLITE_REG_CISTATUS3) &
		  FLITE_CISTATUS3_PRESENT_MASK;

	if (present >= 1 && present <= FLITE_NUM_SLOTS && fl->odma_on &&
	    fl->slot[present - 1]) {
		fl->writing = present - 1;
	} else {
		fl->writing = -1;
		fl->drops++;
	}
}

static void flite_frame_end(struct flite *fl)
{
	struct flite_buffer *buf;
	int slot = fl->writing;

	fl->writing = -1;
	if (slot < 0 || !fl->slot[slot])
		return;

	buf = fl->slot[slot];
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = fl->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	flite_refill_slot(fl, slot);
}

static irqreturn_t flite_irq(int irq, void *priv)
{
	struct flite *fl = priv;
	u32 status;

	spin_lock(&fl->slock);

	status = flite_read(fl, FLITE_REG_CISTATUS);
	flite_write(fl, FLITE_REG_CISTATUS, status);

	/* Both bits set: the end of one frame and the start of the next. */
	if (status & FLITE_CISTATUS_IRQ_FRMEND)
		flite_frame_end(fl);
	if (status & FLITE_CISTATUS_IRQ_FRMSTART)
		flite_frame_start(fl);

	if (status & FLITE_CISTATUS_IRQ_LASTCAPEND) {
		flite_update(fl, FLITE_REG_CISTATUS2,
			     FLITE_CISTATUS2_LASTCAPEND, 0);
		WRITE_ONCE(fl->stopped, true);
		wake_up(&fl->stop_wq);
	}

	if (status & FLITE_CISTATUS_OVF_ALL) {
		u32 clr = 0;

		if (status & FLITE_CISTATUS_OVFIY)
			clr |= FLITE_CIWDOFST_CLROVIY;
		if (status & FLITE_CISTATUS_OVFICB)
			clr |= FLITE_CIWDOFST_CLROVFICB;
		if (status & FLITE_CISTATUS_OVFICR)
			clr |= FLITE_CIWDOFST_CLROVFICR;
		flite_update(fl, FLITE_REG_CIWDOFST, 0, clr);
		flite_update(fl, FLITE_REG_CIWDOFST, clr, 0);
		if (!fl->overflows++)
			dev_err(fl->dev, "FIFO overflow, status 0x%08x\n",
				status);
		/* Stop an interrupt storm; the counter keeps the rest. */
		flite_update(fl, FLITE_REG_CIGCTRL, 0,
			     FLITE_CIGCTRL_IRQ_OVFEN_DIS);
	}

	spin_unlock(&fl->slock);

	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------
 * vb2
 */

static int flite_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct flite *fl = vb2_get_drv_priv(vq);
	unsigned int size = fl->pix.sizeimage;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int flite_buf_prepare(struct vb2_buffer *vb)
{
	struct flite *fl = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < fl->pix.sizeimage)
		return -EINVAL;
	/* The slot registers are 32 bits wide. */
	if (upper_32_bits(vb2_dma_contig_plane_dma_addr(vb, 0)))
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, fl->pix.sizeimage);
	return 0;
}

static void flite_buf_queue(struct vb2_buffer *vb)
{
	struct flite *fl = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct flite_buffer *buf = container_of(vbuf, struct flite_buffer, vb);
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&fl->slock, flags);
	if (vb2_start_streaming_called(vb->vb2_queue)) {
		for (i = 0; i < FLITE_NUM_SLOTS; i++) {
			if (!fl->slot[i]) {
				flite_arm_slot(fl, i, buf);
				goto out;
			}
		}
	}
	list_add_tail(&buf->list, &fl->pending);
out:
	spin_unlock_irqrestore(&fl->slock, flags);
}

static void flite_return_buffers(struct flite *fl, enum vb2_buffer_state state)
{
	struct flite_buffer *buf, *tmp;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&fl->slock, flags);
	for (i = 0; i < FLITE_NUM_SLOTS; i++) {
		if (fl->slot[i]) {
			vb2_buffer_done(&fl->slot[i]->vb.vb2_buf, state);
			fl->slot[i] = NULL;
		}
	}
	list_for_each_entry_safe(buf, tmp, &fl->pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&fl->slock, flags);
}

static int flite_validate_source(struct flite *fl)
{
	struct v4l2_subdev_format sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct media_pad *pad;
	int ret;

	pad = media_pad_remote_pad_first(&fl->vdev_pad);
	if (!pad || !fl->src_sd)
		return -ENOLINK;

	sd_fmt.pad = pad->index;
	ret = v4l2_subdev_call_state_active(fl->src_sd, pad, get_fmt, &sd_fmt);
	if (ret)
		return ret;

	if (sd_fmt.format.width != fl->pix.width ||
	    sd_fmt.format.height != fl->pix.height ||
	    sd_fmt.format.code != fl->fmt->code) {
		dev_err(fl->dev, "source %ux%u 0x%04x, capture %ux%u 0x%04x\n",
			sd_fmt.format.width, sd_fmt.format.height,
			sd_fmt.format.code, fl->pix.width, fl->pix.height,
			fl->fmt->code);
		return -EPIPE;
	}

	return 0;
}

static int flite_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct flite *fl = vb2_get_drv_priv(vq);
	unsigned long flags;
	unsigned int i;
	int ret;

	ret = flite_validate_source(fl);
	if (ret)
		goto err_return;

	ret = pm_runtime_resume_and_get(fl->dev);
	if (ret)
		goto err_return;

	ret = video_device_pipeline_start(&fl->vdev, &fl->pipe);
	if (ret)
		goto err_pm;

	flite_hw_reset(fl);

	spin_lock_irqsave(&fl->slock, flags);
	flite_hw_init_slots(fl);
	fl->odma_on = false;
	fl->writing = -1;
	fl->sequence = 0;
	fl->frames = 0;
	fl->drops = 0;
	fl->overflows = 0;
	for (i = 0; i < FLITE_NUM_SLOTS; i++)
		flite_refill_slot(fl, i);
	flite_hw_start(fl);
	spin_unlock_irqrestore(&fl->slock, flags);

	ret = v4l2_subdev_call(fl->src_sd, video, s_stream, 1);
	if (ret) {
		dev_err(fl->dev, "source stream on failed: %d\n", ret);
		goto err_stop;
	}

	return 0;

err_stop:
	flite_hw_stop(fl);
	video_device_pipeline_stop(&fl->vdev);
err_pm:
	pm_runtime_put(fl->dev);
err_return:
	flite_return_buffers(fl, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void flite_stop_streaming(struct vb2_queue *vq)
{
	struct flite *fl = vb2_get_drv_priv(vq);

	flite_hw_stop(fl);
	v4l2_subdev_call(fl->src_sd, video, s_stream, 0);
	video_device_pipeline_stop(&fl->vdev);

	dev_info(fl->dev, "stopped: %u frames, %u completed, %u dropped, %u overflows\n",
		 fl->frames, fl->sequence, fl->drops, fl->overflows);

	pm_runtime_put(fl->dev);
	flite_return_buffers(fl, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops flite_vb2_ops = {
	.queue_setup = flite_queue_setup,
	.buf_prepare = flite_buf_prepare,
	.buf_queue = flite_buf_queue,
	.start_streaming = flite_start_streaming,
	.stop_streaming = flite_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* -------------------------------------------------------------------------
 * Video device
 */

static const struct flite_fmt *flite_try_fmt(struct v4l2_pix_format *pix)
{
	const struct flite_fmt *fmt = flite_find_fourcc(pix->pixelformat);

	if (!fmt)
		fmt = &flite_formats[0];

	pix->pixelformat = fmt->fourcc;
	pix->width = clamp(ALIGN(pix->width, 2), FLITE_MIN_WIDTH, FLITE_MAX_SIZE);
	pix->height = clamp(ALIGN(pix->height, 2), FLITE_MIN_HEIGHT,
			    FLITE_MAX_SIZE);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = ALIGN(pix->width * fmt->bpp, 16);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;

	return fmt;
}

static int flite_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, FLITE_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Exynos7420 FIMC-LITE", sizeof(cap->card));
	return 0;
}

static int flite_enum_fmt(struct file *file, void *priv,
			  struct v4l2_fmtdesc *f)
{
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(flite_formats); i++) {
		if (f->mbus_code && f->mbus_code != flite_formats[i].code)
			continue;
		if (n++ == f->index) {
			f->pixelformat = flite_formats[i].fourcc;
			return 0;
		}
	}
	return -EINVAL;
}

static int flite_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !flite_find_fourcc(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = FLITE_MIN_WIDTH;
	fsize->stepwise.max_width = FLITE_MAX_SIZE;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = FLITE_MIN_HEIGHT;
	fsize->stepwise.max_height = FLITE_MAX_SIZE;
	fsize->stepwise.step_height = 2;
	return 0;
}

static int flite_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct flite *fl = video_drvdata(file);

	f->fmt.pix = fl->pix;
	return 0;
}

static int flite_try_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	flite_try_fmt(&f->fmt.pix);
	return 0;
}

static int flite_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct flite *fl = video_drvdata(file);

	if (vb2_is_busy(&fl->queue))
		return -EBUSY;

	fl->fmt = flite_try_fmt(&f->fmt.pix);
	fl->pix = f->fmt.pix;
	return 0;
}

static const struct v4l2_ioctl_ops flite_ioctl_ops = {
	.vidioc_querycap = flite_querycap,
	.vidioc_enum_fmt_vid_cap = flite_enum_fmt,
	.vidioc_enum_framesizes = flite_enum_framesizes,
	.vidioc_g_fmt_vid_cap = flite_g_fmt,
	.vidioc_try_fmt_vid_cap = flite_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = flite_s_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations flite_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int flite_video_init(struct flite *fl)
{
	struct video_device *vdev = &fl->vdev;
	struct vb2_queue *q = &fl->queue;
	int ret;

	fl->pix.pixelformat = flite_formats[0].fourcc;
	fl->pix.width = 2608;
	fl->pix.height = 1960;
	fl->fmt = flite_try_fmt(&fl->pix);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = fl;
	q->buf_struct_size = sizeof(struct flite_buffer);
	q->ops = &flite_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &fl->lock;
	q->min_buffers_needed = FLITE_NUM_SLOTS;
	q->dev = fl->dev;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	fl->vdev_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &fl->vdev_pad);
	if (ret)
		return ret;

	strscpy(vdev->name, dev_name(fl->dev), sizeof(vdev->name));
	vdev->fops = &flite_fops;
	vdev->ioctl_ops = &flite_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->v4l2_dev = &fl->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->lock = &fl->lock;
	vdev->queue = q;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	video_set_drvdata(vdev, fl);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		media_entity_cleanup(&vdev->entity);
		return ret;
	}

	dev_info(fl->dev, "capture node /dev/%s\n", video_device_node_name(vdev));
	return 0;
}

/* -------------------------------------------------------------------------
 * Async binding to the CSIS receiver
 */

static inline struct flite *notifier_to_flite(struct v4l2_async_notifier *n)
{
	return container_of(n, struct flite, notifier);
}

static int flite_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *sd,
			      struct v4l2_async_connection *asc)
{
	struct flite *fl = notifier_to_flite(notifier);
	unsigned int pad;

	for (pad = 0; pad < sd->entity.num_pads; pad++)
		if (sd->entity.pads[pad].flags & MEDIA_PAD_FL_SOURCE)
			break;
	if (pad == sd->entity.num_pads)
		return -EINVAL;

	fl->src_sd = sd;
	return media_create_pad_link(&sd->entity, pad, &fl->vdev.entity, 0,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static void flite_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_connection *asc)
{
	struct flite *fl = notifier_to_flite(notifier);

	fl->src_sd = NULL;
}

static int flite_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct flite *fl = notifier_to_flite(notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&fl->v4l2_dev);
	if (ret)
		return ret;

	return media_device_register(&fl->mdev);
}

static const struct v4l2_async_notifier_operations flite_notify_ops = {
	.bound = flite_notify_bound,
	.unbind = flite_notify_unbind,
	.complete = flite_notify_complete,
};

static int flite_async_register(struct flite *fl)
{
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	int ret;

	v4l2_async_nf_init(&fl->notifier, &fl->v4l2_dev);

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(fl->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return dev_err_probe(fl->dev, -ENOTCONN, "no input endpoint\n");

	asc = v4l2_async_nf_add_fwnode_remote(&fl->notifier, ep,
					      struct v4l2_async_connection);
	fwnode_handle_put(ep);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto err_cleanup;
	}

	fl->notifier.ops = &flite_notify_ops;
	ret = v4l2_async_nf_register(&fl->notifier);
	if (ret)
		goto err_cleanup;

	return 0;

err_cleanup:
	v4l2_async_nf_cleanup(&fl->notifier);
	return ret;
}

static const struct media_device_ops flite_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

/* -------------------------------------------------------------------------
 * Probe
 */

static int flite_runtime_suspend(struct device *dev)
{
	struct flite *fl = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(fl->num_clks, fl->clks);
	return 0;
}

static int flite_runtime_resume(struct device *dev)
{
	struct flite *fl = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(fl->num_clks, fl->clks);
}

static int flite_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct flite *fl;
	int irq, ret;

	fl = devm_kzalloc(dev, sizeof(*fl), GFP_KERNEL);
	if (!fl)
		return -ENOMEM;

	fl->dev = dev;
	mutex_init(&fl->lock);
	spin_lock_init(&fl->slock);
	INIT_LIST_HEAD(&fl->pending);
	init_waitqueue_head(&fl->stop_wq);
	platform_set_drvdata(pdev, fl);

	fl->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fl->regs))
		return PTR_ERR(fl->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	fl->num_clks = devm_clk_bulk_get_all(dev, &fl->clks);
	if (fl->num_clks < 0)
		return dev_err_probe(dev, fl->num_clks, "clocks\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "memory-region\n");

	ret = devm_request_irq(dev, irq, flite_irq, 0, dev_name(dev), fl);
	if (ret)
		goto err_mem;

	pm_runtime_enable(dev);

	fl->mdev.dev = dev;
	fl->mdev.ops = &flite_media_ops;
	strscpy(fl->mdev.model, "exynos7-camera", sizeof(fl->mdev.model));
	media_device_init(&fl->mdev);

	fl->v4l2_dev.mdev = &fl->mdev;
	ret = v4l2_device_register(dev, &fl->v4l2_dev);
	if (ret)
		goto err_media;

	ret = flite_video_init(fl);
	if (ret)
		goto err_v4l2;

	ret = flite_async_register(fl);
	if (ret)
		goto err_video;

	return 0;

err_video:
	video_unregister_device(&fl->vdev);
	media_entity_cleanup(&fl->vdev.entity);
err_v4l2:
	v4l2_device_unregister(&fl->v4l2_dev);
err_media:
	media_device_cleanup(&fl->mdev);
	pm_runtime_disable(dev);
err_mem:
	of_reserved_mem_device_release(dev);
	return ret;
}

static void flite_remove(struct platform_device *pdev)
{
	struct flite *fl = platform_get_drvdata(pdev);

	media_device_unregister(&fl->mdev);
	v4l2_async_nf_unregister(&fl->notifier);
	v4l2_async_nf_cleanup(&fl->notifier);
	video_unregister_device(&fl->vdev);
	media_entity_cleanup(&fl->vdev.entity);
	v4l2_device_unregister(&fl->v4l2_dev);
	media_device_cleanup(&fl->mdev);
	pm_runtime_disable(&pdev->dev);
	of_reserved_mem_device_release(&pdev->dev);
}

static const struct dev_pm_ops flite_pm_ops = {
	RUNTIME_PM_OPS(flite_runtime_suspend, flite_runtime_resume, NULL)
};

static const struct of_device_id flite_of_match[] = {
	{ .compatible = "samsung,exynos7420-fimc-lite" },
	{ }
};
MODULE_DEVICE_TABLE(of, flite_of_match);

static struct platform_driver flite_driver = {
	.probe = flite_probe,
	.remove_new = flite_remove,
	.driver = {
		.name = FLITE_DRV_NAME,
		.of_match_table = flite_of_match,
		.pm = pm_ptr(&flite_pm_ops),
	},
};
module_platform_driver(flite_driver);

MODULE_DESCRIPTION("Exynos7420 FIMC-LITE raw capture driver");
MODULE_LICENSE("GPL");
