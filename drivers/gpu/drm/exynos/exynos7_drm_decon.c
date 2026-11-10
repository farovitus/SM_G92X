// SPDX-License-Identifier: GPL-2.0-or-later
/* drivers/gpu/drm/exynos/exynos7_drm_decon.c
 *
 * Copyright (C) 2014 Samsung Electronics Co.Ltd
 * Authors:
 *	Akshu Agarwal <akshua@gmail.com>
 *	Ajay Kumar <ajaykumar.rs@samsung.com>
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/interconnect.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_vblank.h>
#include <drm/exynos_drm.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"
#include "regs-decon7.h"

/*
 * DECON stands for Display and Enhancement controller.
 */

#define MIN_FB_WIDTH_FOR_16WORD_BURST 128

#define WINDOWS_NR	2

/* a command-mode panel holds its frame: release the bus after this */
#define DECON_BW_IDLE_MS	100

/*
 * Command mode with the panel's tearing-effect (TE) line: each frame is
 * sent at a TE edge, when the panel starts reading its frame memory, and
 * vblank counts TE edges, so vblank events and flip completions follow
 * the panel's own refresh. Without TE a frame is sent as soon as it is
 * committed: the transfer then starts at an arbitrary point of the
 * panel's refresh, and the time between frames on screen follows the
 * commit times. te_sync=0 keeps that behaviour; a change applies the next
 * time the display is turned on. te_active reports whether frames
 * currently go out at TE.
 */
static bool decon_te_enable = true;
module_param_named(te_sync, decon_te_enable, bool, 0644);
static bool decon_te_active;
module_param_named(te_active, decon_te_active, bool, 0444);

/*
 * Shadow-latch check: commits programmed while a transfer was running, and
 * frame-done interrupts that found such a commit already applied (its
 * DECON_UPDATE standalone bit cleared before its own trigger). The second
 * must stay 0: a nonzero value means the active registers changed under a
 * running transfer.
 */
static unsigned int decon_te_mid_xfer;
module_param_named(te_mid_xfer, decon_te_mid_xfer, uint, 0444);
static unsigned int decon_te_early_latch;
module_param_named(te_early_latch, decon_te_early_latch, uint, 0444);

/* The TE period in use for stamping flips ahead of TE, in ns. */
static unsigned int decon_te_period;
module_param_named(te_period_ns, decon_te_period, uint, 0444);

/* a pending frame waits at most this long for TE before it is sent anyway */
#define DECON_TE_TIMEOUT_MS	50
/* after this many such frames in a row, frames go out on commit again */
#define DECON_TE_MISS_MAX	3

/* Identity map size when S-Boot scans below the /chosen framebuffer */
#define BOOT_FB_FALLBACK_SIZE	SZ_64M

struct decon_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	void				*dma_priv;
	struct exynos_drm_crtc		*crtc;
	struct exynos_drm_plane		planes[WINDOWS_NR];
	struct exynos_drm_plane_config	configs[WINDOWS_NR];
	struct clk			*pclk;
	struct clk			*aclk;
	struct clk			*eclk;
	struct clk			*vclk;
	void __iomem			*regs;
	unsigned long			irq_flags;
	bool				i80_if;
	bool				bootloader_display;
	u32				sleep_trigcon;
	bool				sleep_stopped;
	wait_queue_head_t		wait_vsync_queue;
	atomic_t			wait_vsync_event;

	/* bootloader framebuffer from /chosen, kept in scanout at handover */
	phys_addr_t			boot_fb_base;
	size_t				boot_fb_size;
	bool				iommu_ready;

	struct drm_encoder *encoder;

	/* display bus bandwidth, kB/s, requested for the last frame */
	struct icc_path			*icc_path;
	u32				icc_bw;
	struct delayed_work		bw_idle_work;

	/* TE-synchronised frame transfers, under te_lock */
	int				te_irq;
	spinlock_t			te_lock;
	bool				te_sync;	/* TE seen: send frames at TE */
	bool				te_irq_on;
	bool				vblank_on;
	bool				trig_pending;	/* committed, waits for TE */
	bool				xfer_busy;	/* sending, until frame done */
	unsigned int			te_count;
	unsigned int			te_misses;	/* watchdog sends in a row */
	ktime_t				te_last;	/* last TE edge */
	u32				te_period_ns;	/* measured, 0 = not yet */
	/* i80: completes once DECON scans the committed state, under te_lock */
	struct drm_pending_vblank_event	*flip_event;
	wait_queue_head_t		te_wq;
	struct delayed_work		te_watchdog;
};

static const struct of_device_id decon_driver_dt_match[] = {
	{.compatible = "samsung,exynos7-decon"},
	{},
};
MODULE_DEVICE_TABLE(of, decon_driver_dt_match);

static const uint32_t decon_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
};

static const enum drm_plane_type decon_win_types[WINDOWS_NR] = {
	DRM_PLANE_TYPE_PRIMARY,
	DRM_PLANE_TYPE_CURSOR,
};

/**
 * decon_shadow_protect_win() - disable updating values from shadow registers at vsync
 *
 * @ctx: display and enhancement controller context
 * @win: window to protect registers for
 * @protect: 1 to protect (disable updates)
 */
static void decon_shadow_protect_win(struct decon_context *ctx,
				     unsigned int win, bool protect)
{
	u32 bits, val;

	bits = SHADOWCON_WINx_PROTECT(win);

	val = readl(ctx->regs + SHADOWCON);
	if (protect)
		val |= bits;
	else
		val &= ~bits;
	writel(val, ctx->regs + SHADOWCON);
}

static void decon_wait_for_vblank(struct decon_context *ctx)
{
	atomic_set(&ctx->wait_vsync_event, 1);

	/*
	 * wait for DECON to signal VSYNC interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(ctx->wait_vsync_queue,
				!atomic_read(&ctx->wait_vsync_event),
				HZ/20))
		DRM_DEV_DEBUG_KMS(ctx->dev, "vblank wait timed out.\n");
}

static void decon_clear_channels(struct decon_context *ctx)
{
	unsigned int win, ch_enabled = 0;
	u32 val;

	/* Check if any channel is enabled. */
	for (win = 0; win < WINDOWS_NR; win++) {
		val = readl(ctx->regs + WINCON(win));

		if (val & WINCONx_ENWIN) {
			decon_shadow_protect_win(ctx, win, true);

			val &= ~WINCONx_ENWIN;
			writel(val, ctx->regs + WINCON(win));
			ch_enabled = 1;

			decon_shadow_protect_win(ctx, win, false);
		}
	}

	val = readl(ctx->regs + DECON_UPDATE);
	val |= DECON_UPDATE_STANDALONE_F;
	writel(val, ctx->regs + DECON_UPDATE);

	/* Wait for vsync, as disable channel takes effect at next vsync */
	if (ch_enabled)
		decon_wait_for_vblank(ctx);
}

static int decon_ctx_initialize(struct decon_context *ctx,
			struct drm_device *drm_dev)
{
	ctx->drm_dev = drm_dev;

	decon_clear_channels(ctx);

	return exynos_drm_register_dma(drm_dev, ctx->dev, &ctx->dma_priv);
}

static void decon_ctx_remove(struct decon_context *ctx)
{
	/* detach this sub driver from iommu mapping if supported. */
	exynos_drm_unregister_dma(ctx->drm_dev, ctx->dev, &ctx->dma_priv);
}

static u32 decon_calc_clkdiv(struct decon_context *ctx,
		const struct drm_display_mode *mode)
{
	unsigned long ideal_clk = mode->clock * 1000;	/* kHz to Hz */
	u32 clkdiv;

	/* Find the clock divider value that gets us closest to ideal_clk */
	clkdiv = DIV_ROUND_UP(clk_get_rate(ctx->vclk), ideal_clk);

	return (clkdiv < 0x100) ? clkdiv : 0xff;
}

static void decon_commit(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	struct drm_display_mode *mode = &crtc->base.state->adjusted_mode;
	u32 val, clkdiv;

	/* nothing to do if we haven't set the mode yet */
	if (mode->htotal == 0 || mode->vtotal == 0)
		return;

	if (!ctx->i80_if) {
		int vsync_len, vbpd, vfpd, hsync_len, hbpd, hfpd;
	      /* setup vertical timing values. */
		vsync_len = mode->crtc_vsync_end - mode->crtc_vsync_start;
		vbpd = mode->crtc_vtotal - mode->crtc_vsync_end;
		vfpd = mode->crtc_vsync_start - mode->crtc_vdisplay;

		val = VIDTCON0_VBPD(vbpd - 1) | VIDTCON0_VFPD(vfpd - 1);
		writel(val, ctx->regs + VIDTCON0);

		val = VIDTCON1_VSPW(vsync_len - 1);
		writel(val, ctx->regs + VIDTCON1);

		/* setup horizontal timing values.  */
		hsync_len = mode->crtc_hsync_end - mode->crtc_hsync_start;
		hbpd = mode->crtc_htotal - mode->crtc_hsync_end;
		hfpd = mode->crtc_hsync_start - mode->crtc_hdisplay;

		/* setup horizontal timing values.  */
		val = VIDTCON2_HBPD(hbpd - 1) | VIDTCON2_HFPD(hfpd - 1);
		writel(val, ctx->regs + VIDTCON2);

		val = VIDTCON3_HSPW(hsync_len - 1);
		writel(val, ctx->regs + VIDTCON3);
	}

	/* setup horizontal and vertical display size. */
	val = VIDTCON4_LINEVAL(mode->vdisplay - 1) |
	       VIDTCON4_HOZVAL(mode->hdisplay - 1);
	writel(val, ctx->regs + VIDTCON4);

	writel(mode->vdisplay - 1, ctx->regs + LINECNT_OP_THRESHOLD);

	/*
	 * fields of register with prefix '_F' would be updated
	 * at vsync(same as dma start)
	 */
	/* keep the i80 and output path bits the bootloader set up */
	val = readl(ctx->regs + VIDCON0);
	val |= VIDCON0_ENVID | VIDCON0_ENVID_F;
	writel(val, ctx->regs + VIDCON0);

	clkdiv = decon_calc_clkdiv(ctx, mode);
	if (clkdiv > 1) {
		val = VCLKCON1_CLKVAL_NUM_VCLK(clkdiv - 1);
		writel(val, ctx->regs + VCLKCON1);
		writel(val, ctx->regs + VCLKCON2);
	}

	val = readl(ctx->regs + DECON_UPDATE);
	val |= DECON_UPDATE_STANDALONE_F;
	writel(val, ctx->regs + DECON_UPDATE);
}

/* Called with te_lock held. */
static void decon_te_irq_get(struct decon_context *ctx)
{
	if (!ctx->te_irq_on) {
		ctx->te_irq_on = true;
		enable_irq(ctx->te_irq);
	}
}

#define DECON_VSTATUS_MASK		(0x7 << 13)

/*
 * Whether DECON has stopped reading the buffers of the state before the
 * last commit. It copies the shadow registers to the active set at the
 * start of a frame and then clears DECON_UPDATE_STANDALONE_F. While that
 * request is still pending, no frame in flight means the next one starts
 * with the new state; a frame with no window enabled may never start at
 * all, so the bit alone is not enough.
 */
static bool decon_old_state_done(struct decon_context *ctx)
{
	u32 val;

	if (!(readl(ctx->regs + DECON_UPDATE) & DECON_UPDATE_STANDALONE_F))
		return true;
	val = readl(ctx->regs + VIDCON1(0));
	return !VIDCON1_LINECNT_GET(val) && !(val & DECON_VSTATUS_MASK);
}

/*
 * A flip completes, and its old framebuffers may go, only once DECON is
 * done with them. Completing it at the TE after the commit, as before, let
 * a framebuffer be freed and unmapped while a transfer still fetched it:
 * the SysMMU then faulted on every retry of the read. Called with te_lock
 * held; the caller sends the event.
 */
static struct drm_pending_vblank_event *
decon_take_latched_flip(struct decon_context *ctx)
{
	struct drm_pending_vblank_event *event = ctx->flip_event;

	if (!event || !decon_old_state_done(ctx))
		return NULL;
	ctx->flip_event = NULL;
	return event;
}

/* Send a flip event taken from ctx->flip_event. Not with te_lock held. */
static void decon_send_flip(struct decon_context *ctx,
			    struct drm_pending_vblank_event *event)
{
	struct drm_crtc *crtc = &ctx->crtc->base;
	unsigned long flags;

	if (!event)
		return;
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	drm_crtc_vblank_put(crtc);
}

/*
 * Send a flip whose frame goes out at the next TE, stamped with that TE:
 * its vblank count and the time the panel starts showing the frame. The
 * event and the out-fence carry this stamp, as drm_crtc_send_vblank_event()
 * would give them at that TE. Returns false, sending nothing, when the TE
 * period is not known yet or that TE is already past. Not with te_lock held.
 */
static bool decon_send_flip_next_te(struct decon_context *ctx,
				    struct drm_pending_vblank_event *e,
				    u32 period_ns)
{
	struct drm_crtc *crtc = &ctx->crtc->base;
	struct drm_device *dev = crtc->dev;
	struct timespec64 tv;
	unsigned long flags;
	ktime_t when;
	u64 seq;

	if (!period_ns)
		return false;
	seq = drm_crtc_vblank_count_and_time(crtc, &when) + 1;
	when = ktime_add_ns(when, period_ns);
	if (ktime_before(when, ktime_get()))
		return false;

	tv = ktime_to_timespec64(when);
	spin_lock_irqsave(&dev->event_lock, flags);
	e->pipe = drm_crtc_index(crtc);
	switch (e->event.base.type) {
	case DRM_EVENT_VBLANK:
	case DRM_EVENT_FLIP_COMPLETE:
		e->event.vbl.sequence = seq;
		e->event.vbl.tv_sec = tv.tv_sec;
		e->event.vbl.tv_usec = tv.tv_nsec / 1000;
		break;
	case DRM_EVENT_CRTC_SEQUENCE:
		e->event.seq.sequence = seq;
		e->event.seq.time_ns = ktime_to_ns(when);
		break;
	}
	drm_send_event_timestamp_locked(dev, &e->base, when);
	spin_unlock_irqrestore(&dev->event_lock, flags);
	drm_crtc_vblank_put(crtc);
	return true;
}

static int decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	unsigned long flags;
	u32 val;

	if (ctx->te_sync) {
		spin_lock_irqsave(&ctx->te_lock, flags);
		ctx->vblank_on = true;
		decon_te_irq_get(ctx);
		spin_unlock_irqrestore(&ctx->te_lock, flags);
	}

	if (!test_and_set_bit(0, &ctx->irq_flags)) {
		val = readl(ctx->regs + VIDINTCON0);

		val |= VIDINTCON0_INT_ENABLE;

		if (ctx->i80_if) {
			/* frame done is the only frame event in command mode */
			val |= VIDINTCON0_INT_I80_EN;
		} else {
			val |= VIDINTCON0_INT_FRAME;
			val &= ~VIDINTCON0_FRAMESEL0_MASK;
			val |= VIDINTCON0_FRAMESEL0_VSYNC;
		}

		writel(val, ctx->regs + VIDINTCON0);
	}

	return 0;
}

static void decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	unsigned long flags;
	u32 val;

	if (ctx->te_sync) {
		/*
		 * Vblank counts TE edges. The frame-done interrupt stays on:
		 * it ends each transfer. The TE handler turns its interrupt
		 * off once no frame waits for it.
		 */
		spin_lock_irqsave(&ctx->te_lock, flags);
		ctx->vblank_on = false;
		spin_unlock_irqrestore(&ctx->te_lock, flags);
		return;
	}

	if (test_and_clear_bit(0, &ctx->irq_flags)) {
		val = readl(ctx->regs + VIDINTCON0);

		val &= ~VIDINTCON0_INT_ENABLE;
		if (ctx->i80_if)
			val &= ~VIDINTCON0_INT_I80_EN;
		else
			val &= ~VIDINTCON0_INT_FRAME;

		writel(val, ctx->regs + VIDINTCON0);
	}
}

static void decon_win_set_pixfmt(struct decon_context *ctx, unsigned int win,
				 struct drm_framebuffer *fb)
{
	unsigned long val;
	int padding;

	val = readl(ctx->regs + WINCON(win));
	val &= ~WINCONx_BPPMODE_MASK;

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		val |= WINCONx_BPPMODE_16BPP_565;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_XRGB8888:
		val |= WINCONx_BPPMODE_24BPP_xRGB;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_XBGR8888:
		val |= WINCONx_BPPMODE_24BPP_xBGR;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_RGBX8888:
		val |= WINCONx_BPPMODE_24BPP_RGBx;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_BGRX8888:
		val |= WINCONx_BPPMODE_24BPP_BGRx;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_ARGB8888:
		val |= WINCONx_BPPMODE_32BPP_ARGB | WINCONx_BLD_PIX |
			WINCONx_ALPHA_SEL;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_ABGR8888:
		val |= WINCONx_BPPMODE_32BPP_ABGR | WINCONx_BLD_PIX |
			WINCONx_ALPHA_SEL;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_RGBA8888:
		val |= WINCONx_BPPMODE_32BPP_RGBA | WINCONx_BLD_PIX |
			WINCONx_ALPHA_SEL;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_BGRA8888:
	default:
		val |= WINCONx_BPPMODE_32BPP_BGRA | WINCONx_BLD_PIX |
			WINCONx_ALPHA_SEL;
		val |= WINCONx_BURSTLEN_16WORD;
		break;
	}

	DRM_DEV_DEBUG_KMS(ctx->dev, "cpp = %d\n", fb->format->cpp[0]);

	/*
	 * In case of exynos, setting dma-burst to 16Word causes permanent
	 * tearing for very small buffers, e.g. cursor buffer. Burst Mode
	 * switching which is based on plane size is not recommended as
	 * plane size varies a lot towards the end of the screen and rapid
	 * movement causes unstable DMA which results into iommu crash/tear.
	 */

	padding = (fb->pitches[0] / fb->format->cpp[0]) - fb->width;
	if (fb->width + padding < MIN_FB_WIDTH_FOR_16WORD_BURST) {
		val &= ~WINCONx_BURSTLEN_MASK;
		val |= WINCONx_BURSTLEN_8WORD;
	}

	writel(val, ctx->regs + WINCON(win));
}

static void decon_win_set_colkey(struct decon_context *ctx, unsigned int win)
{
	unsigned int keycon0 = 0, keycon1 = 0;

	keycon0 = ~(WxKEYCON0_KEYBL_EN | WxKEYCON0_KEYEN_F |
			WxKEYCON0_DIRCON) | WxKEYCON0_COMPKEY(0);

	keycon1 = WxKEYCON1_COLVAL(0xffffffff);

	writel(keycon0, ctx->regs + WKEYCON0_BASE(win));
	writel(keycon1, ctx->regs + WKEYCON1_BASE(win));
}

static void decon_release_bandwidth(struct decon_context *ctx)
{
	icc_set_bw(ctx->icc_path, 0, 0);
	ctx->icc_bw = 0;
}

static void decon_bw_idle_work(struct work_struct *work)
{
	struct decon_context *ctx = container_of(to_delayed_work(work),
						 struct decon_context,
						 bw_idle_work);

	decon_release_bandwidth(ctx);
}

/*
 * Ask the display bus for the bandwidth the enabled windows read: their
 * source size times the refresh rate, with half again as headroom for
 * the burst at the start of each frame. While the previous frame may
 * still be scanned out, keep the larger of its request and this one;
 * a drop takes effect at the next update.
 *
 * In command mode DECON reads memory only while it sends a frame, so the
 * request is released once no update came for DECON_BW_IDLE_MS. The
 * pending release is cancelled before a new frame raises the request.
 */
static void decon_update_bandwidth(struct decon_context *ctx)
{
	struct drm_crtc *crtc = &ctx->crtc->base;
	struct drm_plane *plane;
	u64 bytes = 0;
	u32 bw;

	cancel_delayed_work_sync(&ctx->bw_idle_work);

	drm_for_each_plane_mask(plane, crtc->dev, crtc->state->plane_mask) {
		const struct exynos_drm_plane_state *state =
					to_exynos_plane_state(plane->state);

		if (!state->base.fb)
			continue;
		/* the clipped source the window really fetches */
		bytes += (u64)state->src.w * state->src.h *
			 state->base.fb->format->cpp[0];
	}

	bw = div_u64(bytes * drm_mode_vrefresh(&crtc->state->adjusted_mode),
		     1000);
	icc_set_bw(ctx->icc_path, kBps_to_icc(max(bw, ctx->icc_bw)),
		   kBps_to_icc(max(bw, ctx->icc_bw) / 2 * 3));
	ctx->icc_bw = bw;
}

static void decon_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	int i;

	/*
	 * No wait for the previous transfer: the window registers written
	 * here are shadow copies. DECON copies them to the active set only
	 * when the next frame is triggered, so a transfer in progress keeps
	 * reading the old state (as in exynos5433_drm_decon and the 3.10
	 * decon driver, which both reprogram while a frame is going out).
	 */
	if (ctx->te_sync && READ_ONCE(ctx->xfer_busy))
		WRITE_ONCE(decon_te_mid_xfer, decon_te_mid_xfer + 1);

	decon_update_bandwidth(ctx);

	for (i = 0; i < WINDOWS_NR; i++)
		decon_shadow_protect_win(ctx, i, true);
}

static void decon_update_plane(struct exynos_drm_crtc *crtc,
			       struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
				to_exynos_plane_state(plane->base.state);
	struct decon_context *ctx = crtc->ctx;
	struct drm_framebuffer *fb = state->base.fb;
	int padding;
	unsigned long val, alpha;
	unsigned int last_x;
	unsigned int last_y;
	unsigned int win = plane->index;
	unsigned int cpp = fb->format->cpp[0];
	unsigned int pitch = fb->pitches[0];

	/*
	 * SHADOWCON/PRTCON register is used for enabling timing.
	 *
	 * for example, once only width value of a register is set,
	 * if the dma is started then decon hardware could malfunction so
	 * with protect window setting, the register fields with prefix '_F'
	 * wouldn't be updated at vsync also but updated once unprotect window
	 * is set.
	 */

	/*
	 * buffer start address: the hardware cycles through all three buffer
	 * slots, and a stale address in one of them shows as flicker between
	 * old and new frames.
	 */
	val = (unsigned long)exynos_drm_fb_dma_addr(fb, 0);
	writel(val, ctx->regs + VIDW_BUF_START(win));
	writel(val, ctx->regs + VIDW_BUF_START1(win));
	writel(val, ctx->regs + VIDW_BUF_START2(win));

	padding = (pitch / cpp) - fb->width;

	/* buffer size */
	writel(fb->width + padding, ctx->regs + VIDW_WHOLE_X(win));
	writel(fb->height, ctx->regs + VIDW_WHOLE_Y(win));

	/* offset from the start of the buffer to read */
	writel(state->src.x, ctx->regs + VIDW_OFFSET_X(win));
	writel(state->src.y, ctx->regs + VIDW_OFFSET_Y(win));

	DRM_DEV_DEBUG_KMS(ctx->dev, "start addr = 0x%lx\n",
			(unsigned long)val);
	DRM_DEV_DEBUG_KMS(ctx->dev, "ovl_width = %d, ovl_height = %d\n",
			state->crtc.w, state->crtc.h);

	val = VIDOSDxA_TOPLEFT_X(state->crtc.x) |
		VIDOSDxA_TOPLEFT_Y(state->crtc.y);
	writel(val, ctx->regs + VIDOSD_A(win));

	last_x = state->crtc.x + state->crtc.w;
	if (last_x)
		last_x--;
	last_y = state->crtc.y + state->crtc.h;
	if (last_y)
		last_y--;

	val = VIDOSDxB_BOTRIGHT_X(last_x) | VIDOSDxB_BOTRIGHT_Y(last_y);

	writel(val, ctx->regs + VIDOSD_B(win));

	DRM_DEV_DEBUG_KMS(ctx->dev, "osd pos: tx = %d, ty = %d, bx = %d, by = %d\n",
			state->crtc.x, state->crtc.y, last_x, last_y);

	/* OSD alpha */
	alpha = VIDOSDxC_ALPHA0_R_F(0x0) |
			VIDOSDxC_ALPHA0_G_F(0x0) |
			VIDOSDxC_ALPHA0_B_F(0x0);

	writel(alpha, ctx->regs + VIDOSD_C(win));

	alpha = VIDOSDxD_ALPHA1_R_F(0xff) |
			VIDOSDxD_ALPHA1_G_F(0xff) |
			VIDOSDxD_ALPHA1_B_F(0xff);

	writel(alpha, ctx->regs + VIDOSD_D(win));

	decon_win_set_pixfmt(ctx, win, fb);

	/* hardware window 0 doesn't support color key. */
	if (win != 0)
		decon_win_set_colkey(ctx, win);

	/* wincon */
	val = readl(ctx->regs + WINCON(win));
	val |= WINCONx_TRIPLE_BUF_MODE;
	val |= WINCONx_ENWIN;
	writel(val, ctx->regs + WINCON(win));

	/* Enable DMA channel and unprotect windows */
	decon_shadow_protect_win(ctx, win, false);

	val = readl(ctx->regs + DECON_UPDATE);
	val |= DECON_UPDATE_STANDALONE_F;
	writel(val, ctx->regs + DECON_UPDATE);
}

static void decon_disable_plane(struct exynos_drm_crtc *crtc,
				struct exynos_drm_plane *plane)
{
	struct decon_context *ctx = crtc->ctx;
	unsigned int win = plane->index;
	u32 val;

	/* protect windows */
	decon_shadow_protect_win(ctx, win, true);

	/* wincon */
	val = readl(ctx->regs + WINCON(win));
	val &= ~WINCONx_ENWIN;
	writel(val, ctx->regs + WINCON(win));

	val = readl(ctx->regs + DECON_UPDATE);
	val |= DECON_UPDATE_STANDALONE_F;
	writel(val, ctx->regs + DECON_UPDATE);
}

/*
 * Start one frame transfer to a command mode (i80) panel. The panel has no
 * free-running timing, so nothing reaches it until a frame is triggered.
 */
static void decon_i80_trigger(struct decon_context *ctx)
{
	u32 val;

	val = readl(ctx->regs + TRIGCON);
	val &= ~(TRIGCON_HWTRIG_INV_I80_RGB | TRIGCON_HWTRIGEN_I80_RGB |
		 TRIGCON_HWTRIGMASK_DISPIF0 | TRIGCON_HWTRIGMASK_DISPIF1);
	writel(val, ctx->regs + TRIGCON);

	val |= TRIGCON_SWTRIGEN_I80_RGB | TRIGCON_SWTRIGCMD_I80_RGB |
	       TRIGCON_HWTRIGEN_I80_RGB | TRIGCON_HWTRIGMASK_DISPIF0 |
	       TRIGCON_HWTRIGMASK_DISPIF1 | TRIGCON_HWTRIG_AUTO_MASK;
	writel(val, ctx->regs + TRIGCON);

	/* latch the shadow registers */
	writel(DECON_UPDATE_STANDALONE_F, ctx->regs + DECON_UPDATE);

	val = readl(ctx->regs + TRIGCON);
	val |= TRIGCON_SWTRIGCMD_I80_RGB;
	writel(val, ctx->regs + TRIGCON);
}

/* Start sending the committed frame. Called with te_lock held. */
static void decon_te_send(struct decon_context *ctx)
{
	decon_i80_trigger(ctx);
	ctx->trig_pending = false;
	ctx->xfer_busy = true;
}

/*
 * TE sync: leave the transfer to the next TE edge. The flip completes at
 * the frame done of a transfer still running, stamped with the TE that
 * will send the new frame, or else at that TE. Its event and the trigger
 * are armed together, so a TE in between never sees the event without the
 * frame. te_lock is not held around event_lock, which the vblank core
 * takes before calling disable_vblank. Returns an event left over from a
 * flip_done timeout.
 */
static struct drm_pending_vblank_event *
decon_te_flush(struct decon_context *ctx, struct drm_pending_vblank_event *event)
{
	struct drm_pending_vblank_event *old = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ctx->te_lock, flags);
	if (event) {
		old = ctx->flip_event;
		ctx->flip_event = event;
	}
	ctx->trig_pending = true;
	decon_te_irq_get(ctx);
	spin_unlock_irqrestore(&ctx->te_lock, flags);

	mod_delayed_work(system_highpri_wq, &ctx->te_watchdog,
			 msecs_to_jiffies(DECON_TE_TIMEOUT_MS));
	return old;
}

/* A frame that got no TE in time is sent anyway, so the display never stalls. */
static void decon_te_watchdog(struct work_struct *work)
{
	struct decon_context *ctx = container_of(to_delayed_work(work),
						 struct decon_context,
						 te_watchdog);
	unsigned long flags;
	bool sent = false, off = false;

	spin_lock_irqsave(&ctx->te_lock, flags);
	if (ctx->trig_pending) {
		/* a lost frame-done must not hold the display either */
		decon_te_send(ctx);
		sent = true;
		if (++ctx->te_misses >= DECON_TE_MISS_MAX) {
			/* frame done feeds vblank again, as without TE */
			ctx->te_sync = false;
			WRITE_ONCE(decon_te_active, false);
			ctx->xfer_busy = false;
			if (ctx->te_irq_on) {
				ctx->te_irq_on = false;
				disable_irq_nosync(ctx->te_irq);
			}
			off = true;
		}
	}
	spin_unlock_irqrestore(&ctx->te_lock, flags);

	if (sent) {
		dev_warn_ratelimited(ctx->dev, "no TE for %u ms, frame sent without it\n",
				     DECON_TE_TIMEOUT_MS);
		drm_crtc_handle_vblank(&ctx->crtc->base);
	}
	if (off) {
		wake_up(&ctx->te_wq);
		dev_warn(ctx->dev, "TE lost, frames sent on commit\n");
	}
}

static irqreturn_t decon_te_irq_handler(int irq, void *dev_id)
{
	struct decon_context *ctx = dev_id;
	struct drm_pending_vblank_event *flip;
	ktime_t now = ktime_get();
	bool vblank;

	spin_lock(&ctx->te_lock);
	ctx->te_count++;
	/*
	 * The panel's own refresh period, for stamping flips ahead of TE.
	 * It starts from the display mode and follows the measured TE
	 * intervals; a gap that is not one period (a missed TE, the
	 * interrupt off) is not a sample.
	 */
	if (ctx->te_last) {
		s64 d = ktime_to_ns(ktime_sub(now, ctx->te_last));
		s64 p = ctx->te_period_ns;

		if (!p && d >= 8000000 && d <= 34000000)
			ctx->te_period_ns = d;
		else if (p && d > p * 7 / 8 && d < p * 9 / 8)
			ctx->te_period_ns = p + (d - p) / 16;
		WRITE_ONCE(decon_te_period, ctx->te_period_ns);
	}
	ctx->te_last = now;
	if (ctx->te_sync && ctx->trig_pending && !ctx->xfer_busy) {
		decon_te_send(ctx);
		ctx->te_misses = 0;
		/*
		 * A commit made after the last frame done: its frame goes
		 * out from this TE, so complete its flip here, stamped with
		 * this TE.
		 */
		flip = ctx->flip_event;
		ctx->flip_event = NULL;
	} else if (ctx->te_sync && ctx->trig_pending) {
		flip = NULL;
	} else {
		flip = decon_take_latched_flip(ctx);
	}
	vblank = ctx->te_sync && ctx->vblank_on;
	if (ctx->te_sync && !ctx->trig_pending && !vblank) {
		disable_irq_nosync(irq);
		ctx->te_irq_on = false;
	}
	spin_unlock(&ctx->te_lock);

	if (!ctx->te_sync)
		wake_up(&ctx->te_wq);
	if (vblank)
		drm_crtc_handle_vblank(&ctx->crtc->base);
	/* after the vblank count, so the event carries this TE's time */
	decon_send_flip(ctx, flip);

	return IRQ_HANDLED;
}

/*
 * S-Boot leaves the panel running with TE on, but nothing guarantees it:
 * use TE only once edges arrive at a sane rate. Called when the CRTC is
 * enabled.
 */
static void decon_te_start(struct decon_context *ctx)
{
	const struct drm_display_mode *mode;
	unsigned long flags;
	u32 period;
	ktime_t t0;
	s64 us;
	u32 val;

	if (!ctx->i80_if || ctx->te_irq <= 0 || !decon_te_enable)
		return;

	mode = &ctx->crtc->base.state->adjusted_mode;
	period = mode->clock ? div_u64((u64)mode->htotal * mode->vtotal *
				       1000000, mode->clock) : 0;

	spin_lock_irqsave(&ctx->te_lock, flags);
	ctx->te_sync = false;
	WRITE_ONCE(decon_te_active, false);
	ctx->te_last = 0;
	ctx->te_period_ns = period;
	ctx->te_count = 0;
	ctx->te_misses = 0;
	ctx->trig_pending = false;
	ctx->xfer_busy = false;
	decon_te_irq_get(ctx);
	spin_unlock_irqrestore(&ctx->te_lock, flags);

	t0 = ktime_get();
	wait_event_timeout(ctx->te_wq, READ_ONCE(ctx->te_count) >= 7,
			   msecs_to_jiffies(200));
	us = ktime_us_delta(ktime_get(), t0);

	spin_lock_irqsave(&ctx->te_lock, flags);
	/* 6 TE periods at 30..120 Hz; anything faster is not TE */
	if (ctx->te_count >= 7 && us >= 6 * 8000 && us <= 7 * 34000) {
		/* the frame-done interrupt ends every transfer from now on */
		val = readl(ctx->regs + VIDINTCON0);
		writel(val | VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_I80_EN,
		       ctx->regs + VIDINTCON0);
		/* vblank may already be on: from now on TE feeds it */
		ctx->vblank_on = test_and_set_bit(0, &ctx->irq_flags);
		ctx->te_sync = true;
		WRITE_ONCE(decon_te_active, true);
	} else {
		ctx->te_irq_on = false;
		disable_irq_nosync(ctx->te_irq);
	}
	spin_unlock_irqrestore(&ctx->te_lock, flags);

	if (ctx->te_sync)
		dev_info(ctx->dev, "TE sync on: %u edges in %lld us\n",
			 ctx->te_count, us);
	else
		dev_warn(ctx->dev, "no TE (%u edges in %lld us), frames sent on commit\n",
			 ctx->te_count, us);
}

static void decon_te_stop(struct decon_context *ctx)
{
	unsigned long flags;

	if (ctx->te_irq <= 0)
		return;
	cancel_delayed_work_sync(&ctx->te_watchdog);
	spin_lock_irqsave(&ctx->te_lock, flags);
	ctx->te_sync = false;
	WRITE_ONCE(decon_te_active, false);
	ctx->trig_pending = false;
	ctx->xfer_busy = false;
	if (ctx->te_irq_on) {
		ctx->te_irq_on = false;
		disable_irq_nosync(ctx->te_irq);
	}
	spin_unlock_irqrestore(&ctx->te_lock, flags);
}

static void decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	int i;

	for (i = 0; i < WINDOWS_NR; i++)
		decon_shadow_protect_win(ctx, i, false);

	if (ctx->i80_if) {
		struct drm_crtc *crtc_base = &crtc->base;
		struct drm_pending_vblank_event *event = crtc_base->state->event;
		struct drm_pending_vblank_event *old = NULL;
		bool te_sync = READ_ONCE(ctx->te_sync);
		unsigned long flags;

		/*
		 * Request the latch only now that every window is unprotected,
		 * so a cleared STANDALONE_F means all of this commit is active.
		 */
		writel(readl(ctx->regs + DECON_UPDATE) |
		       DECON_UPDATE_STANDALONE_F, ctx->regs + DECON_UPDATE);

		if (event) {
			crtc_base->state->event = NULL;
			WARN_ON(drm_crtc_vblank_get(crtc_base) != 0);
		}

		if (ctx->icc_bw)
			mod_delayed_work(system_wq, &ctx->bw_idle_work,
					 msecs_to_jiffies(DECON_BW_IDLE_MS));

		if (te_sync) {
			old = decon_te_flush(ctx, event);
		} else if (event) {
			spin_lock_irqsave(&ctx->te_lock, flags);
			old = ctx->flip_event;
			ctx->flip_event = event;
			spin_unlock_irqrestore(&ctx->te_lock, flags);
		}
		/* only after a flip_done timeout: do not leak it */
		if (WARN_ON_ONCE(old))
			decon_send_flip(ctx, old);

		/* without TE, the frame done of this transfer completes the flip */
		if (!te_sync)
			decon_i80_trigger(ctx);
		return;
	}

	exynos_crtc_handle_event(crtc);
}

static void decon_init(struct decon_context *ctx)
{
	u32 val;

	/*
	 * In i80 mode keep what the bootloader set up (DSIM link, panel
	 * timing, trigger setup): a software reset drops it and the panel
	 * stops updating.
	 */
	if (ctx->i80_if)
		return;

	writel(VIDCON0_SWRESET, ctx->regs + VIDCON0);

	val = VIDOUTCON0_DISP_IF_0_ON;
	if (!ctx->i80_if)
		val |= VIDOUTCON0_RGBIF;
	writel(val, ctx->regs + VIDOUTCON0);

	writel(VCLKCON0_CLKVALUP | VCLKCON0_VCLKFREE, ctx->regs + VCLKCON0);

	if (!ctx->i80_if)
		writel(VIDCON1_VCLK_HOLD, ctx->regs + VIDCON1(0));
}

/*
 * Take over the scanout the bootloader left running: map what DECON is
 * still fetching 1:1 in its IOMMU domain, then turn translation on. Doing
 * it the other way round faults every fetch.
 */
static void decon_iommu_takeover(struct decon_context *ctx)
{
	phys_addr_t base = 0;
	size_t size = 0;
	unsigned int win;
	int ret;

	for (win = 0; win < WINDOWS_NR; win++) {
		u32 addr = readl(ctx->regs + VIDW_BUF_START(win));

		if (addr && (!base || addr < base))
			base = addr;
	}

	if (ctx->boot_fb_size && (!base || ctx->boot_fb_base <= base)) {
		base = ctx->boot_fb_base;
		size = ctx->boot_fb_size;
	} else if (base) {
		size = BOOT_FB_FALLBACK_SIZE;
	}

	if (size) {
		ret = exynos_iommu_map_identity(ctx->dev, base, size);
		if (ret == -ENODEV)
			dev_dbg(ctx->dev, "no IOMMU domain, scanout not mapped\n");
		else if (ret)
			dev_err(ctx->dev, "failed to map scanout %pa: %d\n",
				&base, ret);
	}

	exynos_iommu_master_ready(ctx->dev);
	ctx->iommu_ready = true;
}

static void decon_atomic_enable(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	int ret;

	ret = pm_runtime_resume_and_get(ctx->dev);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "failed to enable DECON device.\n");
		return;
	}

	/*
	 * Once only: later resumes enable translation from the SysMMU's own
	 * runtime PM, and the window registers then hold IOVAs.
	 */
	if (ctx->i80_if && !ctx->iommu_ready)
		decon_iommu_takeover(ctx);

	decon_init(ctx);

	/* if vblank was enabled status, enable it again. */
	if (test_and_clear_bit(0, &ctx->irq_flags))
		decon_enable_vblank(ctx->crtc);

	decon_commit(ctx->crtc);
	decon_te_start(ctx);
}

/*
 * The helpers skip atomic_begin/flush for a CRTC being turned off, so the
 * window disables above are still shadow-protected and unlatched, and in
 * command mode the TE trigger goes on fetching the old buffers from the
 * active registers. Unprotect them, request a frame and wait until DECON
 * no longer reads the old buffers, so they can be freed once this returns.
 */
static void decon_stop_windows(struct decon_context *ctx)
{
	struct drm_pending_vblank_event *flip;
	unsigned long flags;
	bool done;
	int i, ret;

	for (i = 0; i < WINDOWS_NR; i++)
		decon_shadow_protect_win(ctx, i, false);
	decon_i80_trigger(ctx);

	ret = read_poll_timeout(decon_old_state_done, done, done, 100, 100000,
				false, ctx);
	if (ret)
		dev_warn(ctx->dev, "window disable not latched\n");

	/* a flip still waiting for its latch: the CRTC is off now */
	spin_lock_irqsave(&ctx->te_lock, flags);
	flip = ctx->flip_event;
	ctx->flip_event = NULL;
	spin_unlock_irqrestore(&ctx->te_lock, flags);
	decon_send_flip(ctx, flip);
}

static void decon_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	int i;

	decon_te_stop(ctx);

	/*
	 * We need to make sure that all windows are disabled before we
	 * suspend that connector. Otherwise we might try to scan from
	 * a destroyed buffer later.
	 */
	for (i = 0; i < WINDOWS_NR; i++)
		decon_disable_plane(crtc, &ctx->planes[i]);

	if (ctx->i80_if)
		decon_stop_windows(ctx);

	pm_runtime_put_sync(ctx->dev);

	cancel_delayed_work_sync(&ctx->bw_idle_work);
	decon_release_bandwidth(ctx);
}

static const struct exynos_drm_crtc_ops decon_crtc_ops = {
	.atomic_enable = decon_atomic_enable,
	.atomic_disable = decon_atomic_disable,
	.enable_vblank = decon_enable_vblank,
	.disable_vblank = decon_disable_vblank,
	.atomic_begin = decon_atomic_begin,
	.update_plane = decon_update_plane,
	.disable_plane = decon_disable_plane,
	.atomic_flush = decon_atomic_flush,
};


static irqreturn_t decon_irq_handler(int irq, void *dev_id)
{
	struct decon_context *ctx = (struct decon_context *)dev_id;
	struct drm_pending_vblank_event *flip;
	u32 val, clear_bit;

	val = readl(ctx->regs + VIDINTCON1);

	clear_bit = ctx->i80_if ? VIDINTCON1_INT_I80 : VIDINTCON1_INT_FRAME;
	if (val & clear_bit)
		writel(clear_bit, ctx->regs + VIDINTCON1);

	/* check the crtc is detached already from encoder */
	if (!ctx->drm_dev)
		goto out;

	/* check if crtc and vblank have been initialized properly */
	if (!drm_dev_has_vblank(ctx->drm_dev))
		goto out;

	if (ctx->te_sync) {
		/* vblank counts TE edges; frame done only ends the transfer */
		spin_lock(&ctx->te_lock);
		if (ctx->trig_pending &&
		    !(readl(ctx->regs + DECON_UPDATE) & DECON_UPDATE_STANDALONE_F))
			decon_te_early_latch++;
		ctx->xfer_busy = false;
		spin_unlock(&ctx->te_lock);
		wake_up(&ctx->te_wq);
	} else {
		/*
		 * In command (i80) mode without TE the end of a frame
		 * transfer stands in for vsync.
		 */
		drm_crtc_handle_vblank(&ctx->crtc->base);
	}

	if (ctx->i80_if) {
		bool ahead;
		u32 period;

		spin_lock(&ctx->te_lock);
		flip = decon_take_latched_flip(ctx);
		ahead = ctx->te_sync && ctx->trig_pending;
		period = ctx->te_period_ns;
		spin_unlock(&ctx->te_lock);
		/*
		 * A commit made while this transfer ran: DECON is done with
		 * the old framebuffers now, so complete the flip and let them
		 * go, but its frame only goes out at the next TE. Stamp the
		 * event and out-fence with that TE: the vblank count would
		 * give the TE before, a refresh earlier than the panel shows
		 * the frame, and SurfaceFlinger, which takes the out-fence
		 * time as the present time, saw such frames as presented
		 * early, and two in a row with one time as a duplicate.
		 */
		if (!flip || !ahead || !decon_send_flip_next_te(ctx, flip, period))
			decon_send_flip(ctx, flip);
	}

	/* set wait vsync event to zero and wake up queue. */
	if (atomic_read(&ctx->wait_vsync_event)) {
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
	}
out:
	return IRQ_HANDLED;
}

static int decon_bind(struct device *dev, struct device *master, void *data)
{
	struct decon_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_plane *exynos_plane;
	unsigned int i;
	int ret;

	ret = decon_ctx_initialize(ctx, drm_dev);
	if (ret) {
		DRM_DEV_ERROR(dev, "decon_ctx_initialize failed.\n");
		return ret;
	}

	for (i = 0; i < WINDOWS_NR; i++) {
		ctx->configs[i].pixel_formats = decon_formats;
		ctx->configs[i].num_pixel_formats = ARRAY_SIZE(decon_formats);
		ctx->configs[i].zpos = i;
		ctx->configs[i].type = decon_win_types[i];

		ret = exynos_plane_init(drm_dev, &ctx->planes[i], i,
					&ctx->configs[i]);
		if (ret)
			return ret;
	}

	exynos_plane = &ctx->planes[DEFAULT_WIN];
	ctx->crtc = exynos_drm_crtc_create(drm_dev, &exynos_plane->base,
			EXYNOS_DISPLAY_TYPE_LCD, &decon_crtc_ops, ctx);
	if (IS_ERR(ctx->crtc)) {
		decon_ctx_remove(ctx);
		return PTR_ERR(ctx->crtc);
	}

	if (ctx->encoder) {
		ret = exynos_dpi_bind(drm_dev, ctx->encoder);
		if (ret) {
			decon_ctx_remove(ctx);
			return ret;
		}
	}

	return 0;
}

static void decon_unbind(struct device *dev, struct device *master,
			void *data)
{
	struct decon_context *ctx = dev_get_drvdata(dev);

	decon_atomic_disable(ctx->crtc);

	if (ctx->encoder)
		exynos_dpi_remove(ctx->encoder);

	decon_ctx_remove(ctx);
}

static const struct component_ops decon_component_ops = {
	.bind	= decon_bind,
	.unbind = decon_unbind,
};

/* The framebuffer the bootloader scans out, as described in /chosen */
static void decon_get_boot_fb(struct decon_context *ctx)
{
	struct device_node *chosen, *np;
	const __be32 *reg;
	u64 size;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return;

	for_each_child_of_node(chosen, np) {
		if (!of_device_is_compatible(np, "simple-framebuffer") ||
		    !of_device_is_available(np))
			continue;

		/* /chosen may lack ranges: read the address untranslated */
		reg = of_get_address(np, 0, &size, NULL);
		if (reg && size) {
			ctx->boot_fb_base = of_read_number(reg,
							   of_n_addr_cells(np));
			ctx->boot_fb_size = size;
			of_node_put(np);
			break;
		}
	}

	of_node_put(chosen);
}

static int decon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct decon_context *ctx;
	struct device_node *i80_if_timings;
	struct icc_path *icc_path;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	/* before the boot framebuffer is claimed: this can defer */
	icc_path = devm_of_icc_get(dev, NULL);
	if (IS_ERR(icc_path))
		return dev_err_probe(dev, PTR_ERR(icc_path),
				     "failed to get the display bus path\n");

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->icc_path = icc_path;
	INIT_DELAYED_WORK(&ctx->bw_idle_work, decon_bw_idle_work);

	i80_if_timings = of_get_child_by_name(dev->of_node, "i80-if-timings");
	if (i80_if_timings)
		ctx->i80_if = true;
	of_node_put(i80_if_timings);
	ctx->bootloader_display = of_property_read_bool(dev->of_node,
						"samsung,bootloader-display");

	if (ctx->i80_if)
		decon_get_boot_fb(ctx);

	ctx->regs = of_iomap(dev->of_node, 0);
	if (!ctx->regs)
		return -ENOMEM;

	ctx->pclk = devm_clk_get(dev, "pclk_decon0");
	if (IS_ERR(ctx->pclk)) {
		dev_err(dev, "failed to get bus clock pclk\n");
		ret = PTR_ERR(ctx->pclk);
		goto err_iounmap;
	}

	ctx->aclk = devm_clk_get(dev, "aclk_decon0");
	if (IS_ERR(ctx->aclk)) {
		dev_err(dev, "failed to get bus clock aclk\n");
		ret = PTR_ERR(ctx->aclk);
		goto err_iounmap;
	}

	ctx->eclk = devm_clk_get(dev, "decon0_eclk");
	if (IS_ERR(ctx->eclk)) {
		dev_err(dev, "failed to get eclock\n");
		ret = PTR_ERR(ctx->eclk);
		goto err_iounmap;
	}

	ctx->vclk = devm_clk_get(dev, "decon0_vclk");
	if (IS_ERR(ctx->vclk)) {
		dev_err(dev, "failed to get vclock\n");
		ret = PTR_ERR(ctx->vclk);
		goto err_iounmap;
	}

	ret =  platform_get_irq_byname(pdev, ctx->i80_if ? "lcd_sys" : "vsync");
	if (ret < 0)
		goto err_iounmap;

	ret = devm_request_irq(dev, ret, decon_irq_handler, 0, "drm_decon", ctx);
	if (ret) {
		dev_err(dev, "irq request failed.\n");
		goto err_iounmap;
	}

	init_waitqueue_head(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	spin_lock_init(&ctx->te_lock);
	init_waitqueue_head(&ctx->te_wq);
	INIT_DELAYED_WORK(&ctx->te_watchdog, decon_te_watchdog);
	if (ctx->i80_if) {
		struct gpio_desc *te = devm_gpiod_get_optional(dev, "te", GPIOD_IN);

		if (IS_ERR(te)) {
			ret = dev_err_probe(dev, PTR_ERR(te), "TE gpio\n");
			goto err_iounmap;
		}
		if (te) {
			ctx->te_irq = gpiod_to_irq(te);
			if (ctx->te_irq > 0)
				ret = devm_request_irq(dev, ctx->te_irq,
						       decon_te_irq_handler,
						       IRQF_TRIGGER_RISING |
						       IRQF_NO_AUTOEN,
						       "decon-te", ctx);
			else
				ret = ctx->te_irq;
			if (ret) {
				dev_warn(dev, "TE irq: %d, frames sent on commit\n", ret);
				ctx->te_irq = 0;
			}
		}
	}

	platform_set_drvdata(pdev, ctx);

	ctx->encoder = exynos_dpi_probe(dev);
	if (IS_ERR(ctx->encoder)) {
		ret = PTR_ERR(ctx->encoder);
		goto err_iounmap;
	}

	pm_runtime_enable(dev);

	ret = component_add(dev, &decon_component_ops);
	if (ret)
		goto err_disable_pm_runtime;

	return ret;

err_disable_pm_runtime:
	pm_runtime_disable(dev);

err_iounmap:
	iounmap(ctx->regs);

	return ret;
}

static int decon_remove(struct platform_device *pdev)
{
	struct decon_context *ctx = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);

	iounmap(ctx->regs);

	component_del(&pdev->dev, &decon_component_ops);

	return 0;
}

static int exynos7_decon_suspend(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);

	clk_disable_unprepare(ctx->vclk);
	clk_disable_unprepare(ctx->eclk);
	clk_disable_unprepare(ctx->aclk);
	clk_disable_unprepare(ctx->pclk);

	return 0;
}

static int exynos7_decon_resume(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(ctx->pclk);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to prepare_enable the pclk [%d]\n",
			      ret);
		goto err_pclk_enable;
	}

	ret = clk_prepare_enable(ctx->aclk);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to prepare_enable the aclk [%d]\n",
			      ret);
		goto err_aclk_enable;
	}

	ret = clk_prepare_enable(ctx->eclk);
	if  (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to prepare_enable the eclk [%d]\n",
			      ret);
		goto err_eclk_enable;
	}

	ret = clk_prepare_enable(ctx->vclk);
	if  (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to prepare_enable the vclk [%d]\n",
			      ret);
		goto err_vclk_enable;
	}

	return 0;

err_vclk_enable:
	clk_disable_unprepare(ctx->eclk);
err_eclk_enable:
	clk_disable_unprepare(ctx->aclk);
err_aclk_enable:
	clk_disable_unprepare(ctx->pclk);
err_pclk_enable:
	return ret;
}

/*
 * In command mode with the TE trigger enabled (TRIGCON HWTRIGMASK_DISPIF0
 * set), DECON fetches a whole frame from DRAM on every TE pulse, screen on
 * or off. That traffic keeps the memory interface busy, so the SoC cannot
 * enter sleep. Stop it the way Samsung's decon_reg_stop() did for command
 * mode: disable the TE trigger, let the current frame finish, then clear
 * ENVID and wait for DECON to report stopped. Without the software reset
 * Samsung followed it with, the configuration survives for the restart.
 */
static void exynos7_decon_stop_output(struct decon_context *ctx)
{
	u32 val;
	int ret;

	val = readl(ctx->regs + VIDCON0);
	ctx->sleep_stopped = false;
	if (!(val & VIDCON0_DECON_STOP_STATUS))
		return;

	ctx->sleep_trigcon = readl(ctx->regs + TRIGCON);
	writel(ctx->sleep_trigcon & ~(TRIGCON_HWTRIGMASK_DISPIF0 |
				      TRIGCON_HWTRIGMASK_DISPIF1),
	       ctx->regs + TRIGCON);

	ret = readl_poll_timeout(ctx->regs + VIDCON1(0), val,
				 !VIDCON1_LINECNT_GET(val) &&
				 !(val & DECON_VSTATUS_MASK), 10, 50000);
	if (ret)
		dev_warn(ctx->dev, "frame did not finish: VIDCON1 %#x\n", val);

	writel(readl(ctx->regs + VIDCON0) &
	       ~(VIDCON0_ENVID | VIDCON0_ENVID_F), ctx->regs + VIDCON0);
	ret = readl_poll_timeout(ctx->regs + VIDCON0, val,
				 !(val & VIDCON0_DECON_STOP_STATUS), 10, 20000);
	if (ret)
		dev_warn(ctx->dev, "did not stop: VIDCON0 %#x\n", val);
	ctx->sleep_stopped = true;
	dev_dbg(ctx->dev, "output stopped for sleep (TRIGCON %#x)\n",
		 ctx->sleep_trigcon);
}

static void exynos7_decon_start_output(struct decon_context *ctx)
{
	if (!ctx->sleep_stopped)
		return;

	writel(readl(ctx->regs + VIDCON0) | VIDCON0_ENVID | VIDCON0_ENVID_F,
	       ctx->regs + VIDCON0);
	writel(ctx->sleep_trigcon, ctx->regs + TRIGCON);
	ctx->sleep_stopped = false;
	dev_dbg(ctx->dev, "output restarted after sleep\n");
}

/*
 * With "samsung,bootloader-display" the DSI link and panel are only set up
 * by the bootloader. Gating DECON's clocks through system sleep stalled the
 * panel after resume (one frame, then no frame-done), so keep them running;
 * only the frame fetch is stopped.
 */
static int exynos7_decon_sys_suspend(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);

	if (!ctx->bootloader_display)
		return pm_runtime_force_suspend(dev);
	if (ctx->i80_if && pm_runtime_active(dev))
		exynos7_decon_stop_output(ctx);
	return 0;
}

static int exynos7_decon_sys_resume(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);

	if (!ctx->bootloader_display)
		return pm_runtime_force_resume(dev);
	exynos7_decon_start_output(ctx);
	return 0;
}

static const struct dev_pm_ops exynos7_decon_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(exynos7_decon_sys_suspend, exynos7_decon_sys_resume)
	RUNTIME_PM_OPS(exynos7_decon_suspend, exynos7_decon_resume, NULL)
};

struct platform_driver decon_driver = {
	.probe		= decon_probe,
	.remove		= decon_remove,
	.driver		= {
		.name	= "exynos-decon",
		.pm	= pm_ptr(&exynos7_decon_pm_ops),
		.of_match_table = decon_driver_dt_match,
	},
};
