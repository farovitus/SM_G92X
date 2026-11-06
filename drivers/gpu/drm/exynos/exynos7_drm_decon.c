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
#include <linux/interconnect.h>
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

static int decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	u32 val;

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
	u32 val;

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

static void decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	int i;

	for (i = 0; i < WINDOWS_NR; i++)
		decon_shadow_protect_win(ctx, i, false);

	if (ctx->i80_if && ctx->icc_bw)
		mod_delayed_work(system_wq, &ctx->bw_idle_work,
				 msecs_to_jiffies(DECON_BW_IDLE_MS));

	/*
	 * Arm the event before the transfer starts: in i80 mode the frame
	 * done interrupt of this transfer is the vblank that completes it.
	 */
	exynos_crtc_handle_event(crtc);

	if (ctx->i80_if)
		decon_i80_trigger(ctx);
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
}

static void decon_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	int i;

	/*
	 * We need to make sure that all windows are disabled before we
	 * suspend that connector. Otherwise we might try to scan from
	 * a destroyed buffer later.
	 */
	for (i = 0; i < WINDOWS_NR; i++)
		decon_disable_plane(crtc, &ctx->planes[i]);

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

	/* in i80 mode the end of a frame transfer stands in for vsync */
	drm_crtc_handle_vblank(&ctx->crtc->base);

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
 * With "samsung,bootloader-display" the DSI link and panel are only set up
 * by the bootloader. Gating DECON's clocks through system sleep stalled the
 * panel after resume (one frame, then no frame-done), so keep them running.
 */
static int exynos7_decon_sys_suspend(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);

	return ctx->bootloader_display ? 0 : pm_runtime_force_suspend(dev);
}

static int exynos7_decon_sys_resume(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);

	return ctx->bootloader_display ? 0 : pm_runtime_force_resume(dev);
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
