// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Exynos7420 MIPI Low Latency Interface (LLI) host for the SS333 modem.
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors: Yulgon Kim <yulgon.kim@samsung.com>,
 *          Byeongjo Park <bjo.park@samsung.com>
 *
 * 6.6 port of the stock tw70-stock mipi-lli.c, exynos-mipi-lli.c,
 * exynos7420-mipi-lli.c and exynos-mipi-lli-mphy.c, merged into one
 * platform driver. Register sequences, values and their order are stock.
 *
 * Changes from stock:
 *  - one DT node: LLI, remote LLI, FSYS0 sysreg, M-PHY and combo PHY PMA
 *    registers by name; the PMU LLI_PHY_CONTROL register through the PMU
 *    syscon; clocks by name, prepared once at probe;
 *  - the 8 MiB IPC carve-out comes from a reserved-memory region and is
 *    mapped write-combined with devm_ioremap_wc() (stock vmap()ed the
 *    struct pages of a memblock region);
 *  - the controller interrupt is fully threaded (stock did the SW-reset
 *    setting, including the PLL/CDR lock polls, in hard IRQ context);
 *  - both interrupts stay disabled until the first power-up has ungated the
 *    clocks, so a pending line never makes the CPU read a gated block;
 *  - no LPA notifier, CMU register dumps, debugfs, ion or 5430/5433 code.
 *
 * The consumer API (include/linux/mipi-lli.h) keeps the stock global calls:
 * the SoC has one LLI. Calls documented as atomic-safe there only touch
 * registers, spinlocks and the workqueue.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/mfd/syscon.h>
#include <linux/mipi-lli.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "exynos-mipi-lli.h"
#include "exynos-mipi-lli-mphy.h"
#include "exynos-mipi-lli-mphy-cal.h"

#define LLI_INTR_ALL			0x3FFFF
/* 10 ns: the LLI configuration clock is 100 MHz */
#define SYSTEM_CLOCK_PERIOD		10

/* Combo PHY PMA: AFC result, logged on every mount (stock) */
#define PMA_AFC_VALUE			(0x1F * 4)

static unsigned int credit_cnt = 10;
module_param(credit_cnt, uint, 0644);
MODULE_PARM_DESC(credit_cnt, "ms to wait for TX credits after a mount");

/* Stock enable order; disabled in reverse */
static const char * const lli_clk_names[] = {
	"aclk_xiu_modemx",
	"aclk_combo_phy_modem_pcs_pclk",
	"sclk_phy_fsys0",
	"sclk_combo_phy_modem_26m",
	"pclk_async_combo_phy_modem",
	"aclk_xiu_llisfrx",
	"aclk_axius_lli_be",
	"aclk_axius_lli_ll",
	"aclk_axius_llisfrx_llill",
	"aclk_axius_llisfrx_llibe",
	"aclk_lli_svc_loc",
	"aclk_lli_svc_rem",
	"aclk_lli_ll_init",
	"aclk_lli_be_init",
	"aclk_lli_ll_targ",
	"aclk_lli_be_targ",
	"user_phyclk_lli_tx0_symbol",
	"user_phyclk_lli_rx0_symbol",
};

struct exynos_lli {
	struct device *dev;

	void __iomem *regs;		/* local LLI */
	void __iomem *remote_regs;	/* CP-side LLI, reached over the link */
	void __iomem *sys_regs;		/* FSYS0 sysreg */
	void __iomem *phy_regs;		/* M-PHY attributes */
	void __iomem *pma_regs;		/* combo PHY PMA */
	struct regmap *pmu;
	u32 pmu_offset;			/* LLI_PHY_CONTROL */

	int irq;
	int irq_sig;
	bool irqs_enabled;		/* under lock */

	struct clk_bulk_data clks[ARRAY_SIZE(lli_clk_names)];
	bool clk_enabled;		/* written under lock */

	phys_addr_t shm_phys;
	unsigned long shm_size;
	void __iomem *shm;

	struct workqueue_struct *wq;
	struct work_struct sys_init_work;
	struct work_struct dump_work;

	/* Serialises power-up, power-down and clock gating */
	struct mutex lock;
	/* Guards is_suspended, resume_pending and irq_sig_active */
	spinlock_t slock;

	spinlock_t hd_lock;
	mipi_lli_handler_t handler;
	void *handler_data;

	atomic_t state;			/* enum mipi_lli_link_status */
	atomic_t init_start;		/* a power-up is queued or running */
	bool is_suspended;
	bool resume_pending;
	bool is_master;
	bool irq_sig_active;
	bool debug_possible;

	/* Board settings */
	bool scrambler;
	bool automode;
	bool shared_refclk;
	u32 mphy_mode;			/* OPMODE_* | GEAR_* | HS_RATE_* */

	unsigned int roe_cnt;
	unsigned int mnt_cnt;
	unsigned int mnt_fail_cnt;
	unsigned int pa_err_cnt;
};

static struct exynos_lli *g_lli;

static inline struct exynos_lli *lli_get(void)
{
	return smp_load_acquire(&g_lli);
}

static void lli_notify(struct exynos_lli *lli, enum mipi_lli_event event,
		       u32 intr)
{
	mipi_lli_handler_t handler;
	unsigned long flags;
	void *data;

	spin_lock_irqsave(&lli->hd_lock, flags);
	handler = lli->handler;
	data = lli->handler_data;
	spin_unlock_irqrestore(&lli->hd_lock, flags);

	if (handler)
		handler(data, event, intr);
}

/* IAL address remap: base >> log2(size), size a power of two, base aligned */
static u32 lli_calc_remap(phys_addr_t base, unsigned long size)
{
	unsigned int bit_pos;
	u64 remap;

	if (!size || !is_power_of_2(size) || (base & (size - 1)))
		return 0;

	bit_pos = __ffs(size);
	remap = ((u64)base >> bit_pos) << LLI_REMAP_BASE_ADDR_SHIFT;
	remap |= bit_pos;
	remap |= LLI_REMAP_ENABLE;

	return remap > U32_MAX ? 0 : (u32)remap;
}

/* --- M-PHY ---------------------------------------------------------------- */

static void lli_mphy_block_powerdown(struct exynos_lli *lli, bool down)
{
	u32 cmn = readl(lli->pma_regs + PHY_CMN_POWER_DOWN);

	if (!down) {
		/* TRSV block on, then CMN block on (bits 6..3 = 0) */
		writel(0x80, lli->pma_regs + PHY_TRSV_POWER_DOWN_LOW);
		writel(0x00, lli->pma_regs + PHY_TRSV_POWER_DOWN_HIGH);
		writel(cmn & 0xFFFFFF87, lli->pma_regs + PHY_CMN_POWER_DOWN);
	} else {
		writel(0xFF, lli->pma_regs + PHY_TRSV_POWER_DOWN_LOW);
		writel(0x3F, lli->pma_regs + PHY_TRSV_POWER_DOWN_HIGH);
		writel(cmn | 0x78, lli->pma_regs + PHY_CMN_POWER_DOWN);
	}
}

/* Stock combo M-PHY driver: init, cmn_init, ovtm_init, pma_init */
static void lli_mphy_setup(struct exynos_lli *lli)
{
	struct refclk_info refclk = {
		.freq = LLI_MPHY_REFERENCE_CLK_IN_26MHZ,
		.is_shared = lli->shared_refclk,
	};
	int ret;

	exynos_mphy_cal_init(lli->phy_regs, lli->mphy_mode);

	writel(0x1, lli->regs + EXYNOS_PA_MPHY_CMN_ENABLE);
	exynos_mphy_cal_cmn_init(lli->phy_regs, &refclk);
	writel(0x0, lli->regs + EXYNOS_PA_MPHY_CMN_ENABLE);

	writel(0x1, lli->regs + EXYNOS_PA_MPHY_OV_TM_ENABLE);
	exynos_mphy_cal_ovtm_init(lli->phy_regs);
	writel(0x0, lli->regs + EXYNOS_PA_MPHY_OV_TM_ENABLE);

	ret = exynos_mphy_cal_pma_init(lli->pma_regs, lli->phy_regs, &refclk,
				       lli->mphy_mode);
	if (ret)
		dev_err(lli->dev, "M-PHY PMA PLL/CDR lock timeout (%d)\n", ret);
}

/* --- controller ----------------------------------------------------------- */

static void lli_system_config(struct exynos_lli *lli)
{
	/* MODEM AXI demux: 0 = PCIe, 1 = LLI */
	writel(1, lli->sys_regs + FSYS0_AXI_SEL_DEMUX);
	writel(SIG_INT_MASK0, lli->sys_regs + FSYS0_LLI_SIG_INT_MASK0_1);
	writel(SIG_INT_MASK1, lli->sys_regs + FSYS0_LLI_SIG_INT_MASK2_3);
}

/* After SW_RESET_DONE: program PA, IAL remap and the M-PHY (stock order) */
static void lli_setting(struct exynos_lli *lli)
{
	u32 remap = lli_calc_remap(lli->shm_phys, lli->shm_size);

	atomic_set(&lli->state, LLI_WAITFORMOUNT);

	writel(SYSTEM_CLOCK_PERIOD, lli->regs + EXYNOS_PA_SYSTEM_CLK_PERIOD);
	writel(0xf, lli->regs + EXYNOS_PA_DRIVE_TACTIVATE_DURATION);

	/* probe rejected a carve-out that cannot be remapped */
	writel(remap, lli->regs + EXYNOS_IAL_LL_INIT_ADDR_REMAP);
	writel(remap, lli->regs + EXYNOS_IAL_BE_INIT_ADDR_REMAP);

	writel(LLI_INTR_ALL, lli->regs + EXYNOS_DME_LLI_INTR_ENABLE);

	writel(0, lli->regs + EXYNOS_DME_BE_TC_DISABLE);
	writel(0, lli->regs + EXYNOS_PA_DBG_PA_BREAK_POINT_ENABLE);
	writel(1, lli->regs + EXYNOS_PA_PHIT_ERR_COUNT_ENABLE);
	writel(1, lli->regs + EXYNOS_PA_PHIT_RECEIVE_COUNT_ENABLE);
	writel(1, lli->regs + EXYNOS_PA_TX_COUNT);
	writel(1, lli->regs + EXYNOS_PA_RX_COUNT);
	/* M-RX to PA latch: 2 = bypass */
	writel(2, lli->regs + EXYNOS_PA_PA_DBG_RX_LATCH_MODE);
	writel(0x40, lli->regs + EXYNOS_PA_NACK_RTT);
	writel(0x0, lli->regs + EXYNOS_PA_MK0_INSERTION_ENABLE);
	/* SS300/SS303/SS333 value */
	writel((128 << 0) | (15 << 8) | (1 << 12),
	       lli->regs + EXYNOS_PA_MK0_CONTROL);
	if (lli->scrambler)
		writel(1, lli->regs + EXYNOS_PA_USR_SCRAMBLER_ENABLE);

	lli_mphy_setup(lli);

	/* Apply the M-PHY standard attributes */
	writel(0xFFFFFFFF, lli->regs + EXYNOS_PA_CONFIG_UPDATE);

	writel((0x1F << 1) | 1, lli->regs + EXYNOS_IAL_LL_SNF_FIFO);
	writel((0x1F << 1) | 1, lli->regs + EXYNOS_IAL_BE_SNF_FIFO);
	writel(0x1000, lli->regs + EXYNOS_PA_WORSTCASE_RTT);
}

static void lli_sys_init_work(struct work_struct *work)
{
	struct exynos_lli *lli = container_of(work, struct exynos_lli,
					      sys_init_work);
	bool resume;
	int ret;

	mutex_lock(&lli->lock);

	regmap_write(lli->pmu, lli->pmu_offset, 1);	/* LLI_PHY_CONTROL */
	lli_system_config(lli);

	if (!lli->clk_enabled) {
		ret = clk_bulk_enable(ARRAY_SIZE(lli->clks), lli->clks);
		if (ret) {
			dev_err(lli->dev, "clock enable failed (%d)\n", ret);
			atomic_set(&lli->init_start, 0);
			mutex_unlock(&lli->lock);
			return;
		}
		WRITE_ONCE(lli->clk_enabled, true);
		dev_info(lli->dev, "clocks on\n");
	}

	lli_mphy_block_powerdown(lli, false);

	spin_lock_irq(&lli->slock);
	lli->is_suspended = false;
	resume = lli->resume_pending;
	lli->resume_pending = false;
	spin_unlock_irq(&lli->slock);

	writel(LLI_INTR_ALL, lli->regs + EXYNOS_DME_LLI_INTR_ENABLE);
	writel(1, lli->regs + EXYNOS_DME_LLI_RESET);

	/* First power-up: the block is clocked, reading it is safe now */
	if (!lli->irqs_enabled) {
		enable_irq(lli->irq);
		enable_irq(lli->irq_sig);
		lli->irqs_enabled = true;
	}

	mutex_unlock(&lli->lock);

	if (resume)
		lli_notify(lli, LLI_EVENT_RESUME, 0);
}

/* Atomic-safe: queue one power-up and LLI reset */
static void lli_init(struct exynos_lli *lli)
{
	if (atomic_cmpxchg(&lli->init_start, 0, 1) == 0)
		queue_work(lli->wq, &lli->sys_init_work);
}

static void lli_set_master(struct exynos_lli *lli, bool is_master)
{
	lli->is_master = is_master;
	writel(is_master << 1, lli->regs + EXYNOS_DME_CSA_SYSTEM_SET);
}

static int lli_link_startup_mount(struct exynos_lli *lli)
{
	u32 val;
	int ret;

	if (lli->is_master) {
		if (readl(lli->regs + EXYNOS_DME_CSA_SYSTEM_STATUS) &
		    CSA_LLI_MOUNTED)
			return 0;
	} else {
		ret = readl_poll_timeout_atomic(lli->regs +
						EXYNOS_DME_LLI_INTR_STATUS, val,
						val & INTR_MPHY_HIBERN8_EXIT_DONE,
						1, 1000);
		if (ret) {
			dev_err(lli->dev, "HIBERN8 exit failed\n");
			return ret;
		}
	}

	val = readl(lli->regs + EXYNOS_DME_CSA_SYSTEM_SET);
	writel(LLI_MOUNT_CTRL | val, lli->regs + EXYNOS_DME_CSA_SYSTEM_SET);
	return 0;
}

static int lli_send_signal(struct exynos_lli *lli, u32 cmd)
{
	u32 status = 0;

	if (atomic_read(&lli->state) == LLI_MOUNTED) {
		/* an unmount may be in progress: LLI_MOUNT_CTRL must be set */
		status = readl(lli->regs + EXYNOS_DME_CSA_SYSTEM_STATUS);
		if (status & LLI_MOUNT_CTRL) {
			writel(cmd, lli->remote_regs + EXYNOS_TL_SIGNAL_SET_LSB);
			return 0;
		}
	}

	dev_err_ratelimited(lli->dev, "send signal: not mounted (%x)\n",
			    status);
	return -EIO;
}

static void lli_reset_signal(struct exynos_lli *lli)
{
	writel(0xFFFFFFFF, lli->regs + EXYNOS_TL_SIGNAL_CLR_LSB);
	writel(0xFFFFFFFF, lli->regs + EXYNOS_TL_SIGNAL_CLR_MSB);
}

static u32 lli_read_signal(struct exynos_lli *lli)
{
	u32 lsb = readl(lli->regs + EXYNOS_TL_SIGNAL_STATUS_LSB);
	u32 msb = readl(lli->regs + EXYNOS_TL_SIGNAL_STATUS_MSB);

	if (lsb)
		writel(lsb, lli->regs + EXYNOS_TL_SIGNAL_CLR_LSB);
	if (msb)
		writel(msb, lli->regs + EXYNOS_TL_SIGNAL_CLR_MSB);

	dev_dbg(lli->dev, "signal LSB %x MSB %x\n", lsb, msb);
	return lsb;
}

/* May sleep: gates clocks and powers the PHY down */
static void lli_power_down(struct exynos_lli *lli)
{
	mutex_lock(&lli->lock);

	spin_lock_irq(&lli->slock);
	if (lli->is_suspended) {
		spin_unlock_irq(&lli->slock);
		mutex_unlock(&lli->lock);
		return;
	}
	spin_unlock_irq(&lli->slock);

	if (lli->clk_enabled) {
		writel(0x0, lli->regs + EXYNOS_DME_LLI_INTR_ENABLE);
		lli_reset_signal(lli);
		lli_mphy_block_powerdown(lli, true);
		clk_bulk_disable(ARRAY_SIZE(lli->clks), lli->clks);
		WRITE_ONCE(lli->clk_enabled, false);
	}
	regmap_write(lli->pmu, lli->pmu_offset, 0);

	spin_lock_irq(&lli->slock);
	lli->is_suspended = true;
	spin_unlock_irq(&lli->slock);
	lli->debug_possible = false;

	mutex_unlock(&lli->lock);

	lli_notify(lli, LLI_EVENT_SUSPEND, 0);
}

static void lli_dump_work(struct work_struct *work)
{
	struct exynos_lli *lli = container_of(work, struct exynos_lli,
					      dump_work);

	mutex_lock(&lli->lock);
	if (!lli->clk_enabled) {
		mutex_unlock(&lli->lock);
		dev_info(lli->dev, "dump: clocks off\n");
		return;
	}

	dev_info(lli->dev,
		 "dump: csa=%08x intr_en=%08x intr=%08x dme=%08x pa=%08x\n",
		 readl(lli->regs + EXYNOS_DME_CSA_SYSTEM_STATUS),
		 readl(lli->regs + EXYNOS_DME_LLI_INTR_ENABLE),
		 readl(lli->regs + EXYNOS_DME_LLI_INTR_STATUS),
		 readl(lli->regs + EXYNOS_DME_DBG_DME_STATE),
		 readl(lli->regs + EXYNOS_PA_CSA_PA_STATUS));
	dev_info(lli->dev,
		 "dump: tl=%08x dl=%08x pa=%08x ial=%08x/%08x credits=%08x\n",
		 readl(lli->regs + EXYNOS_DME_LLI_TL_INTR_REASON),
		 readl(lli->regs + EXYNOS_DME_LLI_DL_INTR_REASON),
		 readl(lli->regs + EXYNOS_DME_LLI_PA_INTR_REASON),
		 readl(lli->regs + EXYNOS_DME_LLI_IAL_INTR_REASON0),
		 readl(lli->regs + EXYNOS_DME_LLI_IAL_INTR_REASON1),
		 readl(lli->regs + EXYNOS_DL_DBG_TX_CREDTIS));
	dev_info(lli->dev,
		 "dump: phy tx=%x rx=%x pma pll=%x afc=%x; mount ok=%u fail=%u roe=%u pa_err=%u\n",
		 readl(lli->phy_regs + PHY_TX_FSM_STATE(0)),
		 readl(lli->phy_regs + PHY_RX_FSM_STATE(0)),
		 readl(lli->pma_regs + 0x1E * 4),
		 readl(lli->pma_regs + PMA_AFC_VALUE),
		 lli->mnt_cnt, lli->mnt_fail_cnt, lli->roe_cnt,
		 lli->pa_err_cnt);

	mutex_unlock(&lli->lock);
}

/* --- interrupts ----------------------------------------------------------- */

static void lli_handle_mount(struct exynos_lli *lli)
{
	u32 credit, i;

	credit = readl(lli->regs + EXYNOS_DL_DBG_TX_CREDTIS);
	dev_info(lli->dev, "mount: rx=%x tx=%x afc=%x status=%x pa_err=%u\n",
		 readl(lli->phy_regs + PHY_RX_FSM_STATE(0)),
		 readl(lli->phy_regs + PHY_TX_FSM_STATE(0)),
		 readl(lli->pma_regs + PMA_AFC_VALUE),
		 readl(lli->regs + EXYNOS_DME_CSA_SYSTEM_STATUS),
		 lli->pa_err_cnt);

	for (i = 0; !credit && i < credit_cnt; i++) {
		usleep_range(1000, 1100);
		credit = readl(lli->regs + EXYNOS_DL_DBG_TX_CREDTIS);
	}
	if (!credit) {
		dev_err(lli->dev, "mount failed, no TX credits after %u ms (ok %u fail %u roe %u)\n",
			i, lli->mnt_cnt, ++lli->mnt_fail_cnt, lli->roe_cnt);
		queue_work(lli->wq, &lli->dump_work);
		return;
	}

	atomic_set(&lli->state, LLI_MOUNTED);
	atomic_set(&lli->init_start, 0);
	lli_notify(lli, LLI_EVENT_MOUNTED, 0);

	dev_info(lli->dev, "mounted: ok %u fail %u roe %u pa_err %u\n",
		 ++lli->mnt_cnt, lli->mnt_fail_cnt, lli->roe_cnt,
		 lli->pa_err_cnt);
}

static void lli_handle_unmount(struct exynos_lli *lli)
{
	mutex_lock(&lli->lock);

	/* Leave the clocks on when a power-up is already queued */
	if (lli->clk_enabled && atomic_read(&lli->init_start) == 0) {
		clk_bulk_disable(ARRAY_SIZE(lli->clks), lli->clks);
		WRITE_ONCE(lli->clk_enabled, false);
		dev_info(lli->dev, "clocks gated\n");
	}

	lli_notify(lli, LLI_EVENT_UNMOUNTED, 0);
	dev_info(lli->dev, "unmounted\n");

	mutex_unlock(&lli->lock);
}

/*
 * Controller interrupt, threaded and oneshot: the line stays masked while
 * the stock hard-IRQ body runs here, so the M-PHY lock polls may sleep.
 */
static irqreturn_t lli_irq_thread(int irq, void *data)
{
	struct exynos_lli *lli = data;
	u32 status;

	status = readl(lli->regs + EXYNOS_DME_LLI_INTR_STATUS);
	if (!status)
		return IRQ_NONE;

	if (status & INTR_SW_RESET_DONE) {
		dev_info(lli->dev, "SW reset done\n");
		lli_setting(lli);
		lli->debug_possible = true;
		lli_notify(lli, LLI_EVENT_RESET, 0);

		if (status & INTR_RESET_ON_ERROR_DETECTED) {
			dev_err(lli->dev, "waiting for mount after line reset\n");
			atomic_set(&lli->state, LLI_WAITFORMOUNT);
			lli_notify(lli, LLI_EVENT_WAITFORMOUNT, 0);
		}
	}

	if (status & INTR_MPHY_HIBERN8_EXIT_DONE) {
		dev_info(lli->dev, "HIBERN8 exit: rx=%x tx=%x\n",
			 readl(lli->phy_regs + PHY_RX_FSM_STATE(0)),
			 readl(lli->phy_regs + PHY_TX_FSM_STATE(0)));
		usleep_range(1000, 1100);	/* stock mdelay(1) */
		writel(LLI_MOUNT_CTRL, lli->regs + EXYNOS_DME_CSA_SYSTEM_SET);
	}

	if (status & INTR_MPHY_HIBERN8_ENTER_DONE)
		dev_info(lli->dev, "HIBERN8 enter\n");

	if (status & INTR_PA_PLU_DETECTED)
		dev_info(lli->dev, "PLU detected\n");

	if (status & INTR_PA_PLU_DONE) {
		dev_info(lli->dev, "PLU done\n");
		if (lli->automode)
			writel(CSA_AUTO_MODE, lli->regs + EXYNOS_PA_CSA_PA_SET);
	}

	if (status & INTR_RESET_ON_ERROR_DETECTED)
		dev_err(lli->dev, "error detected, roe %u\n", ++lli->roe_cnt);

	if (status & INTR_RESET_ON_ERROR_SENT) {
		dev_err(lli->dev, "error sent, roe %u\n", ++lli->roe_cnt);
		/* stock: reset the LLI and leave the status to the reset */
		writel(1, lli->regs + EXYNOS_DME_LLI_RESET);
		return IRQ_HANDLED;
	}

	if (status & INTR_PA_ERROR_INDICATION) {
		dev_err_ratelimited(lli->dev, "PA reason %x\n",
				    readl(lli->regs +
					  EXYNOS_DME_LLI_PA_INTR_REASON));
		lli->pa_err_cnt++;
	}

	if (status & INTR_DL_ERROR_INDICATION)
		dev_err(lli->dev, "DL reason %x\n",
			readl(lli->regs + EXYNOS_DME_LLI_DL_INTR_REASON));

	if (status & INTR_TL_ERROR_INDICATION)
		dev_err(lli->dev, "TL reason %x\n",
			readl(lli->regs + EXYNOS_DME_LLI_TL_INTR_REASON));

	if (status & INTR_IAL_ERROR_INDICATION)
		dev_err(lli->dev, "IAL reason %x %x\n",
			readl(lli->regs + EXYNOS_DME_LLI_IAL_INTR_REASON0),
			readl(lli->regs + EXYNOS_DME_LLI_IAL_INTR_REASON1));

	if (status & INTR_LLI_MOUNT_DONE) {
		writel(status, lli->regs + EXYNOS_DME_LLI_INTR_STATUS);
		lli_handle_mount(lli);
		return IRQ_HANDLED;
	}

	if (status & INTR_LLI_UNMOUNT_DONE) {
		atomic_set(&lli->state, LLI_UNMOUNTED);
		writel(status, lli->regs + EXYNOS_DME_LLI_INTR_STATUS);
		lli->debug_possible = false;
		lli_handle_unmount(lli);
		return IRQ_HANDLED;
	}

	writel(status, lli->regs + EXYNOS_DME_LLI_INTR_STATUS);
	return IRQ_HANDLED;
}

/* CP-to-AP sideband signal: hard IRQ, as stock (feeds the modem IRQ path) */
static irqreturn_t lli_sig_irq(int irq, void *data)
{
	struct exynos_lli *lli = data;

	lli_notify(lli, LLI_EVENT_SIG, lli_read_signal(lli));
	return IRQ_HANDLED;
}

/* --- consumer API (include/linux/mipi-lli.h) ------------------------------ */

bool mipi_lli_ready(void)
{
	return lli_get() != NULL;
}
EXPORT_SYMBOL_GPL(mipi_lli_ready);

void __iomem *mipi_lli_request_sh_region(unsigned long sh_addr,
					 unsigned long size)
{
	struct exynos_lli *lli = lli_get();

	if (!lli)
		return NULL;
	if (sh_addr != lli->shm_phys || size > lli->shm_size) {
		dev_err(lli->dev, "bad shared region %#lx+%#lx\n", sh_addr, size);
		return NULL;
	}
	return lli->shm;
}
EXPORT_SYMBOL_GPL(mipi_lli_request_sh_region);

void mipi_lli_release_sh_region(void __iomem *rgn)
{
	/* devm-mapped for the life of the LLI device */
}
EXPORT_SYMBOL_GPL(mipi_lli_release_sh_region);

phys_addr_t mipi_lli_get_phys_base(void)
{
	struct exynos_lli *lli = lli_get();

	return lli ? lli->shm_phys : 0;
}
EXPORT_SYMBOL_GPL(mipi_lli_get_phys_base);

unsigned long mipi_lli_get_phys_size(void)
{
	struct exynos_lli *lli = lli_get();

	return lli ? lli->shm_size : 0;
}
EXPORT_SYMBOL_GPL(mipi_lli_get_phys_size);

int mipi_lli_get_link_status(void)
{
	struct exynos_lli *lli = lli_get();

	return lli ? atomic_read(&lli->state) : LLI_UNMOUNTED;
}
EXPORT_SYMBOL_GPL(mipi_lli_get_link_status);

int mipi_lli_set_link_status(int status)
{
	struct exynos_lli *lli = lli_get();

	if (!lli)
		return -ENODEV;

	atomic_set(&lli->state, status);
	if (status == LLI_WAITFORMOUNT)
		lli_notify(lli, LLI_EVENT_WAITFORMOUNT, 0);
	return 0;
}
EXPORT_SYMBOL_GPL(mipi_lli_set_link_status);

int mipi_lli_suspended(void)
{
	struct exynos_lli *lli = lli_get();

	return lli ? READ_ONCE(lli->is_suspended) : 1;
}
EXPORT_SYMBOL_GPL(mipi_lli_suspended);

int mipi_lli_register_handler(mipi_lli_handler_t handler, void *data)
{
	struct exynos_lli *lli = lli_get();
	unsigned long flags;

	if (!lli)
		return -ENODEV;
	if (!handler)
		return -EINVAL;

	spin_lock_irqsave(&lli->hd_lock, flags);
	lli->handler_data = data;
	lli->handler = handler;
	spin_unlock_irqrestore(&lli->hd_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(mipi_lli_register_handler);

/* May sleep: waits for running handlers so the consumer can free its data */
int mipi_lli_unregister_handler(mipi_lli_handler_t handler)
{
	struct exynos_lli *lli = lli_get();
	unsigned long flags;

	if (!lli)
		return -ENODEV;

	spin_lock_irqsave(&lli->hd_lock, flags);
	if (!handler || lli->handler != handler) {
		spin_unlock_irqrestore(&lli->hd_lock, flags);
		return -EINVAL;
	}
	lli->handler = NULL;
	lli->handler_data = NULL;
	spin_unlock_irqrestore(&lli->hd_lock, flags);

	synchronize_irq(lli->irq);
	synchronize_irq(lli->irq_sig);
	flush_workqueue(lli->wq);
	return 0;
}
EXPORT_SYMBOL_GPL(mipi_lli_unregister_handler);

void mipi_lli_send_interrupt(u32 cmd)
{
	struct exynos_lli *lli = lli_get();

	if (lli)
		lli_send_signal(lli, cmd);
}
EXPORT_SYMBOL_GPL(mipi_lli_send_interrupt);

void mipi_lli_reset_interrupt(void)
{
	struct exynos_lli *lli = lli_get();

	if (lli && READ_ONCE(lli->clk_enabled))
		lli_reset_signal(lli);
}
EXPORT_SYMBOL_GPL(mipi_lli_reset_interrupt);

u32 mipi_lli_read_interrupt(void)
{
	struct exynos_lli *lli = lli_get();

	if (!lli || !READ_ONCE(lli->clk_enabled))
		return 0;
	return lli_read_signal(lli);
}
EXPORT_SYMBOL_GPL(mipi_lli_read_interrupt);

void mipi_lli_debug_info(void)
{
	struct exynos_lli *lli = lli_get();

	if (lli && lli->debug_possible)
		queue_work(lli->wq, &lli->dump_work);
}
EXPORT_SYMBOL_GPL(mipi_lli_debug_info);

void mipi_lli_intr_enable(void)
{
	struct exynos_lli *lli = lli_get();

	if (!lli || READ_ONCE(lli->is_suspended))
		return;

	if (atomic_read(&lli->state) != LLI_UNMOUNTED &&
	    READ_ONCE(lli->clk_enabled) &&
	    readl(lli->regs + EXYNOS_DME_LLI_INTR_ENABLE) != LLI_INTR_ALL)
		writel(LLI_INTR_ALL, lli->regs + EXYNOS_DME_LLI_INTR_ENABLE);
}
EXPORT_SYMBOL_GPL(mipi_lli_intr_enable);

void mipi_lli_intr_disable(void)
{
	struct exynos_lli *lli = lli_get();

	if (!lli || READ_ONCE(lli->is_suspended))
		return;

	if (atomic_read(&lli->state) != LLI_UNMOUNTED &&
	    READ_ONCE(lli->clk_enabled))
		writel(0x0, lli->regs + EXYNOS_DME_LLI_INTR_ENABLE);
}
EXPORT_SYMBOL_GPL(mipi_lli_intr_disable);

void mipi_lli_reset(void)
{
	struct exynos_lli *lli = lli_get();

	if (lli)
		lli_init(lli);
}
EXPORT_SYMBOL_GPL(mipi_lli_reset);

void mipi_lli_reload(void)
{
	struct exynos_lli *lli = lli_get();

	if (lli)
		lli_init(lli);
}
EXPORT_SYMBOL_GPL(mipi_lli_reload);

void mipi_lli_suspend(void)
{
	struct exynos_lli *lli = lli_get();

	if (lli)
		lli_power_down(lli);
}
EXPORT_SYMBOL_GPL(mipi_lli_suspend);

void mipi_lli_resume(void)
{
	struct exynos_lli *lli = lli_get();
	unsigned long flags;

	if (!lli)
		return;

	spin_lock_irqsave(&lli->slock, flags);
	if (lli->is_suspended) {
		lli->resume_pending = true;
		queue_work(lli->wq, &lli->sys_init_work);
	}
	spin_unlock_irqrestore(&lli->slock, flags);
}
EXPORT_SYMBOL_GPL(mipi_lli_resume);

void mipi_lli_mask_sb_intr(bool mask)
{
	struct exynos_lli *lli = lli_get();
	unsigned long flags;

	if (!lli)
		return;

	spin_lock_irqsave(&lli->slock, flags);
	if (mask && lli->irq_sig_active) {
		writel(SIG_INT_MASK1, lli->sys_regs + FSYS0_LLI_SIG_INT_MASK0_1);
		lli->irq_sig_active = false;
	} else if (!mask && !lli->irq_sig_active) {
		writel(SIG_INT_MASK0, lli->sys_regs + FSYS0_LLI_SIG_INT_MASK0_1);
		/* stock never set this back, so a second mask was lost */
		lli->irq_sig_active = true;
	}
	spin_unlock_irqrestore(&lli->slock, flags);
}
EXPORT_SYMBOL_GPL(mipi_lli_mask_sb_intr);

/* --- sysfs: lli_control (stock commands) ---------------------------------- */

static ssize_t lli_control_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct exynos_lli *lli = dev_get_drvdata(dev);

	return sysfs_emit(buf, "MIPI-LLI %x\n", atomic_read(&lli->state));
}

static void lli_dump_shm(struct exynos_lli *lli, unsigned long offset)
{
	u8 *tmp = kmalloc(512, GFP_KERNEL);

	if (!tmp)
		return;
	memcpy_fromio(tmp, lli->shm + offset, 512);
	print_hex_dump(KERN_INFO, "llimem: ", DUMP_PREFIX_OFFSET, 16, 1,
		       tmp, 512, true);
	kfree(tmp);
}

/*
 * 0 debug dump, 1 power up and reset, 2 set master, 3 start-up mount,
 * 4 power down (stock "exit" had no implementation), 5 signal test,
 * 6 loopback test (stock no-op), 98/99 dump IPC memory +1K / +1K+512.
 * Commands that touch the controller need its clocks on.
 */
static ssize_t lli_control_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct exynos_lli *lli = dev_get_drvdata(dev);
	int cmd, ret = 0, i;

	if (kstrtoint(buf, 10, &cmd))
		return -EINVAL;

	switch (cmd) {
	case 0:
		queue_work(lli->wq, &lli->dump_work);
		break;
	case 1:
		lli_init(lli);
		break;
	case 2:
	case 3:
	case 5:
		mutex_lock(&lli->lock);
		if (!lli->clk_enabled) {
			ret = -EIO;
		} else if (cmd == 2) {
			lli_set_master(lli, true);
		} else if (cmd == 3) {
			ret = lli_link_startup_mount(lli);
		} else {
			for (i = 0; i < 32; i++)
				lli_send_signal(lli, BIT(i));
		}
		mutex_unlock(&lli->lock);
		break;
	case 4:
		lli_power_down(lli);
		break;
	case 6:
		break;
	case 98:
		lli_dump_shm(lli, SZ_1K);
		break;
	case 99:
		lli_dump_shm(lli, SZ_1K + 512);
		break;
	default:
		dev_err(dev, "unsupported control command %d\n", cmd);
		ret = -EINVAL;
	}

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(lli_control);

static struct attribute *lli_attrs[] = {
	&dev_attr_lli_control.attr,
	NULL
};
ATTRIBUTE_GROUPS(lli);

/* --- probe ---------------------------------------------------------------- */

static int lli_parse_dt(struct exynos_lli *lli)
{
	struct device_node *np = lli->dev->of_node;
	u32 gear[3];

	lli->scrambler = of_property_read_bool(np, "samsung,lli-scrambler");
	lli->automode = of_property_read_bool(np, "samsung,lli-automode");
	lli->shared_refclk = of_property_read_bool(np,
						   "samsung,mphy-shared-refclk");

	/* <opmode gear series>: opmode 2 = HS, series 2 = rate B (stock) */
	if (of_property_read_u32_array(np, "samsung,mphy-init-gear", gear, 3)) {
		lli->mphy_mode = OPMODE_PWM | GEAR_1 | HS_RATE_A;
	} else {
		lli->mphy_mode = gear[0] == 2 ? OPMODE_HS : OPMODE_PWM;
		lli->mphy_mode |= (gear[1] > 0 && gear[1] < 8) ? gear[1] : GEAR_1;
		lli->mphy_mode |= gear[2] == 2 ? HS_RATE_B : HS_RATE_A;
	}

	dev_info(lli->dev, "M-PHY %s-G%u%s, shared refclk %d, scrambler %d, automode %d\n",
		 lli->mphy_mode & OPMODE_HS ? "HS" : "PWM",
		 lli->mphy_mode & 0x7, lli->mphy_mode & HS_RATE_B ? "B" : "A",
		 lli->shared_refclk, lli->scrambler, lli->automode);
	return 0;
}

static int lli_map_shm(struct exynos_lli *lli)
{
	struct device *dev = lli->dev;
	struct reserved_mem *rmem;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return dev_err_probe(dev, -ENODEV, "no memory-region\n");
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return dev_err_probe(dev, -ENODEV, "memory-region not reserved\n");

	if (!lli_calc_remap(rmem->base, rmem->size))
		return dev_err_probe(dev, -EINVAL,
				     "IPC region %pa+%pa: size must be a power of two and the base aligned to it\n",
				     &rmem->base, &rmem->size);

	lli->shm_phys = rmem->base;
	lli->shm_size = rmem->size;
	lli->shm = devm_ioremap_wc(dev, rmem->base, rmem->size);
	if (!lli->shm)
		return -ENOMEM;

	dev_info(dev, "IPC memory %pa, %lu KiB\n", &lli->shm_phys,
		 lli->shm_size >> 10);
	return 0;
}

static void lli_clk_unprepare(void *data)
{
	struct exynos_lli *lli = data;

	clk_bulk_unprepare(ARRAY_SIZE(lli->clks), lli->clks);
}

static void lli_destroy_wq(void *data)
{
	struct exynos_lli *lli = data;

	cancel_work_sync(&lli->sys_init_work);
	cancel_work_sync(&lli->dump_work);
	destroy_workqueue(lli->wq);
}

static int exynos_lli_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_lli *lli;
	int i, ret;

	if (lli_get())
		return -EBUSY;

	lli = devm_kzalloc(dev, sizeof(*lli), GFP_KERNEL);
	if (!lli)
		return -ENOMEM;
	lli->dev = dev;

	lli->regs = devm_platform_ioremap_resource_byname(pdev, "lli");
	if (IS_ERR(lli->regs))
		return PTR_ERR(lli->regs);
	lli->remote_regs = devm_platform_ioremap_resource_byname(pdev, "remote");
	if (IS_ERR(lli->remote_regs))
		return PTR_ERR(lli->remote_regs);
	lli->sys_regs = devm_platform_ioremap_resource_byname(pdev, "sysreg");
	if (IS_ERR(lli->sys_regs))
		return PTR_ERR(lli->sys_regs);
	lli->phy_regs = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(lli->phy_regs))
		return PTR_ERR(lli->phy_regs);
	lli->pma_regs = devm_platform_ioremap_resource_byname(pdev, "pma");
	if (IS_ERR(lli->pma_regs))
		return PTR_ERR(lli->pma_regs);

	lli->pmu = syscon_regmap_lookup_by_phandle_args(dev->of_node,
							"samsung,pmu-syscon",
							1, &lli->pmu_offset);
	if (IS_ERR(lli->pmu))
		return dev_err_probe(dev, PTR_ERR(lli->pmu), "PMU syscon\n");

	lli->irq = platform_get_irq_byname(pdev, "lli");
	if (lli->irq < 0)
		return lli->irq;
	lli->irq_sig = platform_get_irq_byname(pdev, "sig");
	if (lli->irq_sig < 0)
		return lli->irq_sig;

	for (i = 0; i < ARRAY_SIZE(lli_clk_names); i++)
		lli->clks[i].id = lli_clk_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(lli->clks), lli->clks);
	if (ret)
		return dev_err_probe(dev, ret, "clocks\n");
	ret = clk_bulk_prepare(ARRAY_SIZE(lli->clks), lli->clks);
	if (ret)
		return dev_err_probe(dev, ret, "clock prepare\n");
	ret = devm_add_action_or_reset(dev, lli_clk_unprepare, lli);
	if (ret)
		return ret;

	ret = lli_map_shm(lli);
	if (ret)
		return ret;

	lli_parse_dt(lli);

	mutex_init(&lli->lock);
	spin_lock_init(&lli->slock);
	spin_lock_init(&lli->hd_lock);
	INIT_WORK(&lli->sys_init_work, lli_sys_init_work);
	INIT_WORK(&lli->dump_work, lli_dump_work);
	atomic_set(&lli->state, LLI_UNMOUNTED);
	atomic_set(&lli->init_start, 0);
	lli->is_suspended = true;
	lli->irq_sig_active = true;

	lli->wq = alloc_workqueue("mipi_lli", WQ_UNBOUND | WQ_HIGHPRI, 1);
	if (!lli->wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, lli_destroy_wq, lli);
	if (ret)
		return ret;

	/* Both lines are enabled by the first power-up, with clocks on */
	ret = devm_request_threaded_irq(dev, lli->irq, NULL, lli_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"mipi-lli", lli);
	if (ret)
		return dev_err_probe(dev, ret, "LLI IRQ\n");
	ret = devm_request_irq(dev, lli->irq_sig, lli_sig_irq, IRQF_NO_AUTOEN,
			       "mipi-lli-sig", lli);
	if (ret)
		return dev_err_probe(dev, ret, "signal IRQ\n");

	/* Stock wrote the AXI demux and signal masks at probe too */
	lli_system_config(lli);

	platform_set_drvdata(pdev, lli);
	smp_store_release(&g_lli, lli);

	dev_info(dev, "MIPI-LLI ready, link down until the modem powers it up\n");
	return 0;
}

static const struct of_device_id exynos_lli_of_match[] = {
	{ .compatible = "samsung,exynos7420-mipi-lli" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_lli_of_match);

static struct platform_driver exynos_lli_driver = {
	.probe = exynos_lli_probe,
	.driver = {
		.name = "exynos-mipi-lli",
		.of_match_table = exynos_lli_of_match,
		.dev_groups = lli_groups,
		/* consumers hold the global API and the IPC mapping */
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(exynos_lli_driver);

MODULE_DESCRIPTION("Exynos7420 MIPI-LLI driver");
MODULE_AUTHOR("Yulgon Kim <yulgon.kim@samsung.com>");
MODULE_LICENSE("GPL");
