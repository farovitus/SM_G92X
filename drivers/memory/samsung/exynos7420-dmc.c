// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos7420 DRAM controller (DREX + LPDDR4 PHY) frequency switching.
 *
 * The 7420 has no firmware call for DRAM DVFS; the stock kernel switched
 * it from Linux and this driver does the same. The DRAM clock (MIF) is
 * half of mout_aclk_mif_pll, normally fed by fout_mif_pll. To change the
 * rate, the controller moves to the BUS0 PLL path ("safe PLL") on timing
 * set 1 while fout_mif_pll is reprogrammed, then returns on timing set 0.
 * Each move goes through the CMU pause handshake, which stalls DRAM
 * traffic while the clock changes, and the VT monitor issues the MR13
 * write that flips the LPDDR4 frequency set point at the same time.
 *
 * The register sequence, timing tables (LPDDR4 Samsung 6Gb die, DQS
 * mode 2) and per-level CCORE and HPM clocks are those of the stock
 * 3.10 driver, exynos7420_bus_mif.c.
 *
 * LV0..LV4 switch over a 1600 MHz safe PLL (DRAM 800 MHz, LV4 timings);
 * any move that involves LV5..LV12 uses 800 MHz (DRAM 400 MHz, LV7
 * timings). BUCK1 (vdd_mif) follows the OPP table: raised before the
 * clock goes up, lowered after it comes down, set to the target on the
 * safe PLL before the target clock (so DQS gate retraining runs at the
 * new voltage, as stock did for differential DQS), with PHY clock gating
 * off around each change. The 800 MHz safe level gets 56.25 mV more
 * (stock CMOS_SE_OFFSET, applied in DQS mode 2).
 *
 * Demand: a devfreq device with simple_ondemand on the DREX PPMUs (CPU
 * and general port of each channel; the busiest channel counts), plus
 * minimum rates from interconnect votes. The driver is an interconnect
 * provider with one node: CPU OPPs carry the stock cpufreq->MIF floors as
 * opp-peak-kBps, and the display bus links to it. The GPU floor follows
 * the GPU devfreq's transition notifier and the stock G3D table
 * (samsung,gpu-mif-floors), not an interconnect path: a devfreq target()
 * runs under dev_pm_qos_mtx when a PM QoS limit (thermal) moves it, and
 * the exynos-bus interconnect provider takes that mutex under icc_lock,
 * so a GPU OPP with bandwidth deadlocks against the display's votes. For
 * the same reason the floor is applied inside this driver, never through
 * dev_pm_qos. A vote converts at 16
 * bytes per DRAM clock (four 16-bit LPDDR4 channels, double data rate).
 * The devfreq userspace governor switches by hand; debugfs
 * exynos7420-dmc/state shows the registers.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>

#define NR_CH			4

/* CMU_TOPC */
#define TOPC_DIV_STAT_TOPC0	0x0700
#define TOPC_DIV_CCORE_532_BUSY	BIT(0)
#define TOPC_DIV_CCORE_133_BUSY	BIT(4)
#define TOPC_MUX_STAT_TOPC5	0x0414
#define TOPC_MIF_MUX_BUSY	GENMASK(27, 24)
#define TOPC_PAUSE		0x1008
#define TOPC_PAUSE_BUSY		GENMASK(18, 16)

/* CMU_MIFn */
#define MIF_DIV_MIF		0x0600
#define MIF_DIV_SCLK_HPM	GENMASK(6, 4)
#define MIF_DIV_STAT_MIF	0x0700
#define MIF_DIV_STAT_HPM	BIT(4)
#define MIF_PAUSE		0x1000
#define MIF_PAUSE_BUSY		GENMASK(18, 16)
#define MIF_MUX_CTRL_BY_CMUC	BIT(20)

/* DREX */
#define DREX_MEMCONTROL		0x0004
#define DREX_PB_REF_EN		BIT(27)
#define DREX_DSREF_EN		BIT(5)
#define DREX_CGCONTROL		0x0008
#define DREX_PHY_CG_EN		BIT(4)
#define DREX_DIRECTCMD		0x0010
#define DREX_TIMINGRFCPB	0x0020
#define DREX_PWRDNCONFIG	0x0028
#define DREX_DSREF_CYC		GENMASK(31, 16)
#define DREX_TIMINGROW_0	0x0034
#define DREX_TIMINGDATA_0	0x0038
#define DREX_TIMINGPOWER_0	0x003c
#define DREX_RDFETCH_0		0x004c
#define DREX_RDFETCH_1		0x0050
#define DREX_ETCCONTROL_0	0x0058
#define DREX_ETCCONTROL_1	0x005c
#define DREX_TIMINGROW_1	0x00e4
#define DREX_TIMINGDATA_1	0x00e8
#define DREX_TIMINGPOWER_1	0x00ec
#define DREX_TRAIN_TIMING_0	0x0410
#define DREX_TRAIN_TIMING_1	0x0414
#define DREX_HW_PTRAIN_PERIOD_0	0x0420
#define DREX_HW_PTRAIN_PERIOD_1	0x0424
#define DREX_HWPR_TRAIN_CONFIG_0 0x0440
#define DREX_HWPR_TRAIN_CONFIG_1 0x0444
#define DREX_HWPR_TRAIN_CONTROL_0 0x0450
#define DREX_HWPR_TRAIN_CONTROL_1 0x0454

/* LPDDR4 PHY */
#define PHY_DVFS_CON(n)		(0x00b8 + 4 * (n))
#define PHY_ZQ_CON9		0x03ec

/* VT monitor */
#define VTMON_CON		0x0014
#define VTMON_AREF_UPDATE_EN	BIT(28)
#define VTMON_PER_MRS_EN	BIT(17)
#define VTMON_MIF_DREX_WDATA0	0x001c
#define VTMON_DREX_TIMING_SW_SET 0x003c
#define VTMON_PER_UPDATE_STATUS	0x0064
#define VTMON_CNT_LIMIT_SET0_SW0 0x0080
#define VTMON_CNT_LIMIT_SET1_SW0 0x0084
#define VTMON_CNT_LIMIT_SET0_SW1 0x0088
#define VTMON_CNT_LIMIT_SET1_SW1 0x008c

#define POLL_TIMEOUT_US		1000

/* Safe levels: 1600 MHz PLL for LV0..LV4 moves, 800 MHz for the rest */
#define LV_SAFE_HIGH		4
#define LV_SAFE_LOW		7
#define SAFE_LOW_OFFSET_UV	56250
#define NR_LEVELS		13

/* Bytes per DRAM clock over all channels, to turn kB/s votes into kHz */
#define BYTES_PER_CLK		16
/* icc node id; exynos-bus icc nodes use small platform device ids */
#define DMC_ICC_ID		7420
#define FLOOR_HOLD_MS		50
#define MAX_EDEV		(2 * NR_CH)

struct drex_timing {
	u32 rfcpb, row, data, power, rdfetch, etc;
	u32 train_timing, hw_ptrain_period, hwpr_config, hwpr_control;
	u32 mr13, mr2, mr22, mr11;
};

#define DREX_T(rfc, row, data, pwr, fetch, ttrain, period, cfg, ctl, mr2v,	\
	       mr22v, mr11v)							\
	{ rfc, row, data, pwr, fetch, 0, ttrain, period, cfg, ctl,		\
	  0x00211400, 0x00200800 | ((mr2v) << 2),				\
	  0x00221800 | ((mr22v) << 2), 0x00210c00 | ((mr11v) << 2) }

/* LPDDR4 Samsung 6Gb die, DQS mode 2, LV0..LV12 */
static const struct drex_timing drex_6gb[NR_LEVELS] = {
	DREX_T(0x2323, 0x46588652, 0x4732bbe0, 0x404a0336, 0x4e03, 0x319ba14a,
	       0x5dc0, 0x01010101, 3, 0x2d, 0x26, 0x04),
	DREX_T(0x2121, 0x42588611, 0x4732bbe0, 0x3c460336, 0x4a03, 0x319ba14a,
	       0x5dc0, 0x01010101, 3, 0x2d, 0x26, 0x04),
	DREX_T(0x1d1d, 0x3947754f, 0x4632b3dc, 0x343d0335, 0x4003, 0x319ba13a,
	       0x5dc0, 0x01010101, 3, 0x24, 0x26, 0x04),
	DREX_T(0x1919, 0x3146648c, 0x3532b39c, 0x2c340335, 0x3602, 0x319ba13a,
	       0x5dc0, 0x01010101, 3, 0x24, 0x26, 0x04),
	DREX_T(0x1313, 0x2635538a, 0x3422aad6, 0x24290234, 0x2a02, 0x219ba12a,
	       0xffffff, 0, 0, 0x1b, 0x26, 0x04),
	DREX_T(0x0f0f, 0x1d2442c8, 0x2322a310, 0x1c1f0233, 0x2002, 0x019ba10b,
	       0xffffff, 0, 0, 0x12, 0x26, 0x04),
	DREX_T(0x0d0d, 0x19233247, 0x23229acc, 0x181f0233, 0x1c02, 0x019ba10a,
	       0xffffff, 0, 0, 0x09, 0x26, 0x04),
	DREX_T(0x0a0a, 0x192331c5, 0x22229acc, 0x141f0233, 0x1502, 0x019ba0fa,
	       0xffffff, 0, 0, 0x09, 0x24, 0x00),
	DREX_T(0x0808, 0x19232185, 0x22229acc, 0x101f0233, 0x1202, 0x019ba0fa,
	       0xffffff, 0, 0, 0x09, 0x24, 0x00),
	DREX_T(0x0606, 0x19232144, 0x22229286, 0x0c1f0233, 0x0d02, 0x019ba0ea,
	       0xffffff, 0, 0, 0x00, 0x24, 0x00),
	DREX_T(0x0404, 0x192320c3, 0x22229286, 0x081f0233, 0x0902, 0x019ba0ea,
	       0xffffff, 0, 0, 0x00, 0x24, 0x00),
	DREX_T(0x0303, 0x192320c2, 0x22229286, 0x081f0233, 0x0702, 0x019ba0ea,
	       0xffffff, 0, 0, 0x00, 0x24, 0x00),
	DREX_T(0x0303, 0x192320c2, 0x22229286, 0x081f0233, 0x0502, 0x019ba0ea,
	       0xffffff, 0, 0, 0x00, 0x24, 0x00),
};

/* PHY_DVFS_CON0..5 field masks for each timing set */
static const u32 phy_mask[2][6] = {
	{	/* set 0 */
		BIT(30) | BIT(28) | (3 << 24) | BIT(22) | BIT(20) | BIT(18) |
			BIT(16) | 0xff,
		BIT(30) | (0x3f << 16) | (0xf << 4) | 0xf,
		0x7f << 18,
		BIT(30) | BIT(28) | BIT(26) | 0xfff,
		(0x3f << 6) | 0x3f,
		BIT(30) | (0xf << 16) | (0xf << 8) | 0xf,
	}, {	/* set 1 */
		BIT(31) | BIT(29) | (3 << 24) | BIT(23) | BIT(21) | BIT(19) |
			BIT(17) | (0xff << 8),
		BIT(31) | (0x3f << 24) | (0xf << 12) | (0xf << 8),
		0x7f << 25,
		BIT(31) | BIT(29) | BIT(27) | (0xfff << 12),
		(0x3f << 18) | (0x3f << 12),
		BIT(31) | (0xf << 20) | (0xf << 12) | (0xf << 4),
	},
};

/* PHY_DVFS_CON0..5 values, [level][set][con] */
static const u32 phy_6gb[NR_LEVELS][2][6] = {
	{ { 0x40100000, 0x002000b0, 0x00400000, 0x0400060e, 0, 0x40070000 },
	  { 0x80200000, 0x2000b000, 0x20000000, 0x0860e000, 0, 0x80700000 } },
	{ { 0x41100000, 0x002000b0, 0x00400000, 0x040005b8, 0, 0x40070000 },
	  { 0x82200000, 0x2000b000, 0x20000000, 0x085b8000, 0, 0x80700000 } },
	{ { 0x41100000, 0x001c00b0, 0x00400000, 0x040004f0, 0, 0x40070000 },
	  { 0x82200000, 0x1c00b000, 0x20000000, 0x084f0000, 0, 0x80700000 } },
	{ { 0x41100000, 0x001c00b0, 0x00400000, 0x0400042c, 0, 0x40070000 },
	  { 0x82200000, 0x1c00b000, 0x20000000, 0x0842c000, 0, 0x80700000 } },
	{ { 0x41100000, 0x00160040, 0x00400000, 0x00000344, 0, 0x00050100 },
	  { 0x82200000, 0x16004000, 0x20000000, 0x00344000, 0, 0x00501000 } },
	{ { 0x41100000, 0x00100040, 0x00400000, 0x0000027b, 0, 0x00050100 },
	  { 0x82200000, 0x10004000, 0x20000000, 0x0027b000, 0, 0x00501000 } },
	{ { 0x41100000, 0x000c0040, 0x00400000, 0x0000021f, 0, 0x00050100 },
	  { 0x82200000, 0x0c004000, 0x20000000, 0x0021f000, 0, 0x00501000 } },
	{ { 0x41500000, 0x400c0040, 0x00400000, 0x000001a2, 0, 0x0005010f },
	  { 0x82200000, 0x0c004000, 0x20000000, 0x001a2000, 0, 0x00501000 } },
	{ { 0x41540000, 0x400c0040, 0x00400000, 0x0000015c, 0, 0x0005010f },
	  { 0x82a80000, 0x8c004000, 0x20000000, 0x0015c000, 0, 0x005010f0 } },
	{ { 0x41540000, 0x40060040, 0x00400000, 0x00000103, 0, 0x0005010f },
	  { 0x82a80000, 0x86004000, 0x20000000, 0x00103000, 0, 0x005010f0 } },
	{ { 0x41400030, 0x40060040, 0x00400000, 0x000000a7, 0, 0x0005010f },
	  { 0x82803000, 0x86004000, 0x20000000, 0x000a7000, 0, 0x005010f0 } },
	{ { 0x41400030, 0x40060040, 0x00400000, 0x00000085, 0, 0x0005010f },
	  { 0x82803000, 0x86004000, 0x20000000, 0x00085000, 0, 0x005010f0 } },
	{ { 0x41400030, 0x40060040, 0x00400000, 0x00000064, 0, 0x0005010f },
	  { 0x82803000, 0x86004000, 0x20000000, 0x00064000, 0, 0x005010f0 } },
};

/* VT monitor count limits, the same for every level */
#define VTMON_LIMIT_SET0	0x00480060
#define VTMON_LIMIT_SET1	0x000003e8

struct mif_level {
	unsigned long khz;		/* DRAM clock; fout_mif_pll is twice */
	u8 ccore_532_parent;		/* index in mout_aclk_ccore_532 */
	unsigned long ccore_532_hz;
	u8 hpm_div;			/* DIV_MIF SCLK_HPM field */
	unsigned long ccore_133_hz;
};

/* mout_aclk_ccore_532 parents: 2 = CC PLL (532), 5 = BUS1 PLL / 2 (334) */
static const struct mif_level levels[NR_LEVELS] = {
	{ 1552000, 2, 532000000, 2, 80000000 },
	{ 1464000, 2, 532000000, 2, 80000000 },
	{ 1264000, 3, 468000000, 1, 80000000 },
	{ 1026000, 0, 400000000, 1, 80000000 },
	{  836000, 5, 334000000, 1, 80000000 },
	{  635000, 5, 334000000, 1, 67000000 },
	{  543000, 6, 266000000, 1, 50000000 },
	{  418000, 4, 200000000, 0, 50000000 },
	{  348000, 9, 167000000, 0, 50000000 },
	{  259000, 9, 167000000, 0, 50000000 },
	{  167000, 10, 133000000, 0, 50000000 },
	{  133000, 8, 100000000, 0, 50000000 },
	{  100000, 8, 100000000, 0, 50000000 },
};

struct exynos7420_dmc {
	struct device *dev;
	void __iomem *drex[NR_CH];
	void __iomem *phy[NR_CH];
	void __iomem *vtmon[NR_CH];
	void __iomem *cmu_mif[NR_CH];
	void __iomem *topc;

	struct clk *fout_mif_pll;
	struct clk *mout_aclk_mif_pll;
	struct clk *mout_topc_mif_pll;
	struct clk *sclk_bus0_pll_mif;
	struct clk *mout_sclk_bus0_pll_mif;
	struct clk *dout_sclk_bus0_pll_mif;
	struct clk *mout_aclk_ccore_532;
	struct clk *dout_aclk_ccore_532;
	struct clk *dout_aclk_ccore_133;

	struct regulator *vdd;
	int uv[NR_LEVELS];		/* BUCK1 voltage per level, from the OPPs */
	int cur_uv;

	struct mutex lock;
	int cur;			/* current level, -1 if unknown */
	bool can_switch;
	unsigned int nr_switch;
	s64 last_us, max_us;		/* switch_level() time */
	struct dentry *debugfs;

	struct devfreq *df;
	struct devfreq_simple_ondemand_data ondemand;
	struct devfreq_event_dev *edev[MAX_EDEV];	/* channel n: n, n + 4 */
	int nr_edev;
	u32 saturation;			/* percent of PPMU cycles = full load */
	u32 last_load[NR_CH];		/* ppm of PPMU cycles, last sample */

	struct icc_provider icc;
	struct icc_node *icc_node;
	u32 icc_khz;			/* floor from interconnect votes */
	u32 gpu_khz;			/* floor from the GPU clock */
	u32 floor_khz;			/* floor dmc_target() applies */
	struct mutex floor_lock;
	struct delayed_work floor_work;	/* applies a lower floor late */

	struct devfreq *gpu_df;
	struct notifier_block gpu_nb;
	struct delayed_work gpu_work;	/* finds the GPU devfreq */
	int gpu_tries;
	u32 gpu_floor[NR_LEVELS][2];	/* GPU kHz, MIF kHz; ascending */
	int nr_gpu_floor;
};

/* /sys/module/exynos7420_dmc/parameters/trace */
static bool trace;
module_param(trace, bool, 0644);
MODULE_PARM_DESC(trace, "log every level switch");

static int wait_clear(void __iomem *reg, u32 mask, const char *what,
		      struct device *dev)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(reg, val, !(val & mask), 1,
					POLL_TIMEOUT_US);
	if (ret)
		dev_err(dev, "timeout waiting for %s (0x%08x)\n", what, val);
	return ret;
}

static int wait_pause(struct exynos7420_dmc *dmc)
{
	int ch, ret;

	ret = wait_clear(dmc->topc + TOPC_PAUSE, TOPC_PAUSE_BUSY,
			 "CMU_TOPC pause", dmc->dev);
	for (ch = 0; ch < NR_CH && !ret; ch++)
		ret = wait_clear(dmc->cmu_mif[ch] + MIF_PAUSE, MIF_PAUSE_BUSY,
				 "CMU_MIF pause", dmc->dev);
	if (!ret)
		ret = wait_clear(dmc->topc + TOPC_MUX_STAT_TOPC5,
				 TOPC_MIF_MUX_BUSY, "MIF PLL mux", dmc->dev);
	return ret;
}

static bool mux_ctrl_by_cmuc(struct exynos7420_dmc *dmc)
{
	int ch;

	for (ch = 0; ch < NR_CH; ch++)
		if (!(readl(dmc->cmu_mif[ch] + MIF_PAUSE) & MIF_MUX_CTRL_BY_CMUC))
			return false;
	return true;
}

static void set_hpm_div(struct exynos7420_dmc *dmc, int lv)
{
	int ch;
	u32 v;

	for (ch = 0; ch < NR_CH; ch++) {
		v = readl(dmc->cmu_mif[ch] + MIF_DIV_MIF);
		v &= ~MIF_DIV_SCLK_HPM;
		v |= FIELD_PREP(MIF_DIV_SCLK_HPM, levels[lv].hpm_div);
		writel(v, dmc->cmu_mif[ch] + MIF_DIV_MIF);
		wait_clear(dmc->cmu_mif[ch] + MIF_DIV_STAT_MIF,
			   MIF_DIV_STAT_HPM, "SCLK_HPM divider", dmc->dev);
	}
}

static int set_ccore(struct exynos7420_dmc *dmc, int lv)
{
	struct clk_hw *mux = __clk_get_hw(dmc->mout_aclk_ccore_532);
	struct clk_hw *parent;
	int ret;

	parent = clk_hw_get_parent_by_index(mux, levels[lv].ccore_532_parent);
	if (!parent)
		return -EINVAL;
	ret = clk_set_parent(dmc->mout_aclk_ccore_532, parent->clk);
	/* the generic divider does not wait for DIV_STAT; stock did */
	if (!ret)
		ret = clk_set_rate(dmc->dout_aclk_ccore_532,
				   levels[lv].ccore_532_hz);
	if (!ret)
		ret = wait_clear(dmc->topc + TOPC_DIV_STAT_TOPC0,
				 TOPC_DIV_CCORE_532_BUSY, "CCORE 532 divider",
				 dmc->dev);
	if (!ret)
		ret = clk_set_rate(dmc->dout_aclk_ccore_133,
				   levels[lv].ccore_133_hz);
	if (!ret)
		ret = wait_clear(dmc->topc + TOPC_DIV_STAT_TOPC0,
				 TOPC_DIV_CCORE_133_BUSY, "CCORE 133 divider",
				 dmc->dev);
	return ret;
}

static void rmw(void __iomem *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | (val & mask), reg);
}

/* Load level @lv into timing set @set of every channel. */
static void preset(struct exynos7420_dmc *dmc, int lv, int set)
{
	const struct drex_timing *t = &drex_6gb[lv];
	const u32 *phy = phy_6gb[lv][set];
	const u32 *mask = phy_mask[set];
	int ch, n;

	for (ch = 0; ch < NR_CH; ch++) {
		void __iomem *drex = dmc->drex[ch];
		void __iomem *vt = dmc->vtmon[ch];
		u32 zq = readl(dmc->phy[ch] + PHY_ZQ_CON9);
		u32 vref1 = (zq >> 8) & 0x3f, vref0 = zq & 0x3f;

		if (set) {
			rmw(drex + DREX_TIMINGRFCPB, 0xff00, t->rfcpb);
			writel(t->row, drex + DREX_TIMINGROW_1);
			writel(t->data, drex + DREX_TIMINGDATA_1);
			writel(t->power, drex + DREX_TIMINGPOWER_1);
			writel(t->rdfetch, drex + DREX_RDFETCH_1);
			writel(BIT(7) | t->etc, drex + DREX_ETCCONTROL_1);
			writel(t->train_timing, drex + DREX_TRAIN_TIMING_1);
			writel(t->hw_ptrain_period, drex + DREX_HW_PTRAIN_PERIOD_1);
			writel(t->hwpr_config, drex + DREX_HWPR_TRAIN_CONFIG_1);
			writel(t->hwpr_control, drex + DREX_HWPR_TRAIN_CONTROL_1);
		} else {
			rmw(drex + DREX_TIMINGRFCPB, 0x00ff, t->rfcpb);
			writel(t->row, drex + DREX_TIMINGROW_0);
			writel(t->data, drex + DREX_TIMINGDATA_0);
			writel(t->power, drex + DREX_TIMINGPOWER_0);
			writel(t->rdfetch, drex + DREX_RDFETCH_0);
			writel(t->etc, drex + DREX_ETCCONTROL_0);
			writel(t->train_timing, drex + DREX_TRAIN_TIMING_0);
			writel(t->hw_ptrain_period, drex + DREX_HW_PTRAIN_PERIOD_0);
			writel(t->hwpr_config, drex + DREX_HWPR_TRAIN_CONFIG_0);
			writel(t->hwpr_control, drex + DREX_HWPR_TRAIN_CONTROL_0);
		}

		for (n = 0; n < 6; n++) {
			u32 v = phy[n];

			/* CON4 carries the trained SoC Vref of both slices */
			if (n == 4)
				v |= set ? (vref1 << 18) | (vref0 << 12) :
					   (vref1 << 6) | vref0;
			rmw(dmc->phy[ch] + PHY_DVFS_CON(n), mask[n], v);
		}

		writel(VTMON_LIMIT_SET0, vt + (set ? VTMON_CNT_LIMIT_SET0_SW1 :
						     VTMON_CNT_LIMIT_SET0_SW0));
		writel(VTMON_LIMIT_SET1, vt + (set ? VTMON_CNT_LIMIT_SET1_SW1 :
						     VTMON_CNT_LIMIT_SET1_SW0));

		/*
		 * MR13 selects the LPDDR4 set point the following mode
		 * registers program; the VT monitor writes MR13 again at
		 * the clock switch to make that set point the operating one.
		 */
		writel((set ? 0x50 << 2 : 0x90 << 2) | t->mr13,
		       drex + DREX_DIRECTCMD);
		writel(t->mr2, drex + DREX_DIRECTCMD);
		writel(t->mr22, drex + DREX_DIRECTCMD);
		writel(t->mr11, drex + DREX_DIRECTCMD);

		writel(set ? 1 : 0, vt + VTMON_DREX_TIMING_SW_SET);
		writel((set ? 0xd0 << 2 : 0x10 << 2) | t->mr13,
		       vt + VTMON_MIF_DREX_WDATA0);
	}
}

static u32 vtmon_comp_off(struct exynos7420_dmc *dmc)
{
	u32 saved = readl(dmc->vtmon[0] + VTMON_CON) &
		    (VTMON_AREF_UPDATE_EN | VTMON_PER_MRS_EN);
	int ch;

	for (ch = 0; ch < NR_CH; ch++) {
		rmw(dmc->vtmon[ch] + VTMON_CON,
		    VTMON_AREF_UPDATE_EN | VTMON_PER_MRS_EN, 0);
		wait_clear(dmc->vtmon[ch] + VTMON_PER_UPDATE_STATUS, 0x1f,
			   "VT monitor update", dmc->dev);
	}
	return saved;
}

static void vtmon_comp_restore(struct exynos7420_dmc *dmc, u32 saved)
{
	int ch;

	for (ch = 0; ch < NR_CH; ch++) {
		writel(readl(dmc->vtmon[ch] + VTMON_CON) | saved,
		       dmc->vtmon[ch] + VTMON_CON);
		wait_clear(dmc->vtmon[ch] + VTMON_PER_UPDATE_STATUS, 0x1f,
			   "VT monitor update", dmc->dev);
	}
}

static void set_pbr(struct exynos7420_dmc *dmc, bool en)
{
	int ch;

	for (ch = 0; ch < NR_CH; ch++)
		rmw(dmc->drex[ch] + DREX_MEMCONTROL, DREX_PB_REF_EN,
		    en ? DREX_PB_REF_EN : 0);
}

/* BUCK1 change, with the PHY clock gating off as stock did around it */
static int set_volt(struct exynos7420_dmc *dmc, int uv)
{
	u32 cg = readl(dmc->drex[0] + DREX_CGCONTROL) & DREX_PHY_CG_EN;
	int ch, ret;

	if (uv == dmc->cur_uv)
		return 0;
	for (ch = 0; ch < NR_CH; ch++)
		rmw(dmc->drex[ch] + DREX_CGCONTROL, DREX_PHY_CG_EN, 0);
	ret = regulator_set_voltage(dmc->vdd, uv, uv);
	for (ch = 0; ch < NR_CH; ch++)
		rmw(dmc->drex[ch] + DREX_CGCONTROL, DREX_PHY_CG_EN, cg);
	if (ret) {
		dev_err(dmc->dev, "vdd_mif to %d uV: %d\n", uv, ret);
		dmc->cur_uv = regulator_get_voltage(dmc->vdd);
		return ret;
	}
	dmc->cur_uv = uv;
	return 0;
}

static void set_dsref_cycle(struct exynos7420_dmc *dmc, int lv)
{
	u32 cyc = lv <= 4 ? 0x3ff : lv <= 7 ? 0x1ff : 0x90;
	int ch;

	for (ch = 0; ch < NR_CH; ch++)
		rmw(dmc->drex[ch] + DREX_MEMCONTROL, DREX_DSREF_EN, 0);
	for (ch = 0; ch < NR_CH; ch++)
		rmw(dmc->drex[ch] + DREX_PWRDNCONFIG, DREX_DSREF_CYC,
		    FIELD_PREP(DREX_DSREF_CYC, cyc));
	for (ch = 0; ch < NR_CH; ch++)
		rmw(dmc->drex[ch] + DREX_MEMCONTROL, DREX_DSREF_EN,
		    DREX_DSREF_EN);
}

/*
 * Move from the current level to @lv through the safe PLL. Called with
 * dmc->lock held.
 *
 * The DRAM runs LPDDR4 set point 0 on fout_mif_pll and set point 1 on
 * the safe PLL. Set 0 may only be reloaded while the DRAM is on the safe
 * PLL: its MR13 write makes set point 1 the operating one.
 */
static int switch_level(struct exynos7420_dmc *dmc, int lv)
{
	int old = dmc->cur, safe, safe_uv, ccore, final, ret;
	unsigned long rate, safe_rate;
	struct clk_hw *hw;
	u32 vt;

	if (lv < 0 || lv >= NR_LEVELS || old < 0 || !dmc->uv[lv])
		return -EINVAL;
	rate = levels[lv].khz * 2000;
	if (!mux_ctrl_by_cmuc(dmc)) {
		dev_err(dmc->dev, "MIF muxes not under CMU_TOPC control\n");
		return -EIO;
	}

	if (old <= LV_SAFE_HIGH && lv <= LV_SAFE_HIGH) {
		safe = LV_SAFE_HIGH;
		safe_rate = 1600000000;
		safe_uv = dmc->uv[safe];
	} else {
		safe = LV_SAFE_LOW;
		safe_rate = 800000000;
		safe_uv = dmc->uv[safe] + SAFE_LOW_OFFSET_UV;
	}

	/* Safe PLL: BUS0 PLL (1600 MHz), undivided or halved */
	hw = clk_hw_get_parent_by_index(__clk_get_hw(dmc->mout_sclk_bus0_pll_mif), 0);
	if (!hw)
		return -EINVAL;
	ret = clk_set_parent(dmc->mout_sclk_bus0_pll_mif, hw->clk);
	if (!ret)
		ret = clk_set_rate(dmc->dout_sclk_bus0_pll_mif, safe_rate);
	if (!ret)
		ret = clk_prepare_enable(dmc->sclk_bus0_pll_mif);
	if (ret)
		return ret;

	/* per-bank refresh only above LV4 */
	if (lv >= 4)
		set_pbr(dmc, false);
	vt = vtmon_comp_off(dmc);

	/* The old level and the safe one must both hold */
	ret = set_volt(dmc, max(dmc->uv[old], safe_uv));
	if (ret)
		goto abort;

	/*
	 * CCORE never runs above the DRAM's level: lower it before the
	 * DRAM clock goes down, raise it after the DRAM clock goes up.
	 */
	ccore = (safe >= old && safe >= lv) ? safe : lv >= old ? lv : -1;
	if (ccore >= 0) {
		ret = set_ccore(dmc, ccore);
		if (ret)
			goto abort;
	}

	preset(dmc, safe, 1);
	set_hpm_div(dmc, safe);
	ret = clk_set_parent(dmc->mout_aclk_mif_pll, dmc->sclk_bus0_pll_mif);
	if (ret)
		goto abort;
	if (wait_pause(dmc)) {
		ret = -ETIMEDOUT;
		goto back;
	}

	/* The target voltage before the target clock */
	ret = set_volt(dmc, max(safe_uv, dmc->uv[lv]));
	if (!ret && clk_get_rate(dmc->fout_mif_pll) != rate) {
		ret = clk_set_rate(dmc->fout_mif_pll, rate);
		if (ret)
			dev_err(dmc->dev, "MIF PLL to %lu Hz: %d\n", rate, ret);
	}

back:
	/* Return to the MIF PLL whatever failed, on the level it now feeds */
	final = clk_get_rate(dmc->fout_mif_pll) == rate ? lv : old;
	if (set_volt(dmc, max(safe_uv, dmc->uv[final])))
		goto stuck;
	preset(dmc, final, 0);
	set_hpm_div(dmc, final);
	if (clk_set_parent(dmc->mout_aclk_mif_pll, dmc->mout_topc_mif_pll) ||
	    wait_pause(dmc))
		goto stuck;
	clk_disable_unprepare(dmc->sclk_bus0_pll_mif);
	set_ccore(dmc, final);
	vtmon_comp_restore(dmc, vt);
	set_volt(dmc, dmc->uv[final]);
	set_pbr(dmc, final < 4);
	set_dsref_cycle(dmc, final);
	dmc->cur = final;
	dmc->nr_switch++;
	return final == lv ? ret : (ret ?: -EIO);

stuck:
	/* DRAM may still be on the safe PLL: keep its gate on */
	dev_crit(dmc->dev, "cannot return to the MIF PLL\n");
	dmc->can_switch = false;
	dmc->cur = -1;
	return -EIO;

abort:
	/* Still on the MIF PLL in set point 0: undo the clock changes */
	set_hpm_div(dmc, old);
	set_ccore(dmc, old);
	clk_disable_unprepare(dmc->sclk_bus0_pll_mif);
	vtmon_comp_restore(dmc, vt);
	set_volt(dmc, dmc->uv[old]);
	set_pbr(dmc, old < 4);
	return ret;
}

/* Identify the running level from DREX0 timing set 0. */
static int detect_level(struct exynos7420_dmc *dmc)
{
	u32 row = readl(dmc->drex[0] + DREX_TIMINGROW_0);
	u32 data = readl(dmc->drex[0] + DREX_TIMINGDATA_0);
	u32 power = readl(dmc->drex[0] + DREX_TIMINGPOWER_0);
	int lv;

	for (lv = 0; lv < NR_LEVELS; lv++)
		if (drex_6gb[lv].row == row && drex_6gb[lv].data == data &&
		    drex_6gb[lv].power == power)
			return lv;
	return -1;
}

static int state_show(struct seq_file *s, void *unused)
{
	struct exynos7420_dmc *dmc = s->private;
	int ch, n;

	mutex_lock(&dmc->lock);
	seq_printf(s, "level %d (%lu kHz) switches %u (last %lld us, max %lld us) can_switch %d\n",
		   dmc->cur, dmc->cur >= 0 ? levels[dmc->cur].khz : 0,
		   dmc->nr_switch, dmc->last_us, dmc->max_us, dmc->can_switch);
	seq_printf(s, "fout_mif_pll %lu mout_aclk_mif_pll %lu (%s) ccore_532 %lu ccore_133 %lu vdd_mif %d (%d)\n",
		   clk_get_rate(dmc->fout_mif_pll),
		   clk_get_rate(dmc->mout_aclk_mif_pll),
		   __clk_get_name(clk_get_parent(dmc->mout_aclk_mif_pll)),
		   clk_get_rate(dmc->dout_aclk_ccore_532),
		   clk_get_rate(dmc->dout_aclk_ccore_133),
		   dmc->cur_uv, regulator_get_voltage(dmc->vdd));
	seq_printf(s, "load ppm %u %u %u %u, icc floor %u kHz, gpu floor %u kHz\n",
		   dmc->last_load[0], dmc->last_load[1], dmc->last_load[2],
		   dmc->last_load[3], dmc->icc_khz, dmc->gpu_khz);
	for (ch = 0; ch < NR_CH; ch++) {
		void __iomem *d = dmc->drex[ch];

		seq_printf(s, "ch%d memctl %08x pwrdn %08x set0 %08x %08x %08x set1 %08x %08x %08x rfcpb %08x\n",
			   ch, readl(d + DREX_MEMCONTROL),
			   readl(d + DREX_PWRDNCONFIG),
			   readl(d + DREX_TIMINGROW_0),
			   readl(d + DREX_TIMINGDATA_0),
			   readl(d + DREX_TIMINGPOWER_0),
			   readl(d + DREX_TIMINGROW_1),
			   readl(d + DREX_TIMINGDATA_1),
			   readl(d + DREX_TIMINGPOWER_1),
			   readl(d + DREX_TIMINGRFCPB));
		seq_printf(s, "ch%d phy", ch);
		for (n = 0; n < 6; n++)
			seq_printf(s, " %08x", readl(dmc->phy[ch] + PHY_DVFS_CON(n)));
		seq_printf(s, " zq9 %08x vtmon %08x div_mif %08x\n",
			   readl(dmc->phy[ch] + PHY_ZQ_CON9),
			   readl(dmc->vtmon[ch] + VTMON_CON),
			   readl(dmc->cmu_mif[ch] + MIF_DIV_MIF));
	}
	mutex_unlock(&dmc->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(state);

static int level_of_khz(unsigned long khz)
{
	int lv;

	for (lv = 0; lv < NR_LEVELS; lv++)
		if (levels[lv].khz == khz)
			return lv;
	return -EINVAL;
}

static int dmc_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct exynos7420_dmc *dmc = dev_get_drvdata(dev);
	unsigned long floor = (unsigned long)READ_ONCE(dmc->floor_khz) * 1000;
	struct dev_pm_opp *opp;
	int lv, ret = 0;

	if (*freq < floor) {
		*freq = floor;
		flags &= ~DEVFREQ_FLAG_LEAST_UPPER_BOUND;
	}
	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);
	if (*freq < floor) {
		/* the floor rounds up to the next level */
		opp = devfreq_recommended_opp(dev, &floor, 0);
		if (IS_ERR(opp))
			return PTR_ERR(opp);
		dev_pm_opp_put(opp);
		*freq = floor;
	}

	lv = level_of_khz(*freq / 1000);
	if (lv < 0)
		return lv;

	mutex_lock(&dmc->lock);
	if (lv != dmc->cur) {
		int old = dmc->cur;
		ktime_t t0 = ktime_get();

		ret = dmc->can_switch ? switch_level(dmc, lv) : -EPERM;
		dmc->last_us = ktime_us_delta(ktime_get(), t0);
		dmc->max_us = max(dmc->max_us, dmc->last_us);
		if (ret)
			dev_err(dev, "LV%d -> LV%d: %d\n", old, lv, ret);
		else if (trace)
			dev_info(dev, "LV%d -> LV%d, %lld us, vdd_mif %d uV\n",
				 old, lv, dmc->last_us, dmc->cur_uv);
	}
	if (dmc->cur >= 0)
		*freq = levels[dmc->cur].khz * 1000;
	mutex_unlock(&dmc->lock);
	return ret;
}

static int dmc_cur_freq(struct device *dev, unsigned long *freq)
{
	struct exynos7420_dmc *dmc = dev_get_drvdata(dev);

	*freq = dmc->cur >= 0 ? levels[dmc->cur].khz * 1000 : 0;
	return 0;
}

/*
 * Load of the busiest channel: data beats on its CPU and general ports
 * per PPMU cycle. Each PPMU counts against its own cycle counter, so the
 * ratio stays right when bus_int restarts the general-port PPMUs on its
 * own schedule. Saturation is the beats-per-cycle ratio of a full channel.
 */
static int dmc_get_dev_status(struct device *dev,
			      struct devfreq_dev_status *stat)
{
	struct exynos7420_dmc *dmc = dev_get_drvdata(dev);
	struct devfreq_event_data ed;
	u32 load[NR_CH] = { }, busy = 0;
	int i, ret;

	for (i = 0; i < dmc->nr_edev; i++) {
		ret = devfreq_event_get_event(dmc->edev[i], &ed);
		if (!ret && ed.total_count)
			load[i % NR_CH] += min_t(u64, div64_ul((u64)ed.load_count *
						 1000000, ed.total_count), 1000000);
		ret = devfreq_event_set_event(dmc->edev[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < NR_CH; i++) {
		dmc->last_load[i] = load[i];
		busy = max(busy, load[i]);
	}

	dmc_cur_freq(dev, &stat->current_frequency);
	stat->total_time = 1000000;
	stat->busy_time = min_t(u64, (u64)busy * 100 / dmc->saturation, 1000000);
	return 0;
}

static struct devfreq_dev_profile dmc_profile = {
	.polling_ms = 100,
	.target = dmc_target,
	.get_dev_status = dmc_get_dev_status,
	.get_cur_freq = dmc_cur_freq,
};

static int dmc_apply_floor(struct exynos7420_dmc *dmc, u32 khz)
{
	int ret = 0;

	WRITE_ONCE(dmc->floor_khz, khz);
	if (!dmc->df)
		return 0;
	mutex_lock(&dmc->df->lock);
	ret = update_devfreq(dmc->df);
	mutex_unlock(&dmc->df->lock);
	return ret;
}

/*
 * schedutil moves the CPUs often, and each move changes their vote; a
 * DRAM switch pauses traffic for about a millisecond. Raise the floor at
 * once (the OPP core votes before a CPU speeds up) but lower it only once
 * the lower vote has held for FLOOR_HOLD_MS.
 */
static int dmc_update_floor(struct exynos7420_dmc *dmc)
{
	u32 want;
	int ret = 0;

	mutex_lock(&dmc->floor_lock);
	want = max(dmc->icc_khz, dmc->gpu_khz);
	if (want >= dmc->floor_khz) {
		cancel_delayed_work(&dmc->floor_work);
		if (want != dmc->floor_khz)
			ret = dmc_apply_floor(dmc, want);
	} else {
		queue_delayed_work(system_wq, &dmc->floor_work,
				   msecs_to_jiffies(FLOOR_HOLD_MS));
	}
	mutex_unlock(&dmc->floor_lock);
	return ret;
}

static void dmc_floor_work(struct work_struct *work)
{
	struct exynos7420_dmc *dmc = container_of(to_delayed_work(work),
						  struct exynos7420_dmc,
						  floor_work);

	mutex_lock(&dmc->floor_lock);
	dmc_apply_floor(dmc, max(dmc->icc_khz, dmc->gpu_khz));
	mutex_unlock(&dmc->floor_lock);
}

static u32 dmc_gpu_floor(struct exynos7420_dmc *dmc, unsigned long hz)
{
	u32 khz = hz / 1000, mif = 0;
	int i;

	for (i = 0; i < dmc->nr_gpu_floor; i++)
		if (khz >= dmc->gpu_floor[i][0])
			mif = dmc->gpu_floor[i][1];
	return mif;
}

/*
 * Called from the GPU devfreq's target path. Raise before the GPU speeds
 * up, settle on the new rate after the change (dmc_update_floor() holds
 * a lower floor back).
 */
static int dmc_gpu_notify(struct notifier_block *nb, unsigned long event,
			  void *data)
{
	struct exynos7420_dmc *dmc = container_of(nb, struct exynos7420_dmc,
						  gpu_nb);
	struct devfreq_freqs *f = data;

	if (event == DEVFREQ_PRECHANGE)
		dmc->gpu_khz = dmc_gpu_floor(dmc, max(f->old, f->new));
	else
		dmc->gpu_khz = dmc_gpu_floor(dmc, f->new);
	dmc_update_floor(dmc);
	return NOTIFY_OK;
}

/* The GPU probes after this driver; look for its devfreq for 30 s. */
static void dmc_gpu_work(struct work_struct *work)
{
	struct exynos7420_dmc *dmc = container_of(to_delayed_work(work),
						  struct exynos7420_dmc,
						  gpu_work);
	struct devfreq *df;

	df = devfreq_get_devfreq_by_phandle(dmc->dev, "samsung,gpu", 0);
	if (IS_ERR(df)) {
		/* -ENODEV until panfrost has added its devfreq */
		if ((PTR_ERR(df) == -EPROBE_DEFER || PTR_ERR(df) == -ENODEV) &&
		    ++dmc->gpu_tries < 30)
			schedule_delayed_work(&dmc->gpu_work, HZ);
		else
			dev_warn(dmc->dev, "no GPU devfreq (%ld)\n", PTR_ERR(df));
		return;
	}
	dmc->gpu_nb.notifier_call = dmc_gpu_notify;
	if (devfreq_register_notifier(df, &dmc->gpu_nb,
				      DEVFREQ_TRANSITION_NOTIFIER)) {
		dev_warn(dmc->dev, "GPU devfreq notifier failed\n");
		return;
	}
	dmc->gpu_df = df;
	dmc->gpu_khz = dmc_gpu_floor(dmc, df->previous_freq);
	dmc_update_floor(dmc);
	dev_info(dmc->dev, "GPU floors on %s\n", dev_name(df->dev.parent));
}

static void dmc_gpu_init(struct exynos7420_dmc *dmc)
{
	struct device_node *np = dmc->dev->of_node;
	int n;

	n = of_property_count_u32_elems(np, "samsung,gpu-mif-floors");
	if (n <= 0 || n % 2 || n / 2 > NR_LEVELS)
		return;
	of_property_read_u32_array(np, "samsung,gpu-mif-floors",
				   &dmc->gpu_floor[0][0], n);
	dmc->nr_gpu_floor = n / 2;
	INIT_DELAYED_WORK(&dmc->gpu_work, dmc_gpu_work);
	schedule_delayed_work(&dmc->gpu_work, 0);
}

static void dmc_gpu_exit(struct exynos7420_dmc *dmc)
{
	if (!dmc->nr_gpu_floor)
		return;
	cancel_delayed_work_sync(&dmc->gpu_work);
	if (dmc->gpu_df)
		devfreq_unregister_notifier(dmc->gpu_df, &dmc->gpu_nb,
					    DEVFREQ_TRANSITION_NOTIFIER);
	dmc->gpu_df = NULL;
}

static int dmc_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct exynos7420_dmc *dmc = dst->provider->data;
	u32 bw = max(dst->avg_bw, dst->peak_bw);

	/*
	 * A path from another provider (the display bus) reaches only this
	 * set(); let that provider apply its own node as well.
	 */
	if (src != dst && src->provider != dst->provider && src->provider->set)
		src->provider->set(src, src);

	/* the boot value is INT_MAX per request, summed: compute in 64 bits */
	dmc->icc_khz = min_t(u64, DIV_ROUND_UP_ULL((u64)bw, BYTES_PER_CLK),
			     S32_MAX);
	return dmc_update_floor(dmc);
}

static struct icc_node *dmc_icc_xlate(struct of_phandle_args *spec, void *data)
{
	struct exynos7420_dmc *dmc = data;

	return dmc->icc_node;
}

static int dmc_icc_init(struct exynos7420_dmc *dmc)
{
	struct icc_provider *p = &dmc->icc;
	int ret;

	p->dev = dmc->dev;
	p->set = dmc_icc_set;
	p->aggregate = icc_std_aggregate;
	p->xlate = dmc_icc_xlate;
	p->inter_set = true;
	p->data = dmc;
	icc_provider_init(p);

	dmc->icc_node = icc_node_create(DMC_ICC_ID);
	if (IS_ERR(dmc->icc_node))
		return PTR_ERR(dmc->icc_node);
	dmc->icc_node->name = "dmc";
	icc_node_add(dmc->icc_node, p);

	ret = icc_provider_register(p);
	if (ret) {
		icc_nodes_remove(p);
		return ret;
	}
	return 0;
}

static int dmc_devfreq_init(struct exynos7420_dmc *dmc)
{
	struct device *dev = dmc->dev;
	int i, ret;

	/* one or two PPMU ports per channel, in channel order */
	dmc->nr_edev = devfreq_event_get_edev_count(dev, "devfreq-events");
	if (dmc->nr_edev != NR_CH && dmc->nr_edev != MAX_EDEV)
		return dev_err_probe(dev, -EINVAL, "need %d or %d devfreq-events\n",
				     NR_CH, MAX_EDEV);
	for (i = 0; i < dmc->nr_edev; i++) {
		dmc->edev[i] = devfreq_event_get_edev_by_phandle(dev,
							"devfreq-events", i);
		if (IS_ERR(dmc->edev[i]))
			return dev_err_probe(dev, PTR_ERR(dmc->edev[i]),
					     "devfreq-event %d\n", i);
		ret = devfreq_event_enable_edev(dmc->edev[i]);
		if (!ret)
			ret = devfreq_event_set_event(dmc->edev[i]);
		if (ret)
			return dev_err_probe(dev, ret, "devfreq-event %d\n", i);
	}
	if (of_property_read_u32(dev->of_node, "exynos,saturation-ratio",
				 &dmc->saturation) || !dmc->saturation)
		dmc->saturation = 100;

	/*
	 * Hold the top level until the interconnect votes are in: with no
	 * get_bw() the core keeps this node at INT_MAX until every provider
	 * reaches sync_state, as stock started MIF at its maximum with the
	 * cpufreq floors in place. A lower level while the CPUs run fast
	 * hangs the SoC.
	 */
	dmc->floor_khz = levels[0].khz;
	dmc->icc_khz = levels[0].khz;

	dmc_profile.initial_freq = levels[dmc->cur].khz * 1000;
	dmc->ondemand.upthreshold = 70;
	dmc->ondemand.downdifferential = 20;
	dmc->df = devm_devfreq_add_device(dev, &dmc_profile,
					  DEVFREQ_GOV_SIMPLE_ONDEMAND,
					  &dmc->ondemand);
	if (IS_ERR(dmc->df)) {
		ret = PTR_ERR(dmc->df);
		dmc->df = NULL;
		return dev_err_probe(dev, ret, "devfreq\n");
	}
	return 0;
}

static void __iomem *map(struct platform_device *pdev, const char *name, int ch)
{
	char res_name[16];
	struct resource *res;

	snprintf(res_name, sizeof(res_name), ch < 0 ? "%s" : "%s%d", name, ch);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, res_name);
	if (!res)
		return IOMEM_ERR_PTR(-ENODEV);
	/* CMU_TOPC also belongs to the clock driver: map, do not request */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?:
		IOMEM_ERR_PTR(-ENOMEM);
}

static int exynos7420_dmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7420_dmc *dmc;
	struct clk_bulk_data clks[] = {
		{ .id = "fout_mif_pll" },
		{ .id = "mout_aclk_mif_pll" },
		{ .id = "sclk_bus0_pll_mif" },
		{ .id = "mout_sclk_bus0_pll_mif" },
		{ .id = "dout_sclk_bus0_pll_mif" },
		{ .id = "mout_aclk_ccore_532" },
		{ .id = "dout_aclk_ccore_532" },
		{ .id = "dout_aclk_ccore_133" },
	};
	struct clk_hw *pll_mux;
	unsigned long pll_rate;
	int ch, lv, ret;

	dmc = devm_kzalloc(dev, sizeof(*dmc), GFP_KERNEL);
	if (!dmc)
		return -ENOMEM;
	dmc->dev = dev;
	mutex_init(&dmc->lock);
	mutex_init(&dmc->floor_lock);
	INIT_DELAYED_WORK(&dmc->floor_work, dmc_floor_work);

	for (ch = 0; ch < NR_CH; ch++) {
		dmc->drex[ch] = map(pdev, "drex", ch);
		dmc->phy[ch] = map(pdev, "phy", ch);
		dmc->vtmon[ch] = map(pdev, "vtmon", ch);
		dmc->cmu_mif[ch] = map(pdev, "cmu-mif", ch);
		if (IS_ERR(dmc->drex[ch]) || IS_ERR(dmc->phy[ch]) ||
		    IS_ERR(dmc->vtmon[ch]) || IS_ERR(dmc->cmu_mif[ch]))
			return dev_err_probe(dev, -ENODEV,
					     "channel %d registers\n", ch);
	}
	dmc->topc = map(pdev, "cmu-topc", -1);
	if (IS_ERR(dmc->topc))
		return dev_err_probe(dev, PTR_ERR(dmc->topc), "CMU_TOPC\n");

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(clks), clks);
	if (ret)
		return dev_err_probe(dev, ret, "clocks\n");
	dmc->fout_mif_pll = clks[0].clk;
	dmc->mout_aclk_mif_pll = clks[1].clk;
	dmc->sclk_bus0_pll_mif = clks[2].clk;
	dmc->mout_sclk_bus0_pll_mif = clks[3].clk;
	dmc->dout_sclk_bus0_pll_mif = clks[4].clk;
	dmc->mout_aclk_ccore_532 = clks[5].clk;
	dmc->dout_aclk_ccore_532 = clks[6].clk;
	dmc->dout_aclk_ccore_133 = clks[7].clk;

	dmc->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dmc->vdd))
		return dev_err_probe(dev, PTR_ERR(dmc->vdd), "vdd_mif\n");
	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "OPP table\n");
	for (lv = 0; lv < NR_LEVELS; lv++) {
		unsigned long hz = levels[lv].khz * 1000;
		struct dev_pm_opp *opp = dev_pm_opp_find_freq_exact(dev, hz, true);

		/* a level without an OPP is never used */
		if (IS_ERR(opp))
			continue;
		dmc->uv[lv] = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);
	}
	if (!dmc->uv[0] || !dmc->uv[LV_SAFE_HIGH] || !dmc->uv[LV_SAFE_LOW])
		return dev_err_probe(dev, -EINVAL, "no OPP for LV0, LV4 or LV7\n");
	dmc->cur_uv = regulator_get_voltage(dmc->vdd);

	pll_mux = clk_hw_get_parent_by_index(__clk_get_hw(dmc->mout_aclk_mif_pll), 1);
	if (!pll_mux)
		return -EINVAL;
	dmc->mout_topc_mif_pll = pll_mux->clk;

	/*
	 * Switch only from a state the tables describe: LV0 in timing set
	 * 0 of every channel, DRAM on fout_mif_pll at its LV0 rate.
	 */
	dmc->cur = detect_level(dmc);
	pll_rate = clk_get_rate(dmc->fout_mif_pll);
	dmc->can_switch = dmc->cur == 0 &&
		clk_get_parent(dmc->mout_aclk_mif_pll) == dmc->mout_topc_mif_pll &&
		pll_rate == levels[0].khz * 2000 && mux_ctrl_by_cmuc(dmc) &&
		dmc->cur_uv >= dmc->uv[0];
	for (ch = 1; ch < NR_CH; ch++)
		if (readl(dmc->drex[ch] + DREX_TIMINGROW_0) !=
		    readl(dmc->drex[0] + DREX_TIMINGROW_0))
			dmc->can_switch = false;

	platform_set_drvdata(pdev, dmc);

	dmc->debugfs = debugfs_create_dir("exynos7420-dmc", NULL);
	debugfs_create_file("state", 0400, dmc->debugfs, dmc, &state_fops);

	/*
	 * The CPUs and the display bus take paths to this provider, so it
	 * registers even when switching is off; votes then do nothing.
	 */
	if (dmc->can_switch) {
		ret = dmc_devfreq_init(dmc);
		if (ret == -EPROBE_DEFER)
			goto err_debugfs;
		if (ret) {
			dev_err(dev, "no devfreq (%d): level fixed\n", ret);
			dmc->can_switch = false;
		}
	}
	ret = dmc_icc_init(dmc);
	if (ret)
		goto err_floor;
	if (dmc->df)
		dmc_gpu_init(dmc);

	dev_info(dev, "LV%d, MIF PLL %lu Hz on %s, vdd_mif %d uV, switching %s\n",
		 dmc->cur, pll_rate,
		 __clk_get_name(clk_get_parent(dmc->mout_aclk_mif_pll)),
		 dmc->cur_uv,
		 dmc->can_switch ? "on demand" : "disabled");
	return 0;

err_floor:
	cancel_delayed_work_sync(&dmc->floor_work);
err_debugfs:
	debugfs_remove_recursive(dmc->debugfs);
	return ret;
}

static void exynos7420_dmc_remove(struct platform_device *pdev)
{
	struct exynos7420_dmc *dmc = platform_get_drvdata(pdev);

	dmc_gpu_exit(dmc);
	icc_provider_deregister(&dmc->icc);
	icc_nodes_remove(&dmc->icc);
	cancel_delayed_work_sync(&dmc->floor_work);
	debugfs_remove_recursive(dmc->debugfs);
}

/*
 * Hand DRAM to the next boot on LV0 at its voltage, as stock did from its
 * reboot notifier. System suspend reaches the same level through devfreq
 * and the OPP marked opp-suspend.
 */
static void exynos7420_dmc_shutdown(struct platform_device *pdev)
{
	struct exynos7420_dmc *dmc = platform_get_drvdata(pdev);

	if (!dmc->df)
		return;
	dmc_gpu_exit(dmc);
	devfreq_suspend_device(dmc->df);
	cancel_delayed_work_sync(&dmc->floor_work);
	mutex_lock(&dmc->lock);
	if (dmc->cur != 0)
		dev_warn(&pdev->dev, "shutdown on LV%d\n", dmc->cur);
	dmc->can_switch = false;
	mutex_unlock(&dmc->lock);
}

static const struct of_device_id exynos7420_dmc_of_match[] = {
	{ .compatible = "samsung,exynos7420-dmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos7420_dmc_of_match);

static struct platform_driver exynos7420_dmc_driver = {
	.probe = exynos7420_dmc_probe,
	.remove_new = exynos7420_dmc_remove,
	.shutdown = exynos7420_dmc_shutdown,
	.driver = {
		.name = "exynos7420-dmc",
		.of_match_table = exynos7420_dmc_of_match,
		.suppress_bind_attrs = true,
		/* the core holds all icc nodes at INT_MAX until every provider syncs */
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(exynos7420_dmc_driver);

MODULE_DESCRIPTION("Exynos7420 DRAM controller frequency switching");
MODULE_LICENSE("GPL");
