// SPDX-License-Identifier: GPL-2.0
/*
 * Command-only MIPI DSI host for the Exynos7420 DSIM that the bootloader
 * left running.
 *
 * On the Galaxy S6, S-Boot sets up DSIM0 (PLL, D-PHY, lanes, command mode
 * and MIC) and the panel, and DECON keeps sending frames through it. Linux
 * has no driver for that link. This host lets a mainline panel driver send
 * its DCS and generic write commands (brightness, mostly) through the
 * DSIM's SFR packet FIFOs; the DSIM interleaves them with the frame
 * traffic. It cannot read: rx requests are rejected.
 *
 * It keeps S-Boot's setup as found at probe. Deep sleep powers the DISP
 * block off and the DSIM loses it: before such a sleep the lanes go to
 * ULPS, and on resume the link is rebuilt from that copy the way Samsung's
 * dsim_exit_ulps() did (PHY reset, PLL, lanes, ULPS exit). Otherwise it
 * never touches the PLL, PHY, lanes or video setup.
 *
 * Nothing here sends the panel's init sequence. The panel drivers bound
 * to it must not reset the panel; after deep sleep they bring it back
 * themselves, once the link is up again.
 *
 * A packet goes out only once the SFR FIFOs are empty, and, when DECON is
 * sending frames, right after a frame ends, so it lands between frames
 * instead of among the pixel data. The DSIM arbitrates at packet level in
 * any case; the 3.10 driver relied on that alone.
 *
 * exynos7420_dsim_cmd.enable=0 on the command line keeps the host from
 * binding, as does status = "disabled" on its DT node.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>

#include <drm/drm_mipi_dsi.h>

#define DSIM_STATUS			0x04
#define DSIM_STATUS_PLL_STABLE		BIT(31)
#define DSIM_STATUS_ULPS_CLK		BIT(9)
#define DSIM_STATUS_STOP_CLK		BIT(8)
#define DSIM_STATUS_ULPS_DAT		GENMASK(7, 4)
#define DSIM_STATUS_STOP_DAT		GENMASK(3, 0)
#define DSIM_SWRST			0x0c
#define DSIM_SWRST_FUNCRST		BIT(16)
#define DSIM_SWRST_SWRST		BIT(0)
#define DSIM_CLKCTRL			0x10
#define DSIM_CLKCTRL_TX_HSCLK		BIT(31)
#define DSIM_CLKCTRL_ESC_EN		BIT(28)
#define DSIM_CLKCTRL_BYTE_EN		BIT(24)
#define DSIM_CLKCTRL_LANE_ESC_EN	GENMASK(23, 19)
#define DSIM_TIMEOUT			0x14
#define DSIM_CONFIG			0x18
#define DSIM_CONFIG_LANES		GENMASK(4, 0)
#define DSIM_ESCMODE			0x1c
#define DSIM_ESCMODE_CMD_LPDT		BIT(7)
#define DSIM_ESCMODE_TX_LPDT		BIT(6)
#define DSIM_ESCMODE_ULPS_CLK		BIT(3)
#define DSIM_ESCMODE_ULPS_CLK_EXIT	BIT(2)
#define DSIM_ESCMODE_ULPS_DAT		BIT(1)
#define DSIM_ESCMODE_ULPS_DAT_EXIT	BIT(0)
#define DSIM_ESCMODE_ULPS		(DSIM_ESCMODE_ULPS_CLK | DSIM_ESCMODE_ULPS_DAT)
#define DSIM_ESCMODE_ULPS_EXIT		(DSIM_ESCMODE_ULPS_CLK_EXIT | \
					 DSIM_ESCMODE_ULPS_DAT_EXIT)
#define DSIM_MDRESOL			0x20
#define DSIM_MDRESOL_STANDBY		BIT(31)
#define DSIM_INTSRC			0x34
#define DSIM_INTSRC_PLL_STABLE		BIT(31)
#define DSIM_INTSRC_SFR_PL_FIFO_EMPTY	BIT(29)
#define DSIM_INTSRC_SFR_PH_FIFO_EMPTY	BIT(28)
#define DSIM_INTSRC_FRAME_DONE		BIT(24)
#define DSIM_INTMSK			0x38
#define DSIM_PKTHDR			0x3c
#define DSIM_PAYLOAD			0x40
#define DSIM_FIFOCTRL			0x4c
#define DSIM_FIFOCTRL_FULL_PH_SFR	BIT(23)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR	BIT(22)
#define DSIM_FIFOCTRL_FULL_PL_SFR	BIT(21)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR	BIT(20)
#define DSIM_MULTI_PKT			0x78
#define DSIM_MULTI_PKT_GO_EN		BIT(29)
#define DSIM_MULTI_PKT_GO_RDY		BIT(28)
#define DSIM_PLLCTRL			0x94
#define DSIM_PLLCTRL_PLL_EN		BIT(23)
#define DSIM_PLLCTRL1			0x98
#define DSIM_PLLTMR			0xa0
#define DSIM_PHYCTRL			0xa4
#define DSIM_PHYTIMING			0xb4
#define DSIM_PHYTIMING1			0xb8
#define DSIM_PHYTIMING2			0xbc

/* S-Boot's link setup, kept at probe to rebuild the link after sleep */
static const u16 dsim_boot_regs[] = {
	DSIM_CLKCTRL, DSIM_TIMEOUT, DSIM_CONFIG, DSIM_ESCMODE, DSIM_MDRESOL,
	DSIM_INTMSK, DSIM_MULTI_PKT, DSIM_PLLCTRL, DSIM_PLLCTRL1, DSIM_PLLTMR,
	DSIM_PHYCTRL, DSIM_PHYTIMING, DSIM_PHYTIMING1, DSIM_PHYTIMING2,
};

/* PLL lock, lane stop state and ULPS changes take microseconds */
#define DSIM_LINK_TIMEOUT_US		20000

/* A frame takes ~17 ms at 60 Hz; packets go out between frames. */
#define DSIM_CMD_TIMEOUT_US		50000
/* No frame end within a frame period: DECON is not sending one. */
#define DSIM_FRAME_WAIT_US		20000
/*
 * A packet sent this soon after the previous one belongs to the same
 * command sequence (a brightness update is ten packets): it follows at
 * once instead of waiting for another frame end.
 */
#define DSIM_BURST_US			1000

static bool enable = true;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Bind to the DSIM (0 leaves the link alone)");

struct dsim_cmd {
	struct device *dev;
	void __iomem *reg;
	struct clk *pclk;
	struct phy *phy;
	struct mipi_dsi_host host;
	struct mutex lock;	/* one packet in the FIFOs at a time */
	ktime_t last_tx;	/* end of the last packet, under lock */
	u32 boot[ARRAY_SIZE(dsim_boot_regs)];
	bool ulps;		/* lanes put in ULPS for deep sleep */
};

static struct dsim_cmd *host_to_dsim(struct mipi_dsi_host *host)
{
	return container_of(host, struct dsim_cmd, host);
}

static int dsim_cmd_attach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *dsi)
{
	struct dsim_cmd *dsim = host_to_dsim(host);

	dev_info(dsim->dev, "%s attached (link kept as the bootloader set it)\n",
		 dsi->name);
	return 0;
}

static int dsim_cmd_detach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *dsi)
{
	return 0;
}

static int dsim_cmd_wait(struct dsim_cmd *dsim, u32 reg, u32 mask, bool set)
{
	u32 val;

	return readl_poll_timeout(dsim->reg + reg, val,
				  set ? (val & mask) == mask : !(val & mask),
				  10, DSIM_CMD_TIMEOUT_US);
}

/*
 * Wait for the FIFOs to drain, then, at the start of a command sequence,
 * for the end of the frame in flight, if any. Called with dsim->lock held.
 */
static int dsim_cmd_wait_idle(struct dsim_cmd *dsim)
{
	u32 val;
	int ret;

	ret = dsim_cmd_wait(dsim, DSIM_FIFOCTRL, DSIM_FIFOCTRL_EMPTY_PH_SFR |
			    DSIM_FIFOCTRL_EMPTY_PL_SFR, true);
	if (ret)
		return ret;

	if (ktime_us_delta(ktime_get(), dsim->last_tx) < DSIM_BURST_US)
		return 0;

	writel(DSIM_INTSRC_FRAME_DONE, dsim->reg + DSIM_INTSRC);
	ret = readl_poll_timeout(dsim->reg + DSIM_INTSRC, val,
				 val & DSIM_INTSRC_FRAME_DONE, 10,
				 DSIM_FRAME_WAIT_US);
	if (ret)
		dev_dbg(dsim->dev, "no frame in flight\n");
	else
		writel(DSIM_INTSRC_FRAME_DONE, dsim->reg + DSIM_INTSRC);

	return 0;
}

static ssize_t dsim_cmd_transfer(struct mipi_dsi_host *host,
				 const struct mipi_dsi_msg *msg)
{
	struct dsim_cmd *dsim = host_to_dsim(host);
	struct mipi_dsi_packet pkt;
	u32 done = DSIM_INTSRC_SFR_PH_FIFO_EMPTY;
	size_t i;
	int ret;

	if (msg->rx_len || msg->rx_buf)
		return -EOPNOTSUPP;

	ret = mipi_dsi_create_packet(&pkt, msg);
	if (ret)
		return ret;

	mutex_lock(&dsim->lock);

	ret = dsim_cmd_wait_idle(dsim);
	if (ret)
		goto timeout;

	if (pkt.payload_length)
		done |= DSIM_INTSRC_SFR_PL_FIFO_EMPTY;
	writel(done, dsim->reg + DSIM_INTSRC);

	for (i = 0; i < pkt.payload_length; i += 4) {
		u32 word = 0;
		size_t j;

		for (j = 0; j < 4 && i + j < pkt.payload_length; j++)
			word |= (u32)pkt.payload[i + j] << (8 * j);

		ret = dsim_cmd_wait(dsim, DSIM_FIFOCTRL,
				    DSIM_FIFOCTRL_FULL_PL_SFR, false);
		if (ret)
			goto timeout;
		writel(word, dsim->reg + DSIM_PAYLOAD);
	}

	/* The DSIM adds the ECC byte. */
	writel(pkt.header[0] | pkt.header[1] << 8 | pkt.header[2] << 16,
	       dsim->reg + DSIM_PKTHDR);

	/* With packet-go mode on, queued packets wait for this. */
	if (readl(dsim->reg + DSIM_MULTI_PKT) & DSIM_MULTI_PKT_GO_EN)
		writel(readl(dsim->reg + DSIM_MULTI_PKT) |
		       DSIM_MULTI_PKT_GO_RDY, dsim->reg + DSIM_MULTI_PKT);

	ret = dsim_cmd_wait(dsim, DSIM_INTSRC, done, true);
	if (ret)
		goto timeout;
	writel(done, dsim->reg + DSIM_INTSRC);

	dsim->last_tx = ktime_get();
	mutex_unlock(&dsim->lock);
	return msg->tx_len;

timeout:
	dev_err_ratelimited(dsim->dev,
			    "packet 0x%02x timed out: fifoctrl %08x intsrc %08x status %08x\n",
			    pkt.header[0], readl(dsim->reg + DSIM_FIFOCTRL),
			    readl(dsim->reg + DSIM_INTSRC),
			    readl(dsim->reg + DSIM_STATUS));
	mutex_unlock(&dsim->lock);
	return -ETIMEDOUT;
}

static u32 dsim_boot(struct dsim_cmd *dsim, u16 reg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dsim_boot_regs); i++)
		if (dsim_boot_regs[i] == reg)
			return dsim->boot[i];
	return 0;
}

static int dsim_link_wait(struct dsim_cmd *dsim, u32 mask, u32 want,
			  const char *what)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(dsim->reg + DSIM_STATUS, val,
				 (val & mask) == want, 10, DSIM_LINK_TIMEOUT_US);
	if (ret)
		dev_err(dsim->dev, "%s timed out: status %08x\n", what, val);
	return ret;
}

/* The lane stop and ULPS bits of the lanes S-Boot enabled */
static u32 dsim_data_lanes(struct dsim_cmd *dsim)
{
	return (dsim_boot(dsim, DSIM_CONFIG) & DSIM_CONFIG_LANES) >> 1;
}

static int dsim_ulps_enter(struct dsim_cmd *dsim)
{
	u32 lanes = dsim_data_lanes(dsim);

	writel(readl(dsim->reg + DSIM_ESCMODE) | DSIM_ESCMODE_ULPS,
	       dsim->reg + DSIM_ESCMODE);
	return dsim_link_wait(dsim, DSIM_STATUS_ULPS_DAT | DSIM_STATUS_ULPS_CLK,
			      (lanes << 4) | DSIM_STATUS_ULPS_CLK, "ULPS entry");
}

static int dsim_ulps_exit(struct dsim_cmd *dsim)
{
	u32 val;
	int ret;

	val = readl(dsim->reg + DSIM_ESCMODE);
	writel(val | DSIM_ESCMODE_ULPS_EXIT, dsim->reg + DSIM_ESCMODE);
	ret = dsim_link_wait(dsim, DSIM_STATUS_ULPS_DAT | DSIM_STATUS_ULPS_CLK,
			     0, "ULPS exit");
	/* the exit state must last at least 1 ms */
	usleep_range(1000, 1100);
	val = readl(dsim->reg + DSIM_ESCMODE);
	writel(val & ~DSIM_ESCMODE_ULPS_EXIT, dsim->reg + DSIM_ESCMODE);
	writel(val & ~(DSIM_ESCMODE_ULPS_EXIT | DSIM_ESCMODE_ULPS),
	       dsim->reg + DSIM_ESCMODE);
	return ret;
}

/*
 * Bring S-Boot's link back after the DISP block lost power, in the order
 * of Samsung's dsim_exit_ulps(): PHY reset, DSIM reset, PLL, clocks,
 * lanes, then ULPS entry and exit, which the panel left in ULPS (or in an
 * unknown lane state) follows. The panel itself is not touched. Called
 * with dsim->lock held.
 */
static int dsim_relink(struct dsim_cmd *dsim)
{
	void __iomem *r = dsim->reg;
	u32 clk = dsim_boot(dsim, DSIM_CLKCTRL);
	u32 cfg = dsim_boot(dsim, DSIM_CONFIG);
	u32 pll = dsim_boot(dsim, DSIM_PLLCTRL);
	u32 lanes = dsim_data_lanes(dsim);
	u32 val;
	int ret;

	if (dsim->phy) {
		phy_power_off(dsim->phy);
		udelay(10);
		ret = phy_power_on(dsim->phy);
		if (ret)
			dev_err(dsim->dev, "D-PHY power on: %d\n", ret);
	}

	writel(DSIM_SWRST_SWRST, r + DSIM_SWRST);
	ret = readl_poll_timeout(r + DSIM_SWRST, val, !(val & DSIM_SWRST_SWRST),
				 10, DSIM_LINK_TIMEOUT_US);
	if (ret) {
		dev_err(dsim->dev, "DSIM reset timed out\n");
		return ret;
	}

	writel(dsim_boot(dsim, DSIM_ESCMODE) &
	       ~(DSIM_ESCMODE_ULPS | DSIM_ESCMODE_ULPS_EXIT |
		 DSIM_ESCMODE_TX_LPDT | DSIM_ESCMODE_CMD_LPDT),
	       r + DSIM_ESCMODE);
	writel(dsim_boot(dsim, DSIM_TIMEOUT), r + DSIM_TIMEOUT);
	writel(dsim_boot(dsim, DSIM_MULTI_PKT) &
	       ~(DSIM_MULTI_PKT_GO_EN | DSIM_MULTI_PKT_GO_RDY),
	       r + DSIM_MULTI_PKT);
	writel(dsim_boot(dsim, DSIM_MDRESOL) & ~DSIM_MDRESOL_STANDBY,
	       r + DSIM_MDRESOL);
	writel(cfg & ~DSIM_CONFIG_LANES, r + DSIM_CONFIG);
	writel(dsim_boot(dsim, DSIM_PLLCTRL1), r + DSIM_PLLCTRL1);

	/* PLL and D-PHY timing with the PLL and every clock still off */
	writel(clk & ~(DSIM_CLKCTRL_TX_HSCLK | DSIM_CLKCTRL_ESC_EN |
		       DSIM_CLKCTRL_BYTE_EN | DSIM_CLKCTRL_LANE_ESC_EN),
	       r + DSIM_CLKCTRL);
	writel(pll & ~DSIM_PLLCTRL_PLL_EN, r + DSIM_PLLCTRL);
	writel(dsim_boot(dsim, DSIM_PLLTMR), r + DSIM_PLLTMR);
	writel(dsim_boot(dsim, DSIM_PHYCTRL), r + DSIM_PHYCTRL);
	writel(dsim_boot(dsim, DSIM_PHYTIMING), r + DSIM_PHYTIMING);
	writel(dsim_boot(dsim, DSIM_PHYTIMING1), r + DSIM_PHYTIMING1);
	writel(dsim_boot(dsim, DSIM_PHYTIMING2), r + DSIM_PHYTIMING2);

	writel(DSIM_INTSRC_PLL_STABLE, r + DSIM_INTSRC);
	writel(pll | DSIM_PLLCTRL_PLL_EN, r + DSIM_PLLCTRL);
	ret = dsim_link_wait(dsim, DSIM_STATUS_PLL_STABLE,
			     DSIM_STATUS_PLL_STABLE, "PLL lock");
	if (ret)
		return ret;

	/* byte, escape and lane escape clocks; HS clock later */
	writel(clk & ~DSIM_CLKCTRL_TX_HSCLK, r + DSIM_CLKCTRL);

	writel(cfg, r + DSIM_CONFIG);
	udelay(300);
	ret = dsim_link_wait(dsim, DSIM_STATUS_STOP_DAT | DSIM_STATUS_STOP_CLK,
			     lanes | DSIM_STATUS_STOP_CLK, "lane stop state");
	if (ret)
		return ret;

	ret = dsim_ulps_enter(dsim);
	if (!ret)
		ret = dsim_ulps_exit(dsim);
	if (ret)
		return ret;

	writel(readl(r + DSIM_ESCMODE) &
	       ~(DSIM_ESCMODE_TX_LPDT | DSIM_ESCMODE_CMD_LPDT),
	       r + DSIM_ESCMODE);
	writel(readl(r + DSIM_CLKCTRL) | DSIM_CLKCTRL_TX_HSCLK,
	       r + DSIM_CLKCTRL);
	writel(dsim_boot(dsim, DSIM_MDRESOL), r + DSIM_MDRESOL);
	writel(readl(r + DSIM_SWRST) | DSIM_SWRST_FUNCRST, r + DSIM_SWRST);
	writel(dsim_boot(dsim, DSIM_INTMSK), r + DSIM_INTMSK);
	writel(dsim_boot(dsim, DSIM_MULTI_PKT), r + DSIM_MULTI_PKT);

	return 0;
}

static bool dsim_link_lost(struct dsim_cmd *dsim)
{
	return readl(dsim->reg + DSIM_CONFIG) != dsim_boot(dsim, DSIM_CONFIG) ||
	       !(readl(dsim->reg + DSIM_STATUS) & DSIM_STATUS_PLL_STABLE);
}

static const struct mipi_dsi_host_ops dsim_cmd_ops = {
	.attach = dsim_cmd_attach,
	.detach = dsim_cmd_detach,
	.transfer = dsim_cmd_transfer,
};

static int dsim_cmd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dsim_cmd *dsim;
	int i, ret;

	if (!enable) {
		dev_info(dev, "disabled on the command line\n");
		return -ENODEV;
	}

	dsim = devm_kzalloc(dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim)
		return -ENOMEM;

	dsim->dev = dev;
	mutex_init(&dsim->lock);

	/*
	 * Hold the register clock before the first access: a gated DSIM
	 * does not answer and stalls the bus. S-Boot leaves it on; this
	 * keeps it on once unused clocks are switched off.
	 */
	dsim->pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(dsim->pclk))
		return dev_err_probe(dev, PTR_ERR(dsim->pclk),
				     "cannot enable the DSIM clock\n");

	dsim->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsim->reg))
		return PTR_ERR(dsim->reg);

	/* Held on, as S-Boot left it; it is reset only to rebuild the link. */
	dsim->phy = devm_phy_optional_get(dev, "dsim");
	if (IS_ERR(dsim->phy))
		return dev_err_probe(dev, PTR_ERR(dsim->phy),
				     "cannot get the D-PHY\n");
	ret = phy_power_on(dsim->phy);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power the D-PHY\n");

	for (i = 0; i < ARRAY_SIZE(dsim_boot_regs); i++)
		dsim->boot[i] = readl(dsim->reg + dsim_boot_regs[i]);

	dev_info(dev, "status %08x multi_pkt %08x\n",
		 readl(dsim->reg + DSIM_STATUS),
		 readl(dsim->reg + DSIM_MULTI_PKT));

	dsim->host.dev = dev;
	dsim->host.ops = &dsim_cmd_ops;
	platform_set_drvdata(pdev, dsim);

	ret = mipi_dsi_host_register(&dsim->host);
	if (ret) {
		phy_power_off(dsim->phy);
		return dev_err_probe(dev, ret, "cannot register the DSI host\n");
	}

	return 0;
}

static void dsim_cmd_remove(struct platform_device *pdev)
{
	struct dsim_cmd *dsim = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsim->host);
	phy_power_off(dsim->phy);
}

/*
 * DECON has stopped sending frames by now (it suspends first). Before deep
 * sleep, park the lanes in ULPS as Samsung's driver did, so the panel sees
 * a clean ULPS entry before the DISP block loses power, not dead lanes.
 * s2idle keeps the link as it is.
 */
static int dsim_cmd_suspend(struct device *dev)
{
	struct dsim_cmd *dsim = dev_get_drvdata(dev);

	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;

	mutex_lock(&dsim->lock);
	dsim->ulps = !dsim_ulps_enter(dsim);
	mutex_unlock(&dsim->lock);
	return 0;
}

static int dsim_cmd_resume(struct device *dev)
{
	struct dsim_cmd *dsim = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&dsim->lock);
	if (dsim_link_lost(dsim)) {
		ret = dsim_relink(dsim);
		if (ret)
			dev_err(dev, "link not rebuilt: %d\n", ret);
		else
			dev_info(dev, "link rebuilt after sleep: status %08x\n",
				 readl(dsim->reg + DSIM_STATUS));
	} else if (dsim->ulps) {
		ret = dsim_ulps_exit(dsim);
	}
	dsim->ulps = false;
	mutex_unlock(&dsim->lock);

	/* a dead link must not hold the resume of everything else */
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(dsim_cmd_pm_ops, dsim_cmd_suspend,
				dsim_cmd_resume);

static const struct of_device_id dsim_cmd_of_match[] = {
	{ .compatible = "samsung,exynos7420-dsim-cmd" },
	{ }
};
MODULE_DEVICE_TABLE(of, dsim_cmd_of_match);

static struct platform_driver dsim_cmd_driver = {
	.probe = dsim_cmd_probe,
	.remove_new = dsim_cmd_remove,
	.driver = {
		.name = "exynos7420-dsim-cmd",
		.of_match_table = dsim_cmd_of_match,
		.pm = pm_sleep_ptr(&dsim_cmd_pm_ops),
	},
};
module_platform_driver(dsim_cmd_driver);

MODULE_DESCRIPTION("Exynos7420 command-only DSI host for a bootloader-set link");
MODULE_LICENSE("GPL");
