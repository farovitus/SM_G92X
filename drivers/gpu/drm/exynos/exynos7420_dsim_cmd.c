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
 * traffic. It never touches the PLL, PHY, lanes or video setup, and it
 * cannot read: rx requests are rejected.
 *
 * The panel drivers bound to it must not reset or re-initialise the panel,
 * since nothing here can bring the link back up.
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
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <drm/drm_mipi_dsi.h>

#define DSIM_STATUS			0x04
#define DSIM_INTSRC			0x34
#define DSIM_INTSRC_SFR_PL_FIFO_EMPTY	BIT(29)
#define DSIM_INTSRC_SFR_PH_FIFO_EMPTY	BIT(28)
#define DSIM_INTSRC_FRAME_DONE		BIT(24)
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
	struct mipi_dsi_host host;
	struct mutex lock;	/* one packet in the FIFOs at a time */
	ktime_t last_tx;	/* end of the last packet, under lock */
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

static const struct mipi_dsi_host_ops dsim_cmd_ops = {
	.attach = dsim_cmd_attach,
	.detach = dsim_cmd_detach,
	.transfer = dsim_cmd_transfer,
};

static int dsim_cmd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dsim_cmd *dsim;
	int ret;

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

	dev_info(dev, "status %08x multi_pkt %08x\n",
		 readl(dsim->reg + DSIM_STATUS),
		 readl(dsim->reg + DSIM_MULTI_PKT));

	dsim->host.dev = dev;
	dsim->host.ops = &dsim_cmd_ops;
	platform_set_drvdata(pdev, dsim);

	ret = mipi_dsi_host_register(&dsim->host);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register the DSI host\n");

	return 0;
}

static void dsim_cmd_remove(struct platform_device *pdev)
{
	struct dsim_cmd *dsim = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsim->host);
}

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
	},
};
module_platform_driver(dsim_cmd_driver);

MODULE_DESCRIPTION("Exynos7420 command-only DSI host for a bootloader-set link");
MODULE_LICENSE("GPL");
