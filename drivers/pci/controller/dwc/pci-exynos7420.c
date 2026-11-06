// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for the Samsung Exynos 7420
 *
 * The Exynos 7420 has a DesignWare PCIe core behind Samsung ELBI glue and
 * an embedded combo PHY. On SM-G920F channel 1 carries the BCM4358 Wi-Fi.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "pcie-designware.h"

/* ELBI registers */
#define PCIE_IRQ_PULSE			0x000
#define PCIE_IRQ_LEVEL			0x004
#define PCIE_IRQ_SPECIAL		0x008
#define PCIE_IRQ_EN_PULSE		0x00c
#define PCIE_IRQ_EN_LEVEL		0x010
#define IRQ_MSI_ENABLE			BIT(1)
#define PCIE_IRQ_EN_SPECIAL		0x014
#define PCIE_SW_WAKE			0x018
#define PCIE_BUS_EN			BIT(1)
#define PCIE_APP_LTSSM_ENABLE		0x02c
#define PCIE_ELBI_LTSSM_ENABLE		0x1
#define PCIE_L1_BUG_FIX_ENABLE		0x038
#define PCIE_APP_REQ_EXIT_L1		0x040
#define PCIE_ELBI_RDLH_LINKUP		0x074
#define PCIE_LTSSM_STATE_MASK		0x1f
#define PCIE_LTSSM_L0			0x0d
#define PCIE_LTSSM_L2_IDLE		0x15
#define PCIE_CORE_RESETN_DISABLE	0x0f0
#define PCIE_APP_REQ_EXIT_L1_MODE	0x0f4
#define PCIE_PCS_GLOBAL_RESET		0x130
#define PCIE_PMA_INIT_RESET		0x134

/* PMU PCIE_PHY_CONTROL */
#define PCIE_PHY_ENABLE			BIT(0)

/* Combo PHY (PMA) registers, 32-bit stride */
#define PHY_REG(n)			((n) * 4)
#define PHY_PD_CTRL			PHY_REG(0x15)
#define PHY_PD_CTRL_MASK		GENMASK(6, 3)
#define PHY_PD_TRSV0			PHY_REG(0x4e)
#define PHY_PD_TRSV1			PHY_REG(0x4f)
#define PHY_TRSV_BASE			0x30

/* SYSREG PCIe/PHY configuration word */
#define SYSREG_PHY_CMN_RST		BIT(7)
#define SYSREG_PHY_CFG_A		GENMASK(5, 4)
#define SYSREG_PHY_CFG_B		GENMASK(3, 2)

/* PHY PCS control word */
#define PHY_PCS_CTRL_VAL		0x2

/* CMU_FSYS1 PCIe reference clock select word */
#define CMU_PCIE_REFCLK_SEL		BIT(0)

/* CMU_FSYS1 gate registers used by PCIe channel 1 (wifi1) */
#define ENABLE_ACLK_FSYS1		0x800
#define ENABLE_PCLK_FSYS1		0x900
#define ENABLE_SCLK_FSYS11		0xa04
#define ENABLE_SCLK_FSYS12		0xa08
#define ENABLE_SCLK_FSYS13		0xa0c

struct exynos7420_pcie_gate {
	u32 offset;
	u32 mask;
};

/*
 * Gates are only ever set. Some (bts_embedded, sclk_phy_fsys1) are shared
 * with other FSYS1 users such as UFS, so they are never cleared here.
 */
static const struct exynos7420_pcie_gate exynos7420_pcie_gates[] = {
	{ ENABLE_ACLK_FSYS1,  BIT(9) | BIT(10) | BIT(11) | BIT(12) | BIT(13) |
			      BIT(17) | BIT(18) | BIT(20) | BIT(21) | BIT(24) },
	{ ENABLE_PCLK_FSYS1,  BIT(22) | BIT(26) | BIT(27) | BIT(28) | BIT(29) |
			      BIT(31) },
	{ ENABLE_SCLK_FSYS11, BIT(0) | BIT(8) | BIT(12) | BIT(16) },
	{ ENABLE_SCLK_FSYS12, BIT(8) | BIT(12) },
	{ ENABLE_SCLK_FSYS13, BIT(28) },
};

/* Combo PHY common block, written at phy + i * 4 */
static const u8 exynos7420_pcie_phy_cmn[] = {
	0x01, 0x0f, 0xa6, 0x71, 0x90, 0x62, 0x20, 0x00, 0x00, 0xa7, 0x0a,
	0x37, 0x20, 0x08, 0xef, 0xfc, 0x96, 0x14, 0x00, 0x10, 0x68, 0x01,
	0x00, 0x00, 0x04, 0x20,
};

/* Combo PHY transceiver block, written at phy + (0x30 + i) * 4 */
static const u8 exynos7420_pcie_phy_trsv[] = {
	0x31, 0xf4, 0xf4, 0x80, 0x25, 0x40, 0xc0, 0x03, 0x35, 0x55, 0x4c,
	0xc3, 0x10, 0x54, 0x70, 0xc5, 0x00, 0x2f, 0x44, 0xa4, 0x00, 0x3b,
	0x34, 0xa7, 0x64, 0x00, 0x1f, 0x83, 0x1b, 0x01, 0xe0, 0x00, 0x00,
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x00,
};

struct exynos7420_pcie {
	struct dw_pcie		pci;
	void __iomem		*elbi_base;
	void __iomem		*phy_base;
	void __iomem		*sysreg;
	void __iomem		*pmu;
	void __iomem		*cmu;
	void __iomem		*phy_pcs;
	void __iomem		*gates;
	struct gpio_desc	*perst_gpio;
	struct gpio_desc	*wlen_gpio;
	int			irq;

	/* L1-exit request around DBI and config accesses, may nest */
	raw_spinlock_t		l1_lock;
	unsigned int		l1_depth;

	const struct pci_ops	*dw_child_ops;
	struct pci_ops		child_ops;
};

#define to_exynos7420_pcie(x)	dev_get_drvdata((x)->dev)

static void exynos7420_pcie_l1_exit_get(struct exynos7420_pcie *ep)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ep->l1_lock, flags);
	if (!ep->l1_depth++) {
		writel(1, ep->elbi_base + PCIE_APP_REQ_EXIT_L1);
		writel(0, ep->elbi_base + PCIE_APP_REQ_EXIT_L1_MODE);
	}
	raw_spin_unlock_irqrestore(&ep->l1_lock, flags);
}

static void exynos7420_pcie_l1_exit_put(struct exynos7420_pcie *ep)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ep->l1_lock, flags);
	if (!--ep->l1_depth) {
		writel(1, ep->elbi_base + PCIE_APP_REQ_EXIT_L1_MODE);
		writel(0, ep->elbi_base + PCIE_APP_REQ_EXIT_L1);
	}
	raw_spin_unlock_irqrestore(&ep->l1_lock, flags);
}

static u32 exynos7420_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base,
				    u32 reg, size_t size)
{
	struct exynos7420_pcie *ep = to_exynos7420_pcie(pci);
	u32 val;

	exynos7420_pcie_l1_exit_get(ep);
	dw_pcie_read(base + reg, size, &val);
	exynos7420_pcie_l1_exit_put(ep);

	return val;
}

static void exynos7420_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base,
				      u32 reg, size_t size, u32 val)
{
	struct exynos7420_pcie *ep = to_exynos7420_pcie(pci);

	exynos7420_pcie_l1_exit_get(ep);
	dw_pcie_write(base + reg, size, val);
	exynos7420_pcie_l1_exit_put(ep);
}

static struct exynos7420_pcie *bus_to_exynos7420_pcie(struct pci_bus *bus)
{
	struct dw_pcie_rp *pp = bus->sysdata;

	return to_exynos7420_pcie(to_dw_pcie_from_pp(pp));
}

static int exynos7420_pcie_rd_own_conf(struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 *val)
{
	struct exynos7420_pcie *ep = bus_to_exynos7420_pcie(bus);
	int ret;

	exynos7420_pcie_l1_exit_get(ep);
	ret = pci_generic_config_read(bus, devfn, where, size, val);
	exynos7420_pcie_l1_exit_put(ep);

	return ret;
}

static int exynos7420_pcie_wr_own_conf(struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 val)
{
	struct exynos7420_pcie *ep = bus_to_exynos7420_pcie(bus);
	int ret;

	exynos7420_pcie_l1_exit_get(ep);
	ret = pci_generic_config_write(bus, devfn, where, size, val);
	exynos7420_pcie_l1_exit_put(ep);

	return ret;
}

static struct pci_ops exynos7420_pcie_ops = {
	.map_bus = dw_pcie_own_conf_map_bus,
	.read = exynos7420_pcie_rd_own_conf,
	.write = exynos7420_pcie_wr_own_conf,
};

/* Only device 0 exists on the bus behind the root port. */
static bool exynos7420_pcie_valid_device(struct pci_bus *bus,
					 unsigned int devfn)
{
	return !(pci_is_root_bus(bus->parent) && PCI_SLOT(devfn));
}

static int exynos7420_pcie_rd_other_conf(struct pci_bus *bus,
					 unsigned int devfn, int where,
					 int size, u32 *val)
{
	struct exynos7420_pcie *ep = bus_to_exynos7420_pcie(bus);
	int ret;

	if (!exynos7420_pcie_valid_device(bus, devfn)) {
		PCI_SET_ERROR_RESPONSE(val);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	exynos7420_pcie_l1_exit_get(ep);
	ret = ep->dw_child_ops->read(bus, devfn, where, size, val);
	exynos7420_pcie_l1_exit_put(ep);

	return ret;
}

static int exynos7420_pcie_wr_other_conf(struct pci_bus *bus,
					 unsigned int devfn, int where,
					 int size, u32 val)
{
	struct exynos7420_pcie *ep = bus_to_exynos7420_pcie(bus);
	int ret;

	if (!exynos7420_pcie_valid_device(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	exynos7420_pcie_l1_exit_get(ep);
	ret = ep->dw_child_ops->write(bus, devfn, where, size, val);
	exynos7420_pcie_l1_exit_put(ep);

	return ret;
}

static int exynos7420_pcie_link_up(struct dw_pcie *pci)
{
	struct exynos7420_pcie *ep = to_exynos7420_pcie(pci);
	u32 state = readl(ep->elbi_base + PCIE_ELBI_RDLH_LINKUP) &
		    PCIE_LTSSM_STATE_MASK;

	return state >= PCIE_LTSSM_L0 && state <= PCIE_LTSSM_L2_IDLE;
}

static int exynos7420_pcie_start_link(struct dw_pcie *pci)
{
	struct exynos7420_pcie *ep = to_exynos7420_pcie(pci);

	writel(PCIE_ELBI_LTSSM_ENABLE, ep->elbi_base + PCIE_APP_LTSSM_ENABLE);

	return 0;
}

static void exynos7420_pcie_stop_link(struct dw_pcie *pci)
{
	struct exynos7420_pcie *ep = to_exynos7420_pcie(pci);

	writel(0, ep->elbi_base + PCIE_APP_LTSSM_ENABLE);
}

static irqreturn_t exynos7420_pcie_irq_handler(int irq, void *arg)
{
	struct exynos7420_pcie *ep = arg;
	void __iomem *elbi = ep->elbi_base;
	u32 pulse, level, special;

	pulse = readl(elbi + PCIE_IRQ_PULSE);
	writel(pulse, elbi + PCIE_IRQ_PULSE);

	special = readl(elbi + PCIE_IRQ_SPECIAL);
	writel(special, elbi + PCIE_IRQ_SPECIAL);

	level = readl(elbi + PCIE_IRQ_LEVEL);
	if (level & IRQ_MSI_ENABLE)
		dw_handle_msi_irq(&ep->pci.pp);
	writel(level, elbi + PCIE_IRQ_LEVEL);

	return (pulse | level | special) ? IRQ_HANDLED : IRQ_NONE;
}

static void exynos7420_pcie_irq_mask_all(struct exynos7420_pcie *ep)
{
	void __iomem *elbi = ep->elbi_base;

	writel(0, elbi + PCIE_IRQ_EN_PULSE);
	writel(0, elbi + PCIE_IRQ_EN_LEVEL);
	writel(0, elbi + PCIE_IRQ_EN_SPECIAL);

	writel(readl(elbi + PCIE_IRQ_PULSE), elbi + PCIE_IRQ_PULSE);
	writel(readl(elbi + PCIE_IRQ_LEVEL), elbi + PCIE_IRQ_LEVEL);
	writel(readl(elbi + PCIE_IRQ_SPECIAL), elbi + PCIE_IRQ_SPECIAL);
}

static void exynos7420_pcie_enable_gates(struct exynos7420_pcie *ep)
{
	const struct exynos7420_pcie_gate *g;
	int i;

	for (i = 0; i < ARRAY_SIZE(exynos7420_pcie_gates); i++) {
		g = &exynos7420_pcie_gates[i];
		writel(readl(ep->gates + g->offset) | g->mask,
		       ep->gates + g->offset);
	}
}

static void exynos7420_pcie_phy_power_on(struct exynos7420_pcie *ep)
{
	writel(PCIE_PHY_ENABLE, ep->pmu);

	writel(readl(ep->phy_base + PHY_PD_CTRL) & ~PHY_PD_CTRL_MASK,
	       ep->phy_base + PHY_PD_CTRL);
	writel(0, ep->phy_base + PHY_PD_TRSV0);
	writel(0, ep->phy_base + PHY_PD_TRSV1);
}

static void exynos7420_pcie_phy_power_off(struct exynos7420_pcie *ep)
{
	writel(readl(ep->phy_base + PHY_PD_CTRL) | PHY_PD_CTRL_MASK,
	       ep->phy_base + PHY_PD_CTRL);
	writel(0xff, ep->phy_base + PHY_PD_TRSV0);
	writel(0x3f, ep->phy_base + PHY_PD_TRSV1);

	writel(0, ep->pmu);
}

static void exynos7420_pcie_elbi_pulse(struct exynos7420_pcie *ep, u32 reg)
{
	writel(1, ep->elbi_base + reg);
	udelay(10);
	writel(0, ep->elbi_base + reg);
	udelay(10);
}

static void exynos7420_pcie_phy_config(struct exynos7420_pcie *ep)
{
	u32 val;
	int i;

	val = readl(ep->sysreg);
	val &= ~(SYSREG_PHY_CFG_A | SYSREG_PHY_CFG_B);
	val |= FIELD_PREP(SYSREG_PHY_CFG_B, 1);
	writel(val, ep->sysreg);

	exynos7420_pcie_elbi_pulse(ep, PCIE_PCS_GLOBAL_RESET);
	exynos7420_pcie_elbi_pulse(ep, PCIE_PMA_INIT_RESET);

	for (i = 0; i < ARRAY_SIZE(exynos7420_pcie_phy_cmn); i++)
		writel(exynos7420_pcie_phy_cmn[i], ep->phy_base + PHY_REG(i));

	for (i = 0; i < ARRAY_SIZE(exynos7420_pcie_phy_trsv); i++)
		writel(exynos7420_pcie_phy_trsv[i],
		       ep->phy_base + PHY_REG(PHY_TRSV_BASE + i));

	writel(PHY_PCS_CTRL_VAL, ep->phy_pcs);

	/* Common block reset pulse */
	writel(readl(ep->sysreg) & ~SYSREG_PHY_CMN_RST, ep->sysreg);
	udelay(20);
	writel(readl(ep->sysreg) | SYSREG_PHY_CMN_RST, ep->sysreg);
	udelay(20);
	writel(readl(ep->sysreg) & ~SYSREG_PHY_CMN_RST, ep->sysreg);
	udelay(20);
}

static int exynos7420_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos7420_pcie *ep = to_exynos7420_pcie(pci);
	struct device *dev = pci->dev;
	void __iomem *elbi = ep->elbi_base;
	int ret;

	pp->bridge->ops = &exynos7420_pcie_ops;

	ep->dw_child_ops = pp->bridge->child_ops;
	ep->child_ops = *ep->dw_child_ops;
	ep->child_ops.read = exynos7420_pcie_rd_other_conf;
	ep->child_ops.write = exynos7420_pcie_wr_other_conf;
	pp->bridge->child_ops = &ep->child_ops;

	/* CMU_FSYS1 gates must be open before any PHY, ELBI or DBI access */
	exynos7420_pcie_enable_gates(ep);

	/* Power the BCM4358 and give it time to come up */
	gpiod_set_value_cansleep(ep->wlen_gpio, 1);
	msleep(100);

	exynos7420_pcie_phy_power_on(ep);

	exynos7420_pcie_irq_mask_all(ep);

	ret = devm_request_irq(dev, ep->irq, exynos7420_pcie_irq_handler,
			       IRQF_SHARED, "exynos7420-pcie", ep);
	if (ret) {
		dev_err(dev, "failed to request irq %d: %d\n", ep->irq, ret);
		goto err_power_off;
	}

	/* Release PERST# */
	gpiod_set_value_cansleep(ep->perst_gpio, 0);
	usleep_range(18000, 20000);

	writel(1, elbi + PCIE_APP_REQ_EXIT_L1_MODE);
	writel(1, elbi + PCIE_CORE_RESETN_DISABLE);
	writel(readl(ep->cmu) | CMU_PCIE_REFCLK_SEL, ep->cmu);

	exynos7420_pcie_phy_config(ep);

	writel(1, elbi + PCIE_L1_BUG_FIX_ENABLE);

	writel(readl(elbi + PCIE_SW_WAKE) & ~PCIE_BUS_EN, elbi + PCIE_SW_WAKE);

	writel(0, elbi + PCIE_IRQ_EN_PULSE);
	writel(IRQ_MSI_ENABLE, elbi + PCIE_IRQ_EN_LEVEL);
	writel(0, elbi + PCIE_IRQ_EN_SPECIAL);

	return 0;

err_power_off:
	exynos7420_pcie_phy_power_off(ep);
	gpiod_set_value_cansleep(ep->wlen_gpio, 0);
	return ret;
}

static void exynos7420_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos7420_pcie *ep = to_exynos7420_pcie(pci);

	exynos7420_pcie_irq_mask_all(ep);
	devm_free_irq(pci->dev, ep->irq, ep);

	gpiod_set_value_cansleep(ep->perst_gpio, 1);
	writel(0, ep->elbi_base + PCIE_APP_LTSSM_ENABLE);
	exynos7420_pcie_phy_power_off(ep);
	gpiod_set_value_cansleep(ep->wlen_gpio, 0);

	/* The FSYS1 gates stay on: several of them are shared with UFS. */
}

static const struct dw_pcie_host_ops exynos7420_pcie_host_ops = {
	.host_init = exynos7420_pcie_host_init,
	.host_deinit = exynos7420_pcie_host_deinit,
};

static const struct dw_pcie_ops exynos7420_dw_pcie_ops = {
	.read_dbi = exynos7420_pcie_read_dbi,
	.write_dbi = exynos7420_pcie_write_dbi,
	.link_up = exynos7420_pcie_link_up,
	.start_link = exynos7420_pcie_start_link,
	.stop_link = exynos7420_pcie_stop_link,
};

/*
 * The small control words and the gate block sit inside regions owned by
 * the PMU, SYSREG and CMU_FSYS1 drivers, so map them without claiming.
 */
static void __iomem *exynos7420_pcie_map_shared(struct platform_device *pdev,
						const char *name)
{
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(&pdev->dev, "missing \"%s\" reg\n", name);
		return IOMEM_ERR_PTR(-EINVAL);
	}

	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!base)
		return IOMEM_ERR_PTR(-ENOMEM);

	return base;
}

static int exynos7420_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7420_pcie *ep;
	struct dw_pcie_rp *pp;
	int ret;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->pci.dev = dev;
	ep->pci.ops = &exynos7420_dw_pcie_ops;
	raw_spin_lock_init(&ep->l1_lock);

	ep->elbi_base = devm_platform_ioremap_resource_byname(pdev, "elbi");
	if (IS_ERR(ep->elbi_base))
		return PTR_ERR(ep->elbi_base);

	ep->phy_base = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(ep->phy_base))
		return PTR_ERR(ep->phy_base);

	ep->sysreg = exynos7420_pcie_map_shared(pdev, "block");
	if (IS_ERR(ep->sysreg))
		return PTR_ERR(ep->sysreg);

	ep->pmu = exynos7420_pcie_map_shared(pdev, "pmu");
	if (IS_ERR(ep->pmu))
		return PTR_ERR(ep->pmu);

	ep->cmu = exynos7420_pcie_map_shared(pdev, "cmu");
	if (IS_ERR(ep->cmu))
		return PTR_ERR(ep->cmu);

	ep->phy_pcs = exynos7420_pcie_map_shared(pdev, "phy_pcs");
	if (IS_ERR(ep->phy_pcs))
		return PTR_ERR(ep->phy_pcs);

	ep->gates = exynos7420_pcie_map_shared(pdev, "gates");
	if (IS_ERR(ep->gates))
		return PTR_ERR(ep->gates);

	/* Hold the endpoint in reset and unpowered until host_init. */
	ep->perst_gpio = devm_gpiod_get(dev, "perst", GPIOD_OUT_HIGH);
	if (IS_ERR(ep->perst_gpio))
		return dev_err_probe(dev, PTR_ERR(ep->perst_gpio),
				     "failed to get perst gpio\n");

	ep->wlen_gpio = devm_gpiod_get(dev, "wl-en", GPIOD_OUT_LOW);
	if (IS_ERR(ep->wlen_gpio))
		return dev_err_probe(dev, PTR_ERR(ep->wlen_gpio),
				     "failed to get wl-en gpio\n");

	ep->irq = platform_get_irq(pdev, 0);
	if (ep->irq < 0)
		return ep->irq;

	platform_set_drvdata(pdev, ep);

	pp = &ep->pci.pp;
	pp->ops = &exynos7420_pcie_host_ops;
	/* MSIs are dispatched from the ELBI handler, not a chained IRQ. */
	pp->msi_irq[0] = -ENODEV;

	ret = dw_pcie_host_init(pp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize host\n");

	return 0;
}

static void exynos7420_pcie_remove(struct platform_device *pdev)
{
	struct exynos7420_pcie *ep = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&ep->pci.pp);
}

static const struct of_device_id exynos7420_pcie_of_match[] = {
	{ .compatible = "samsung,exynos7420-pcie", },
	{ },
};
MODULE_DEVICE_TABLE(of, exynos7420_pcie_of_match);

static struct platform_driver exynos7420_pcie_driver = {
	.probe		= exynos7420_pcie_probe,
	.remove_new	= exynos7420_pcie_remove,
	.driver = {
		.name	= "exynos7420-pcie",
		.of_match_table = exynos7420_pcie_of_match,
	},
};
module_platform_driver(exynos7420_pcie_driver);

MODULE_DESCRIPTION("Samsung Exynos 7420 PCIe host controller driver");
MODULE_LICENSE("GPL");
