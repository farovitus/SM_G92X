// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bit-banged SPI host on the GPIO lines of one Samsung pin bank.
 *
 * spi-gpio drives each line through gpiolib. On Samsung pinctrl every
 * gpiolib call is a locked read-modify-write of the bank's DAT register,
 * so a bit costs two such calls for SCK, one for MOSI and one read for
 * MISO: about 2.4 us per bit on the Exynos 7420, or 50 KB/s.
 *
 * When SCK, MOSI and MISO share one bank, a bit needs only two register
 * writes. This driver takes the bank lock once per byte and, for each
 * clock edge, writes DAT with the new SCK and MOSI levels and reads it
 * back. The read-back makes the write reach the pins before the next edge,
 * which paces the clock, and it carries the MISO level. Other lines of the
 * bank keep working through gpiolib, which takes the same lock.
 *
 * Chip select is a GPIO handled by the SPI core. The clock runs as fast as
 * the bank's register bus allows (a few MHz) and ignores the requested
 * speed, so setup() refuses devices rated below MIN_DEVICE_HZ.
 */

#include <linux/bits.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/soc/samsung/exynos-gpio-port.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

/* Above the fastest edge rate this driver reaches, with margin. */
#define MIN_DEVICE_HZ	10000000

struct exynos_spi_gpio {
	struct gpio_desc *sck;
	void __iomem *dat;
	raw_spinlock_t *lock;
	u32 sck_mask;
	u32 mosi_mask;
	u32 miso_mask;
};

/* Set the pins for one edge and return DAT once the write has landed. */
static inline u32 exynos_spi_gpio_edge(void __iomem *dat, u32 val)
{
	writel_relaxed(val, dat);
	return readl(dat);
}

static u8 exynos_spi_gpio_byte(struct exynos_spi_gpio *bus, u8 out, bool cpol,
			       bool cpha)
{
	u32 idle = cpol ? bus->sck_mask : 0;
	u32 active = idle ^ bus->sck_mask;
	unsigned long flags;
	u32 base, mosi = 0, r;
	u8 in = 0;
	int bit;

	raw_spin_lock_irqsave(bus->lock, flags);
	/* Re-read under the lock: other drivers own the bank's other lines. */
	base = readl_relaxed(bus->dat) & ~(bus->sck_mask | bus->mosi_mask);

	for (bit = 7; bit >= 0; bit--) {
		mosi = out & BIT(bit) ? bus->mosi_mask : 0;
		if (cpha) {
			/*
			 * The leading edge shifts out; sample on the trailing
			 * edge. The device changes MISO only on the next
			 * leading edge, so the read-back after the trailing
			 * edge holds this bit.
			 */
			exynos_spi_gpio_edge(bus->dat, base | mosi | active);
			r = exynos_spi_gpio_edge(bus->dat, base | mosi | idle);
		} else {
			/* MOSI is set while SCK idles; sample on the leading edge. */
			exynos_spi_gpio_edge(bus->dat, base | mosi | idle);
			r = exynos_spi_gpio_edge(bus->dat, base | mosi | active);
		}
		if (r & bus->miso_mask)
			in |= BIT(bit);
	}
	if (!cpha)
		exynos_spi_gpio_edge(bus->dat, base | mosi | idle);

	raw_spin_unlock_irqrestore(bus->lock, flags);
	return in;
}

static int exynos_spi_gpio_transfer_one(struct spi_controller *host,
					struct spi_device *spi,
					struct spi_transfer *xfer)
{
	struct exynos_spi_gpio *bus = spi_controller_get_devdata(host);
	bool cpol = spi->mode & SPI_CPOL;
	bool cpha = spi->mode & SPI_CPHA;
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;
	unsigned int i;
	u8 in;

	for (i = 0; i < xfer->len; i++) {
		in = exynos_spi_gpio_byte(bus, tx ? tx[i] : 0, cpol, cpha);
		if (rx)
			rx[i] = in;
	}
	return 0;
}

/* The core asserts chip select after this, so SCK idles at the right level. */
static int exynos_spi_gpio_prepare_message(struct spi_controller *host,
					   struct spi_message *msg)
{
	struct exynos_spi_gpio *bus = spi_controller_get_devdata(host);

	gpiod_set_value_cansleep(bus->sck, !!(msg->spi->mode & SPI_CPOL));
	return 0;
}

static int exynos_spi_gpio_setup(struct spi_device *spi)
{
	if (spi->max_speed_hz && spi->max_speed_hz < MIN_DEVICE_HZ) {
		dev_err(&spi->dev, "rated %u Hz, this bus may clock faster\n",
			spi->max_speed_hz);
		return -EINVAL;
	}
	return 0;
}

static int exynos_spi_gpio_get(struct device *dev, const char *con,
			       enum gpiod_flags flags, struct gpio_desc **desc,
			       struct samsung_gpio_port *port)
{
	int ret;

	*desc = devm_gpiod_get(dev, con, flags);
	if (IS_ERR(*desc))
		return dev_err_probe(dev, PTR_ERR(*desc), "%s gpio\n", con);
	if (gpiod_is_active_low(*desc))
		return dev_err_probe(dev, -EINVAL, "%s gpio is active low\n", con);
	ret = samsung_gpio_get_port(*desc, port);
	if (ret)
		return dev_err_probe(dev, ret, "%s is not a Samsung GPIO\n", con);
	return 0;
}

static int exynos_spi_gpio_probe(struct platform_device *pdev)
{
	struct samsung_gpio_port sck, mosi, miso;
	struct device *dev = &pdev->dev;
	struct gpio_desc *mosi_desc, *miso_desc;
	struct spi_controller *host;
	struct exynos_spi_gpio *bus;
	int ret;

	host = devm_spi_alloc_host(dev, sizeof(*bus));
	if (!host)
		return -ENOMEM;
	bus = spi_controller_get_devdata(host);

	ret = exynos_spi_gpio_get(dev, "sck", GPIOD_OUT_LOW, &bus->sck, &sck);
	if (ret)
		return ret;
	ret = exynos_spi_gpio_get(dev, "mosi", GPIOD_OUT_LOW, &mosi_desc, &mosi);
	if (ret)
		return ret;
	ret = exynos_spi_gpio_get(dev, "miso", GPIOD_IN, &miso_desc, &miso);
	if (ret)
		return ret;
	if (sck.dat != mosi.dat || sck.dat != miso.dat)
		return dev_err_probe(dev, -EINVAL,
				     "sck, mosi and miso must share a bank\n");

	bus->dat = sck.dat;
	bus->lock = sck.lock;
	bus->sck_mask = sck.mask;
	bus->mosi_mask = mosi.mask;
	bus->miso_mask = miso.mask;

	host->dev.of_node = dev->of_node;
	host->bus_num = -1;
	host->num_chipselect = 1;
	host->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	host->bits_per_word_mask = SPI_BPW_MASK(8);
	host->use_gpio_descriptors = true;
	host->setup = exynos_spi_gpio_setup;
	host->prepare_message = exynos_spi_gpio_prepare_message;
	host->transfer_one = exynos_spi_gpio_transfer_one;

	return devm_spi_register_controller(dev, host);
}

static const struct of_device_id exynos_spi_gpio_of_match[] = {
	{ .compatible = "samsung,exynos-spi-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_spi_gpio_of_match);

static struct platform_driver exynos_spi_gpio_driver = {
	.driver = {
		.name = "exynos-spi-gpio",
		.of_match_table = exynos_spi_gpio_of_match,
	},
	.probe = exynos_spi_gpio_probe,
};
module_platform_driver(exynos_spi_gpio_driver);

MODULE_DESCRIPTION("Bit-banged SPI host on a Samsung GPIO bank");
MODULE_LICENSE("GPL");
