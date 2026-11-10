// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5C73C2 ("73C2") camera companion of the Galaxy S6 rear module.
 *
 * The rear sensor (Sony IMX240, or Samsung S5K2P2 on some units) sends its
 * MIPI stream through this chip, which applies the module calibration and
 * forwards the stream to CSIS0. It has no bypass: nothing reaches the SoC
 * until its firmware runs, and the firmware is loaded by the host on every
 * power-up.
 *
 * The chip answers on the rear camera I2C bus (ID 0x73c2 at register 0,
 * 16-bit big-endian addresses and data), and takes register writes and its
 * firmware over SPI:
 *   write: 0x02, addr[15:8], addr[7:0], data[15:8], data[7:0]
 *   burst: 0x02, 0x6f, 0x12, bytes...   (0x6f12 is the RAM data port)
 * Reads here go over I2C. The SPI pins are bit-banged, which moves the
 * 200 KB firmware in about 5 s.
 *
 * The node owns the whole rear module power sequence for now: probe powers
 * it in the stock order, identifies the sensor, uploads the firmware for it,
 * has the chip compute the CRC32 of what it received and compares it with
 * the file, starts the chip's ARM, applies the master and mode setfiles and
 * powers the module off again. Streaming and the V4L2 side come with the
 * capture path.
 */

#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

#define S5C73C2_CHIP_ID			0x73c2

#define S5C73C2_REG_ID			0x0000
#define S5C73C2_REG_VERSION		0x0002
#define S5C73C2_REG_DOWNLOAD		0x0008
#define S5C73C2_REG_SFLASH_CMD		0x0014
#define S5C73C2_REG_SFLASH_SRAM_ADDR	0x001c
#define S5C73C2_REG_SFLASH_SIZE		0x0020
#define S5C73C2_REG_SFLASH_CRC32	0x002c
#define S5C73C2_REG_M2M_CAL		0x0044
#define S5C73C2_REG_ARM_GO		0x6014
#define S5C73C2_REG_ARM_REMAP		0x6048
#define S5C73C2_REG_RAM_ADDR_HI		0x6428
#define S5C73C2_REG_RAM_ADDR_LO		0x642a
#define S5C73C2_REG_HOST_INTRP		0x6806
#define S5C73C2_REG_RAM_DATA		0x6f12

#define S5C73C2_VERSION_EVT1		0x00b0
#define S5C73C2_FW_ADDR			0x00080000
#define S5C73C2_SFLASH_CMD_CRC32	0x000c
#define S5C73C2_SIGNATURE_CLEAR		0xbeef

#define S5C73C2_SPI_WRITE		0x02
/* One SPI packet: the command, the data port address, 253 data bytes. */
#define S5C73C2_BURST_LEN		256
#define S5C73C2_BURST_HDR		3
/* The firmware ends with its version string; the last 4 bytes are a CRC. */
#define S5C73C2_FW_TAIL			16
#define S5C73C2_FW_CRC_LEN		4
/* A setfile is (addr, value) pairs followed by a 16-byte version. */
#define S5C73C2_SETFILE_TAIL		16

struct s5c73c2_supply {
	const char *name;
	int microvolt;		/* 0: keep the rail's fixed level */
	unsigned int delay_us;	/* after enabling */
};

/* Stock power-on order of the IMX240 module; IMX240 levels. */
static const struct s5c73c2_supply s5c73c2_supplies[] = {
	{ "vddaf",	2800000, 2000 },
	{ "vdda",	2800000, 0 },
	{ "ois-vdd",	0, 0 },
	{ "ois-vm",	0, 2500 },
	{ "vddio",	0, 0 },
	{ "vddd",	1050000, 0 },
	{ "comp-ret",	1000000, 0 },
	{ "comp-vddio",	0, 0 },
	{ "comp-vdda",	0, 0 },
	{ "comp-vddd",	0, 0 },
	{ "comp-noret",	0, 0 },
	{ "comp-core",	800000, 0 },
};

struct s5c73c2_sensor {
	const char *name;
	u16 addr;
	u16 id_reg;
	u16 id;
};

static const struct s5c73c2_sensor s5c73c2_sensors[] = {
	{ "imx240", 0x1a, 0x0016, 0x0240 },
	/* S5K2P2: its ID is not known here, so any answer at its address. */
	{ "2p2", 0x2d, 0x0000, 0 },
};

struct s5c73c2 {
	struct device *dev;
	struct i2c_client *client;
	struct spi_device *spi;
	struct regulator *supplies[ARRAY_SIZE(s5c73c2_supplies)];
	struct clk *mclk;
	struct clk *sensor_mclk;
	struct gpio_desc *reset;
	struct gpio_desc *sensor_reset;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_off;
	struct pinctrl_state *pins_on;
	u8 *tx;
};

static int s5c73c2_read(struct s5c73c2 *c2, u16 addr, u16 reg, u16 *val)
{
	u8 wbuf[2], rbuf[2];
	struct i2c_msg msgs[2] = {
		{ .addr = addr, .len = 2, .buf = wbuf },
		{ .addr = addr, .flags = I2C_M_RD, .len = 2, .buf = rbuf },
	};
	int ret;

	put_unaligned_be16(reg, wbuf);
	ret = i2c_transfer(c2->client->adapter, msgs, 2);
	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = get_unaligned_be16(rbuf);
	return 0;
}

static int s5c73c2_write(struct s5c73c2 *c2, u16 reg, u16 val)
{
	c2->tx[0] = S5C73C2_SPI_WRITE;
	put_unaligned_be16(reg, &c2->tx[1]);
	put_unaligned_be16(val, &c2->tx[3]);
	return spi_write(c2->spi, c2->tx, 5);
}

/* Write a 32-bit value as two 16-bit registers, high half first. */
static int s5c73c2_write32(struct s5c73c2 *c2, u16 reg, u32 val)
{
	int ret = s5c73c2_write(c2, reg, val >> 16);

	return ret ?: s5c73c2_write(c2, reg + 2, val & 0xffff);
}

static int s5c73c2_burst(struct s5c73c2 *c2, const u8 *data, size_t size)
{
	const size_t chunk = S5C73C2_BURST_LEN - S5C73C2_BURST_HDR;
	size_t off, len;
	int ret;

	c2->tx[0] = S5C73C2_SPI_WRITE;
	put_unaligned_be16(S5C73C2_REG_RAM_DATA, &c2->tx[1]);
	for (off = 0; off < size; off += len) {
		len = min(chunk, size - off);
		memcpy(&c2->tx[S5C73C2_BURST_HDR], data + off, len);
		ret = spi_write(c2->spi, c2->tx, S5C73C2_BURST_HDR + len);
		if (ret)
			return ret;
	}
	return 0;
}

static void s5c73c2_power_off(struct s5c73c2 *c2, int nsup)
{
	gpiod_set_value_cansleep(c2->sensor_reset, 1);
	gpiod_set_value_cansleep(c2->reset, 1);
	usleep_range(10, 20);
	pinctrl_select_state(c2->pinctrl, c2->pins_off);
	clk_disable_unprepare(c2->sensor_mclk);
	clk_disable_unprepare(c2->mclk);
	while (nsup--)
		regulator_disable(c2->supplies[nsup]);
}

static int s5c73c2_power_on(struct s5c73c2 *c2)
{
	int i, ret;

	gpiod_set_value_cansleep(c2->sensor_reset, 1);
	gpiod_set_value_cansleep(c2->reset, 1);

	for (i = 0; i < ARRAY_SIZE(s5c73c2_supplies); i++) {
		const struct s5c73c2_supply *s = &s5c73c2_supplies[i];

		if (s->microvolt) {
			ret = regulator_set_voltage(c2->supplies[i],
						    s->microvolt, s->microvolt);
			if (ret)
				goto err_supply;
		}
		ret = regulator_enable(c2->supplies[i]);
		if (ret)
			goto err_supply;
		if (s->delay_us)
			usleep_range(s->delay_us, s->delay_us + 100);
	}
	usleep_range(1000, 1100);

	ret = clk_set_rate(c2->mclk, 24000000);
	ret = ret ?: clk_prepare_enable(c2->mclk);
	if (ret)
		goto err_supply;
	ret = clk_set_rate(c2->sensor_mclk, 24000000);
	ret = ret ?: clk_prepare_enable(c2->sensor_mclk);
	if (ret)
		goto err_mclk;
	ret = pinctrl_select_state(c2->pinctrl, c2->pins_on);
	if (ret)
		goto err_sensor_mclk;
	usleep_range(2000, 2100);

	gpiod_set_value_cansleep(c2->reset, 0);
	usleep_range(300, 400);
	gpiod_set_value_cansleep(c2->sensor_reset, 0);
	usleep_range(2000, 2100);
	return 0;

err_sensor_mclk:
	clk_disable_unprepare(c2->sensor_mclk);
err_mclk:
	clk_disable_unprepare(c2->mclk);
err_supply:
	dev_err(c2->dev, "power on at %s: %d\n",
		i < ARRAY_SIZE(s5c73c2_supplies) ? s5c73c2_supplies[i].name :
		"mclk", ret);
	while (i--)
		regulator_disable(c2->supplies[i]);
	return ret;
}

static const struct s5c73c2_sensor *s5c73c2_find_sensor(struct s5c73c2 *c2)
{
	int i;
	u16 id;

	for (i = 0; i < ARRAY_SIZE(s5c73c2_sensors); i++) {
		const struct s5c73c2_sensor *s = &s5c73c2_sensors[i];

		if (s5c73c2_read(c2, s->addr, s->id_reg, &id))
			continue;
		if (!s->id || id == s->id)
			return s;
		dev_warn(c2->dev, "0x%02x: ID 0x%04x, not %s\n", s->addr, id,
			 s->name);
	}
	return NULL;
}

/* Have the chip compute the CRC32 of its RAM and compare it with the file. */
static int s5c73c2_check_crc(struct s5c73c2 *c2, u32 addr, u32 size, u32 want)
{
	u16 cmd, hi, lo;
	int ret, i;
	u32 got;

	ret = s5c73c2_write32(c2, S5C73C2_REG_SFLASH_CRC32, 0);
	ret = ret ?: s5c73c2_write32(c2, S5C73C2_REG_SFLASH_SRAM_ADDR, addr);
	ret = ret ?: s5c73c2_write32(c2, S5C73C2_REG_SFLASH_SIZE, size);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_SFLASH_CMD,
				   S5C73C2_SFLASH_CMD_CRC32);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_HOST_INTRP, 1);
	if (ret)
		return ret;

	for (i = 0; i < 1000; i++) {
		ret = s5c73c2_read(c2, c2->client->addr,
				   S5C73C2_REG_SFLASH_CMD, &cmd);
		if (!ret && !cmd)
			break;
		usleep_range(1000, 1100);
	}
	if (ret || cmd) {
		dev_err(c2->dev, "CRC command not done (cmd 0x%04x, %d)\n",
			cmd, ret);
		return ret ?: -ETIMEDOUT;
	}

	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_SFLASH_CRC32,
			   &hi);
	ret = ret ?: s5c73c2_read(c2, c2->client->addr,
				  S5C73C2_REG_SFLASH_CRC32 + 2, &lo);
	if (ret)
		return ret;
	/* High half first; equal to the file's trailer read little-endian. */
	got = (u32)hi << 16 | lo;

	if (got != want) {
		dev_err(c2->dev, "CRC32 %08x, file %08x\n", got, want);
		return -EBADMSG;
	}
	return 0;
}

static int s5c73c2_load_fw(struct s5c73c2 *c2, const struct firmware *fw)
{
	size_t len = fw->size - S5C73C2_FW_CRC_LEN;
	u32 want = get_unaligned_le32(fw->data + len);
	ktime_t start;
	int ret;
	u16 val;

	ret = s5c73c2_write(c2, S5C73C2_REG_DOWNLOAD, 1);
	if (ret)
		return ret;
	usleep_range(1000, 1100);
	ret = s5c73c2_write32(c2, S5C73C2_REG_RAM_ADDR_HI, S5C73C2_FW_ADDR);
	if (ret)
		return ret;

	/* Fail now rather than after the upload if SPI writes do not land. */
	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_RAM_ADDR_HI, &val);
	if (ret)
		return ret;
	if (val != S5C73C2_FW_ADDR >> 16) {
		dev_err(c2->dev, "SPI write not seen over I2C (0x%04x)\n", val);
		return -EIO;
	}

	start = ktime_get();
	ret = s5c73c2_burst(c2, fw->data, fw->size);
	if (ret)
		return ret;
	dev_dbg(c2->dev, "firmware %zu bytes in %lld ms\n", fw->size,
		ktime_ms_delta(ktime_get(), start));

	return s5c73c2_check_crc(c2, S5C73C2_FW_ADDR, len, want);
}

static int s5c73c2_load_setfile(struct s5c73c2 *c2, const char *name)
{
	const struct firmware *fw;
	size_t i, len;
	int ret;

	ret = request_firmware(&fw, name, c2->dev);
	if (ret)
		return ret;
	if (fw->size < S5C73C2_SETFILE_TAIL ||
	    (fw->size - S5C73C2_SETFILE_TAIL) % 4) {
		ret = -EINVAL;
		goto out;
	}
	len = fw->size - S5C73C2_SETFILE_TAIL;
	for (i = 0; i < len && !ret; i += 4)
		ret = s5c73c2_write(c2, get_unaligned_be16(fw->data + i),
				    get_unaligned_be16(fw->data + i + 2));
	dev_dbg(c2->dev, "%s: %zu writes, %.16s\n", name, len / 4,
		fw->data + len);
out:
	release_firmware(fw);
	return ret;
}

static int s5c73c2_boot(struct s5c73c2 *c2, const char *sensor)
{
	const struct firmware *fw;
	char name[48];
	int ret, try;
	u16 val;

	snprintf(name, sizeof(name), "companion_fw_%s_evt1.bin", sensor);
	ret = request_firmware(&fw, name, c2->dev);
	if (ret)
		return ret;
	if (fw->size <= S5C73C2_FW_TAIL) {
		ret = -EINVAL;
		goto out;
	}
	if ((crc32_le(~0, fw->data, fw->size - S5C73C2_FW_CRC_LEN) ^ ~0) !=
	    get_unaligned_le32(fw->data + fw->size - S5C73C2_FW_CRC_LEN)) {
		dev_err(c2->dev, "%s: CRC does not match its contents\n", name);
		ret = -EBADMSG;
		goto out;
	}
	dev_info(c2->dev, "%s: version %.12s\n", name,
		 fw->data + fw->size - S5C73C2_FW_TAIL);

	for (try = 0; try < 2; try++) {
		ret = s5c73c2_load_fw(c2, fw);
		if (ret != -EBADMSG)
			break;
	}
	if (ret)
		goto out;

	ret = s5c73c2_write(c2, S5C73C2_REG_ARM_REMAP, 1);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_ID, S5C73C2_SIGNATURE_CLEAR);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_ARM_GO, 1);
	if (ret)
		goto out;
	usleep_range(1000, 1100);
	/* The firmware replaces the signature with the chip ID once it runs. */
	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_ID, &val);
	if (ret)
		goto out;
	if (val != S5C73C2_CHIP_ID) {
		dev_err(c2->dev, "firmware did not start (0x%04x)\n", val);
		ret = -EIO;
		goto out;
	}
	/* No calibration from the module's FROM yet. */
	ret = s5c73c2_write(c2, S5C73C2_REG_M2M_CAL, 0);
	if (ret)
		goto out;

	snprintf(name, sizeof(name), "companion_%s_master_setfile.bin", sensor);
	ret = s5c73c2_load_setfile(c2, name);
	if (ret)
		goto out;
	usleep_range(1000, 1100);
	snprintf(name, sizeof(name), "companion_%s_mode_setfile.bin", sensor);
	ret = s5c73c2_load_setfile(c2, name);
out:
	release_firmware(fw);
	return ret;
}

static int s5c73c2_identify_and_boot(struct s5c73c2 *c2)
{
	const struct s5c73c2_sensor *sensor;
	u16 id, ver;
	int ret;

	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_ID, &id);
	ret = ret ?: s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_VERSION,
				  &ver);
	if (ret)
		return dev_err_probe(c2->dev, ret, "no answer on I2C\n");
	if (id != S5C73C2_CHIP_ID)
		return dev_err_probe(c2->dev, -ENODEV, "ID 0x%04x\n", id);
	if (ver != S5C73C2_VERSION_EVT1)
		return dev_err_probe(c2->dev, -ENODEV,
				     "version 0x%04x, only EVT1 is known\n",
				     ver);

	sensor = s5c73c2_find_sensor(c2);
	if (!sensor)
		return dev_err_probe(c2->dev, -ENODEV, "no rear sensor\n");
	dev_info(c2->dev, "73C2 EVT1, sensor %s\n", sensor->name);

	ret = s5c73c2_boot(c2, sensor->name);
	if (ret)
		return dev_err_probe(c2->dev, ret, "firmware start failed\n");
	dev_info(c2->dev, "firmware running\n");
	return 0;
}

static void s5c73c2_put_spi(void *data)
{
	put_device(data);
}

static int s5c73c2_get_spi(struct s5c73c2 *c2)
{
	struct device_node *np;
	struct device *dev;
	int ret;

	np = of_parse_phandle(c2->dev->of_node, "samsung,spi-device", 0);
	if (!np)
		return dev_err_probe(c2->dev, -EINVAL, "no samsung,spi-device\n");
	dev = bus_find_device_by_of_node(&spi_bus_type, np);
	of_node_put(np);
	if (!dev)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(c2->dev, s5c73c2_put_spi, dev);
	if (ret)
		return ret;
	c2->spi = to_spi_device(dev);
	return 0;
}

static int s5c73c2_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5c73c2 *c2;
	int i, ret;

	c2 = devm_kzalloc(dev, sizeof(*c2), GFP_KERNEL);
	if (!c2)
		return -ENOMEM;
	c2->dev = dev;
	c2->client = client;

	c2->tx = devm_kmalloc(dev, S5C73C2_BURST_LEN, GFP_KERNEL);
	if (!c2->tx)
		return -ENOMEM;

	ret = s5c73c2_get_spi(c2);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(s5c73c2_supplies); i++) {
		c2->supplies[i] = devm_regulator_get(dev,
						     s5c73c2_supplies[i].name);
		if (IS_ERR(c2->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(c2->supplies[i]),
					     "%s supply\n",
					     s5c73c2_supplies[i].name);
	}

	c2->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(c2->mclk))
		return dev_err_probe(dev, PTR_ERR(c2->mclk), "mclk\n");
	c2->sensor_mclk = devm_clk_get(dev, "sensor-mclk");
	if (IS_ERR(c2->sensor_mclk))
		return dev_err_probe(dev, PTR_ERR(c2->sensor_mclk),
				     "sensor-mclk\n");

	c2->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(c2->reset))
		return dev_err_probe(dev, PTR_ERR(c2->reset), "reset\n");
	c2->sensor_reset = devm_gpiod_get(dev, "sensor-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(c2->sensor_reset))
		return dev_err_probe(dev, PTR_ERR(c2->sensor_reset),
				     "sensor-reset\n");

	c2->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(c2->pinctrl))
		return dev_err_probe(dev, PTR_ERR(c2->pinctrl), "pinctrl\n");
	c2->pins_off = pinctrl_lookup_state(c2->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(c2->pins_off))
		return dev_err_probe(dev, PTR_ERR(c2->pins_off), "default pins\n");
	c2->pins_on = pinctrl_lookup_state(c2->pinctrl, "active");
	if (IS_ERR(c2->pins_on))
		return dev_err_probe(dev, PTR_ERR(c2->pins_on), "active pins\n");

	ret = s5c73c2_power_on(c2);
	if (ret)
		return ret;
	ret = s5c73c2_identify_and_boot(c2);
	s5c73c2_power_off(c2, ARRAY_SIZE(s5c73c2_supplies));
	return ret;
}

static const struct of_device_id s5c73c2_of_match[] = {
	{ .compatible = "samsung,s5c73c2" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5c73c2_of_match);

static struct i2c_driver s5c73c2_driver = {
	.driver = {
		.name = "s5c73c2",
		.of_match_table = s5c73c2_of_match,
	},
	.probe = s5c73c2_probe,
};
module_i2c_driver(s5c73c2_driver);

MODULE_DESCRIPTION("Samsung S5C73C2 camera companion");
MODULE_LICENSE("GPL");
