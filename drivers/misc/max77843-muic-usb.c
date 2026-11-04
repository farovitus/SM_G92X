// SPDX-License-Identifier: GPL-2.0-only
/*
 * MAX77843 MUIC: route the micro-USB data lines to the SoC's USB PHY.
 *
 * The MUIC's COM switches connect the connector's D+/D- to the USB PHY,
 * a UART or nothing. S-Boot leaves them open (CONTROL1 = 0x00), so the
 * host never sees the USB device. The mainline extcon-max77843 driver
 * needs the MAX77843 MFD, which this board does not use: the fuel gauge
 * and charger are separate I2C clients on a bit-banged bus. This driver
 * binds to the MUIC address on that bus and sets the USB path at probe,
 * with the same register writes as extcon-max77843's set_path.
 *
 * There is no attach/detach handling yet: the switch stays on USB. The
 * MUIC's charger-type detection runs on its own, so charging still works.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>

#define MUIC_REG_ID		0x00
#define MUIC_REG_STATUS2	0x05
#define MUIC_REG_CONTROL1	0x0c
#define MUIC_REG_CONTROL2	0x0d

#define CONTROL1_COMN1SW	GENMASK(2, 0)
#define CONTROL1_COMP2SW	GENMASK(5, 3)
#define CONTROL1_COM_USB	0x1
#define CONTROL1_SW_USB		(FIELD_PREP(CONTROL1_COMN1SW, CONTROL1_COM_USB) | \
				 FIELD_PREP(CONTROL1_COMP2SW, CONTROL1_COM_USB))
#define CONTROL2_LOWPWR		BIT(0)
#define CONTROL2_CPEN		BIT(2)
#define STATUS2_CHGTYP		GENMASK(2, 0)

static int muic_update(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		return ret;
	return i2c_smbus_write_byte_data(client, reg, (ret & ~mask) | val);
}

static int max77843_muic_usb_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int id, status2, ret;

	id = i2c_smbus_read_byte_data(client, MUIC_REG_ID);
	if (id < 0)
		return dev_err_probe(dev, id, "MUIC does not respond\n");
	status2 = i2c_smbus_read_byte_data(client, MUIC_REG_STATUS2);

	ret = muic_update(client, MUIC_REG_CONTROL1,
			  CONTROL1_COMN1SW | CONTROL1_COMP2SW, CONTROL1_SW_USB);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot switch to USB\n");

	ret = muic_update(client, MUIC_REG_CONTROL2,
			  CONTROL2_LOWPWR | CONTROL2_CPEN, CONTROL2_CPEN);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot leave low-power mode\n");

	dev_info(dev, "ID 0x%02x, charger type %d, data lines on USB\n", id,
		 status2 < 0 ? status2 : (int)FIELD_GET(STATUS2_CHGTYP, status2));
	return 0;
}

static const struct of_device_id max77843_muic_usb_of_match[] = {
	{ .compatible = "maxim,max77843-muic-usb" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77843_muic_usb_of_match);

static struct i2c_driver max77843_muic_usb_driver = {
	.driver = {
		.name = "max77843-muic-usb",
		.of_match_table = max77843_muic_usb_of_match,
	},
	.probe = max77843_muic_usb_probe,
};
module_i2c_driver(max77843_muic_usb_driver);

MODULE_DESCRIPTION("MAX77843 MUIC USB data-path switch");
MODULE_LICENSE("GPL");
