// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cypress CY8CMBR31x5 capacitive touch key driver
 *
 * Supports the two-key (Recents / Back) controller of the Galaxy S6
 * (SM-G920F) running its factory firmware. The key status register
 * holds a 2-bit field per key: an odd value is a press, 2 is a release
 * and 0 means no change.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pm.h>
#include <linux/property.h>

#define MBR_REG_KEY_STATUS	0x03
#define MBR_REG_FW_VER		0x04

#define MBR_MAX_KEYS		4
#define MBR_KEY_FIELD_BITS	2
#define MBR_KEY_FIELD_MASK	GENMASK(1, 0)

struct mbr_touchkey {
	struct i2c_client *client;
	struct input_dev *input;
	unsigned int num_keys;
	u32 keycodes[MBR_MAX_KEYS];
};

static const u32 mbr_default_keycodes[] = {
	KEY_APPSELECT,		/* slot 0, bits [1:0] */
	KEY_BACK,		/* slot 1, bits [3:2] */
};

static irqreturn_t mbr_touchkey_irq(int irq, void *data)
{
	struct mbr_touchkey *tk = data;
	unsigned int i, field;
	int status;

	status = i2c_smbus_read_byte_data(tk->client, MBR_REG_KEY_STATUS);
	if (status < 0) {
		dev_err_ratelimited(&tk->client->dev,
				    "failed to read key status: %d\n", status);
		return IRQ_HANDLED;
	}

	for (i = 0; i < tk->num_keys; i++) {
		field = (status >> (i * MBR_KEY_FIELD_BITS)) & MBR_KEY_FIELD_MASK;
		if (!field)
			continue;

		input_report_key(tk->input, tk->keycodes[i], field & 1);
	}
	input_sync(tk->input);

	return IRQ_HANDLED;
}

static void mbr_touchkey_release_keys(struct mbr_touchkey *tk)
{
	unsigned int i;

	for (i = 0; i < tk->num_keys; i++)
		input_report_key(tk->input, tk->keycodes[i], 0);
	input_sync(tk->input);
}

static int mbr_touchkey_parse_keymap(struct mbr_touchkey *tk)
{
	struct device *dev = &tk->client->dev;
	int count, ret;

	count = device_property_count_u32(dev, "linux,keycodes");
	if (count > MBR_MAX_KEYS) {
		dev_err(dev, "too many keycodes: %d (max %d)\n",
			count, MBR_MAX_KEYS);
		return -EINVAL;
	}

	if (count > 0) {
		ret = device_property_read_u32_array(dev, "linux,keycodes",
						     tk->keycodes, count);
		if (!ret) {
			tk->num_keys = count;
			return 0;
		}
	}

	dev_info(dev, "no usable linux,keycodes, using the default keymap\n");
	memcpy(tk->keycodes, mbr_default_keycodes,
	       sizeof(mbr_default_keycodes));
	tk->num_keys = ARRAY_SIZE(mbr_default_keycodes);

	return 0;
}

static int mbr_touchkey_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mbr_touchkey *tk;
	struct input_dev *input;
	u8 fw[2];
	unsigned int i;
	int ret;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return dev_err_probe(dev, -ENXIO,
				     "I2C adapter lacks SMBus support\n");

	tk = devm_kzalloc(dev, sizeof(*tk), GFP_KERNEL);
	if (!tk)
		return -ENOMEM;

	tk->client = client;
	i2c_set_clientdata(client, tk);

	ret = i2c_smbus_read_i2c_block_data(client, MBR_REG_FW_VER,
					    sizeof(fw), fw);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to read firmware version\n");
	if (ret != sizeof(fw))
		return dev_err_probe(dev, -EIO,
				     "short firmware version read\n");

	dev_info(dev, "firmware 0x%02x, module 0x%02x\n", fw[0], fw[1]);

	ret = mbr_touchkey_parse_keymap(tk);
	if (ret)
		return ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	tk->input = input;
	input->name = "sec_touchkey";
	input->id.bustype = BUS_I2C;
	input->keycode = tk->keycodes;
	input->keycodesize = sizeof(tk->keycodes[0]);
	input->keycodemax = tk->num_keys;

	for (i = 0; i < tk->num_keys; i++)
		input_set_capability(input, EV_KEY, tk->keycodes[i]);

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register input device\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					mbr_touchkey_irq, IRQF_ONESHOT,
					dev_name(dev), tk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	return 0;
}

static int mbr_touchkey_suspend(struct device *dev)
{
	struct mbr_touchkey *tk = dev_get_drvdata(dev);

	disable_irq(tk->client->irq);
	mbr_touchkey_release_keys(tk);

	return 0;
}

static int mbr_touchkey_resume(struct device *dev)
{
	struct mbr_touchkey *tk = dev_get_drvdata(dev);

	enable_irq(tk->client->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mbr_touchkey_pm_ops,
				mbr_touchkey_suspend, mbr_touchkey_resume);

static const struct of_device_id mbr_touchkey_of_match[] = {
	{ .compatible = "cypress,mbr31x5-touchkey" },
	{ }
};
MODULE_DEVICE_TABLE(of, mbr_touchkey_of_match);

static const struct i2c_device_id mbr_touchkey_i2c_id[] = {
	{ "mbr31x5-touchkey" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mbr_touchkey_i2c_id);

static struct i2c_driver mbr_touchkey_driver = {
	.driver = {
		.name = "cypress-mbr-touchkey",
		.of_match_table = mbr_touchkey_of_match,
		.pm = pm_sleep_ptr(&mbr_touchkey_pm_ops),
	},
	.probe = mbr_touchkey_probe,
	.id_table = mbr_touchkey_i2c_id,
};
module_i2c_driver(mbr_touchkey_driver);

MODULE_DESCRIPTION("Cypress CY8CMBR31x5 capacitive touch key driver");
MODULE_LICENSE("GPL");
