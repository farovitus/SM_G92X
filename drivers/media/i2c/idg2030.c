// SPDX-License-Identifier: GPL-2.0-only
/*
 * InvenSense IDG2030 Optical Image Stabilization (OIS) companion driver
 *
 * Powers the OIS driver IC over its dedicated rails and exposes simple
 * sysfs controls under /sys/class/camera/ois/ to enable or center the
 * OIS actuator while the rear camera is active.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <soc/samsung/camera_class.h>

#define IDG2030_REG_CTRL		0x0000
#define IDG2030_REG_MODE		0x0002
#define IDG2030_REG_STATUS		0x0006
#define IDG2030_REG_FW_VER_LO		0x00fc
#define IDG2030_REG_FW_VER_HI		0x00fd

#define IDG2030_MODE_NORMAL		0x00
#define IDG2030_MODE_CENTER		0x02
#define IDG2030_CTRL_ENABLE		0x01

static const char * const idg2030_supply_names[] = {
	"vddio",
	"ois-vdd",
	"ois-vm",
};

struct idg2030_dev {
	struct i2c_client *client;
	struct regulator_bulk_data supplies[ARRAY_SIZE(idg2030_supply_names)];
	struct device *cam_dev;
	struct mutex lock;
	bool powered;
	int mode;
	u8 fw_lo;
	u8 fw_hi;
	bool fw_read;
};

static int idg2030_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	u8 tx[2] = { (reg >> 8) & 0xff, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(tx),
			.buf = tx,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static int idg2030_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3] = { (reg >> 8) & 0xff, reg & 0xff, val };
	int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		return ret;
	return ret == sizeof(buf) ? 0 : -EIO;
}

static int idg2030_set_power(struct idg2030_dev *ois, int mode)
{
	int ret = 0;

	mutex_lock(&ois->lock);

	if (mode <= 0) {
		if (ois->powered) {
			idg2030_write_reg(ois->client, IDG2030_REG_CTRL, 0x00);
			regulator_bulk_disable(ARRAY_SIZE(idg2030_supply_names),
					       ois->supplies);
			ois->powered = false;
			ois->mode = 0;
		}
		goto out;
	}

	if (!ois->powered) {
		ret = regulator_bulk_enable(ARRAY_SIZE(idg2030_supply_names),
					    ois->supplies);
		if (ret)
			goto out;
		msleep(20);
		ois->powered = true;
	}

	if (!ois->fw_read) {
		u8 lo = 0, hi = 0, st = 0;

		if (!idg2030_read_reg(ois->client, IDG2030_REG_FW_VER_LO, &lo) &&
		    !idg2030_read_reg(ois->client, IDG2030_REG_FW_VER_HI, &hi)) {
			idg2030_read_reg(ois->client, IDG2030_REG_STATUS, &st);
			ois->fw_lo = lo;
			ois->fw_hi = hi;
			ois->fw_read = true;
			dev_info(&ois->client->dev,
				 "IDG2030 OIS detected (fw=0x%02x%02x status=0x%02x)\n",
				 hi, lo, st);
		}
	}

	ret = idg2030_write_reg(ois->client, IDG2030_REG_MODE,
				mode == 2 ? IDG2030_MODE_CENTER :
					    IDG2030_MODE_NORMAL);
	ret = ret ?: idg2030_write_reg(ois->client, IDG2030_REG_CTRL,
				       IDG2030_CTRL_ENABLE);
	if (!ret) {
		ois->mode = mode;
	} else {
		dev_warn(&ois->client->dev, "OIS mode=%d i2c failed: %d\n",
			 mode, ret);
		regulator_bulk_disable(ARRAY_SIZE(idg2030_supply_names),
				       ois->supplies);
		ois->powered = false;
		ois->mode = 0;
	}

out:
	mutex_unlock(&ois->lock);
	return ret;
}

static ssize_t ois_power_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct idg2030_dev *ois = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", ois->mode);
}

static ssize_t ois_power_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct idg2030_dev *ois = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	ret = idg2030_set_power(ois, (int)val);
	return ret ? ret : count;
}

static ssize_t oisfw_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct idg2030_dev *ois = dev_get_drvdata(dev);

	if (!ois->fw_read)
		return sysfs_emit(buf, "unprobed\n");
	return sysfs_emit(buf, "0x%02x%02x\n", ois->fw_hi, ois->fw_lo);
}

static DEVICE_ATTR(ois_power, 0664, ois_power_show, ois_power_store);
static DEVICE_ATTR_RO(oisfw);

static struct attribute *idg2030_attrs[] = {
	&dev_attr_ois_power.attr,
	&dev_attr_oisfw.attr,
	NULL,
};
ATTRIBUTE_GROUPS(idg2030);

static int idg2030_probe(struct i2c_client *client)
{
	struct idg2030_dev *ois;
	struct class *cam_cls;
	unsigned int i;
	int ret;

	ois = devm_kzalloc(&client->dev, sizeof(*ois), GFP_KERNEL);
	if (!ois)
		return -ENOMEM;

	ois->client = client;
	mutex_init(&ois->lock);
	for (i = 0; i < ARRAY_SIZE(idg2030_supply_names); i++)
		ois->supplies[i].supply = idg2030_supply_names[i];

	ret = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(idg2030_supply_names),
				      ois->supplies);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to get OIS supplies\n");

	i2c_set_clientdata(client, ois);

	cam_cls = samsung_camera_class();
	if (cam_cls) {
		ois->cam_dev = device_create_with_groups(cam_cls, NULL, 0, ois,
							 idg2030_groups, "ois");
		if (IS_ERR(ois->cam_dev)) {
			dev_warn(&client->dev, "camera/ois sysfs failed: %ld\n",
				 PTR_ERR(ois->cam_dev));
			ois->cam_dev = NULL;
		}
	}

	return 0;
}

static void idg2030_remove(struct i2c_client *client)
{
	struct idg2030_dev *ois = i2c_get_clientdata(client);

	idg2030_set_power(ois, 0);
	if (!IS_ERR_OR_NULL(ois->cam_dev))
		device_unregister(ois->cam_dev);
}

static const struct of_device_id idg2030_of_match[] = {
	{ .compatible = "invensense,idg2030" },
	{ }
};
MODULE_DEVICE_TABLE(of, idg2030_of_match);

static const struct i2c_device_id idg2030_i2c_id[] = {
	{ "idg2030" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, idg2030_i2c_id);

static struct i2c_driver idg2030_driver = {
	.driver = {
		.name = "idg2030",
		.of_match_table = idg2030_of_match,
	},
	.probe = idg2030_probe,
	.remove = idg2030_remove,
	.id_table = idg2030_i2c_id,
};
module_i2c_driver(idg2030_driver);

MODULE_DESCRIPTION("InvenSense IDG2030 OIS companion driver");
MODULE_LICENSE("GPL");
