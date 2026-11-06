// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S2MPB02 sub-PMIC regulator driver
 *
 * Only LDO17 is described. On the Galaxy S6 (SM-G920F) it supplies the
 * touchscreen analog rail. The other outputs, the IRQ block and the flash
 * LED are left as the boot loader configured them.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define S2MPB02_REG_ID			0x00
#define S2MPB02_REG_L17CTRL		0x2e

#define S2MPB02_LDO_VSEL_MASK		GENMASK(6, 0)
#define S2MPB02_LDO_ENABLE_MASK		BIT(7)
#define S2MPB02_LDO_N_VOLTAGES		128
#define S2MPB02_LDO_MIN_UV		600000
#define S2MPB02_LDO_STEP_UV		25000
#define S2MPB02_LDO_ENABLE_TIME_US	180

static bool s2mpb02_readable_reg(struct device *dev, unsigned int reg)
{
	return reg == S2MPB02_REG_ID || reg == S2MPB02_REG_L17CTRL;
}

static bool s2mpb02_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg == S2MPB02_REG_L17CTRL;
}

static const struct regmap_config s2mpb02_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = S2MPB02_REG_L17CTRL,
	.readable_reg = s2mpb02_readable_reg,
	.writeable_reg = s2mpb02_writeable_reg,
	.cache_type = REGCACHE_NONE,
};

static const struct regulator_ops s2mpb02_ldo_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc s2mpb02_ldo17_desc = {
	.name = "ldo17",
	.of_match = "ldo17",
	.regulators_node = "regulators",
	.id = 17,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &s2mpb02_ldo_ops,
	.n_voltages = S2MPB02_LDO_N_VOLTAGES,
	.min_uV = S2MPB02_LDO_MIN_UV,
	.uV_step = S2MPB02_LDO_STEP_UV,
	.vsel_reg = S2MPB02_REG_L17CTRL,
	.vsel_mask = S2MPB02_LDO_VSEL_MASK,
	.enable_reg = S2MPB02_REG_L17CTRL,
	.enable_mask = S2MPB02_LDO_ENABLE_MASK,
	.enable_time = S2MPB02_LDO_ENABLE_TIME_US,
};

static int s2mpb02_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regmap *regmap;
	unsigned int id;
	int ret;

	regmap = devm_regmap_init_i2c(client, &s2mpb02_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to initialise regmap\n");

	ret = regmap_read(regmap, S2MPB02_REG_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");

	dev_info(dev, "S2MPB02 chip ID 0x%02x\n", id);

	config.dev = dev;
	config.regmap = regmap;

	rdev = devm_regulator_register(dev, &s2mpb02_ldo17_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register %s\n",
				     s2mpb02_ldo17_desc.name);

	return 0;
}

static const struct of_device_id s2mpb02_of_match[] = {
	{ .compatible = "samsung,s2mpb02" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpb02_of_match);

static const struct i2c_device_id s2mpb02_i2c_id[] = {
	{ "s2mpb02-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mpb02_i2c_id);

static struct i2c_driver s2mpb02_regulator_driver = {
	.driver = {
		.name = "s2mpb02-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = s2mpb02_of_match,
	},
	.probe = s2mpb02_probe,
	.id_table = s2mpb02_i2c_id,
};
module_i2c_driver(s2mpb02_regulator_driver);

MODULE_DESCRIPTION("Samsung S2MPB02 sub-PMIC regulator driver");
MODULE_LICENSE("GPL");
