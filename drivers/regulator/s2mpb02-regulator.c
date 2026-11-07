// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S2MPB02 sub-PMIC regulator driver
 *
 * LDO1-18 and BUCK1/2 are described. On the Galaxy S6 (SM-G920F) LDO17
 * supplies the touchscreen analog rail and most of the others the camera
 * modules. Outputs without a device-tree node have no constraints, so the
 * regulator core leaves them as the boot loader set them. The buck-boost,
 * the IRQ block and the flash LED are not handled here.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define S2MPB02_REG_ID			0x00
#define S2MPB02_REG_B1CTRL1		0x0b
#define S2MPB02_REG_B1CTRL2		0x0c
#define S2MPB02_REG_B2CTRL1		0x0d
#define S2MPB02_REG_B2CTRL2		0x0e
#define S2MPB02_REG_L1CTRL		0x1e
#define S2MPB02_REG_L18CTRL		0x2f

#define S2MPB02_LDO_VSEL_MASK		GENMASK(6, 0)
#define S2MPB02_LDO_ENABLE_MASK		BIT(7)
#define S2MPB02_LDO_N_VOLTAGES		128
#define S2MPB02_LDO_MIN_UV		600000
#define S2MPB02_LDO_STEP1_UV		12500	/* LDO1-5 */
#define S2MPB02_LDO_STEP2_UV		25000	/* LDO6-18 */
#define S2MPB02_LDO_ENABLE_TIME_US	180

#define S2MPB02_BUCK_VSEL_MASK		GENMASK(7, 0)
#define S2MPB02_BUCK_ENABLE_MASK	GENMASK(7, 6)
#define S2MPB02_BUCK_N_VOLTAGES		256
#define S2MPB02_BUCK_MIN_UV		400000
#define S2MPB02_BUCK_STEP_UV		6250
#define S2MPB02_BUCK_ENABLE_TIME_US	100

static bool s2mpb02_readable_reg(struct device *dev, unsigned int reg)
{
	return reg == S2MPB02_REG_ID ||
	       (reg >= S2MPB02_REG_B1CTRL1 && reg <= S2MPB02_REG_B2CTRL2) ||
	       (reg >= S2MPB02_REG_L1CTRL && reg <= S2MPB02_REG_L18CTRL);
}

static bool s2mpb02_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg != S2MPB02_REG_ID && s2mpb02_readable_reg(dev, reg);
}

static const struct regmap_config s2mpb02_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = S2MPB02_REG_L18CTRL,
	.readable_reg = s2mpb02_readable_reg,
	.writeable_reg = s2mpb02_writeable_reg,
	.cache_type = REGCACHE_NONE,
};

static const struct regulator_ops s2mpb02_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

#define S2MPB02_LDO(_n, _step) {					\
	.name = "ldo" #_n,						\
	.of_match = "ldo" #_n,						\
	.regulators_node = "regulators",				\
	.id = (_n),							\
	.type = REGULATOR_VOLTAGE,					\
	.owner = THIS_MODULE,						\
	.ops = &s2mpb02_ops,						\
	.n_voltages = S2MPB02_LDO_N_VOLTAGES,				\
	.min_uV = S2MPB02_LDO_MIN_UV,					\
	.uV_step = (_step),						\
	.vsel_reg = S2MPB02_REG_L1CTRL + (_n) - 1,			\
	.vsel_mask = S2MPB02_LDO_VSEL_MASK,				\
	.enable_reg = S2MPB02_REG_L1CTRL + (_n) - 1,			\
	.enable_mask = S2MPB02_LDO_ENABLE_MASK,				\
	.enable_time = S2MPB02_LDO_ENABLE_TIME_US,			\
}

#define S2MPB02_BUCK(_n, _id) {						\
	.name = "buck" #_n,						\
	.of_match = "buck" #_n,						\
	.regulators_node = "regulators",				\
	.id = (_id),							\
	.type = REGULATOR_VOLTAGE,					\
	.owner = THIS_MODULE,						\
	.ops = &s2mpb02_ops,						\
	.n_voltages = S2MPB02_BUCK_N_VOLTAGES,				\
	.min_uV = S2MPB02_BUCK_MIN_UV,					\
	.uV_step = S2MPB02_BUCK_STEP_UV,				\
	.vsel_reg = S2MPB02_REG_B##_n##CTRL2,				\
	.vsel_mask = S2MPB02_BUCK_VSEL_MASK,				\
	.enable_reg = S2MPB02_REG_B##_n##CTRL1,				\
	.enable_mask = S2MPB02_BUCK_ENABLE_MASK,			\
	.enable_time = S2MPB02_BUCK_ENABLE_TIME_US,			\
}

static const struct regulator_desc s2mpb02_regulators[] = {
	S2MPB02_LDO(1, S2MPB02_LDO_STEP1_UV),
	S2MPB02_LDO(2, S2MPB02_LDO_STEP1_UV),
	S2MPB02_LDO(3, S2MPB02_LDO_STEP1_UV),
	S2MPB02_LDO(4, S2MPB02_LDO_STEP1_UV),
	S2MPB02_LDO(5, S2MPB02_LDO_STEP1_UV),
	S2MPB02_LDO(6, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(7, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(8, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(9, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(10, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(11, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(12, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(13, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(14, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(15, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(16, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(17, S2MPB02_LDO_STEP2_UV),
	S2MPB02_LDO(18, S2MPB02_LDO_STEP2_UV),
	S2MPB02_BUCK(1, 19),
	S2MPB02_BUCK(2, 20),
};

static int s2mpb02_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regmap *regmap;
	unsigned int id;
	int i, ret;

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

	for (i = 0; i < ARRAY_SIZE(s2mpb02_regulators); i++) {
		rdev = devm_regulator_register(dev, &s2mpb02_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     s2mpb02_regulators[i].name);
	}

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
