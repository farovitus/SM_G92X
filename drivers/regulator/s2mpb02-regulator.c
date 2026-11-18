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
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <soc/samsung/camera_class.h>

#define S2MPB02_REG_ID			0x00
#define S2MPB02_REG_B1CTRL1		0x0b
#define S2MPB02_REG_B1CTRL2		0x0c
#define S2MPB02_REG_B2CTRL1		0x0d
#define S2MPB02_REG_B2CTRL2		0x0e
#define S2MPB02_REG_FLED_CTRL1		0x12
#define S2MPB02_REG_FLED_CTRL2		0x13
#define S2MPB02_REG_FLED_CUR1		0x14
#define S2MPB02_REG_FLED_TIME1		0x15
#define S2MPB02_REG_FLED_IRD2		0x1b
#define S2MPB02_REG_L1CTRL		0x1e
#define S2MPB02_REG_L18CTRL		0x2f

#define S2MPB02_FLED_EN_MASK		GENMASK(7, 6)
#define S2MPB02_FLED_OFF		0x00
#define S2MPB02_FLED_FLASH_ON		0x80
#define S2MPB02_FLED_TORCH_ON		0xc0
#define S2MPB02_FLED_LV_EN_MASK		BIT(3)
#define S2MPB02_FLED_FLASH_CUR_MASK	GENMASK(7, 4)
#define S2MPB02_FLED_TORCH_CUR_MASK	GENMASK(3, 0)
#define S2MPB02_FLED_FLASH_CUR_SHIFT	4
#define S2MPB02_FLED_TORCH_CUR_SHIFT	0
#define S2MPB02_FLED_FLASH_TIME_MASK	GENMASK(7, 4)
#define S2MPB02_FLED_TORCH_TIME_MASK	GENMASK(3, 0)
#define S2MPB02_FLED_FLASH_TIME_250MS	0x30
#define S2MPB02_FLED_TORCH_TIME_MAX	0x0f
#define S2MPB02_FLED_IRD2_ACT_DISCH	BIT(1)
#define S2MPB02_FLED_MAX_STEP		15
#define S2MPB02_FLED_DEF_TORCH		6	/* 120 mA */
#define S2MPB02_FLED_DEF_FLASH		8	/* 800 mA */

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
	       (reg >= S2MPB02_REG_FLED_CTRL1 && reg <= S2MPB02_REG_FLED_IRD2) ||
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

struct s2mpb02_fled {
	struct regmap *regmap;
	struct mutex lock;
	struct led_classdev torch_cdev;
	struct led_classdev flash_cdev;
	struct device *cam_dev;
	u8 torch_level;
	u8 flash_level;
};

static struct class *s2mpb02_cam_class;
static DEFINE_MUTEX(s2mpb02_cam_class_lock);

struct class *samsung_camera_class(void)
{
	struct class *cls;

	mutex_lock(&s2mpb02_cam_class_lock);
	if (!s2mpb02_cam_class) {
		cls = class_create("camera");
		if (!IS_ERR(cls))
			s2mpb02_cam_class = cls;
	}
	cls = s2mpb02_cam_class;
	mutex_unlock(&s2mpb02_cam_class_lock);

	return cls;
}
EXPORT_SYMBOL_GPL(samsung_camera_class);

static int s2mpb02_fled_apply(struct s2mpb02_fled *fled, u8 flash_lvl,
			      u8 torch_lvl)
{
	int ret;

	mutex_lock(&fled->lock);

	flash_lvl = min_t(u8, flash_lvl, S2MPB02_FLED_MAX_STEP);
	torch_lvl = min_t(u8, torch_lvl, S2MPB02_FLED_MAX_STEP);

	/* Transition through OFF before engaging flash to reset one-shot timer. */
	ret = regmap_update_bits(fled->regmap, S2MPB02_REG_FLED_CTRL1,
				 S2MPB02_FLED_EN_MASK | S2MPB02_FLED_LV_EN_MASK,
				 S2MPB02_FLED_OFF);
	if (ret)
		goto out;

	if (flash_lvl) {
		ret = regmap_update_bits(fled->regmap, S2MPB02_REG_FLED_CUR1,
					 S2MPB02_FLED_FLASH_CUR_MASK,
					 flash_lvl << S2MPB02_FLED_FLASH_CUR_SHIFT);
		ret = ret ?: regmap_update_bits(fled->regmap,
						S2MPB02_REG_FLED_TIME1,
						S2MPB02_FLED_FLASH_TIME_MASK,
						S2MPB02_FLED_FLASH_TIME_250MS);
		ret = ret ?: regmap_update_bits(fled->regmap,
						S2MPB02_REG_FLED_CTRL1,
						S2MPB02_FLED_EN_MASK |
						S2MPB02_FLED_LV_EN_MASK,
						S2MPB02_FLED_FLASH_ON);
	} else if (torch_lvl) {
		ret = regmap_update_bits(fled->regmap, S2MPB02_REG_FLED_CUR1,
					 S2MPB02_FLED_TORCH_CUR_MASK,
					 torch_lvl << S2MPB02_FLED_TORCH_CUR_SHIFT);
		ret = ret ?: regmap_update_bits(fled->regmap,
						S2MPB02_REG_FLED_TIME1,
						S2MPB02_FLED_TORCH_TIME_MASK,
						S2MPB02_FLED_TORCH_TIME_MAX);
		ret = ret ?: regmap_update_bits(fled->regmap,
						S2MPB02_REG_FLED_CTRL1,
						S2MPB02_FLED_EN_MASK |
						S2MPB02_FLED_LV_EN_MASK,
						S2MPB02_FLED_TORCH_ON);
	} else {
		ret = regmap_update_bits(fled->regmap, S2MPB02_REG_FLED_IRD2,
					 S2MPB02_FLED_IRD2_ACT_DISCH,
					 S2MPB02_FLED_IRD2_ACT_DISCH);
	}

	if (!ret) {
		fled->flash_level = flash_lvl;
		fled->torch_level = flash_lvl ? 0 : torch_lvl;
	}

out:
	mutex_unlock(&fled->lock);
	return ret;
}

static int s2mpb02_torch_cdev_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct s2mpb02_fled *fled =
		container_of(cdev, struct s2mpb02_fled, torch_cdev);

	return s2mpb02_fled_apply(fled, 0, min_t(u8, brightness, S2MPB02_FLED_MAX_STEP));
}

static int s2mpb02_flash_cdev_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct s2mpb02_fled *fled =
		container_of(cdev, struct s2mpb02_fled, flash_cdev);

	return s2mpb02_fled_apply(fled, min_t(u8, brightness, S2MPB02_FLED_MAX_STEP), 0);
}

static ssize_t rear_torch_flash_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct s2mpb02_fled *fled = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", fled->torch_level);
}

static ssize_t rear_torch_flash_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct s2mpb02_fled *fled = dev_get_drvdata(dev);
	unsigned int val;
	u8 lvl;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val == 0)
		lvl = 0;
	else if (val == 1)
		lvl = S2MPB02_FLED_DEF_TORCH;
	else if (val == 100)
		lvl = 12; /* 240 mA factory bright torch */
	else if (val >= 1001 && val <= 1010)
		lvl = val - 1000;
	else
		lvl = min_t(u8, val, S2MPB02_FLED_MAX_STEP);

	ret = s2mpb02_fled_apply(fled, 0, lvl);
	return ret ? ret : count;
}

static ssize_t rear_flash_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct s2mpb02_fled *fled = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", fled->flash_level);
}

static ssize_t rear_flash_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct s2mpb02_fled *fled = dev_get_drvdata(dev);
	unsigned int val;
	u8 lvl;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val == 0)
		lvl = 0;
	else if (val == 1)
		lvl = S2MPB02_FLED_DEF_FLASH;
	else if (val == 100)
		lvl = S2MPB02_FLED_MAX_STEP;
	else
		lvl = min_t(u8, val, S2MPB02_FLED_MAX_STEP);

	ret = s2mpb02_fled_apply(fled, lvl, 0);
	return ret ? ret : count;
}

static DEVICE_ATTR(rear_torch_flash, 0664, rear_torch_flash_show,
		   rear_torch_flash_store);
static DEVICE_ATTR(rear_flash, 0664, rear_flash_show, rear_flash_store);

static struct attribute *s2mpb02_flash_attrs[] = {
	&dev_attr_rear_torch_flash.attr,
	&dev_attr_rear_flash.attr,
	NULL,
};
ATTRIBUTE_GROUPS(s2mpb02_flash);

static void s2mpb02_fled_cleanup(void *data)
{
	struct s2mpb02_fled *fled = data;

	s2mpb02_fled_apply(fled, 0, 0);
	if (!IS_ERR_OR_NULL(fled->cam_dev))
		device_unregister(fled->cam_dev);
}

static int s2mpb02_fled_probe(struct device *dev, struct regmap *regmap)
{
	struct s2mpb02_fled *fled;
	struct class *cam_cls;
	int ret;

	fled = devm_kzalloc(dev, sizeof(*fled), GFP_KERNEL);
	if (!fled)
		return -ENOMEM;

	fled->regmap = regmap;
	mutex_init(&fled->lock);

	ret = regmap_update_bits(regmap, S2MPB02_REG_FLED_IRD2,
				 S2MPB02_FLED_IRD2_ACT_DISCH,
				 S2MPB02_FLED_IRD2_ACT_DISCH);
	if (ret)
		return ret;

	ret = s2mpb02_fled_apply(fled, 0, 0);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_LEDS_CLASS)) {
		fled->torch_cdev.name = "torch-sec1";
		fled->torch_cdev.max_brightness = S2MPB02_FLED_MAX_STEP;
		fled->torch_cdev.brightness_set_blocking = s2mpb02_torch_cdev_set;
		ret = devm_led_classdev_register(dev, &fled->torch_cdev);
		if (ret)
			dev_warn(dev, "failed to register torch-sec1: %d\n", ret);

		fled->flash_cdev.name = "flash-sec1";
		fled->flash_cdev.max_brightness = S2MPB02_FLED_MAX_STEP;
		fled->flash_cdev.brightness_set_blocking = s2mpb02_flash_cdev_set;
		ret = devm_led_classdev_register(dev, &fled->flash_cdev);
		if (ret)
			dev_warn(dev, "failed to register flash-sec1: %d\n", ret);
	}

	cam_cls = samsung_camera_class();
	if (cam_cls) {
		fled->cam_dev = device_create_with_groups(cam_cls, NULL, 0,
							  fled,
							  s2mpb02_flash_groups,
							  "flash");
		if (IS_ERR(fled->cam_dev)) {
			dev_warn(dev, "failed to create camera/flash sysfs: %ld\n",
				 PTR_ERR(fled->cam_dev));
			fled->cam_dev = NULL;
		}
	}

	return devm_add_action_or_reset(dev, s2mpb02_fled_cleanup, fled);
}

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

	ret = s2mpb02_fled_probe(dev, regmap);
	if (ret)
		dev_warn(dev, "FLED init failed: %d\n", ret);

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
