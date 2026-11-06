// SPDX-License-Identifier: GPL-2.0+
/*
 * Fuel gauge driver for the Maxim MAX77843
 *
 * The MAX77843 fuel gauge is a ModelGauge m3 block at its own I2C
 * address (0x36) on the PMIC bus. The MFD core does not create a client
 * for it, so this cell driver adds one on the parent adapter. The gauge
 * is only read; its configuration and model are left as the bootloader
 * programmed them.
 */

#include <linux/i2c.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77843-private.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/*
 * Units with the 0.01 ohm sense resistor:
 *   current:  1.5625 uV / Rsense = 156.25 uA per LSB
 *   capacity: 5 uVh / Rsense     = 500 uAh per LSB
 *   voltage:  78.125 uV per LSB
 */
#define MAX77843_FG_CURRENT_LSB_NUM	15625	/* uA x 100 */
#define MAX77843_FG_CAPACITY_LSB	500	/* uAh */
#define MAX77843_FG_VCELL_LSB_NUM	625	/* uV x 8 */

/* Charger state is decided from the current when no charger supply */
#define MAX77843_FG_CHG_THRESHOLD_UA	100000

#define MAX77843_FG_CHARGER_NAME	"max77843-charger"

struct max77843_fg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
};

static const struct regmap_config max77843_fg_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.val_format_endian	= REGMAP_ENDIAN_LITTLE,
	.max_register		= MAX77843_FG_SOCVF,
	.cache_type		= REGCACHE_NONE,
};

static int max77843_fg_read_s16(struct max77843_fg *fg, unsigned int reg,
				int *val)
{
	unsigned int raw;
	int ret;

	ret = regmap_read(fg->regmap, reg, &raw);
	if (ret)
		return ret;

	*val = (s16)raw;
	return 0;
}

static int max77843_fg_current(struct max77843_fg *fg, unsigned int reg,
			       int *val)
{
	int raw, ret;

	ret = max77843_fg_read_s16(fg, reg, &raw);
	if (ret)
		return ret;

	*val = raw * MAX77843_FG_CURRENT_LSB_NUM / 100;
	return 0;
}

static int max77843_fg_capacity(struct max77843_fg *fg, int *val)
{
	unsigned int raw;
	int ret;

	ret = regmap_read(fg->regmap, MAX77843_FG_REG_SOCREP, &raw);
	if (ret)
		return ret;

	*val = clamp_t(int, raw >> 8, 0, 100);
	return 0;
}

static int max77843_fg_status(struct max77843_fg *fg, int *val)
{
	union power_supply_propval prop;
	struct power_supply *charger;
	int ret, ua, cap;

	charger = power_supply_get_by_name(MAX77843_FG_CHARGER_NAME);
	if (charger) {
		ret = power_supply_get_property(charger,
						POWER_SUPPLY_PROP_STATUS,
						&prop);
		power_supply_put(charger);
		if (!ret) {
			*val = prop.intval;
			return 0;
		}
	}

	ret = max77843_fg_current(fg, MAX77843_FG_REG_CURRENT, &ua);
	if (ret)
		return ret;

	if (ua > MAX77843_FG_CHG_THRESHOLD_UA) {
		*val = POWER_SUPPLY_STATUS_CHARGING;
	} else if (ua < -MAX77843_FG_CHG_THRESHOLD_UA) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		ret = max77843_fg_capacity(fg, &cap);
		if (ret)
			return ret;
		*val = cap >= 100 ? POWER_SUPPLY_STATUS_FULL :
				    POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return 0;
}

static enum power_supply_property max77843_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int max77843_fg_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct max77843_fg *fg = power_supply_get_drvdata(psy);
	unsigned int raw;
	int ret, sval;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return max77843_fg_status(fg, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		return max77843_fg_capacity(fg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(fg->regmap, MAX77843_FG_REG_VCELL, &raw);
		if (ret)
			return ret;
		val->intval = raw * MAX77843_FG_VCELL_LSB_NUM / 8;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return max77843_fg_current(fg, MAX77843_FG_REG_CURRENT,
					   &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		return max77843_fg_current(fg, MAX77843_FG_REG_AVG_CURRENT,
					   &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		/* 1/256 degC per LSB, reported in 0.1 degC */
		ret = max77843_fg_read_s16(fg, MAX77843_FG_REG_TEMP, &sval);
		if (ret)
			return ret;
		val->intval = sval * 10 / 256;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(fg->regmap, MAX77843_FG_REG_FULLCAP, &raw);
		if (ret)
			return ret;
		val->intval = raw * MAX77843_FG_CAPACITY_LSB;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = regmap_read(fg->regmap, MAX77843_FG_REG_FULLCAP_NOM,
				  &raw);
		if (ret)
			return ret;
		val->intval = raw * MAX77843_FG_CAPACITY_LSB;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc max77843_fg_desc = {
	.name		= "max77843-fuelgauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= max77843_fg_props,
	.num_properties	= ARRAY_SIZE(max77843_fg_props),
	.get_property	= max77843_fg_get_property,
};

static int max77843_fg_probe(struct platform_device *pdev)
{
	struct max77693_dev *max77843 = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct i2c_client *client;
	struct max77843_fg *fg;
	unsigned int status;
	int ret;

	if (!max77843 || !max77843->i2c)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "missing MAX77843 parent\n");

	fg = devm_kzalloc(&pdev->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = &pdev->dev;
	platform_set_drvdata(pdev, fg);

	client = devm_i2c_new_dummy_device(&pdev->dev,
					   max77843->i2c->adapter,
					   I2C_ADDR_FG);
	if (IS_ERR(client))
		return dev_err_probe(&pdev->dev, PTR_ERR(client),
				     "failed to add fuel gauge I2C client\n");

	fg->regmap = devm_regmap_init_i2c(client, &max77843_fg_regmap_config);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(fg->regmap),
				     "failed to init fuel gauge regmap\n");

	ret = regmap_read(fg->regmap, MAX77843_FG_REG_STATUS, &status);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to read fuel gauge status\n");

	psy_cfg.drv_data = fg;
	psy_cfg.of_node = pdev->dev.of_node;

	fg->psy = devm_power_supply_register(&pdev->dev, &max77843_fg_desc,
					     &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	return 0;
}

static const struct of_device_id max77843_fg_of_match[] = {
	{ .compatible = "maxim,max77843-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77843_fg_of_match);

static const struct platform_device_id max77843_fg_id[] = {
	{ "max77843-fuelgauge", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77843_fg_id);

static struct platform_driver max77843_fg_driver = {
	.driver = {
		.name = "max77843-fuelgauge",
		.of_match_table = max77843_fg_of_match,
	},
	.probe = max77843_fg_probe,
	.id_table = max77843_fg_id,
};
module_platform_driver(max77843_fg_driver);

MODULE_DESCRIPTION("Maxim MAX77843 fuel gauge driver");
MODULE_LICENSE("GPL");
