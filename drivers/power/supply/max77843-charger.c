// SPDX-License-Identifier: GPL-2.0+
/*
 * Charger status driver for the Maxim MAX77843
 *
 * Reports input presence, charge state and the programmed current
 * limits from the CHG block (I2C 0x69) of the MAX77843 PMIC. The
 * charging configuration left by the bootloader is not changed.
 */

#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77843-private.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/* CHG_DTLS_00: CHGIN input state, 3 = valid input */
#define MAX77843_CHGIN_DTLS_MASK	GENMASK(6, 5)
#define MAX77843_CHGIN_DTLS_VALID	3

/* CHG_DTLS_01: charger state */
#define MAX77843_CHG_DTLS_FIELD		GENMASK(3, 0)

/* CHG_CNFG_09: input current limit */
#define MAX77843_CHG_CHGIN_ILIM_MASK	GENMASK(6, 0)

struct max77843_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct regmap_irq_chip_data *irq_data;
};

static int max77843_charger_online(struct max77843_charger *chg, int *val)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(chg->regmap, MAX77843_CHG_REG_CHG_DTLS_00, &reg);
	if (ret)
		return ret;

	*val = FIELD_GET(MAX77843_CHGIN_DTLS_MASK, reg) ==
	       MAX77843_CHGIN_DTLS_VALID;
	return 0;
}

static int max77843_charger_status(struct max77843_charger *chg, int *val)
{
	unsigned int reg;
	int online, ret;

	ret = max77843_charger_online(chg, &online);
	if (ret)
		return ret;

	if (!online) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = regmap_read(chg->regmap, MAX77843_CHG_REG_CHG_DTLS_01, &reg);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77843_CHG_DTLS_FIELD, reg)) {
	case MAX77843_CHG_PQ_MODE:
	case MAX77843_CHG_CC_MODE:
	case MAX77843_CHG_CV_MODE:
		*val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case MAX77843_CHG_TO_MODE:
	case MAX77843_CHG_DO_MODE:
		*val = POWER_SUPPLY_STATUS_FULL;
		break;
	case MAX77843_CHG_HT_MODE:
	case MAX77843_CHG_TF_MODE:
	case MAX77843_CHG_TS_MODE:
	case MAX77843_CHG_OFF_MODE:
	case 0x0a:
	case 0x0b:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		*val = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

/* CHG_CNFG_02[5:0]: 50 mA per step, codes 0-2 give the 100 mA minimum */
static int max77843_charger_fast_current(struct max77843_charger *chg,
					 int *val)
{
	unsigned int reg, code;
	int ret;

	ret = regmap_read(chg->regmap, MAX77843_CHG_REG_CHG_CNFG_02, &reg);
	if (ret)
		return ret;

	code = reg & MAX77843_CHG_FAST_CHG_CURRENT_MASK;
	*val = code <= 2 ? MAX77843_CHG_FAST_CHG_CURRENT_MIN :
	       code * MAX77843_CHG_FAST_CHG_CURRENT_STEP;
	return 0;
}

/*
 * CHG_CNFG_09[6:0]: 100/3 mA per step. Codes 0-3 give 100 mA, codes
 * 0x78 and above give 4000 mA.
 */
static int max77843_charger_input_limit(struct max77843_charger *chg,
					int *val)
{
	unsigned int reg, code;
	int ret;

	ret = regmap_read(chg->regmap, MAX77843_CHG_REG_CHG_CNFG_09, &reg);
	if (ret)
		return ret;

	code = FIELD_GET(MAX77843_CHG_CHGIN_ILIM_MASK, reg);
	if (code <= 3)
		*val = MAX77843_CHG_INPUT_CURRENT_LIMIT_MIN;
	else if (code >= 0x78)
		*val = MAX77843_CHG_INPUT_CURRENT_LIMIT_MAX;
	else
		*val = DIV_ROUND_CLOSEST(code * 100000, 3);

	return 0;
}

static enum power_supply_property max77843_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int max77843_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max77843_charger *chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return max77843_charger_online(chg, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return max77843_charger_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return max77843_charger_fast_current(chg, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return max77843_charger_input_limit(chg, &val->intval);
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc max77843_charger_desc = {
	.name		= "max77843-charger",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= max77843_charger_props,
	.num_properties	= ARRAY_SIZE(max77843_charger_props),
	.get_property	= max77843_charger_get_property,
};

static const struct regmap_irq max77843_chg_irqs[] = {
	REGMAP_IRQ_REG(0, 0, MAX77843_CHG_BYP_I),
	REGMAP_IRQ_REG(1, 0, MAX77843_CHG_BATP_I),
	REGMAP_IRQ_REG(2, 0, MAX77843_CHG_BAT_I),
	REGMAP_IRQ_REG(3, 0, MAX77843_CHG_CHG_I),
	REGMAP_IRQ_REG(4, 0, MAX77843_CHG_WCIN_I),
	REGMAP_IRQ_REG(5, 0, MAX77843_CHG_CHGIN_I),
	REGMAP_IRQ_REG(6, 0, MAX77843_CHG_AICL_I),
};

#define MAX77843_CHG_IRQ_CHG	3
#define MAX77843_CHG_IRQ_CHGIN	5

static const struct regmap_irq_chip max77843_chg_irq_chip = {
	.name		= "max77843-chg",
	.status_base	= MAX77843_CHG_REG_CHG_INT,
	.mask_base	= MAX77843_CHG_REG_CHG_INT_MASK,
	.num_regs	= 1,
	.irqs		= max77843_chg_irqs,
	.num_irqs	= ARRAY_SIZE(max77843_chg_irqs),
};

static irqreturn_t max77843_charger_irq(int irq, void *data)
{
	struct max77843_charger *chg = data;

	pm_wakeup_event(chg->dev, 2000);
	power_supply_changed(chg->psy);

	return IRQ_HANDLED;
}

static int max77843_chg_request_irq(struct max77843_charger *chg, int hwirq,
				    const char *name)
{
	int virq = regmap_irq_get_virq(chg->irq_data, hwirq);

	if (virq < 0)
		return virq;

	return devm_request_threaded_irq(chg->dev, virq, NULL,
					 max77843_charger_irq, IRQF_ONESHOT,
					 name, chg);
}

static void max77843_chg_mask_intsrc(void *data)
{
	struct regmap *topsys = data;

	regmap_update_bits(topsys, MAX77843_SYS_REG_INTSRCMASK,
			   MAX77843_INTSRCMASK_CHGR_MASK,
			   MAX77843_INTSRCMASK_CHGR_MASK);
}

static int max77843_chg_init_irq(struct max77843_charger *chg, int irq,
				 struct regmap *topsys)
{
	struct device *dev = chg->dev;
	int ret;

	if (irq <= 0)
		return 0;

	ret = devm_regmap_add_irq_chip(dev, chg->regmap, irq,
				       IRQF_TRIGGER_LOW | IRQF_ONESHOT |
				       IRQF_SHARED, 0, &max77843_chg_irq_chip,
				       &chg->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add CHG irq chip\n");

	ret = max77843_chg_request_irq(chg, MAX77843_CHG_IRQ_CHGIN,
				       "max77843-chgin");
	if (ret)
		return dev_err_probe(dev, ret, "failed to request CHGIN irq\n");

	ret = max77843_chg_request_irq(chg, MAX77843_CHG_IRQ_CHG,
				       "max77843-chg");
	if (ret)
		return dev_err_probe(dev, ret, "failed to request CHG irq\n");

	device_init_wakeup(dev, true);

	ret = regmap_update_bits(topsys, MAX77843_SYS_REG_INTSRCMASK,
				 MAX77843_INTSRCMASK_CHGR_MASK, 0);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, max77843_chg_mask_intsrc, topsys);
}

static int max77843_charger_probe(struct platform_device *pdev)
{
	struct max77693_dev *max77843 = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct max77843_charger *chg;

	if (!max77843 || !max77843->regmap_chg)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "missing MAX77843 charger regmap\n");

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->regmap = max77843->regmap_chg;
	platform_set_drvdata(pdev, chg);

	psy_cfg.drv_data = chg;
	psy_cfg.of_node = pdev->dev.of_node;

	chg->psy = devm_power_supply_register(&pdev->dev,
					      &max77843_charger_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->psy),
				     "failed to register power supply\n");

	return max77843_chg_init_irq(chg, max77843->irq, max77843->regmap);
}

static const struct of_device_id max77843_charger_of_match[] = {
	{ .compatible = "maxim,max77843-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77843_charger_of_match);

static const struct platform_device_id max77843_charger_id[] = {
	{ "max77843-charger", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77843_charger_id);

static struct platform_driver max77843_charger_driver = {
	.driver = {
		.name = "max77843-charger",
		.of_match_table = max77843_charger_of_match,
	},
	.probe = max77843_charger_probe,
	.id_table = max77843_charger_id,
};
module_platform_driver(max77843_charger_driver);

MODULE_DESCRIPTION("Maxim MAX77843 charger driver");
MODULE_LICENSE("GPL");
