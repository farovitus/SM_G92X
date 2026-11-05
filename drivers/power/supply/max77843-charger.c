// SPDX-License-Identifier: GPL-2.0+
/*
 * Charger status driver for the Maxim MAX77843
 *
 * Reports input presence, charge state and the programmed current
 * limits from the CHG block (I2C 0x69) of the MAX77843 PMIC, and programs
 * the charger from the battery described by the "monitored-battery"
 * phandle (simple-battery binding):
 *
 *   constant-charge-voltage-max-microvolt   float voltage      (CHG_CNFG_04)
 *   constant-charge-current-max-microamp    fast-charge current (CHG_CNFG_02)
 *   charge-term-current-microamp            top-off current    (CHG_CNFG_03)
 *   re-charge-voltage-microvolt             restart threshold  (CHG_CNFG_01)
 *
 * Charging is enabled at probe and again on every input change. Without a
 * monitored battery the driver stays read-only and leaves the boot loader's
 * configuration alone.
 */

#include <linux/bitfield.h>
#include <linux/devm-helpers.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77843-private.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
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

/* CHG_CNFG_00 (0xB7): operating mode and charger watchdog */
#define MAX77843_CNFG00_CHG		BIT(0)
#define MAX77843_CNFG00_OTG		BIT(1)
#define MAX77843_CNFG00_BUCK		BIT(2)
#define MAX77843_CNFG00_BOOST		BIT(3)
#define MAX77843_CNFG00_WDTEN		BIT(4)
/* CHG_CNFG_01 (0xB8): restart threshold below the float voltage */
#define MAX77843_CNFG01_RSTRT_MASK	GENMASK(5, 4)
/* CHG_CNFG_03 (0xBA): top-off current and top-off time */
#define MAX77843_CNFG03_TO_ITH_MASK	GENMASK(2, 0)
#define MAX77843_CNFG03_TO_TIME_MASK	GENMASK(5, 3)
/* CHG_CNFG_04 (0xBB): float voltage, bits [5:0]; MINVSYS above */
#define MAX77843_CHG_CV_MASK		0x3f
/* CHG_CNFG_06 (0xBD): watchdog clear and write protection */
#define MAX77843_CNFG06_WDTCLR_MASK	GENMASK(1, 0)
#define MAX77843_CNFG06_WDTCLR		0x01
#define MAX77843_CNFG06_CHGPROT_MASK	GENMASK(3, 2)

/* TOPSYS PMICREV: the top-off current steps changed with revision 2 */
#define MAX77843_PMICREV_MASK		GENMASK(2, 0)

/* Stock top-off time: 70 minutes in top-off before the charger is done */
#define MAX77843_CHG_TO_TIME_70MIN	7

/* Input limit used until the charger type is known */
#define MAX77843_CHG_DEFAULT_INPUT_UA	500000

struct max77843_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct regmap_irq_chip_data *irq_data;
	struct power_supply_battery_info *bat;
	struct work_struct work;
	struct mutex lock;		/* serialises max77843_chg_apply() */
	unsigned int pmic_rev;
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

/* ---------------------------------------------------------------------- */
/* Charger configuration                                                  */
/* ---------------------------------------------------------------------- */

/*
 * Register codes round towards the safe side: currents and voltages down.
 * Float voltage, from the stock driver: 3.650 V + 25 mV per step up to
 * 0x1b (4.325 V), 0x1c is 4.340 V, and from 0x1d (4.350 V) the 25 mV steps
 * resume one code later.
 */
static unsigned int max77843_chg_cv_code(int uv)
{
	int mv = clamp(uv / 1000, 3650, 4700);

	if (mv < 4340)
		return (mv - 3650) / 25;
	if (mv < 4350)
		return 0x1c;
	return (mv - 3650) / 25 + 1;
}

static unsigned int max77843_chg_cc_code(int ua)
{
	/* Codes 0 to 2 all select 100 mA, then 50 mA per step */
	return clamp(ua / 50000, 2, MAX77843_CHG_FAST_CHG_CURRENT_MASK);
}

static unsigned int max77843_chg_input_code(int ua)
{
	/* 33.3 mA per step, codes 0 to 3 are 100 mA, 0x78 and up 4 A */
	return clamp((int)div_s64((s64)ua * 3, 100000), 3, 0x78);
}

static unsigned int max77843_chg_topoff_code(struct max77843_charger *chg,
					     int ua)
{
	if (chg->pmic_rev >= 2)
		return clamp((ua - 125000) / 75000, 0, 7);
	return clamp((ua - 100000) / 50000, 0, 7);
}

static unsigned int max77843_chg_restart_code(int cv_uv, int restart_uv)
{
	int drop = cv_uv - restart_uv;

	/* 100, 150 or 200 mV below the float voltage, never above the request */
	if (drop < 150000)
		return 0;
	if (drop < 200000)
		return 1;
	return 2;
}

static int max77843_chg_update(struct max77843_charger *chg, u8 reg,
			       unsigned int mask, unsigned int val)
{
	int ret = regmap_update_bits(chg->regmap, reg, mask, val);

	if (ret)
		dev_err(chg->dev, "failed to write register 0x%02x: %d\n",
			reg, ret);
	return ret;
}

/*
 * Unlock the protected configuration registers, clear the charger watchdog
 * and keep it disabled: with the watchdog enabled and not cleared every
 * 80 s the MAX77843 stops charging, and this driver has no periodic host
 * to clear it in suspend.
 */
static int max77843_chg_unlock(struct max77843_charger *chg)
{
	int ret;

	ret = max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_06,
				  MAX77843_CNFG06_CHGPROT_MASK |
				  MAX77843_CNFG06_WDTCLR_MASK,
				  MAX77843_CNFG06_CHGPROT_MASK |
				  MAX77843_CNFG06_WDTCLR);
	if (ret)
		return ret;

	return max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_00,
				   MAX77843_CNFG00_WDTEN, 0);
}

static int max77843_chg_apply(struct max77843_charger *chg)
{
	struct power_supply_battery_info *bat = chg->bat;
	int cv_uv = bat->constant_charge_voltage_max_uv;
	int cc_ua = bat->constant_charge_current_max_ua;
	int input_ua = MAX77843_CHG_DEFAULT_INPUT_UA;
	unsigned int mode;
	int ret;

	mutex_lock(&chg->lock);

	ret = max77843_chg_unlock(chg);
	if (ret)
		goto out;

	/* Input limit first, so the source is never asked for too much */
	ret = max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_09,
				  MAX77843_CHG_CHGIN_ILIM_MASK,
				  max77843_chg_input_code(input_ua));
	if (ret)
		goto out;

	ret = max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_02,
				  MAX77843_CHG_FAST_CHG_CURRENT_MASK,
				  max77843_chg_cc_code(cc_ua));
	if (ret)
		goto out;

	ret = max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_04,
				  MAX77843_CHG_CV_MASK,
				  max77843_chg_cv_code(cv_uv));
	if (ret)
		goto out;

	/* Charger and buck on; OTG and boost off */
	mode = MAX77843_CNFG00_CHG | MAX77843_CNFG00_BUCK;
	ret = max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_00,
				  MAX77843_CNFG00_CHG | MAX77843_CNFG00_OTG |
				  MAX77843_CNFG00_BUCK | MAX77843_CNFG00_BOOST,
				  mode);
	if (ret)
		goto out;

	dev_dbg(chg->dev, "input %d uA, fast charge %d uA, float %d uV\n",
		input_ua, cc_ua, cv_uv);
out:
	mutex_unlock(&chg->lock);
	return ret;
}

/* Settings that do not change with the input or the temperature */
static int max77843_chg_init_hw(struct max77843_charger *chg)
{
	struct power_supply_battery_info *bat = chg->bat;
	int ret;

	ret = max77843_chg_unlock(chg);
	if (ret)
		return ret;

	if (bat->charge_term_current_ua > 0) {
		ret = max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_03,
				MAX77843_CNFG03_TO_ITH_MASK |
				MAX77843_CNFG03_TO_TIME_MASK,
				max77843_chg_topoff_code(chg,
					bat->charge_term_current_ua) |
				FIELD_PREP(MAX77843_CNFG03_TO_TIME_MASK,
					   MAX77843_CHG_TO_TIME_70MIN));
		if (ret)
			return ret;
	}

	if (bat->charge_restart_voltage_uv > 0) {
		ret = max77843_chg_update(chg, MAX77843_CHG_REG_CHG_CNFG_01,
				MAX77843_CNFG01_RSTRT_MASK,
				FIELD_PREP(MAX77843_CNFG01_RSTRT_MASK,
					max77843_chg_restart_code(
					bat->constant_charge_voltage_max_uv,
					bat->charge_restart_voltage_uv)));
		if (ret)
			return ret;
	}

	return max77843_chg_apply(chg);
}

static void max77843_chg_work(struct work_struct *work)
{
	struct max77843_charger *chg = container_of(work,
					struct max77843_charger, work);

	max77843_chg_apply(chg);
	power_supply_changed(chg->psy);
}

static void max77843_chg_get_battery(struct max77843_charger *chg)
{
	struct power_supply_battery_info *bat;
	int ret;

	ret = power_supply_get_battery_info(chg->psy, &bat);
	if (ret) {
		dev_warn(chg->dev,
			 "no monitored-battery (%d), leaving the charger as the boot loader set it\n",
			 ret);
		return;
	}

	if (bat->constant_charge_voltage_max_uv <= 0 ||
	    bat->constant_charge_current_max_ua <= 0) {
		dev_warn(chg->dev,
			 "monitored-battery lacks the constant charge voltage and current, not charging\n");
		return;
	}

	chg->bat = bat;
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
	if (chg->bat)
		schedule_work(&chg->work);
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
	unsigned int reg;
	int ret;

	if (!max77843 || !max77843->regmap_chg)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "missing MAX77843 charger regmap\n");

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->regmap = max77843->regmap_chg;
	chg->pmic_rev = 2;
	mutex_init(&chg->lock);
	platform_set_drvdata(pdev, chg);

	if (!regmap_read(max77843->regmap, MAX77843_SYS_REG_PMICREV, &reg))
		chg->pmic_rev = reg & MAX77843_PMICREV_MASK;

	psy_cfg.drv_data = chg;
	psy_cfg.of_node = pdev->dev.of_node;

	chg->psy = devm_power_supply_register(&pdev->dev,
					      &max77843_charger_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->psy),
				     "failed to register power supply\n");

	/* Cancelled before the power supply goes away */
	ret = devm_work_autocancel(&pdev->dev, &chg->work, max77843_chg_work);
	if (ret)
		return ret;

	max77843_chg_get_battery(chg);
	if (chg->bat) {
		ret = max77843_chg_init_hw(chg);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "cannot configure the charger\n");
	}

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
