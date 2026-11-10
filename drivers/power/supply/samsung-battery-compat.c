// SPDX-License-Identifier: GPL-2.0-only
/*
 * "battery" and "ac" power supplies for Samsung's Android userspace.
 *
 * On the 3.10 kernel, Samsung's sec-battery driver registered the power
 * supplies "battery" and "ac", and the ROM's thermal and health HALs look
 * for those names. The thermal HAL aborts when it cannot find their thermal
 * zones, and system_server then blocks waiting for it. This driver
 * registers both names and forwards them to the MAX77843 fuel gauge and
 * charger. Each has a temperature, so the power-supply core also registers
 * the thermal zones the HAL needs.
 *
 * The "ac" temperature is the fuel gauge's battery temperature: this
 * kernel has no driver for the charger thermistor that 3.10 reported.
 * The battery health is the charger's, which stops or limits charging by
 * the battery temperature.
 *
 * Lineage's health HAL only starts when battery/charging_enabled exists
 * (added to the 3.10 sec_battery driver for Lineage), and system_server
 * waits for that HAL. Writing 0 inhibits charging and 1 resumes it,
 * through the charger's charge_behaviour.
 *
 * batt_slate_mode is stock's switch for running the system from the
 * battery with the input attached, used to measure the load over USB.
 * Writing 1 sets the charger's charge_behaviour to force-discharge; 0
 * returns to what charging_enabled asks for.
 */

#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

struct sec_compat {
	struct power_supply *fg;
	struct power_supply *chg;
	struct mutex lock;	/* serialises the two switches below */
	bool inhibit;		/* charging_enabled 0 */
	bool slate;		/* batt_slate_mode 1 */
};

static void sec_compat_put(void *data)
{
	power_supply_put(data);
}

static int sec_compat_battery_get(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sec_compat *sc = power_supply_get_drvdata(psy);

	/* The charger judges the battery temperature */
	if (psp == POWER_SUPPLY_PROP_HEALTH)
		return power_supply_get_property(sc->chg, psp, val);
	return power_supply_get_property(sc->fg, psp, val);
}

static int sec_compat_ac_get(struct power_supply *psy,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct sec_compat *sc = power_supply_get_drvdata(psy);

	if (psp == POWER_SUPPLY_PROP_TEMP)
		return power_supply_get_property(sc->fg, psp, val);
	return power_supply_get_property(sc->chg, psp, val);
}

static struct sec_compat *sec_compat_of(struct device *dev)
{
	return power_supply_get_drvdata(dev_get_drvdata(dev));
}

/*
 * Slate mode wins over charging_enabled: the health HAL rewrites
 * charging_enabled on battery updates, and that must not end slate mode.
 */
static ssize_t sec_compat_set(struct device *dev, const char *buf,
			      size_t count, bool *flag, bool invert)
{
	struct sec_compat *sc = sec_compat_of(dev);
	union power_supply_propval val;
	bool on, old;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	on ^= invert;
	mutex_lock(&sc->lock);
	old = *flag;
	*flag = on;
	val.intval = sc->slate ? POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE :
		     sc->inhibit ? POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
		     POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
	ret = power_supply_set_property(sc->chg,
					POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, &val);
	if (ret)
		*flag = old;
	mutex_unlock(&sc->lock);
	return ret ? ret : count;
}

static ssize_t charging_enabled_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", !READ_ONCE(sec_compat_of(dev)->inhibit));
}

static ssize_t charging_enabled_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	return sec_compat_set(dev, buf, count, &sec_compat_of(dev)->inhibit,
			      true);
}
static DEVICE_ATTR_RW(charging_enabled);

static ssize_t batt_slate_mode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", READ_ONCE(sec_compat_of(dev)->slate));
}

static ssize_t batt_slate_mode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	return sec_compat_set(dev, buf, count, &sec_compat_of(dev)->slate,
			      false);
}
static DEVICE_ATTR_RW(batt_slate_mode);

static struct attribute *sec_compat_battery_attrs[] = {
	&dev_attr_charging_enabled.attr,
	&dev_attr_batt_slate_mode.attr,
	NULL
};
ATTRIBUTE_GROUPS(sec_compat_battery);

static enum power_supply_property sec_compat_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static enum power_supply_property sec_compat_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc sec_compat_battery_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sec_compat_battery_props,
	.num_properties = ARRAY_SIZE(sec_compat_battery_props),
	.get_property = sec_compat_battery_get,
};

static const struct power_supply_desc sec_compat_ac_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = sec_compat_ac_props,
	.num_properties = ARRAY_SIZE(sec_compat_ac_props),
	.get_property = sec_compat_ac_get,
};

static struct power_supply *sec_compat_get(struct device *dev,
					   const char *name)
{
	struct power_supply *psy = power_supply_get_by_name(name);
	int ret;

	if (!psy)
		return ERR_PTR(dev_err_probe(dev, -EPROBE_DEFER,
					     "waiting for %s\n", name));
	ret = devm_add_action_or_reset(dev, sec_compat_put, psy);
	return ret ? ERR_PTR(ret) : psy;
}

static int sec_compat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	struct power_supply *psy;
	struct sec_compat *sc;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;
	mutex_init(&sc->lock);

	sc->fg = sec_compat_get(dev, "max77843-fuelgauge");
	if (IS_ERR(sc->fg))
		return PTR_ERR(sc->fg);
	sc->chg = sec_compat_get(dev, "max77843-charger");
	if (IS_ERR(sc->chg))
		return PTR_ERR(sc->chg);

	cfg.drv_data = sc;
	cfg.attr_grp = sec_compat_battery_groups;
	psy = devm_power_supply_register(dev, &sec_compat_battery_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "battery\n");
	cfg.attr_grp = NULL;
	psy = devm_power_supply_register(dev, &sec_compat_ac_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy), "ac\n");

	dev_info(dev, "registered battery and ac\n");
	return 0;
}

static const struct of_device_id sec_compat_of_match[] = {
	{ .compatible = "samsung,sec-battery-compat" },
	{ }
};
MODULE_DEVICE_TABLE(of, sec_compat_of_match);

static struct platform_driver sec_compat_driver = {
	.probe = sec_compat_probe,
	.driver = {
		.name = "sec-battery-compat",
		.of_match_table = sec_compat_of_match,
	},
};
module_platform_driver(sec_compat_driver);

MODULE_DESCRIPTION("Samsung battery/ac power-supply names for Android");
MODULE_LICENSE("GPL");
