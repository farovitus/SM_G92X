// SPDX-License-Identifier: GPL-2.0-only
/*
 * Placeholder mDNIe and LCD panel nodes for Lineage's LiveDisplay HAL.
 *
 * The universal7420 LiveDisplay HAL registers each of its interfaces only
 * when the matching 3.10 sysfs node exists: /sys/class/mdnie/mdnie/{mode,
 * mode_max,accessibility,lux,sensorRGB} and
 * /sys/class/lcd/panel/adaptive_control. Lineage's LiveDisplayService then
 * looks the interfaces up with HIDL getService(), which on this ROM waits
 * for an interface that never registers, whether or not the manifest
 * declares it. It does that on the display thread when boot completes, so
 * the screen cannot turn back on and the watchdog restarts the framework.
 *
 * This kernel has no mDNIe driver: the nodes keep the values written to
 * them and change nothing on the panel. Replace them once a real mDNIe
 * driver exists.
 */

#include <linux/device.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

/* The 3.10 S6 mDNIe modes: dynamic, standard, natural, movie, auto. */
#define MDNIE_MODE_MAX		5
#define MDNIE_MODE_AUTO		4

struct mdnie_compat {
	struct class *mdnie_class;
	struct class *lcd_class;
	struct device *mdnie;
	struct device *panel;
	int mode;
	int scenario;
	int accessibility;
	int lux;
	int adaptive_control;
	int rgb[3];
};

static int *mdnie_compat_field(struct device *dev, const char *name)
{
	struct mdnie_compat *mc = dev_get_drvdata(dev);

	if (!strcmp(name, "mode"))
		return &mc->mode;
	if (!strcmp(name, "scenario"))
		return &mc->scenario;
	if (!strcmp(name, "accessibility"))
		return &mc->accessibility;
	if (!strcmp(name, "lux"))
		return &mc->lux;
	return &mc->adaptive_control;
}

static ssize_t int_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sysfs_emit(buf, "%d\n", *mdnie_compat_field(dev, attr->attr.name));
}

static ssize_t int_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;
	if (!strcmp(attr->attr.name, "mode") &&
	    (val < 0 || val >= MDNIE_MODE_MAX))
		return -EINVAL;
	*mdnie_compat_field(dev, attr->attr.name) = val;
	return count;
}

static ssize_t mode_max_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", MDNIE_MODE_MAX);
}

static ssize_t sensorRGB_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct mdnie_compat *mc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d %d %d\n", mc->rgb[0], mc->rgb[1], mc->rgb[2]);
}

static ssize_t sensorRGB_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mdnie_compat *mc = dev_get_drvdata(dev);
	int r, g, b;

	if (sscanf(buf, "%d %d %d", &r, &g, &b) != 3)
		return -EINVAL;
	mc->rgb[0] = r;
	mc->rgb[1] = g;
	mc->rgb[2] = b;
	return count;
}

static struct device_attribute dev_attr_mode = __ATTR(mode, 0644, int_show, int_store);
static struct device_attribute dev_attr_scenario = __ATTR(scenario, 0644, int_show, int_store);
static struct device_attribute dev_attr_accessibility = __ATTR(accessibility, 0644, int_show, int_store);
static struct device_attribute dev_attr_lux = __ATTR(lux, 0644, int_show, int_store);
static struct device_attribute dev_attr_adaptive_control = __ATTR(adaptive_control, 0644, int_show, int_store);
static DEVICE_ATTR_RO(mode_max);
static DEVICE_ATTR_RW(sensorRGB);

static struct attribute *mdnie_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_mode_max.attr,
	&dev_attr_scenario.attr,
	&dev_attr_accessibility.attr,
	&dev_attr_lux.attr,
	&dev_attr_sensorRGB.attr,
	NULL
};
ATTRIBUTE_GROUPS(mdnie);

static struct attribute *panel_attrs[] = {
	&dev_attr_adaptive_control.attr,
	NULL
};
ATTRIBUTE_GROUPS(panel);

static void mdnie_compat_remove(struct platform_device *pdev)
{
	struct mdnie_compat *mc = platform_get_drvdata(pdev);

	device_unregister(mc->panel);
	device_unregister(mc->mdnie);
	class_destroy(mc->lcd_class);
	class_destroy(mc->mdnie_class);
}

static int mdnie_compat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mdnie_compat *mc;
	int ret;

	mc = devm_kzalloc(dev, sizeof(*mc), GFP_KERNEL);
	if (!mc)
		return -ENOMEM;
	mc->mode = MDNIE_MODE_AUTO;
	mc->rgb[0] = mc->rgb[1] = mc->rgb[2] = 255;

	mc->mdnie_class = class_create("mdnie");
	if (IS_ERR(mc->mdnie_class))
		return PTR_ERR(mc->mdnie_class);
	mc->lcd_class = class_create("lcd");
	if (IS_ERR(mc->lcd_class)) {
		ret = PTR_ERR(mc->lcd_class);
		goto err_mdnie_class;
	}

	mc->mdnie = device_create_with_groups(mc->mdnie_class, dev, 0, mc,
					      mdnie_groups, "mdnie");
	if (IS_ERR(mc->mdnie)) {
		ret = PTR_ERR(mc->mdnie);
		goto err_lcd_class;
	}
	mc->panel = device_create_with_groups(mc->lcd_class, dev, 0, mc,
					      panel_groups, "panel");
	if (IS_ERR(mc->panel)) {
		ret = PTR_ERR(mc->panel);
		goto err_mdnie;
	}

	platform_set_drvdata(pdev, mc);
	return 0;

err_mdnie:
	device_unregister(mc->mdnie);
err_lcd_class:
	class_destroy(mc->lcd_class);
err_mdnie_class:
	class_destroy(mc->mdnie_class);
	return ret;
}

static const struct of_device_id mdnie_compat_of_match[] = {
	{ .compatible = "samsung,mdnie-compat" },
	{ }
};
MODULE_DEVICE_TABLE(of, mdnie_compat_of_match);

static struct platform_driver mdnie_compat_driver = {
	.probe = mdnie_compat_probe,
	.remove_new = mdnie_compat_remove,
	.driver = {
		.name = "samsung-mdnie-compat",
		.of_match_table = mdnie_compat_of_match,
	},
};
module_platform_driver(mdnie_compat_driver);

MODULE_DESCRIPTION("Placeholder mDNIe nodes for Lineage LiveDisplay");
MODULE_LICENSE("GPL");
