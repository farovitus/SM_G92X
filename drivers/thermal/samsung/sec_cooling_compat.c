// SPDX-License-Identifier: GPL-2.0-only
/*
 * Placeholder cooling devices for Samsung's Android thermal HAL.
 *
 * The ROM's thermal HAL lists the 3.10 cooling devices "thermal-cpufreq-0",
 * "thermal-cpufreq-1" and "battery", and aborts when one is missing.
 * system_server then blocks waiting for the HAL. This kernel has no
 * cpufreq driver for the Exynos 7420 and no battery charge limiting, so
 * there is nothing to throttle: each device has one state, 0, and
 * ignores writes. Replace them with real cooling devices once those
 * drivers exist.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

static const char * const sec_cooling_names[] = {
	"thermal-cpufreq-0",
	"thermal-cpufreq-1",
	"battery",
};

static int sec_cooling_get_max_state(struct thermal_cooling_device *cdev,
				     unsigned long *state)
{
	*state = 0;
	return 0;
}

static int sec_cooling_get_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long *state)
{
	*state = 0;
	return 0;
}

static int sec_cooling_set_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long state)
{
	return state ? -EINVAL : 0;
}

static const struct thermal_cooling_device_ops sec_cooling_ops = {
	.get_max_state = sec_cooling_get_max_state,
	.get_cur_state = sec_cooling_get_cur_state,
	.set_cur_state = sec_cooling_set_cur_state,
};

static void sec_cooling_unregister(void *cdev)
{
	thermal_cooling_device_unregister(cdev);
}

static int sec_cooling_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(sec_cooling_names); i++) {
		cdev = thermal_cooling_device_register(sec_cooling_names[i],
						       NULL, &sec_cooling_ops);
		if (IS_ERR(cdev))
			return dev_err_probe(&pdev->dev, PTR_ERR(cdev), "%s\n",
					     sec_cooling_names[i]);
		ret = devm_add_action_or_reset(&pdev->dev,
					       sec_cooling_unregister, cdev);
		if (ret)
			return ret;
	}
	return 0;
}

static const struct of_device_id sec_cooling_of_match[] = {
	{ .compatible = "samsung,sec-cooling-compat" },
	{ }
};
MODULE_DEVICE_TABLE(of, sec_cooling_of_match);

static struct platform_driver sec_cooling_driver = {
	.probe = sec_cooling_probe,
	.driver = {
		.name = "sec-cooling-compat",
		.of_match_table = sec_cooling_of_match,
	},
};
module_platform_driver(sec_cooling_driver);

MODULE_DESCRIPTION("Placeholder cooling devices for Samsung's thermal HAL");
MODULE_LICENSE("GPL");
