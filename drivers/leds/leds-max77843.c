// SPDX-License-Identifier: GPL-2.0
/*
 * RGB indicator LED of the Maxim MAX77843 PMIC.
 *
 * The TOPSYS block (the MFD's main I2C client) has four constant-current
 * LED channels with one shared hardware blink timer. LEDEN holds two bits
 * per channel (off, on or blink), LEDxBRT the channel current. The
 * channels in use form one multicolor LED; DT gives each its register
 * index and the current code it gets at full brightness, which carries the
 * board's colour calibration. Channels not described in DT are kept off.
 *
 * Hardware blink takes the timer trigger's delay_on/delay_off, rounded to
 * the nearest LEDBLNK step, so the chip blinks with the CPU asleep.
 */

#include <linux/bits.h>
#include <linux/led-class-multicolor.h>
#include <linux/mfd/max77693-common.h>
#include <linux/mfd/max77843-private.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MAX77843_LED_CHANNELS		4

#define MAX77843_LEDEN_MASK(ch)		(GENMASK(1, 0) << (2 * (ch)))
#define MAX77843_LEDEN_ON(ch)		(BIT(0) << (2 * (ch)))
#define MAX77843_LEDEN_BLINK(ch)	(BIT(1) << (2 * (ch)))

#define MAX77843_LEDBLNK_ON_SHIFT	4

/* Default full-scale current code (the 3.10 driver's normal current). */
#define MAX77843_LED_DEFAULT_MAX	0x14

/* LEDBLNK on time per code, in ms. */
static const u16 max77843_blink_on_ms[16] = {
	100, 200, 300, 400, 500, 750, 1000, 1250,
	1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250,
};

/* LEDBLNK off time per code, in ms; code 0 never turns the LED off. */
static const u16 max77843_blink_off_ms[16] = {
	0, 500, 1000, 1500, 2000, 2500, 3000, 3500,
	4000, 4500, 5000, 6000, 7000, 8000, 10000, 12000,
};

struct max77843_led {
	struct led_classdev_mc mc;
	struct mc_subled subled[MAX77843_LED_CHANNELS];
	u8 full_scale[MAX77843_LED_CHANNELS];
	struct regmap *map;
	struct mutex lock;	/* serialises register updates */
	bool blinking;
};

static struct max77843_led *to_max77843_led(struct led_classdev *cdev)
{
	return container_of(lcdev_to_mccdev(cdev), struct max77843_led, mc);
}

static unsigned int max77843_nearest(const u16 *table, unsigned long ms)
{
	unsigned int i, best = 0;

	for (i = 1; i < 16; i++)
		if (abs((long)table[i] - (long)ms) <
		    abs((long)table[best] - (long)ms))
			best = i;
	return best;
}

/* Write every channel's current and mode. Called with led->lock held. */
static int max77843_led_apply(struct max77843_led *led,
			      enum led_brightness brightness)
{
	unsigned int i, en = 0, mask = 0;
	int ret;

	led_mc_calc_color_components(&led->mc, brightness);

	for (i = 0; i < led->mc.num_colors; i++) {
		struct mc_subled *s = &led->subled[i];
		unsigned int ch = s->channel;
		unsigned int code;

		code = DIV_ROUND_CLOSEST(s->brightness * led->full_scale[i],
					 led->mc.led_cdev.max_brightness);
		if (s->brightness && !code)
			code = 1;

		ret = regmap_write(led->map, MAX77843_LED_REG_LED0BRT + ch,
				   code);
		if (ret)
			return ret;

		mask |= MAX77843_LEDEN_MASK(ch);
		if (code)
			en |= led->blinking ? MAX77843_LEDEN_BLINK(ch) :
					      MAX77843_LEDEN_ON(ch);
	}

	return regmap_update_bits(led->map, MAX77843_LED_REG_LEDEN, mask, en);
}

static int max77843_led_brightness_set(struct led_classdev *cdev,
				       enum led_brightness brightness)
{
	struct max77843_led *led = to_max77843_led(cdev);
	int ret;

	mutex_lock(&led->lock);
	/* Turning the LED off also ends hardware blinking. */
	if (brightness == LED_OFF)
		led->blinking = false;
	ret = max77843_led_apply(led, brightness);
	mutex_unlock(&led->lock);

	return ret;
}

static int max77843_led_blink_set(struct led_classdev *cdev,
				  unsigned long *delay_on,
				  unsigned long *delay_off)
{
	struct max77843_led *led = to_max77843_led(cdev);
	unsigned int on, off;
	int ret;

	if (!*delay_on && !*delay_off) {
		*delay_on = 500;
		*delay_off = 500;
	}

	on = max77843_nearest(max77843_blink_on_ms, *delay_on);
	off = max77843_nearest(max77843_blink_off_ms, *delay_off);
	*delay_on = max77843_blink_on_ms[on];
	*delay_off = max77843_blink_off_ms[off];

	mutex_lock(&led->lock);
	ret = regmap_write(led->map, MAX77843_LED_REG_LEDBLNK,
			   on << MAX77843_LEDBLNK_ON_SHIFT | off);
	if (ret)
		goto out;

	/* Like software blink, blink at full brightness if the LED is off. */
	if (cdev->brightness == LED_OFF)
		cdev->brightness = cdev->max_brightness;
	led->blinking = off != 0;
	ret = max77843_led_apply(led, cdev->brightness);
out:
	mutex_unlock(&led->lock);
	return ret;
}

static int max77843_led_parse_dt(struct device *dev, struct max77843_led *led,
				 struct fwnode_handle *mc_node)
{
	struct fwnode_handle *child;
	unsigned int n = 0;
	u32 reg, color, max;
	int ret;

	fwnode_for_each_child_node(mc_node, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret || reg >= MAX77843_LED_CHANNELS ||
		    n >= MAX77843_LED_CHANNELS) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "bad channel node %pfwP\n", child);
		}

		ret = fwnode_property_read_u32(child, "color", &color);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, ret, "%pfwP: no color\n",
					     child);
		}

		if (fwnode_property_read_u32(child, "maxim,brt-code-max",
					     &max))
			max = MAX77843_LED_DEFAULT_MAX;

		led->subled[n].channel = reg;
		led->subled[n].color_index = color;
		/* White until userspace picks a colour. */
		led->subled[n].intensity = LED_FULL;
		led->full_scale[n] = min_t(u32, max, 0xff);
		n++;
	}

	if (!n)
		return dev_err_probe(dev, -ENODEV, "no LED channels\n");

	led->mc.num_colors = n;
	return 0;
}

static int max77843_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77693_dev *max77843 = dev_get_drvdata(dev->parent);
	struct led_init_data init_data = {};
	struct fwnode_handle *mc_node;
	struct max77843_led *led;
	unsigned int i;
	int ret;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->map = max77843->regmap;
	mutex_init(&led->lock);

	mc_node = device_get_named_child_node(dev, "multi-led");
	if (!mc_node)
		return dev_err_probe(dev, -ENODEV, "no multi-led node\n");

	ret = max77843_led_parse_dt(dev, led, mc_node);
	if (ret)
		goto out;

	/* Start dark: S-Boot may leave channels on, ramping or blinking. */
	ret = regmap_write(led->map, MAX77843_LED_REG_LEDEN, 0);
	for (i = 0; !ret && i < MAX77843_LED_CHANNELS; i++)
		ret = regmap_write(led->map, MAX77843_LED_REG_LED0BRT + i, 0);
	if (!ret)
		ret = regmap_write(led->map, MAX77843_LED_REG_LEDRAMP, 0);
	if (ret) {
		dev_err_probe(dev, ret, "cannot reset the LED block\n");
		goto out;
	}

	led->mc.subled_info = led->subled;
	led->mc.led_cdev.max_brightness = LED_FULL;
	led->mc.led_cdev.brightness_set_blocking = max77843_led_brightness_set;
	led->mc.led_cdev.blink_set = max77843_led_blink_set;

	init_data.fwnode = mc_node;
	init_data.devicename = "max77843";
	init_data.default_label = ":";

	ret = devm_led_classdev_multicolor_register_ext(dev, &led->mc,
							&init_data);
	if (ret)
		dev_err_probe(dev, ret, "cannot register the LED\n");
out:
	fwnode_handle_put(mc_node);
	return ret;
}

static const struct of_device_id max77843_led_of_match[] = {
	{ .compatible = "maxim,max77843-led" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77843_led_of_match);

static struct platform_driver max77843_led_driver = {
	.driver = {
		.name = "max77843-led",
		.of_match_table = max77843_led_of_match,
	},
	.probe = max77843_led_probe,
};
module_platform_driver(max77843_led_driver);

MODULE_DESCRIPTION("Maxim MAX77843 RGB indicator LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:max77843-led");
