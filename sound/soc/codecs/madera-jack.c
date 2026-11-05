// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Accessory detection for Cirrus Logic Madera codecs
 *
 * Jack insertion comes from the MICDET clamp, which the jack detect
 * inputs control. MICDET1 then tells a headset from headphones and
 * reports the headset buttons. Headphone impedance (HPDET) and moisture
 * detection are not done.
 *
 * Based on arizona-jack.c:
 *  Copyright (C) 2012-2014 Wolfson Microelectronics plc
 */

#include <linux/delay.h>
#include <linux/irqchip/irq-madera.h>
#include <linux/mfd/madera/core.h>
#include <linux/mfd/madera/registers.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <sound/jack.h>
#include <sound/soc.h>

#include "madera.h"

#define MADERA_ACCDET_MODE_MIC		0
#define MADERA_ACCDET_MODE_ADC		7

#define MADERA_MICD_MAX_RANGE		8
/* The hardware has 8 ranges, snd_jack has 6 buttons. */
#define MADERA_MICD_MAX_BUTTONS		6
/* Levels of madera_micd_levels usable as button thresholds */
#define MADERA_MICD_BUTTON_LEVELS	64
#define MADERA_MICD_RETRIES		10

#define QUICK_HEADPHONE_MAX_OHM		3
#define MICROPHONE_MIN_OHM		1257
#define MICROPHONE_MAX_OHM		30000

#define MICD_LVL_1_TO_7 (MADERA_MICD_LVL_1 | MADERA_MICD_LVL_2 | \
			 MADERA_MICD_LVL_3 | MADERA_MICD_LVL_4 | \
			 MADERA_MICD_LVL_5 | MADERA_MICD_LVL_6 | \
			 MADERA_MICD_LVL_7)
#define MICD_LVL_0_TO_7 (MADERA_MICD_LVL_0 | MICD_LVL_1_TO_7)
#define MICD_LVL_0_TO_8 (MICD_LVL_0_TO_7 | MADERA_MICD_LVL_8)

#define MADERA_JACK_MASK (SND_JACK_HEADSET | SND_JACK_MECHANICAL)

static const int madera_micd_levels[] = {
	3, 6, 8, 11, 13, 16, 18, 21, 23, 26, 28, 31, 34, 36, 39, 41, 44, 46,
	49, 52, 54, 57, 60, 62, 65, 67, 70, 73, 75, 78, 81, 83, 89, 94, 100,
	105, 111, 116, 122, 127, 139, 150, 161, 173, 186, 196, 209, 220, 245,
	270, 295, 321, 348, 375, 402, 430, 489, 550, 614, 681, 752, 903, 1071,
	1257, 30000,
};

static const char * const madera_micd_bias_widget[] = {
	"MICVDD", "MICBIAS1", "MICBIAS2", "MICBIAS3", "MICBIAS4",
};

struct madera_jack {
	struct madera *madera;
	struct snd_soc_component *component;
	struct snd_soc_jack *jack;
	const struct madera_jack_cfg *cfg;
	struct regulator *micvdd;

	struct mutex lock;
	struct delayed_work micd_detect_work;
	struct delayed_work micd_timeout_work;

	unsigned int last_jackdet;
	int button_mask;
	int retries;
	bool micd_active;
	bool wake;
	bool detecting;
	bool mic;
};

static void madera_jack_pulse_micbias(struct madera_jack *info)
{
	const char *widget = madera_micd_bias_widget[info->cfg->micd_bias];
	struct snd_soc_component *component = info->component;
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_get_dapm(component);
	int ret;

	ret = snd_soc_component_force_enable_pin(component, widget);
	if (ret)
		dev_warn(component->dev, "Failed to enable %s: %d\n",
			 widget, ret);

	snd_soc_dapm_sync(dapm);

	if (!info->cfg->micd_force_micbias) {
		ret = snd_soc_component_disable_pin(component, widget);
		if (ret)
			dev_warn(component->dev, "Failed to disable %s: %d\n",
				 widget, ret);

		snd_soc_dapm_sync(dapm);
	}
}

static void madera_jack_start_mic(struct madera_jack *info)
{
	struct madera *madera = info->madera;
	unsigned int mode;
	int ret;

	if (!info->micd_active) {
		/* Microphone detection can't use idle mode */
		pm_runtime_get_sync(madera->dev);

		ret = regulator_enable(info->micvdd);
		if (ret) {
			dev_err(madera->dev, "Failed to enable MICVDD: %d\n",
				ret);
			pm_runtime_put_autosuspend(madera->dev);
			return;
		}

		info->micd_active = true;
	}

	if (info->detecting) {
		ret = regulator_allow_bypass(info->micvdd, false);
		if (ret)
			dev_err(madera->dev, "Failed to regulate MICVDD: %d\n",
				ret);
	}

	if (info->detecting && info->cfg->micd_software_compare)
		mode = MADERA_ACCDET_MODE_ADC;
	else
		mode = MADERA_ACCDET_MODE_MIC;

	regmap_update_bits(madera->regmap, MADERA_ACCESSORY_DETECT_MODE_1,
			   MADERA_ACCDET_MODE_MASK, mode);

	madera_jack_pulse_micbias(info);

	regmap_update_bits(madera->regmap, MADERA_MIC_DETECT_1_CONTROL_1,
			   MADERA_MICD_ENA, MADERA_MICD_ENA);
}

static void madera_jack_stop_mic(struct madera_jack *info)
{
	const char *widget = madera_micd_bias_widget[info->cfg->micd_bias];
	struct snd_soc_component *component = info->component;
	struct madera *madera = info->madera;
	int ret;

	regmap_update_bits(madera->regmap, MADERA_MIC_DETECT_1_CONTROL_1,
			   MADERA_MICD_ENA, 0);

	ret = snd_soc_component_disable_pin(component, widget);
	if (ret)
		dev_warn(component->dev, "Failed to disable %s: %d\n",
			 widget, ret);

	snd_soc_dapm_sync(snd_soc_component_get_dapm(component));

	if (!info->micd_active)
		return;

	ret = regulator_allow_bypass(info->micvdd, true);
	if (ret)
		dev_err(madera->dev, "Failed to bypass MICVDD: %d\n", ret);

	regulator_disable(info->micvdd);
	info->micd_active = false;

	pm_runtime_mark_last_busy(madera->dev);
	pm_runtime_put_autosuspend(madera->dev);
}

/* Identification is over and found no headset mic. */
static void madera_jack_report_headphone(struct madera_jack *info)
{
	info->detecting = false;
	info->mic = false;
	madera_jack_stop_mic(info);

	dev_dbg(info->madera->dev, "Detected headphones\n");
	snd_soc_jack_report(info->jack, SND_JACK_HEADPHONE, SND_JACK_HEADSET);
}

/* A headset: keep MICDET running, in level mode, for the buttons. */
static void madera_jack_report_headset(struct madera_jack *info)
{
	int ret;

	info->detecting = false;
	info->mic = true;

	dev_dbg(info->madera->dev, "Detected headset\n");
	snd_soc_jack_report(info->jack, SND_JACK_HEADSET, SND_JACK_HEADSET);

	/* Button detection does not need a regulated MICVDD. */
	ret = regulator_allow_bypass(info->micvdd, true);
	if (ret)
		dev_err(info->madera->dev, "Failed to bypass MICVDD: %d\n",
			ret);

	madera_jack_start_mic(info);
}

static void madera_jack_micd_timeout_work(struct work_struct *work)
{
	struct madera_jack *info = container_of(work, struct madera_jack,
						micd_timeout_work.work);

	mutex_lock(&info->lock);

	dev_dbg(info->madera->dev, "MICD timed out, reporting HP\n");
	madera_jack_report_headphone(info);

	mutex_unlock(&info->lock);
}

static int madera_jack_micd_adc_read(struct madera_jack *info)
{
	struct madera *madera = info->madera;
	unsigned int val;
	int ret;

	/* Must disable MICD before we read the ADCVAL */
	regmap_update_bits(madera->regmap, MADERA_MIC_DETECT_1_CONTROL_1,
			   MADERA_MICD_ENA, 0);

	ret = regmap_read(madera->regmap, MADERA_MIC_DETECT_1_CONTROL_4, &val);
	if (ret) {
		dev_err(madera->dev, "Failed to read MICDET_ADCVAL: %d\n", ret);
		return ret;
	}

	dev_dbg(madera->dev, "MICDET_ADCVAL: %x\n", val);

	val &= MADERA_MICDET_ADCVAL_MASK;
	if (val < ARRAY_SIZE(madera_micd_levels))
		val = madera_micd_levels[val];
	else
		val = INT_MAX;

	if (val <= QUICK_HEADPHONE_MAX_OHM)
		return MADERA_MICD_STS | MADERA_MICD_LVL_0;
	else if (val <= MICROPHONE_MIN_OHM)
		return MADERA_MICD_STS | MADERA_MICD_LVL_1;
	else if (val <= MICROPHONE_MAX_OHM)
		return MADERA_MICD_STS | MADERA_MICD_LVL_8;
	else
		return MADERA_MICD_LVL_8;
}

static int madera_jack_micd_read(struct madera_jack *info)
{
	struct madera *madera = info->madera;
	unsigned int val = 0;
	int ret, i;

	for (i = 0; i < 10 && !(val & MICD_LVL_0_TO_8); i++) {
		ret = regmap_read(madera->regmap, MADERA_MIC_DETECT_1_CONTROL_3,
				  &val);
		if (ret) {
			dev_err(madera->dev, "Failed to read MICDET: %d\n",
				ret);
			return ret;
		}

		dev_dbg(madera->dev, "MICDET: %x\n", val);

		if (!(val & MADERA_MICD_VALID)) {
			dev_warn(madera->dev,
				 "Microphone detection state invalid\n");
			return -EINVAL;
		}
	}

	if (i == 10 && !(val & MICD_LVL_0_TO_8)) {
		dev_err(madera->dev, "Failed to get valid MICDET value\n");
		return -EINVAL;
	}

	return val;
}

static void madera_jack_micdet_reading(struct madera_jack *info)
{
	int val;

	if (info->cfg->micd_software_compare)
		val = madera_jack_micd_adc_read(info);
	else
		val = madera_jack_micd_read(info);
	if (val < 0)
		return;

	/* Open circuit: no contact on the mic ring. */
	if (!(val & MADERA_MICD_STS)) {
		dev_dbg(info->madera->dev, "Detected open circuit\n");
		if (info->cfg->micd_open_circuit_declare)
			madera_jack_report_headset(info);
		else
			madera_jack_report_headphone(info);
		return;
	}

	/* A high impedance is a headset mic. */
	if (val & MADERA_MICD_LVL_8) {
		madera_jack_report_headset(info);
		return;
	}

	/*
	 * A middle impedance during insertion may be a plug that is not
	 * fully in yet: measure again, then settle for headphones.
	 */
	if ((val & MICD_LVL_1_TO_7) &&
	    info->retries++ < MADERA_MICD_RETRIES) {
		if (info->cfg->micd_software_compare)
			regmap_update_bits(info->madera->regmap,
					   MADERA_MIC_DETECT_1_CONTROL_1,
					   MADERA_MICD_ENA, MADERA_MICD_ENA);

		queue_delayed_work(system_power_efficient_wq,
				   &info->micd_timeout_work,
				   msecs_to_jiffies(info->cfg->micd_timeout_ms));
		return;
	}

	/* A short on the mic ring, a three-pole plug, or no stable reading */
	madera_jack_report_headphone(info);
}

static void madera_jack_button_reading(struct madera_jack *info)
{
	struct madera *madera = info->madera;
	int val, lvl, key;

	val = madera_jack_micd_read(info);
	if (val < 0)
		return;

	if (val & MICD_LVL_0_TO_7) {
		if (!info->mic) {
			dev_warn(madera->dev, "Button with no mic: %x\n", val);
			return;
		}

		lvl = (val & MADERA_MICD_LVL_MASK) >> MADERA_MICD_LVL_SHIFT;
		key = ffs(lvl) - 1;
		if (lvl && key < info->cfg->num_micd_ranges) {
			dev_dbg(madera->dev, "Mic button %d pressed\n", key);
			snd_soc_jack_report(info->jack, SND_JACK_BTN_0 >> key,
					    info->button_mask);
		} else {
			dev_err(madera->dev, "Button out of range: %x\n", val);
		}
	} else {
		dev_dbg(madera->dev, "Mic button released\n");
		snd_soc_jack_report(info->jack, 0, info->button_mask);
		madera_jack_pulse_micbias(info);
	}
}

static void madera_jack_micd_detect(struct work_struct *work)
{
	struct madera_jack *info = container_of(work, struct madera_jack,
						micd_detect_work.work);

	cancel_delayed_work_sync(&info->micd_timeout_work);

	mutex_lock(&info->lock);

	/* If the cable was removed while measuring ignore the result */
	if (!(info->jack->status & SND_JACK_MECHANICAL)) {
		dev_dbg(info->madera->dev, "Ignoring MICDET for removed cable\n");
		goto out;
	}

	if (info->detecting)
		madera_jack_micdet_reading(info);
	else
		madera_jack_button_reading(info);

out:
	pm_runtime_mark_last_busy(info->madera->dev);
	mutex_unlock(&info->lock);
}

static irqreturn_t madera_jack_micdet(int irq, void *data)
{
	struct madera_jack *info = data;
	unsigned int debounce = info->cfg->micd_detect_debounce_ms;

	cancel_delayed_work_sync(&info->micd_detect_work);
	cancel_delayed_work_sync(&info->micd_timeout_work);

	mutex_lock(&info->lock);
	if (!info->detecting)
		debounce = 0;
	mutex_unlock(&info->lock);

	if (debounce)
		queue_delayed_work(system_power_efficient_wq,
				   &info->micd_detect_work,
				   msecs_to_jiffies(debounce));
	else
		madera_jack_micd_detect(&info->micd_detect_work.work);

	return IRQ_HANDLED;
}

static irqreturn_t madera_jack_jackdet(int irq, void *data)
{
	struct madera_jack *info = data;
	struct madera *madera = info->madera;
	unsigned int val;
	bool cancelled_mic;
	int ret;

	cancelled_mic = cancel_delayed_work_sync(&info->micd_timeout_work);

	pm_runtime_get_sync(madera->dev);

	mutex_lock(&info->lock);

	ret = regmap_read(madera->regmap, MADERA_IRQ1_RAW_STATUS_7, &val);
	if (ret) {
		dev_err(madera->dev, "Failed to read jackdet status: %d\n",
			ret);
		goto out;
	}

	/* The clamp is released while a plug is in. */
	val &= MADERA_MICD_CLAMP_RISE_STS1;
	if (val == info->last_jackdet) {
		dev_dbg(madera->dev, "Suppressing duplicate JACKDET\n");
		if (cancelled_mic)
			queue_delayed_work(system_power_efficient_wq,
					   &info->micd_timeout_work,
					   msecs_to_jiffies(info->cfg->micd_timeout_ms));
		goto out;
	}
	info->last_jackdet = val;

	if (!val) {
		dev_dbg(madera->dev, "Detected jack\n");
		snd_soc_jack_report(info->jack, SND_JACK_MECHANICAL,
				    SND_JACK_MECHANICAL);

		info->detecting = true;
		info->mic = false;
		info->retries = 0;

		if (info->cfg->init_mic_delay_ms)
			msleep(info->cfg->init_mic_delay_ms);

		madera_jack_start_mic(info);

		queue_delayed_work(system_power_efficient_wq,
				   &info->micd_timeout_work,
				   msecs_to_jiffies(info->cfg->micd_timeout_ms));

		regmap_update_bits(madera->regmap, MADERA_INTERRUPT_DEBOUNCE_7,
				   MADERA_MICD_CLAMP_DB | MADERA_JD1_DB, 0);
	} else {
		dev_dbg(madera->dev, "Detected jack removal\n");

		info->detecting = false;
		info->mic = false;
		madera_jack_stop_mic(info);

		snd_soc_jack_report(info->jack, 0,
				    MADERA_JACK_MASK | info->button_mask);

		regmap_update_bits(madera->regmap, MADERA_INTERRUPT_DEBOUNCE_7,
				   MADERA_MICD_CLAMP_DB | MADERA_JD1_DB,
				   MADERA_MICD_CLAMP_DB | MADERA_JD1_DB);
	}

out:
	mutex_unlock(&info->lock);

	pm_runtime_mark_last_busy(madera->dev);
	pm_runtime_put_autosuspend(madera->dev);

	return IRQ_HANDLED;
}

/* Map a level onto a slot in the register bank */
static void madera_jack_set_level(struct madera *madera, int index,
				  unsigned int level)
{
	unsigned int reg = MADERA_MIC_DETECT_1_LEVEL_4 - (index / 2);
	unsigned int mask = 0x3f;

	if (!(index % 2)) {
		mask <<= 8;
		level <<= 8;
	}

	regmap_update_bits(madera->regmap, reg, mask, level);
}

static int madera_jack_check_cfg(struct device *dev,
				 const struct madera_jack_cfg *cfg)
{
	int i;

	if (!cfg->micd_bias ||
	    cfg->micd_bias >= ARRAY_SIZE(madera_micd_bias_widget)) {
		dev_err(dev, "Invalid MICD bias %u\n", cfg->micd_bias);
		return -EINVAL;
	}

	if (cfg->num_micd_ranges > MADERA_MICD_MAX_BUTTONS) {
		dev_err(dev, "Too many MICD ranges: %d > %d\n",
			cfg->num_micd_ranges, MADERA_MICD_MAX_BUTTONS);
		return -EINVAL;
	}

	for (i = 0; i < cfg->num_micd_ranges; i++) {
		if (i && cfg->micd_ranges[i - 1].max > cfg->micd_ranges[i].max) {
			dev_err(dev, "MICD ranges must be sorted\n");
			return -EINVAL;
		}

		if (cfg->micd_ranges[i].max >
		    madera_micd_levels[MADERA_MICD_BUTTON_LEVELS - 1]) {
			dev_err(dev, "Unsupported MICD level %d\n",
				cfg->micd_ranges[i].max);
			return -EINVAL;
		}
	}

	return 0;
}

static void madera_jack_setup(struct madera_jack *info)
{
	const struct madera_jack_cfg *cfg = info->cfg;
	struct regmap *regmap = info->madera->regmap;
	int i, j;

	regmap_update_bits(regmap, MADERA_MIC_DETECT_1_CONTROL_1,
			   MADERA_MICD_BIAS_STARTTIME_MASK |
			   MADERA_MICD_RATE_MASK |
			   MADERA_MICD_BIAS_SRC_MASK |
			   MADERA_MICD_DBTIME_MASK,
			   cfg->micd_bias_start_time <<
			   MADERA_MICD_BIAS_STARTTIME_SHIFT |
			   cfg->micd_rate << MADERA_MICD_RATE_SHIFT |
			   cfg->micd_bias << MADERA_MICD_BIAS_SRC_SHIFT |
			   (cfg->micd_dbtime ? MADERA_MICD_DBTIME : 0));

	regmap_update_bits(regmap, MADERA_ACCESSORY_DETECT_MODE_1,
			   MADERA_ACCDET_SRC, 0);

	/* Disable all buttons by default */
	regmap_update_bits(regmap, MADERA_MIC_DETECT_1_CONTROL_2,
			   MADERA_MICD_LVL_SEL_MASK, 0x81);

	for (i = 0; i < cfg->num_micd_ranges; i++) {
		for (j = 0; j < MADERA_MICD_BUTTON_LEVELS; j++)
			if (madera_micd_levels[j] >= cfg->micd_ranges[i].max)
				break;

		madera_jack_set_level(info->madera, i, j);

		/* SND_JACK_BTN_# masks start with the most significant bit */
		info->button_mask |= SND_JACK_BTN_0 >> i;
		snd_jack_set_key(info->jack->jack, SND_JACK_BTN_0 >> i,
				 cfg->micd_ranges[i].key);

		regmap_update_bits(regmap, MADERA_MIC_DETECT_1_CONTROL_2,
				   1 << i, 1 << i);
	}

	/* Set all the remaining levels to the maximum */
	for (; i < MADERA_MICD_MAX_RANGE; i++)
		madera_jack_set_level(info->madera, i, 0x3f);

	if (cfg->gpsw)
		regmap_update_bits(regmap, MADERA_GP_SWITCH_1,
				   MADERA_SW1_MODE_MASK, cfg->gpsw);

	/* Let the jack detect inputs, not the override, drive the clamp. */
	regmap_update_bits(regmap, MADERA_MICD_CLAMP_CONTROL,
			   MADERA_MICD_CLAMP_OVD | MADERA_MICD_CLAMP_MODE_MASK,
			   cfg->micd_clamp_mode);

	regmap_update_bits(regmap, MADERA_INTERRUPT_DEBOUNCE_7,
			   MADERA_MICD_CLAMP_DB | MADERA_JD1_DB,
			   MADERA_MICD_CLAMP_DB | MADERA_JD1_DB);
	regmap_update_bits(regmap, MADERA_JACK_DETECT_ANALOGUE,
			   MADERA_JD1_ENA, MADERA_JD1_ENA);
}

static int madera_jack_enable(struct madera_priv *priv,
			      struct snd_soc_component *component,
			      struct snd_soc_jack *jack,
			      const struct madera_jack_cfg *cfg)
{
	struct madera *madera = priv->madera;
	struct madera_jack *info;
	int ret;

	if (!cfg)
		return -EINVAL;

	if (priv->jack)
		return -EBUSY;

	ret = madera_jack_check_cfg(component->dev, cfg);
	if (ret)
		return ret;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->madera = madera;
	info->component = component;
	info->jack = jack;
	info->cfg = cfg;
	info->last_jackdet = ~MADERA_MICD_CLAMP_RISE_STS1;
	mutex_init(&info->lock);
	INIT_DELAYED_WORK(&info->micd_detect_work, madera_jack_micd_detect);
	INIT_DELAYED_WORK(&info->micd_timeout_work,
			  madera_jack_micd_timeout_work);

	info->micvdd = regulator_get(component->dev, "MICVDD");
	if (IS_ERR(info->micvdd)) {
		ret = dev_err_probe(component->dev, PTR_ERR(info->micvdd),
				    "Failed to get MICVDD\n");
		goto err_free;
	}

	pm_runtime_get_sync(madera->dev);

	madera_jack_setup(info);

	ret = madera_request_irq(madera, MADERA_IRQ_MICD_CLAMP_RISE,
				 "JACKDET rise", madera_jack_jackdet, info);
	if (ret) {
		dev_err(madera->dev, "Failed to get JACKDET rise IRQ: %d\n",
			ret);
		goto err_pm;
	}

	ret = madera_request_irq(madera, MADERA_IRQ_MICD_CLAMP_FALL,
				 "JACKDET fall", madera_jack_jackdet, info);
	if (ret) {
		dev_err(madera->dev, "Failed to get JACKDET fall IRQ: %d\n",
			ret);
		goto err_rise;
	}

	ret = madera_request_irq(madera, MADERA_IRQ_MICDET1,
				 "MICDET", madera_jack_micdet, info);
	if (ret) {
		dev_err(madera->dev, "Failed to get MICDET IRQ: %d\n", ret);
		goto err_fall;
	}

	info->wake = !madera_set_irq_wake(madera, MADERA_IRQ_MICD_CLAMP_RISE, 1);
	if (info->wake &&
	    madera_set_irq_wake(madera, MADERA_IRQ_MICD_CLAMP_FALL, 1)) {
		madera_set_irq_wake(madera, MADERA_IRQ_MICD_CLAMP_RISE, 0);
		info->wake = false;
	}
	if (!info->wake)
		dev_warn(madera->dev, "Jack detect cannot wake the system\n");

	ret = regulator_allow_bypass(info->micvdd, true);
	if (ret)
		dev_warn(madera->dev, "Failed to set MICVDD to bypass: %d\n",
			 ret);

	priv->jack = info;

	pm_runtime_put(madera->dev);

	/* The clamp IRQs are edges: pick up a plug that is already in. */
	madera_jack_jackdet(0, info);

	return 0;

err_fall:
	madera_free_irq(madera, MADERA_IRQ_MICD_CLAMP_FALL, info);
err_rise:
	madera_free_irq(madera, MADERA_IRQ_MICD_CLAMP_RISE, info);
	/* A jack IRQ may already have started detection. */
	cancel_delayed_work_sync(&info->micd_timeout_work);
	mutex_lock(&info->lock);
	madera_jack_stop_mic(info);
	mutex_unlock(&info->lock);
err_pm:
	regmap_update_bits(madera->regmap, MADERA_JACK_DETECT_ANALOGUE,
			   MADERA_JD1_ENA, 0);
	regmap_update_bits(madera->regmap, MADERA_MICD_CLAMP_CONTROL,
			   MADERA_MICD_CLAMP_MODE_MASK, 0);
	pm_runtime_put(madera->dev);
	regulator_put(info->micvdd);
err_free:
	kfree(info);
	return ret;
}

static int madera_jack_disable(struct madera_priv *priv)
{
	struct madera_jack *info = priv->jack;
	struct madera *madera;

	if (!info)
		return 0;

	madera = info->madera;

	if (info->wake) {
		madera_set_irq_wake(madera, MADERA_IRQ_MICD_CLAMP_RISE, 0);
		madera_set_irq_wake(madera, MADERA_IRQ_MICD_CLAMP_FALL, 0);
	}
	madera_free_irq(madera, MADERA_IRQ_MICDET1, info);
	madera_free_irq(madera, MADERA_IRQ_MICD_CLAMP_RISE, info);
	madera_free_irq(madera, MADERA_IRQ_MICD_CLAMP_FALL, info);
	cancel_delayed_work_sync(&info->micd_detect_work);
	cancel_delayed_work_sync(&info->micd_timeout_work);

	mutex_lock(&info->lock);
	madera_jack_stop_mic(info);
	mutex_unlock(&info->lock);

	regmap_update_bits(madera->regmap, MADERA_JACK_DETECT_ANALOGUE,
			   MADERA_JD1_ENA, 0);
	regmap_update_bits(madera->regmap, MADERA_MICD_CLAMP_CONTROL,
			   MADERA_MICD_CLAMP_MODE_MASK, 0);

	regulator_put(info->micvdd);
	priv->jack = NULL;
	kfree(info);

	return 0;
}

/**
 * madera_jack_set_jack - start or stop accessory detection
 * @component: the Madera codec
 * @jack: jack to report to, with SND_JACK_HEADSET, SND_JACK_MECHANICAL and
 *	  one SND_JACK_BTN_n per button; NULL stops detection
 * @data: the board's struct madera_jack_cfg, which must outlive the jack
 */
int madera_jack_set_jack(struct snd_soc_component *component,
			 struct snd_soc_jack *jack, void *data)
{
	struct madera_priv *priv = snd_soc_component_get_drvdata(component);

	if (jack)
		return madera_jack_enable(priv, component, jack, data);
	else
		return madera_jack_disable(priv);
}
EXPORT_SYMBOL_GPL(madera_jack_set_jack);
