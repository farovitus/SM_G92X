// SPDX-License-Identifier: GPL-2.0
/*
 * Galaxy S6 / S6 edge (zeroflte, zerolte) audio: Exynos7420 I2S0 and a
 * Cirrus Logic WM1840 (Madera, CS47L85 family).
 *
 * The codec clocks the I2S link. Its FLL1 runs from the 32.768 kHz MCLK2
 * and makes a 98.304 MHz SYSCLK, so only 48 kHz-family rates are offered.
 * The FLL runs only while the codec is out of BIAS_OFF. Routes come from
 * the samsung,audio-routing DT property.
 *
 * The loudspeaker has a MAX98505 smart amp (max98925 driver) on the
 * codec's AIF4, a codec-to-codec link: AIF4 is the I2S clock provider at
 * 48 kHz, 16 bit, 2 slots, and the amp runs from its BCLK.
 *
 * The headset jack is the codec's own accessory detection: the MICDET
 * clamp follows JD1, and MICDET1 measures the mic ring through MICBIAS2.
 *
 * The Shannon modem's voice PCM is wired to the codec's AIF2, the CP
 * driving BCLK and LRCLK: I2S, 2 x 16 bit slots at 8 kHz (narrowband) or
 * 16 kHz (wideband). It is a codec-to-codec link from a machine DAI that
 * stands for the CP, so calls need no host PCM; the audio HAL's mixer
 * paths route AIF2 and pick the rate with the link's Params control. The
 * CP clock is not locked to the codec's, so AIF2 runs on ASYNCCLK from
 * FLL2, which follows AIF2 BCLK, and ASRC1 crosses into SYSCLK.
 */

#include <linux/input.h>
#include <linux/mfd/madera/core.h>
#include <linux/mfd/madera/registers.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "i2s.h"
#include "../codecs/madera.h"

#define ZERO_MCLK2_RATE		32768
#define ZERO_SYSCLK_RATE	98304000
#define ZERO_ASYNCCLK_RATE	98304000

/*
 * max98925 set_sysclk: clock ID 1 selects BCLK as the amp's DAI clock.
 * The frequency only picks the driver's divider table; the vendor driver
 * used 12.288 MHz with BCLK as the source too.
 */
#define ZERO_AMP_CLK_BCLK	1
#define ZERO_AMP_SYSCLK_RATE	12288000

struct zero_priv {
	struct snd_soc_component *codec;
	struct snd_soc_jack jack;
	bool fll2_on;
};

/* Stock headset buttons: impedance up to max ohms, and their keys. */
static const struct madera_micd_range zero_micd_ranges[] = {
	{ .max =  116, .key = KEY_MEDIA },
	{ .max =  173, .key = KEY_VOICECOMMAND },
	{ .max =  321, .key = KEY_VOLUMEUP },
	{ .max =  752, .key = KEY_VOLUMEDOWN },
	{ .max = 1257, .key = KEY_SEARCH },
};

#define ZERO_JACK_BUTTONS	(SND_JACK_BTN_0 | SND_JACK_BTN_1 | \
				 SND_JACK_BTN_2 | SND_JACK_BTN_3 | \
				 SND_JACK_BTN_4)

/* The stock accessory detect settings (wlf,* in the vendor DT). */
static const struct madera_jack_cfg zero_jack_cfg = {
	.micd_clamp_mode		= 0x8,
	.micd_bias			= 2,
	.micd_bias_start_time		= 1,
	.micd_rate			= 7,
	.micd_dbtime			= true,
	.micd_detect_debounce_ms	= 300,
	.micd_timeout_ms		= 1000,
	.init_mic_delay_ms		= 30,
	.micd_software_compare		= true,
	.micd_force_micbias		= true,
	.micd_open_circuit_declare	= true,
	.gpsw				= 0x3,
	.micd_ranges			= zero_micd_ranges,
	.num_micd_ranges		= ARRAY_SIZE(zero_micd_ranges),
};

static struct snd_soc_jack_pin zero_jack_pins[] = {
	{ .pin = "HP",		.mask = SND_JACK_HEADPHONE },
	{ .pin = "Headset Mic",	.mask = SND_JACK_MICROPHONE },
};

static int zero_start_sysclk(struct snd_soc_component *codec)
{
	int ret;

	ret = snd_soc_component_set_pll(codec, MADERA_FLL1_REFCLK,
					MADERA_FLL_SRC_MCLK2, ZERO_MCLK2_RATE,
					ZERO_SYSCLK_RATE);
	if (ret) {
		dev_err(codec->dev, "Failed to start FLL1: %d\n", ret);
		return ret;
	}

	ret = snd_soc_component_set_sysclk(codec, MADERA_CLK_SYSCLK_1,
					   MADERA_CLK_SRC_FLL1,
					   ZERO_SYSCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(codec->dev, "Failed to set SYSCLK: %d\n", ret);
		return ret;
	}

	/* madera only warns on a lock timeout; say where the clock stands. */
	dev_info(codec->dev, "FLL1 %s on MCLK2\n",
		 snd_soc_component_read(codec, MADERA_IRQ1_RAW_STATUS_2) &
		 MADERA_FLL1_LOCK_STS1 ? "locked" : "NOT locked");

	return 0;
}

static void zero_stop_sysclk(struct snd_soc_component *codec)
{
	snd_soc_component_set_sysclk(codec, MADERA_CLK_SYSCLK_1,
				     MADERA_CLK_SRC_FLL1, 0, SND_SOC_CLOCK_IN);
	snd_soc_component_set_pll(codec, MADERA_FLL1_REFCLK,
				  MADERA_FLL_SRC_NONE, 0, 0);
}

static int zero_set_bias_level(struct snd_soc_card *card,
			       struct snd_soc_dapm_context *dapm,
			       enum snd_soc_bias_level level)
{
	struct zero_priv *priv = snd_soc_card_get_drvdata(card);

	if (!priv->codec || dapm->dev != priv->codec->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_dapm_get_bias_level(dapm) == SND_SOC_BIAS_OFF)
			return zero_start_sysclk(priv->codec);
		break;
	case SND_SOC_BIAS_OFF:
		zero_stop_sysclk(priv->codec);
		break;
	default:
		break;
	}

	return 0;
}

static const unsigned int zero_rates[] = {
	8000, 16000, 32000, 48000, 96000,
};

static const struct snd_pcm_hw_constraint_list zero_rate_list = {
	.count = ARRAY_SIZE(zero_rates),
	.list = zero_rates,
};

static int zero_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_RATE,
					  &zero_rate_list);
}

/*
 * The codec drives BCLK and LRCLK, so the I2S is a clock consumer: it
 * does not drive CDCLK, and its core runs from PCLK rather than the
 * default CDCLK out, which nothing drives here. The vendor driver and
 * the mainline boards with a codec clock provider (midas, tm2) do the
 * same.
 */
static int zero_aif1_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK, 0,
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	return snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK, 0,
				      SAMSUNG_I2S_OPCLK_PCLK);
}

static int zero_aif1_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct zero_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	/* The codec is still in BIAS_OFF here; SYSCLK starts on STANDBY. */
	priv->codec = snd_soc_rtd_to_codec(rtd, 0)->component;

	ret = snd_soc_card_jack_new_pins(card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_MECHANICAL |
					 ZERO_JACK_BUTTONS,
					 &priv->jack, zero_jack_pins,
					 ARRAY_SIZE(zero_jack_pins));
	if (ret)
		return ret;

	return snd_soc_component_set_jack(priv->codec, &priv->jack,
					  (void *)&zero_jack_cfg);
}

static void zero_aif1_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct zero_priv *priv = snd_soc_card_get_drvdata(rtd->card);

	snd_soc_component_set_jack(priv->codec, NULL, NULL);
}

static const struct snd_soc_ops zero_aif1_ops = {
	.startup = zero_aif1_startup,
	.hw_params = zero_aif1_hw_params,
};

static int zero_amp_init(struct snd_soc_pcm_runtime *rtd)
{
	return snd_soc_dai_set_sysclk(snd_soc_rtd_to_codec(rtd, 0),
				      ZERO_AMP_CLK_BCLK, ZERO_AMP_SYSCLK_RATE,
				      SND_SOC_CLOCK_IN);
}

static const struct snd_soc_pcm_stream zero_amp_params = {
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min	= 48000,
	.rate_max	= 48000,
	.channels_min	= 2,
	.channels_max	= 2,
};

/*
 * FLL2 makes ASYNCCLK for AIF2. It takes MCLK2 as its reference, so it
 * locks before the CP starts BCLK, and follows AIF2 BCLK once it runs.
 * It runs only while the baseband link is powered.
 */
static int zero_baseband_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	struct zero_priv *priv = snd_soc_card_get_drvdata(dai->component->card);
	unsigned int bclk = params_rate(params) * 32;
	int ret;

	if (!priv->codec)
		return -EPROBE_DEFER;

	ret = snd_soc_component_set_pll(priv->codec, MADERA_FLL2_REFCLK,
					MADERA_FLL_SRC_MCLK2, ZERO_MCLK2_RATE,
					ZERO_ASYNCCLK_RATE);
	if (ret) {
		dev_err(dai->dev, "Failed to start FLL2: %d\n", ret);
		return ret;
	}

	ret = snd_soc_component_set_pll(priv->codec, MADERA_FLL2_SYNCCLK,
					MADERA_FLL_SRC_AIF2BCLK, bclk,
					ZERO_ASYNCCLK_RATE);
	if (ret) {
		dev_err(dai->dev, "Failed to sync FLL2 to AIF2 BCLK: %d\n", ret);
		return ret;
	}

	priv->fll2_on = true;
	dev_info(dai->dev, "baseband %u Hz, FLL2 on AIF2 BCLK %u Hz\n",
		 params_rate(params), bclk);

	return 0;
}

static int zero_baseband_hw_free(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct zero_priv *priv = snd_soc_card_get_drvdata(dai->component->card);

	if (!priv->fll2_on)
		return 0;

	snd_soc_component_set_pll(priv->codec, MADERA_FLL2_SYNCCLK,
				  MADERA_FLL_SRC_NONE, 0, 0);
	snd_soc_component_set_pll(priv->codec, MADERA_FLL2_REFCLK,
				  MADERA_FLL_SRC_NONE, 0, 0);
	priv->fll2_on = false;

	return 0;
}

static const struct snd_soc_dai_ops zero_baseband_dai_ops = {
	.hw_params	= zero_baseband_hw_params,
	.hw_free	= zero_baseband_hw_free,
};

#define ZERO_BASEBAND_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000)

static struct snd_soc_dai_driver zero_baseband_dai = {
	.name = "zero-baseband",
	.playback = {
		.stream_name	= "Modem Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= ZERO_BASEBAND_RATES,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name	= "Modem Capture",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= ZERO_BASEBAND_RATES,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &zero_baseband_dai_ops,
};

/* The CP end of the link: it powers whenever a path through AIF2 is set. */
static const struct snd_soc_dapm_widget zero_modem_widgets[] = {
	SND_SOC_DAPM_INPUT("Modem RX"),
	SND_SOC_DAPM_OUTPUT("Modem TX"),
};

static const struct snd_soc_dapm_route zero_modem_routes[] = {
	{ "Modem Capture", NULL, "Modem RX" },
	{ "Modem TX", NULL, "Modem Playback" },
};

static const struct snd_soc_component_driver zero_modem_component = {
	.name			= "zero-modem",
	.dapm_widgets		= zero_modem_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(zero_modem_widgets),
	.dapm_routes		= zero_modem_routes,
	.num_dapm_routes	= ARRAY_SIZE(zero_modem_routes),
	.idle_bias_on		= 1,
	.endianness		= 1,
};

/*
 * AIF2 runs on ASYNCCLK. SYSCLK sample rates 2 and 3 are the narrowband
 * and wideband call rates, for the ASRC and ISRC between the domains.
 */
static int zero_baseband_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *codec = snd_soc_rtd_to_codec(rtd, 0)->component;
	int ret;

	ret = snd_soc_component_set_sysclk(codec, MADERA_CLK_ASYNCCLK_1,
					   MADERA_CLK_SRC_FLL2,
					   ZERO_ASYNCCLK_RATE,
					   SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(codec->dev, "Failed to set ASYNCCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(snd_soc_rtd_to_codec(rtd, 0),
				     MADERA_CLK_ASYNCCLK_1, 0, 0);
	if (ret) {
		dev_err(codec->dev, "Failed to put AIF2 on ASYNCCLK: %d\n", ret);
		return ret;
	}

	snd_soc_component_update_bits(codec, MADERA_SAMPLE_RATE_2,
				      MADERA_SAMPLE_RATE_1_MASK, 0x11);
	snd_soc_component_update_bits(codec, MADERA_SAMPLE_RATE_3,
				      MADERA_SAMPLE_RATE_1_MASK, 0x12);

	return 0;
}

static const struct snd_soc_pcm_stream zero_baseband_params[] = {
	{
		.stream_name	= "NB",
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min	= 8000,
		.rate_max	= 8000,
		.channels_min	= 2,
		.channels_max	= 2,
	},
	{
		.stream_name	= "WB",
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min	= 16000,
		.rate_max	= 16000,
		.channels_min	= 2,
		.channels_max	= 2,
	},
};

#define ZERO_LINK_BASEBAND	1

/*
 * "Voice Call Rate" picks the baseband link's configuration, NB or WB.
 * The link's own Params control would do the same, but it is created
 * after every routing control, and libaudioroute writes changed controls
 * in index order: it would come too late, with the link already powered
 * by the new path. This card control comes before them.
 */
static const char * const zero_voice_rate_text[] = { "NB", "WB" };

static SOC_ENUM_SINGLE_EXT_DECL(zero_voice_rate_enum, zero_voice_rate_text);

static struct snd_soc_pcm_runtime *zero_baseband_rtd(struct snd_soc_card *card)
{
	return snd_soc_get_pcm_runtime(card, &card->dai_link[ZERO_LINK_BASEBAND]);
}

static int zero_voice_rate_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_pcm_runtime *rtd = zero_baseband_rtd(card);

	ucontrol->value.enumerated.item[0] = rtd ? rtd->c2c_params_select : 0;

	return 0;
}

static int zero_voice_rate_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_pcm_runtime *rtd = zero_baseband_rtd(card);
	unsigned int sel = ucontrol->value.enumerated.item[0];
	int stream;

	if (!rtd || sel >= ARRAY_SIZE(zero_voice_rate_text))
		return -EINVAL;

	if (sel == rtd->c2c_params_select)
		return 0;

	for_each_pcm_streams(stream)
		if (rtd->c2c_widget[stream] && rtd->c2c_widget[stream]->power)
			return -EBUSY;

	rtd->c2c_params_select = sel;

	return 1;
}

static const struct snd_soc_dapm_widget zero_dapm_widgets[] = {
	SND_SOC_DAPM_HP("HP", NULL),
	SND_SOC_DAPM_SPK("RCV", NULL),
	SND_SOC_DAPM_SPK("SPK", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Sub Mic", NULL),
	SND_SOC_DAPM_MIC("Third Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_kcontrol_new zero_controls[] = {
	SOC_DAPM_PIN_SWITCH("HP"),
	SOC_DAPM_PIN_SWITCH("RCV"),
	SOC_DAPM_PIN_SWITCH("SPK"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Sub Mic"),
	SOC_DAPM_PIN_SWITCH("Third Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_ENUM_EXT("Voice Call Rate", zero_voice_rate_enum,
		     zero_voice_rate_get, zero_voice_rate_put),
};

SND_SOC_DAILINK_DEFS(aif1,
	DAILINK_COMP_ARRAY(COMP_CPU(SAMSUNG_I2S_DAI)),
	DAILINK_COMP_ARRAY(COMP_CODEC("cs47l85-codec", "cs47l85-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(baseband,
	DAILINK_COMP_ARRAY(COMP_CPU("zero-baseband")),
	DAILINK_COMP_ARRAY(COMP_CODEC("cs47l85-codec", "cs47l85-aif2")));

SND_SOC_DAILINK_DEFS(amp,
	DAILINK_COMP_ARRAY(COMP_CPU("cs47l85-aif4")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "max98925-aif1")));

static struct snd_soc_dai_link zero_dai_links[] = {
	{
		.name		= "WM1840 AIF1",
		.stream_name	= "HiFi",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBP_CFP,
		.init		= zero_aif1_init,
		.exit		= zero_aif1_exit,
		.ops		= &zero_aif1_ops,
		SND_SOC_DAILINK_REG(aif1),
	},
	{
		.name		= "WM1840 AIF2 Baseband",
		.stream_name	= "Baseband",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBC_CFC,
		.c2c_params	= zero_baseband_params,
		.num_c2c_params	= ARRAY_SIZE(zero_baseband_params),
		.init		= zero_baseband_init,
		.ignore_suspend	= 1,
		SND_SOC_DAILINK_REG(baseband),
	},
	{
		.name		= "WM1840 AIF4 Speaker Amp",
		.stream_name	= "Speaker Amp",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBC_CFC,
		.c2c_params	= &zero_amp_params,
		.num_c2c_params	= 1,
		.init		= zero_amp_init,
		SND_SOC_DAILINK_REG(amp),
	},
};

#define ZERO_LINK_AMP	2

static struct snd_soc_card zero_card = {
	.owner			= THIS_MODULE,
	.dai_link		= zero_dai_links,
	.controls		= zero_controls,
	.num_controls		= ARRAY_SIZE(zero_controls),
	.dapm_widgets		= zero_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(zero_dapm_widgets),
	.set_bias_level		= zero_set_bias_level,
};

static void zero_put_node(void *np)
{
	of_node_put(np);
}

static int zero_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &zero_card;
	struct device_node *i2s_node, *amp_node;
	struct zero_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	card->dev = dev;
	snd_soc_card_set_drvdata(card, priv);

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		return ret;

	ret = snd_soc_of_parse_audio_routing(card, "samsung,audio-routing");
	if (ret)
		return ret;

	i2s_node = of_parse_phandle(dev->of_node, "samsung,i2s-controller", 0);
	if (!i2s_node)
		return dev_err_probe(dev, -EINVAL,
				     "samsung,i2s-controller missing\n");

	ret = devm_add_action_or_reset(dev, zero_put_node, i2s_node);
	if (ret)
		return ret;

	zero_dai_links[0].cpus->of_node = i2s_node;
	zero_dai_links[0].platforms->of_node = i2s_node;

	ret = devm_snd_soc_register_component(dev, &zero_modem_component,
					      &zero_baseband_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register the CP DAI\n");

	/* Without an amp node the card has no AIF4 link. */
	amp_node = of_parse_phandle(dev->of_node, "samsung,speaker-amp", 0);
	if (amp_node) {
		ret = devm_add_action_or_reset(dev, zero_put_node, amp_node);
		if (ret)
			return ret;

		zero_dai_links[ZERO_LINK_AMP].codecs->of_node = amp_node;
		card->num_links = ARRAY_SIZE(zero_dai_links);
	} else {
		card->num_links = ZERO_LINK_AMP;
	}

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id zero_audio_of_match[] = {
	{ .compatible = "samsung,zero-wm1840" },
	{ }
};
MODULE_DEVICE_TABLE(of, zero_audio_of_match);

static struct platform_driver zero_audio_driver = {
	.driver = {
		.name		= "zero-wm1840",
		.pm		= &snd_soc_pm_ops,
		.of_match_table	= zero_audio_of_match,
	},
	.probe = zero_audio_probe,
};
module_platform_driver(zero_audio_driver);

MODULE_DESCRIPTION("Galaxy S6 WM1840 audio");
MODULE_LICENSE("GPL");
