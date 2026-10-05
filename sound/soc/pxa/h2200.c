// SPDX-License-Identifier: GPL-2.0-only
/*
 * h2200.c -- SoC audio for the HP iPAQ h2200 (h2210/h2212/h2215)
 *
 * The codec is a Philips/NXP UDA1380 on the PXA25x I2C bus, fed I2S with
 * the PXA as bit, frame and system clock master. The machine driver is
 * modelled on spitz.c, and the GPIO roles and polarities below follow the
 * original 2.6 handheld driver for this board.
 *
 * Board GPIOs this driver owns, taken from properties of its devicetree
 * node:
 *
 *  - "speaker": the onboard mono speaker amplifier enable.
 *
 *  - "tda-mode": driven as a complementary pair with "speaker", never
 *    independently: unmute is speaker=1, tda-mode=0 and mute is
 *    speaker=0, tda-mode=1 on the raw pins. The devicetree declares it
 *    GPIO_ACTIVE_LOW so that h2200_spk_event() can write the same logical
 *    value to both.
 *
 *  - "mic": external microphone bias enable, active low. The UDA1380 has
 *    no bias widget of its own, the bias is entirely external to it on
 *    this board.
 *
 *  - "ear-in": headphone jack detect, a pin of the HAMCOP companion chip
 *    that reads high when a plug is inserted. snd_soc_jack_add_gpiods()
 *    owns the interrupt and debouncing, and the two jack pins disable
 *    the speaker while a plug is in.
 *
 * The codec's own power and reset lines belong to its devicetree node and
 * are handled by the codec driver.
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/soc.h>

static struct gpio_desc *gpiod_speaker, *gpiod_tda_mode, *gpiod_mic;

/*
 * Fires on every DAPM power transition of the "Speaker" widget, that is
 * whenever audio starts or stops being routed to VOUTL/VOUTR.
 */
static int h2200_spk_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *k, int event)
{
	int on = SND_SOC_DAPM_EVENT_ON(event);

	gpiod_set_value_cansleep(gpiod_speaker, on);
	gpiod_set_value_cansleep(gpiod_tda_mode, on);

	return 0;
}

static int h2200_mic_bias(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	gpiod_set_value_cansleep(gpiod_mic, SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget h2200_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", h2200_spk_event),
	SND_SOC_DAPM_MIC("Mic Jack", h2200_mic_bias),
};

static const struct snd_soc_dapm_route h2200_audio_map[] = {
	{"Headphone Jack", NULL, "VOUTLHP"},
	{"Headphone Jack", NULL, "VOUTRHP"},

	{"Speaker", NULL, "VOUTL"},
	{"Speaker", NULL, "VOUTR"},

	{"VINM", NULL, "Mic Jack"},
};

/*
 * The components are matched by devicetree phandle, filled in by
 * h2200_probe(). The I2S block is a combined CPU and platform component,
 * so one phandle serves both roles. The codec DAI is the combined
 * full-duplex one, which is what the single I2S interface uses.
 */
SND_SOC_DAILINK_DEFS(uda1380,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "uda1380-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * Two pins toggled in lockstep with opposite polarity, so exactly one of
 * the headphones and the speaker is enabled at a time.
 */
static struct snd_soc_jack h2200_hp_jack;

static struct snd_soc_jack_pin h2200_hp_jack_pins[] = {
	{ .pin = "Headphone Jack", .mask = SND_JACK_HEADPHONE },
	{ .pin = "Speaker", .mask = SND_JACK_HEADPHONE, .invert = 1 },
};

static struct snd_soc_jack_gpio h2200_hp_jack_gpio = {
	.name = "ear-in",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
};

/*
 * The jack is created from the dai_link init callback because the sound
 * card is not allocated yet when the probe function returns.
 */
static int h2200_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	int ret;

	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack", SND_JACK_HEADPHONE,
					 &h2200_hp_jack, h2200_hp_jack_pins,
					 ARRAY_SIZE(h2200_hp_jack_pins));
	if (ret)
		return ret;

	return snd_soc_jack_add_gpiods(card->dev, &h2200_hp_jack, 1,
				       &h2200_hp_jack_gpio);
}

/*
 * The format is given from the codec's point of view and is flipped
 * before it reaches the CPU DAI, so the PXA ends up the I2S master. The
 * UDA1380 driver rejects anything but codec clock consumer.
 */
static struct snd_soc_dai_link h2200_dai = {
	.name = "uda1380",
	.stream_name = "UDA1380",
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBC_CFC,
	.init = h2200_dai_init,
	SND_SOC_DAILINK_REG(uda1380),
};

static struct snd_soc_card snd_soc_h2200 = {
	.name = "iPAQ h2200",
	.owner = THIS_MODULE,
	.dai_link = &h2200_dai,
	.num_links = 1,

	.dapm_widgets = h2200_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(h2200_dapm_widgets),
	.dapm_routes = h2200_audio_map,
	.num_dapm_routes = ARRAY_SIZE(h2200_audio_map),
	.fully_routed = true,
};

static const struct of_device_id h2200_audio_of_match[] = {
	{ .compatible = "hp,ipaq-h2200-audio", },
	{ }
};
MODULE_DEVICE_TABLE(of, h2200_audio_of_match);

static int h2200_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_h2200;
	struct device *dev = &pdev->dev;
	int ret;

	h2200_dai.cpus[0].of_node = of_parse_phandle(dev->of_node, "cpu-dai", 0);
	if (!h2200_dai.cpus[0].of_node)
		return dev_err_probe(dev, -EINVAL, "missing cpu-dai phandle\n");

	h2200_dai.platforms[0].of_node = h2200_dai.cpus[0].of_node;

	h2200_dai.codecs[0].of_node = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!h2200_dai.codecs[0].of_node) {
		of_node_put(h2200_dai.cpus[0].of_node);
		return dev_err_probe(dev, -EINVAL, "missing audio-codec phandle\n");
	}

	gpiod_speaker = devm_gpiod_get(dev, "speaker", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod_speaker)) {
		ret = PTR_ERR(gpiod_speaker);
		goto err_put_nodes;
	}

	gpiod_tda_mode = devm_gpiod_get(dev, "tda-mode", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod_tda_mode)) {
		ret = PTR_ERR(gpiod_tda_mode);
		goto err_put_nodes;
	}

	gpiod_mic = devm_gpiod_get(dev, "mic", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod_mic)) {
		ret = PTR_ERR(gpiod_mic);
		goto err_put_nodes;
	}

	card->dev = dev;

	ret = devm_snd_soc_register_card(dev, card);
	if (ret) {
		dev_err_probe(dev, ret, "snd_soc_register_card() failed\n");
		goto err_put_nodes;
	}

	return 0;

err_put_nodes:
	of_node_put(h2200_dai.codecs[0].of_node);
	of_node_put(h2200_dai.cpus[0].of_node);
	return ret;
}

/*
 * The card may match components against these nodes for as long as it
 * exists, so they are only released when the driver unbinds.
 */
static void h2200_remove(struct platform_device *pdev)
{
	of_node_put(h2200_dai.codecs[0].of_node);
	of_node_put(h2200_dai.cpus[0].of_node);
}

static struct platform_driver h2200_driver = {
	.driver		= {
		.name		= "h2200-audio",
		.pm		= &snd_soc_pm_ops,
		.of_match_table	= h2200_audio_of_match,
	},
	.probe		= h2200_probe,
	.remove		= h2200_remove,
};

module_platform_driver(h2200_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("ALSA SoC HP iPAQ h2200");
MODULE_LICENSE("GPL");
