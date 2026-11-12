// SPDX-License-Identifier: GPL-2.0-only
/*
 * h3900.c -- SoC audio for the HP iPAQ h3900 family (h3950/h3955/h3970/
 * h3975)
 *
 * The codec is a Philips/NXP UDA1380 on the PXA25x I2C bus at address
 * 0x18, fed I2S with the PXA as bit and frame clock master. The machine
 * driver follows h2200.c.
 *
 * Board GPIOs this driver owns, taken from properties of its devicetree
 * node:
 *
 *  - "speaker": the speaker amplifier enable, an ASIC3 output. Unlike the
 *    h2200 it has no complementary partner line.
 *
 *  - "jack": headphone jack detect, ASIC2 GPIO2 bit 7 (EAR_IN_N), which
 *    reads low while a plug is inserted. It is optional; without it the
 *    speaker is never gated by the jack. snd_soc_jack_add_gpiods() owns
 *    the interrupt and debouncing, and the two jack pins disable the
 *    speaker while a plug is in.
 *
 * The microphone is a built-in electret on the codec's VINM input, biased
 * by the codec's own PON_BIAS, so there is no bias line to drive and the
 * widget below only makes the codec's microphone path reachable. The
 * codec's input mux still has to be set to the microphone from userspace.
 *
 * The codec's own reset and power lines belong to its devicetree node and
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

static struct gpio_desc *gpiod_speaker;

/*
 * Fires on every DAPM power transition of the "Speaker" widget, that is
 * whenever audio starts or stops being routed to VOUTL/VOUTR.
 */
static int h3900_spk_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *k, int event)
{
	gpiod_set_value_cansleep(gpiod_speaker, SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget h3900_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", h3900_spk_event),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
};

static const struct snd_soc_dapm_route h3900_audio_map[] = {
	{"Headphone Jack", NULL, "VOUTLHP"},
	{"Headphone Jack", NULL, "VOUTRHP"},

	{"Speaker", NULL, "VOUTL"},
	{"Speaker", NULL, "VOUTR"},

	{"VINM", NULL, "Internal Mic"},
};

/*
 * The components are matched by devicetree phandle, filled in by
 * h3900_probe(). The I2S block is a combined CPU and platform component,
 * so one phandle serves both roles. The codec DAI is the combined
 * full-duplex one, which is what the single I2S interface uses.
 */
SND_SOC_DAILINK_DEFS(uda1380,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "uda1380-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * Exactly one of the headphones and the speaker is enabled at a time, so
 * h3900_spk_event() switches the amplifier off on insertion.
 */
static struct snd_soc_jack h3900_hp_jack;

static struct snd_soc_jack_pin h3900_hp_jack_pins[] = {
	{ .pin = "Headphone Jack", .mask = SND_JACK_HEADPHONE },
	{ .pin = "Speaker", .mask = SND_JACK_HEADPHONE, .invert = 1 },
};

/* The devicetree declares the line GPIO_ACTIVE_LOW, so no .invert. */
static struct snd_soc_jack_gpio h3900_hp_jack_gpio = {
	.name = "jack",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
};

/*
 * The jack is created from the dai_link init callback because the sound
 * card is not allocated yet when the probe function returns.
 */
static int h3900_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	int ret;

	if (!of_property_present(card->dev->of_node, "jack-gpios"))
		return 0;

	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack",
					 SND_JACK_HEADPHONE, &h3900_hp_jack,
					 h3900_hp_jack_pins,
					 ARRAY_SIZE(h3900_hp_jack_pins));
	if (ret)
		return ret;

	return snd_soc_jack_add_gpiods(card->dev, &h3900_hp_jack, 1,
				       &h3900_hp_jack_gpio);
}

/*
 * The format is given from the codec's point of view and is flipped
 * before it reaches the CPU DAI, so the PXA ends up the I2S master. The
 * UDA1380 driver rejects anything but codec clock consumer.
 */
static struct snd_soc_dai_link h3900_dai = {
	.name = "uda1380",
	.stream_name = "UDA1380",
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBC_CFC,
	.init = h3900_dai_init,
	SND_SOC_DAILINK_REG(uda1380),
};

static struct snd_soc_card snd_soc_h3900 = {
	.name = "iPAQ h3900",
	.owner = THIS_MODULE,
	.dai_link = &h3900_dai,
	.num_links = 1,

	.dapm_widgets = h3900_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(h3900_dapm_widgets),
	.dapm_routes = h3900_audio_map,
	.num_dapm_routes = ARRAY_SIZE(h3900_audio_map),
	.fully_routed = true,
};

static const struct of_device_id h3900_audio_of_match[] = {
	{ .compatible = "hp,ipaq-h3900-audio", },
	{ }
};
MODULE_DEVICE_TABLE(of, h3900_audio_of_match);

static int h3900_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_h3900;
	struct device *dev = &pdev->dev;
	int ret;

	h3900_dai.cpus[0].of_node = of_parse_phandle(dev->of_node, "cpu-dai", 0);
	if (!h3900_dai.cpus[0].of_node)
		return dev_err_probe(dev, -EINVAL, "missing cpu-dai phandle\n");

	h3900_dai.platforms[0].of_node = h3900_dai.cpus[0].of_node;

	h3900_dai.codecs[0].of_node = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!h3900_dai.codecs[0].of_node) {
		of_node_put(h3900_dai.cpus[0].of_node);
		return dev_err_probe(dev, -EINVAL, "missing audio-codec phandle\n");
	}

	gpiod_speaker = devm_gpiod_get(dev, "speaker", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod_speaker)) {
		ret = PTR_ERR(gpiod_speaker);
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
	of_node_put(h3900_dai.codecs[0].of_node);
	of_node_put(h3900_dai.cpus[0].of_node);
	return ret;
}

/*
 * The card may match components against these nodes for as long as it
 * exists, so they are only released when the driver unbinds.
 */
static void h3900_remove(struct platform_device *pdev)
{
	of_node_put(h3900_dai.codecs[0].of_node);
	of_node_put(h3900_dai.cpus[0].of_node);
}

static struct platform_driver h3900_driver = {
	.driver		= {
		.name		= "h3900-audio",
		.pm		= &snd_soc_pm_ops,
		.of_match_table	= h3900_audio_of_match,
	},
	.probe		= h3900_probe,
	.remove		= h3900_remove,
};

module_platform_driver(h3900_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("ALSA SoC HP iPAQ h3900");
MODULE_LICENSE("GPL");
