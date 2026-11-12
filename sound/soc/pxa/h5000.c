// SPDX-License-Identifier: GPL-2.0-only
/*
 * h5000.c -- SoC audio for the HP iPAQ h5xxx (h51xx/h54xx/h55xx)
 *
 * The codec is an Asahi Kasei AK4535 on the PXA25x I2C bus at address
 * 0x10, fed I2S with the PXA as bit and frame clock master. The machine
 * driver follows h2200.c.
 *
 * Board GPIOs this driver owns, taken from properties of its devicetree
 * node:
 *
 *  - "codec-power": supply of the AK4535 itself, a SAMCOP GPB output. The
 *    codec driver has no supply handling and the factory firmware powers
 *    the codec before the first access to its registers, so the line is
 *    asserted in probe, before the card is registered. The codec's
 *    idle_bias_on makes card registration itself write a codec register,
 *    so there is no later point to hook the power-up into.
 *
 *  - "audio-power": a second, independent switch for the output stage and
 *    microphone bias downstream of the codec. It is a DAPM supply that
 *    every output and input widget routes through, so it is on while any
 *    of them is.
 *
 *  - "headphone-detect": jack detect on a PXA GPIO. snd_soc_jack_add_gpiods()
 *    owns the interrupt and debouncing, and the two jack pins disable the
 *    speaker while a plug is in. The polarity is not confirmed.
 *
 * MOUT2 (a codec output) is wired back to MIN (a codec input) and MICOUT
 * (output) to AIN (input) on the board, which the audio map reproduces
 * from the vendor driver. The codec's own "Mic Select" control already
 * chooses between the internal and the external microphone.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/soc.h>

static struct gpio_desc *gpiod_codec_power, *gpiod_audio_power;

static int h5000_audio_power_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *k, int event)
{
	gpiod_set_value_cansleep(gpiod_audio_power, SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget h5000_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_SUPPLY("Audio Power", SND_SOC_NOPM, 0, 0,
			    h5000_audio_power_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route h5000_audio_map[] = {
	{"Speaker", NULL, "SPP"},
	{"Speaker", NULL, "SPN"},

	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},

	{"MICIN", NULL, "Internal Mic"},
	{"MICEXT", NULL, "Mic Jack"},

	/* Board traces between codec pins, not routing inside the codec. */
	{"MIN", NULL, "MOUT2"},
	{"AIN", NULL, "MICOUT"},

	{"Speaker", NULL, "Audio Power"},
	{"Headphone Jack", NULL, "Audio Power"},
	{"Internal Mic", NULL, "Audio Power"},
	{"Mic Jack", NULL, "Audio Power"},
};

/*
 * The components are matched by devicetree phandle, filled in by
 * h5000_probe(). The I2S block is a combined CPU and platform component,
 * so one phandle serves both roles.
 */
SND_SOC_DAILINK_DEFS(ak4535,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "ak4535-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * Two pins toggled in lockstep with opposite polarity, so exactly one of
 * the headphones and the speaker is enabled at a time.
 */
static struct snd_soc_jack h5000_hp_jack;

static struct snd_soc_jack_pin h5000_hp_jack_pins[] = {
	{ .pin = "Headphone Jack", .mask = SND_JACK_HEADPHONE },
	{ .pin = "Speaker", .mask = SND_JACK_HEADPHONE, .invert = 1 },
};

static struct snd_soc_jack_gpio h5000_hp_jack_gpio = {
	.name = "headphone-detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
};

/*
 * The jack is created from the dai_link init callback because the sound
 * card is not allocated yet when the probe function returns.
 */
static int h5000_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	int ret;

	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack", SND_JACK_HEADPHONE,
					 &h5000_hp_jack, h5000_hp_jack_pins,
					 ARRAY_SIZE(h5000_hp_jack_pins));
	if (ret)
		return ret;

	return snd_soc_jack_add_gpiods(card->dev, &h5000_hp_jack, 1,
				       &h5000_hp_jack_gpio);
}

/*
 * The AK4535's master clock is the PXA's I2S SYSCLK pin, and
 * pxa2xx_i2s_hw_params() programs the divider so that it is 256 * fs at
 * every rate it supports. The codec divides the clock it is told by the
 * sample rate to pick its oversampling mode, so it has to be told that
 * clock. The two-entry table of 11.2896 and 12.288 MHz in spitz.c would
 * make it come out 512 at 22.05 kHz and 1024 at 11.025 kHz.
 */
static int h5000_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);

	return snd_soc_dai_set_sysclk(codec_dai, 0, 256 * params_rate(params),
				      SND_SOC_CLOCK_IN);
}

static const struct snd_soc_ops h5000_ops = {
	.hw_params = h5000_hw_params,
};

/*
 * The format is given from the codec's point of view and is flipped
 * before it reaches the CPU DAI, so the PXA ends up the I2S master.
 */
static struct snd_soc_dai_link h5000_dai = {
	.name = "ak4535",
	.stream_name = "AK4535",
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBC_CFC,
	.ops = &h5000_ops,
	.init = h5000_dai_init,
	SND_SOC_DAILINK_REG(ak4535),
};

static struct snd_soc_card snd_soc_h5000 = {
	.name = "iPAQ h5xxx",
	.owner = THIS_MODULE,
	.dai_link = &h5000_dai,
	.num_links = 1,

	.dapm_widgets = h5000_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(h5000_dapm_widgets),
	.dapm_routes = h5000_audio_map,
	.num_dapm_routes = ARRAY_SIZE(h5000_audio_map),
	.fully_routed = true,
};

static const struct of_device_id h5000_audio_of_match[] = {
	{ .compatible = "hp,ipaq-h5000-audio", },
	{ }
};
MODULE_DEVICE_TABLE(of, h5000_audio_of_match);

static void h5000_unpower_codec(void *data)
{
	gpiod_set_value_cansleep(data, 0);
}

/*
 * The card may match components against these nodes for as long as it
 * exists, so they are released by devres only after the card has been
 * unregistered.
 */
static void h5000_put_nodes(void *data)
{
	of_node_put(h5000_dai.codecs[0].of_node);
	of_node_put(h5000_dai.cpus[0].of_node);
}

static int h5000_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_h5000;
	struct device *dev = &pdev->dev;
	int ret;

	h5000_dai.cpus[0].of_node = of_parse_phandle(dev->of_node, "cpu-dai", 0);
	if (!h5000_dai.cpus[0].of_node)
		return dev_err_probe(dev, -EINVAL, "missing cpu-dai phandle\n");

	h5000_dai.platforms[0].of_node = h5000_dai.cpus[0].of_node;

	h5000_dai.codecs[0].of_node = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!h5000_dai.codecs[0].of_node) {
		of_node_put(h5000_dai.cpus[0].of_node);
		return dev_err_probe(dev, -EINVAL, "missing audio-codec phandle\n");
	}

	ret = devm_add_action_or_reset(dev, h5000_put_nodes, NULL);
	if (ret)
		return ret;

	gpiod_audio_power = devm_gpiod_get(dev, "audio-power", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod_audio_power))
		return PTR_ERR(gpiod_audio_power);

	gpiod_codec_power = devm_gpiod_get(dev, "codec-power", GPIOD_OUT_HIGH);
	if (IS_ERR(gpiod_codec_power))
		return PTR_ERR(gpiod_codec_power);

	/*
	 * Registered before the card, so the devres teardown drops the
	 * supply after the card is unregistered: that runs the codec's bias
	 * shutdown, which is I2C traffic to a codec that must still be
	 * powered.
	 */
	ret = devm_add_action_or_reset(dev, h5000_unpower_codec,
				       gpiod_codec_power);
	if (ret)
		return ret;

	/* The power-up time of the AK4535 is not documented. */
	usleep_range(1000, 2000);

	card->dev = dev;

	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		return dev_err_probe(dev, ret, "snd_soc_register_card() failed\n");

	return 0;
}

static struct platform_driver h5000_driver = {
	.driver		= {
		.name		= "h5000-audio",
		.pm		= &snd_soc_pm_ops,
		.of_match_table	= h5000_audio_of_match,
	},
	.probe		= h5000_probe,
};

module_platform_driver(h5000_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("ALSA SoC HP iPAQ h5xxx");
MODULE_LICENSE("GPL");
