// SPDX-License-Identifier: GPL-2.0-only
/*
 * ipaq-h3800.c -- ASoC machine driver for the UDA1341TS audio path of the
 * HP iPAQ h3800 (h38xx) family.
 *
 * Ties sound/soc/sa11x0/sa11x0-ssp.c (the SSP CPU DAI) to
 * sound/soc/codecs/uda1341.c (the codec) and owns everything that neither
 * of those two, nor the audio-clock tap selector of drivers/mfd/asic2.c,
 * can correctly own by itself:
 *
 *  - The 32*fs bit-clock invariant. Reaching 256/384/512 fs needs three
 *    registers in lock-step: the ASIC2 clock tap (a fixed EX1/EX2-derived
 *    rate), the SerClkDiv of the SSP and the STATUS0 SC field of the
 *    codec. Without something enforcing it, it drifts silently rather than
 *    loudly. The hw_params() of this driver runs before the hw_params() of
 *    either DAI (__soc_pcm_hw_params() calls the machine ops first, then
 *    the codec DAI, then the cpu DAI), so setting the ASIC2 tap and the
 *    SYSCLK of both DAIs here is sufficient, with no explicit ordering
 *    between the two DAIs.
 *
 *  - AUD_PWR_ON sequencing around a tap switch: powered off, tap switched,
 *    powered back on, and a one millisecond settle before the first L3
 *    byte, because the reset and register replay of the codec must not
 *    run into a part that has not finished waking.
 *
 *  - ASIC1's speaker/headphone routing and jack detect, a counter-intuitive
 *    truth table: mute is EAR_ON_N set and SPK_ON clear; unmuted with a
 *    jack inserted is both cleared; unmuted with no jack is both set. It
 *    is modelled as plain "set/clear the bit" GPIO writes (both gpios are
 *    GPIO_ACTIVE_HIGH in the devicetree) to match that table literally,
 *    rather than through DAPM enable/disable semantics that would invert
 *    it.
 *
 * DAI format: the SSP drives the bit clock, which it divides from the
 * ASIC2 tap on GPIO19 (SSCR1_ECS), and the frame, and the codec consumes
 * both, so dai_link->dai_fmt is SND_SOC_DAIFMT_CBC_CFC from the codec's
 * side.
 *
 * DSP_A vs DSP_B: the fixed TI hardware frame maps onto either value of
 * ASoC's generic enum, and DSP_A is an untested pick.
 *
 * Capture is not wired: sa11x0_ssp_dai has no capture stream, so this card
 * is playback-only.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define IPAQ_H3800_JACK_DEBOUNCE_MS	2

struct ipaq_h3800_audio {
	struct device *dev;
	struct clk *audio_clk;

	/*
	 * The "sound-dai" phandle references of the "cpu" and "codec" child
	 * nodes, handed to ipaq_h3800_cpus[0]/ipaq_h3800_codecs[0] below and
	 * kept here so that one devm action can release both: those component
	 * arrays are file-scope and outlive any single bound instance.
	 */
	struct device_node *cpu_np;
	struct device_node *codec_np;

	/* ASIC1 SPK_ON/EAR_ON_N/AUD_PWR_ON, ASIC2 GPIO2 EAR_IN_N */
	struct gpio_desc *spk;
	struct gpio_desc *ear;
	struct gpio_desc *aud_pwr;
	struct gpio_desc *jack;
	int jack_irq;

	/*
	 * Jack-detect debounce, in work context rather than a timer:
	 * fix_jack() calls the gpiod_*_cansleep() variants that this driver
	 * uses for all ASIC1/ASIC2 GPIO access, which a timer callback
	 * (softirq context) must not do.
	 */
	struct delayed_work jack_work;

	/* Whether playback is considered muted, driven by .trigger(). */
	bool muted;

	/* The powered-up state as a single flag: true means both AUD_PWR_ON
	 * is raised and exactly one clk_prepare_enable() reference is held
	 * on audio_clk, at cur_tap_rate. Every path that sets it raises both
	 * and every path that clears it drops both, so .shutdown() below can
	 * decide whether a clk_disable_unprepare() is owed from this flag
	 * alone. Both 0/false before the first hw_params().
	 */
	bool powered;
	unsigned long cur_tap_rate;
};

/*
 * Rate -> ASIC2 tap rate: 8k/16k use the 4.096 MHz tap, 32k/48k the
 * 12.288 MHz tap, the 10985 Hz behind the usual "11025" label the 5.6245
 * MHz tap, and 22.05k/44.1k the 11.2896 MHz tap. This matches
 * sa11x0_ssp_rates[] in sa11x0-ssp.c exactly; the tap switch itself
 * happens in the clk_hw of drivers/mfd/asic2.c, and this table only has to
 * agree with it on which rates are valid and which tap rate each implies.
 */
static const struct {
	unsigned int rate;
	unsigned long tap_rate;
} ipaq_h3800_rate_taps[] = {
	{ 8000,   4096000UL },
	{ 16000,  4096000UL },
	{ 32000,  12288000UL },
	{ 48000,  12288000UL },
	{ 10985,  5624500UL },
	{ 22050,  11289600UL },
	{ 44100,  11289600UL },
};

static unsigned long ipaq_h3800_tap_rate(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipaq_h3800_rate_taps); i++)
		if (ipaq_h3800_rate_taps[i].rate == rate)
			return ipaq_h3800_rate_taps[i].tap_rate;

	return 0;
}

/*
 * The routing truth table: muted is EAR_ON_N=1/SPK_ON=0 regardless of the
 * jack; unmuted reads the jack line live and picks headphone (both
 * cleared) or speaker (both set). gpiod_get_value_cansleep() on "jack"
 * already returns the logical, polarity-corrected value (the devicetree
 * marks it GPIO_ACTIVE_LOW), so 1 here really means "jack present".
 */
static void ipaq_h3800_fix_jack(struct ipaq_h3800_audio *priv)
{
	bool jack_in = gpiod_get_value_cansleep(priv->jack) > 0;
	bool ear_on_n, spk_on;

	if (priv->muted) {
		ear_on_n = true;
		spk_on = false;
	} else if (jack_in) {
		ear_on_n = false;
		spk_on = false;
	} else {
		ear_on_n = true;
		spk_on = true;
	}

	gpiod_set_value_cansleep(priv->ear, ear_on_n);
	gpiod_set_value_cansleep(priv->spk, spk_on);

	dev_dbg(priv->dev, "jack %s, muted=%d -> EAR_ON_N=%d SPK_ON=%d\n",
		jack_in ? "in" : "out", priv->muted, ear_on_n, spk_on);
}

static void ipaq_h3800_jack_work(struct work_struct *work)
{
	struct ipaq_h3800_audio *priv =
		container_of(work, struct ipaq_h3800_audio, jack_work.work);

	ipaq_h3800_fix_jack(priv);
}

static irqreturn_t ipaq_h3800_jack_isr(int irq, void *data)
{
	struct ipaq_h3800_audio *priv = data;

	mod_delayed_work(system_wq, &priv->jack_work,
			 msecs_to_jiffies(IPAQ_H3800_JACK_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static int ipaq_h3800_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct ipaq_h3800_audio *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int rate = params_rate(params);
	unsigned long tap_rate = ipaq_h3800_tap_rate(rate);
	int ret;

	if (!tap_rate) {
		dev_err(priv->dev, "unsupported rate %u Hz\n", rate);
		return -EINVAL;
	}

	if (!priv->powered || tap_rate != priv->cur_tap_rate) {
		bool was_powered = priv->powered;

		/*
		 * AUD_PWR_ON is cleared first, the tap switched with power
		 * already off, AUD_PWR_ON set again, then mdelay(1) before the
		 * first L3 byte: the reset and register replay of the codec
		 * must not run into a still-waking part.
		 */
		gpiod_set_value_cansleep(priv->aud_pwr, 0);

		/*
		 * clk_prepare_enable() before clk_set_rate(), and only on the
		 * off->on edge, for two distinct reasons.
		 *
		 * That it happens at all: the audio clk_hw of
		 * drivers/mfd/asic2.c acquires an EX1/EX2 crystal reference as
		 * a side effect of .set_rate(), so a consumer that only calls
		 * clk_set_rate() gets a working clock but no reference of its
		 * own, and no way to ever hand the crystal back. .shutdown()
		 * below releases it, and can only do that against a
		 * clk_prepare_enable() taken here.
		 *
		 * That it comes first: with no tap selected yet, the .prepare()
		 * of that clk_hw acquires the crystal by routing through its
		 * own .set_rate(), so preparing first leaves exactly one
		 * crystal reference for the following clk_set_rate() to
		 * migrate across crystals. Preparing after clk_set_rate()
		 * would stack a second reference on the crystal of the first
		 * tap, and a later cross-crystal tap switch only ever migrates
		 * one, so the other would be stranded for the lifetime of the
		 * module. The .prepare() call may itself select a tap (AUDIO_1,
		 * the default state of that clk_hw), which is why it belongs
		 * inside this AUD_PWR_ON-off bracket alongside the
		 * clk_set_rate() that immediately corrects it.
		 */
		if (!was_powered) {
			ret = clk_prepare_enable(priv->audio_clk);
			if (ret)
				return ret;
		}

		ret = clk_set_rate(priv->audio_clk, tap_rate);
		if (ret) {
			/*
			 * AUD_PWR_ON is already off and the tap did not change,
			 * so drop all the way back to the powered-off state
			 * rather than guess: that releases the one reference
			 * held here (taken just above, or on an earlier call)
			 * exactly once, and makes the next hw_params() re-run
			 * this whole block even if it asks for the tap rate
			 * that is still selected.
			 */
			clk_disable_unprepare(priv->audio_clk);
			priv->powered = false;
			return ret;
		}

		gpiod_set_value_cansleep(priv->aud_pwr, 1);
		mdelay(1);

		priv->powered = true;
		priv->cur_tap_rate = tap_rate;
	}

	/*
	 * The machine hw_params() runs before either DAI's own, and the codec
	 * DAI's before the cpu DAI's, so both SYSCLK values only need to be
	 * in place before those run, and need no specific order here.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, tap_rate, SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	return snd_soc_dai_set_sysclk(cpu_dai, 0, tap_rate, SND_SOC_CLOCK_IN);
}

/*
 * Routing and mute are driven from stream start/stop rather than a DAPM
 * widget of their own: the mute of this hardware is two ASIC1 GPIOs that
 * the machine driver already owns for jack routing, not a codec-internal
 * DAPM path, so folding it into the fix_jack() that the jack interrupt
 * already calls keeps exactly one function owning the whole truth table.
 */
static int ipaq_h3800_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct ipaq_h3800_audio *priv = snd_soc_card_get_drvdata(rtd->card);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		priv->muted = false;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		priv->muted = true;
		break;
	default:
		return 0;
	}

	ipaq_h3800_fix_jack(priv);

	return 0;
}

/*
 * The exact inverse of the power-up block of hw_params(): AUD_PWR_ON goes
 * down first, then the clock, so the codec is never left powered with its
 * system clock already stopped. The card is playback-only with a single
 * substream, so priv->powered is the whole state machine and no refcount
 * across concurrent substreams is needed. It also guarantees that the
 * clk_disable_unprepare() here matches exactly one clk_prepare_enable() in
 * hw_params(): a stream opened and closed without ever reaching
 * hw_params() leaves priv->powered false and takes neither.
 */
static void ipaq_h3800_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct ipaq_h3800_audio *priv = snd_soc_card_get_drvdata(rtd->card);

	if (!priv->powered)
		return;

	gpiod_set_value_cansleep(priv->aud_pwr, 0);
	clk_disable_unprepare(priv->audio_clk);

	priv->powered = false;
}

static const struct snd_soc_ops ipaq_h3800_ops = {
	.shutdown	= ipaq_h3800_shutdown,
	.hw_params	= ipaq_h3800_hw_params,
	.trigger	= ipaq_h3800_trigger,
};

/*
 * Plain component arrays rather than SND_SOC_DAILINK_DEFS(): this DAI link
 * needs no separate "platforms" entry, because sa11x0-ssp.c registers one
 * component that is both the CPU DAI and, through
 * devm_snd_dmaengine_pcm_register(), the platform, so reusing
 * ipaq_h3800_cpus as .platforms resolves to that same component. of_node
 * is filled in at probe time from the "sound-dai" phandles of the "cpu"
 * and "codec" child nodes; dai_name is the one static fact each DAI driver
 * already fixes.
 */
static struct snd_soc_dai_link_component ipaq_h3800_cpus[] = {
	{ .dai_name = "sa11x0-ssp" },
};

static struct snd_soc_dai_link_component ipaq_h3800_codecs[] = {
	{ .dai_name = "uda1341-hifi" },
};

static struct snd_soc_dai_link ipaq_h3800_dai_link = {
	.name		= "UDA1341TS",
	.stream_name	= "UDA1341TS",
	.ops		= &ipaq_h3800_ops,
	.cpus		= ipaq_h3800_cpus,
	.num_cpus	= ARRAY_SIZE(ipaq_h3800_cpus),
	.codecs		= ipaq_h3800_codecs,
	.num_codecs	= ARRAY_SIZE(ipaq_h3800_codecs),
	.platforms	= ipaq_h3800_cpus,
	.num_platforms	= ARRAY_SIZE(ipaq_h3800_cpus),
	.dai_fmt	= SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
			  SND_SOC_DAIFMT_CBC_CFC,
	/*
	 * The component driver of the codec sets .use_pmdown_time, so without
	 * this the DAC's DAPM power-down write is queued and fires up to
	 * pmdown_time (default 5000 ms) after this stream closes, well after
	 * ipaq_h3800_shutdown() has dropped AUD_PWR_ON and the audio clock.
	 * That deferred write would go out over L3 to a part with no power
	 * and no SYSCLK, and it defeats the pop suppression pmdown_time is for.
	 * Forcing the power-down to be synchronous, ahead of hw_free() and
	 * .shutdown(), keeps the DAPM teardown inside the powered window.
	 */
	.ignore_pmdown_time = 1,
	/*
	 * .trigger() routes the outputs through ASIC1/ASIC2 GPIOs with the
	 * sleeping gpiod accessors, which atomic PCM trigger context forbids.
	 */
	.nonatomic = 1,
};

static struct snd_soc_card ipaq_h3800_card = {
	.name		= "ipaq-h3800-audio",
	/*
	 * The ALSA driver field is only 15 characters wide and ASoC falls
	 * back to .name for it, truncating "ipaq-h3800-audio" mid-word.
	 * Userspace uses this field as a lookup key, so pick a whole word.
	 */
	.driver_name	= "ipaq-h3800",
	.owner		= THIS_MODULE,
	.dai_link	= &ipaq_h3800_dai_link,
	.num_links	= 1,
};

/*
 * Suspend cancels the jack work and disables the interrupt, and touches no
 * ASIC register: the mute-on-suspend effect comes from AUD_PWR_ON having
 * been dropped by the generic suspend path of ASIC1, which this driver
 * depends on. Resume re-arms the edge type, forces the muted state through
 * fix_jack(), and only then enables the interrupt. Routing before muting
 * risks driving speaker output from a stale pre-suspend jack state for one
 * interrupt latency.
 */
static int ipaq_h3800_suspend_pre(struct snd_soc_card *card)
{
	struct ipaq_h3800_audio *priv = snd_soc_card_get_drvdata(card);

	disable_irq(priv->jack_irq);
	cancel_delayed_work_sync(&priv->jack_work);

	return 0;
}

static int ipaq_h3800_resume_pre(struct snd_soc_card *card)
{
	struct ipaq_h3800_audio *priv = snd_soc_card_get_drvdata(card);

	irq_set_irq_type(priv->jack_irq, IRQ_TYPE_EDGE_BOTH);

	priv->muted = true;
	ipaq_h3800_fix_jack(priv);

	enable_irq(priv->jack_irq);

	return 0;
}

static const struct of_device_id ipaq_h3800_of_match[] = {
	{ .compatible = "hp,ipaq-h3800-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, ipaq_h3800_of_match);

/*
 * One devm action for both phandle references rather than an of_node_put()
 * pair at every probe error return: registered as soon as both references
 * exist, it covers the rest of probe() and normal unbind alike, and clears
 * the file-scope component arrays so they never keep a pointer to a node
 * this instance no longer holds.
 */
static void ipaq_h3800_put_dai_nodes(void *data)
{
	struct ipaq_h3800_audio *priv = data;

	ipaq_h3800_cpus[0].of_node = NULL;
	ipaq_h3800_codecs[0].of_node = NULL;

	of_node_put(priv->cpu_np);
	of_node_put(priv->codec_np);
}

static struct device_node *ipaq_h3800_get_dai_node(struct device_node *np,
						   const char *child)
{
	struct device_node *dai_np, *cn;

	cn = of_get_child_by_name(np, child);
	if (!cn)
		return NULL;

	dai_np = of_parse_phandle(cn, "sound-dai", 0);
	of_node_put(cn);

	return dai_np;
}

static void ipaq_h3800_cancel_jack_work(void *data)
{
	struct ipaq_h3800_audio *priv = data;

	cancel_delayed_work_sync(&priv->jack_work);
}

static int ipaq_h3800_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *cpu_np, *codec_np;
	struct ipaq_h3800_audio *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	cpu_np = ipaq_h3800_get_dai_node(np, "cpu");
	if (!cpu_np)
		return dev_err_probe(dev, -ENODEV, "no cpu sound-dai\n");

	codec_np = ipaq_h3800_get_dai_node(np, "codec");
	if (!codec_np) {
		of_node_put(cpu_np);
		return dev_err_probe(dev, -ENODEV, "no codec sound-dai\n");
	}

	priv->cpu_np = cpu_np;
	priv->codec_np = codec_np;
	ipaq_h3800_cpus[0].of_node = cpu_np;
	ipaq_h3800_codecs[0].of_node = codec_np;

	ret = devm_add_action_or_reset(dev, ipaq_h3800_put_dai_nodes, priv);
	if (ret)
		return ret;

	priv->audio_clk = devm_clk_get(dev, "audio");
	if (IS_ERR(priv->audio_clk))
		return dev_err_probe(dev, PTR_ERR(priv->audio_clk),
				      "failed to get audio clock\n");

	priv->spk = devm_gpiod_get(dev, "spk", GPIOD_OUT_LOW);
	if (IS_ERR(priv->spk))
		return dev_err_probe(dev, PTR_ERR(priv->spk),
				      "failed to get spk-gpios\n");

	/*
	 * Boot-safe default before the first fix_jack() call below runs:
	 * EAR_ON_N=1 alongside the GPIOD_OUT_LOW default of SPK_ON above is
	 * the muted state of the truth table.
	 */
	priv->ear = devm_gpiod_get(dev, "ear", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->ear))
		return dev_err_probe(dev, PTR_ERR(priv->ear),
				      "failed to get ear-gpios\n");

	priv->aud_pwr = devm_gpiod_get(dev, "aud-pwr", GPIOD_OUT_LOW);
	if (IS_ERR(priv->aud_pwr))
		return dev_err_probe(dev, PTR_ERR(priv->aud_pwr),
				      "failed to get aud-pwr-gpios\n");

	priv->jack = devm_gpiod_get(dev, "jack", GPIOD_IN);
	if (IS_ERR(priv->jack))
		return dev_err_probe(dev, PTR_ERR(priv->jack),
				      "failed to get jack-gpios\n");

	priv->jack_irq = gpiod_to_irq(priv->jack);
	if (priv->jack_irq < 0)
		return dev_err_probe(dev, priv->jack_irq,
				      "failed to get jack IRQ\n");

	INIT_DELAYED_WORK(&priv->jack_work, ipaq_h3800_jack_work);
	ret = devm_add_action_or_reset(dev, ipaq_h3800_cancel_jack_work, priv);
	if (ret)
		return ret;

	/*
	 * Init order: arm the edge type, then fix_jack() with the boot-time
	 * state, then request_irq() last. Requesting first would let a jack
	 * already inserted at boot latch the wrong route until the next
	 * physical unplug. irq_set_irq_type() alone, with no IRQF_TRIGGER_*
	 * flags passed to devm_request_irq() below, arms the type without
	 * unmasking anything.
	 */
	irq_set_irq_type(priv->jack_irq, IRQ_TYPE_EDGE_BOTH);

	priv->muted = true;
	ipaq_h3800_fix_jack(priv);

	ret = devm_request_irq(dev, priv->jack_irq, ipaq_h3800_jack_isr, 0,
			       "ipaq-h3800-jack", priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request jack IRQ\n");

	ipaq_h3800_card.dev = dev;
	ipaq_h3800_card.suspend_pre = ipaq_h3800_suspend_pre;
	ipaq_h3800_card.resume_pre = ipaq_h3800_resume_pre;
	snd_soc_card_set_drvdata(&ipaq_h3800_card, priv);

	ret = devm_snd_soc_register_card(dev, &ipaq_h3800_card);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register sound card\n");

	return 0;
}

static struct platform_driver ipaq_h3800_audio_driver = {
	.driver = {
		.name = "ipaq-h3800-audio",
		.of_match_table = ipaq_h3800_of_match,
	},
	.probe = ipaq_h3800_audio_probe,
};
module_platform_driver(ipaq_h3800_audio_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("HP iPAQ h3800 UDA1341TS audio machine driver");
MODULE_LICENSE("GPL");
