// SPDX-License-Identifier: GPL-2.0-only
/*
 * uda1341.c -- NXP/Philips UDA1341TS ASoC codec driver
 *
 * The UDA1341TS is controlled entirely over L3, a 3-wire bit-banged bus
 * (DATA/MODE/CLOCK) with no register readback of any kind, so the only way
 * to know the state of the chip is to track what was last written to it.
 * This driver embeds the L3 bit-bang directly. The shared L3 core
 * (sound/soc/codecs/l3.c and drivers/l3/) was deleted from mainline in
 * 6dd11b945951 ("ASoC: Delete UDA134x/L3 audio codec") for lack of a
 * remaining user, and its legacy <linux/gpio.h> and board-callback shape
 * would need a full gpiod rewrite to come back anyway. With exactly one L3
 * device on the platform, a shared core with a single consumer is not
 * worth re-adding.
 *
 * The register model (the L3 address/subaddress encoding, the two-phase
 * extended-address write and the STATUS0 SC/format fields) is that of the
 * UDA134x family.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

/* L3 device address 5, subaddress selects which register file */
#define UDA1341_L3ADDR		5
#define UDA1341_DATA0_ADDR	((UDA1341_L3ADDR << 2) | 0)
#define UDA1341_DATA1_ADDR	((UDA1341_L3ADDR << 2) | 1)
#define UDA1341_STATUS_ADDR	((UDA1341_L3ADDR << 2) | 2)

#define UDA1341_EXTADDR_PREFIX	0xc0
#define UDA1341_EXTDATA_PREFIX	0xe0

/* Extended-address register file (each needs two L3 transfers to reach) */
#define UDA1341_EA000		0
#define UDA1341_EA001		1
#define UDA1341_EA010		2
#define UDA1341_EA011		3
#define UDA1341_EA100		4
#define UDA1341_EA101		5
#define UDA1341_EA110		6
#define UDA1341_EA111		7
/* Directly-addressed registers (one L3 transfer each) */
#define UDA1341_STATUS0		8
#define UDA1341_STATUS1		9
#define UDA1341_DATA000		10
#define UDA1341_DATA001		11
#define UDA1341_DATA010		12
#define UDA1341_DATA011		13
#define UDA1341_DATA1		14

#define UDA1341_STATUS0_RST		BIT(6)		/* software reset, held then released */
#define UDA1341_STATUS0_SC_MASK		GENMASK(5, 4)	/* 00=512fs 01=384fs 10=256fs */
#define UDA1341_STATUS0_DAIFMT_MASK	GENMASK(3, 1)
#define UDA1341_DATA010_MUTE		BIT(2)

struct uda1341 {
	struct regmap *regmap;
	struct gpio_desc *data;
	struct gpio_desc *mode;
	struct gpio_desc *clock;

	unsigned int sysclk;
	unsigned int dai_fmt;

	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;
};

/*
 * All six per-edge L3 timings the UDA1341 datasheet specifies (hold,
 * setup, clock-high, mode-hold, mode, mode-setup: 190/30/190/190/250/
 * 250 ns) are comfortably under a microsecond; a flat 1 us per edge,
 * as the historical SA-1100 L3 bit-banger used, safely covers all of
 * them. It does NOT cover the codec's separate internal latch delay of
 * roughly 64/fs (~35 us at typical audio rates) needed between the end
 * of one transfer and the start of the next -- that is not one of
 * those six timed edges and is met explicitly below (UDA1341_L3_GAP_US)
 * rather than folded into the per-bit delay.
 */
#define UDA1341_L3_EDGE_US	1
#define UDA1341_L3_GAP_US	40

static void uda1341_l3_sendbyte(struct uda1341 *uda1341, u8 byte)
{
	int i;

	for (i = 0; i < 8; i++) {
		gpiod_set_value_cansleep(uda1341->clock, 0);
		udelay(UDA1341_L3_EDGE_US);
		gpiod_set_value_cansleep(uda1341->data, byte & 1);
		udelay(UDA1341_L3_EDGE_US);
		gpiod_set_value_cansleep(uda1341->clock, 1);
		udelay(UDA1341_L3_EDGE_US);
		byte >>= 1;
	}
}

/*
 * MODE is pulsed low-then-high between successive data bytes to mark
 * byte boundaries, but never at the start or the end of the transfer. Every
 * register write on this codec sends exactly one data byte after the
 * address byte; the loop exists only to keep the framing faithful to the
 * general L3 protocol.
 */
static void uda1341_l3_sendbytes(struct uda1341 *uda1341, const u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (i) {
			udelay(UDA1341_L3_EDGE_US);
			gpiod_set_value_cansleep(uda1341->mode, 0);
			udelay(UDA1341_L3_EDGE_US);
		}
		gpiod_set_value_cansleep(uda1341->mode, 1);
		udelay(UDA1341_L3_EDGE_US);
		uda1341_l3_sendbyte(uda1341, buf[i]);
	}
}

/* One full L3 transfer: an address byte with MODE low, then data bytes
 * with MODE high.
 */
static int uda1341_l3_write(struct uda1341 *uda1341, u8 addr, const u8 *data, int len)
{
	gpiod_set_value_cansleep(uda1341->clock, 1);
	gpiod_set_value_cansleep(uda1341->data, 1);
	gpiod_set_value_cansleep(uda1341->mode, 1);
	udelay(UDA1341_L3_EDGE_US);

	gpiod_set_value_cansleep(uda1341->mode, 0);
	udelay(UDA1341_L3_EDGE_US);
	uda1341_l3_sendbyte(uda1341, addr);
	udelay(UDA1341_L3_EDGE_US);

	uda1341_l3_sendbytes(uda1341, data, len);

	gpiod_set_value_cansleep(uda1341->clock, 1);
	gpiod_set_value_cansleep(uda1341->data, 1);
	gpiod_set_value_cansleep(uda1341->mode, 0);

	udelay(UDA1341_L3_GAP_US);

	return len;
}

static int uda1341_reg_write(void *context, unsigned int reg, unsigned int value)
{
	struct uda1341 *uda1341 = context;
	u8 addr, data = value, extaddr;

	switch (reg) {
	case UDA1341_STATUS0:
	case UDA1341_STATUS1:
		addr = UDA1341_STATUS_ADDR;
		data |= (reg - UDA1341_STATUS0) << 7;
		break;
	case UDA1341_DATA000:
	case UDA1341_DATA001:
	case UDA1341_DATA010:
	case UDA1341_DATA011:
		addr = UDA1341_DATA0_ADDR;
		data |= (reg - UDA1341_DATA000) << 6;
		break;
	case UDA1341_DATA1:
		addr = UDA1341_DATA1_ADDR;
		break;
	default:
		/* EA000..EA111: latch the extended address, then its data,
		 * as two separate L3 transfers over the DATA0 address.
		 */
		extaddr = reg | UDA1341_EXTADDR_PREFIX;
		uda1341_l3_write(uda1341, UDA1341_DATA0_ADDR, &extaddr, 1);

		addr = UDA1341_DATA0_ADDR;
		data = value | UDA1341_EXTDATA_PREFIX;
		break;
	}

	return uda1341_l3_write(uda1341, addr, &data, 1) == 1 ? 0 : -EIO;
}

static const struct reg_default uda1341_reg_defaults[] = {
	{ UDA1341_EA000,   0x04 },
	{ UDA1341_EA001,   0x04 },
	{ UDA1341_EA010,   0x04 },
	{ UDA1341_EA011,   0x00 },
	{ UDA1341_EA100,   0x00 },
	{ UDA1341_EA101,   0x00 },
	{ UDA1341_EA110,   0x00 },
	{ UDA1341_EA111,   0x00 },
	{ UDA1341_STATUS0, 0x00 },
	{ UDA1341_STATUS1, 0x03 },
	{ UDA1341_DATA000, 0x00 },
	{ UDA1341_DATA001, 0x00 },
	{ UDA1341_DATA010, 0x00 },
	{ UDA1341_DATA011, 0x00 },
	{ UDA1341_DATA1,   0x00 },
};

static const struct regmap_config uda1341_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = UDA1341_DATA1,
	.reg_defaults = uda1341_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(uda1341_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
	.reg_write = uda1341_reg_write,
};

static const char * const uda1341_dsp_setting[] = {
	"Flat", "Minimum1", "Minimum2", "Maximum",
};

static const char * const uda1341_deemph[] = {
	"None", "32Khz", "44.1Khz", "48Khz",
};

static const char * const uda1341_mixmode[] = {
	"Differential", "Analog1", "Analog2", "Both",
};

static const struct soc_enum uda1341_mixer_enum[] = {
	SOC_ENUM_SINGLE(UDA1341_DATA010, 0, 4, uda1341_dsp_setting),
	SOC_ENUM_SINGLE(UDA1341_DATA010, 3, 4, uda1341_deemph),
	SOC_ENUM_SINGLE(UDA1341_EA010, 0, 4, uda1341_mixmode),
};

static const struct snd_kcontrol_new uda1341_snd_controls[] = {
	SOC_SINGLE("Master Playback Volume", UDA1341_DATA000, 0, 0x3f, 1),
	SOC_SINGLE("Analog1 Volume", UDA1341_EA000, 0, 0x1f, 1),
	SOC_SINGLE("Analog2 Volume", UDA1341_EA001, 0, 0x1f, 1),

	SOC_SINGLE("Mic Sensitivity", UDA1341_EA010, 2, 7, 0),
	SOC_SINGLE("Mic Volume", UDA1341_EA101, 0, 0x1f, 0),

	SOC_SINGLE("Tone Control - Bass", UDA1341_DATA001, 2, 0xf, 0),
	SOC_SINGLE("Tone Control - Treble", UDA1341_DATA001, 0, 3, 0),

	SOC_ENUM("Sound Processing Filter", uda1341_mixer_enum[0]),
	SOC_ENUM("PCM Playback De-emphasis", uda1341_mixer_enum[1]),
	SOC_ENUM("Input Mux", uda1341_mixer_enum[2]),

	SOC_SINGLE("AGC Switch", UDA1341_EA100, 4, 1, 0),
	SOC_SINGLE("AGC Target Volume", UDA1341_EA110, 0, 3, 1),
	SOC_SINGLE("AGC Timing", UDA1341_EA110, 2, 7, 0),

	SOC_SINGLE("DAC +6dB Switch", UDA1341_STATUS1, 6, 1, 0),
	SOC_SINGLE("ADC +6dB Switch", UDA1341_STATUS1, 5, 1, 0),
	SOC_SINGLE("ADC Polarity Switch", UDA1341_STATUS1, 4, 1, 0),
	SOC_SINGLE("DAC Polarity Switch", UDA1341_STATUS1, 3, 1, 0),
	SOC_SINGLE("Double Speed Playback Switch", UDA1341_STATUS1, 2, 1, 0),
	SOC_SINGLE("DC Filter Enable Switch", UDA1341_STATUS0, 0, 1, 0),
};

/* UDA1341 keeps DAC/ADC power-down in STATUS1, unlike UDA1340/4/5's DATA011 */
static const struct snd_soc_dapm_widget uda1341_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", UDA1341_STATUS1, 0, 0),
	SND_SOC_DAPM_ADC("ADC", "Capture", UDA1341_STATUS1, 1, 0),

	SND_SOC_DAPM_INPUT("VINL1"),
	SND_SOC_DAPM_INPUT("VINR1"),
	SND_SOC_DAPM_INPUT("VINL2"),
	SND_SOC_DAPM_INPUT("VINR2"),
	SND_SOC_DAPM_OUTPUT("VOUTL"),
	SND_SOC_DAPM_OUTPUT("VOUTR"),
};

static const struct snd_soc_dapm_route uda1341_dapm_routes[] = {
	{ "ADC", NULL, "VINL1" },
	{ "ADC", NULL, "VINR1" },
	{ "ADC", NULL, "VINL2" },
	{ "ADC", NULL, "VINR2" },
	{ "VOUTL", NULL, "DAC" },
	{ "VOUTR", NULL, "DAC" },
};

static int uda1341_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct uda1341 *uda1341 = snd_soc_component_get_drvdata(dai->component);
	struct snd_pcm_runtime *master_runtime;

	/*
	 * DAC and ADC share the one SYSCLK/L3 clock domain (the 32*fs
	 * bit-clock invariant), so a stream
	 * already running pins the rate and sample width for the other
	 * direction.
	 */
	if (uda1341->master_substream) {
		master_runtime = uda1341->master_substream->runtime;

		snd_pcm_hw_constraint_single(substream->runtime,
					     SNDRV_PCM_HW_PARAM_RATE,
					     master_runtime->rate);
		snd_pcm_hw_constraint_single(substream->runtime,
					     SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
					     master_runtime->sample_bits);

		uda1341->slave_substream = substream;
	} else {
		uda1341->master_substream = substream;
	}

	return 0;
}

static void uda1341_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct uda1341 *uda1341 = snd_soc_component_get_drvdata(dai->component);

	if (uda1341->master_substream == substream)
		uda1341->master_substream = uda1341->slave_substream;

	uda1341->slave_substream = NULL;
}

static int uda1341_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct uda1341 *uda1341 = snd_soc_component_get_drvdata(dai->component);
	unsigned int val = 0;

	if (substream == uda1341->slave_substream)
		return 0;

	if (!uda1341->sysclk)
		return -EINVAL;

	/* STATUS0 SC field: 256/384/512 fs */
	switch (uda1341->sysclk / params_rate(params)) {
	case 512:
		break;
	case 384:
		val |= BIT(4);
		break;
	case 256:
		val |= BIT(5);
		break;
	default:
		dev_err(dai->dev, "unsupported sysclk/rate ratio\n");
		return -EINVAL;
	}

	switch (uda1341->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		switch (params_width(params)) {
		case 16:
			val |= BIT(1);
			break;
		case 18:
			val |= BIT(2);
			break;
		case 20:
			val |= BIT(2) | BIT(1);
			break;
		default:
			dev_err(dai->dev, "unsupported width for right-justified format\n");
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val |= BIT(3);
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		/*
		 * Not a distinct STATUS0 format of its own: the IF field of the
		 * UDA1341 has no DSP/TI-frame encoding at all. What ASoC calls
		 * DSP_A/DSP_B here is the electrical shape of the fixed "TI
		 * Synchronous Serial" frame format of the SA-1110 SSP
		 * (SSCR0_FRF_TI in sound/soc/sa11x0/sa11x0-ssp.c), which this
		 * board wires the codec to whether or not the register set of
		 * the codec has a matching name for it. On this fixed wiring
		 * DSP_A/DSP_B is bit-for-bit the LSB-justified 16-bit format
		 * that SND_SOC_DAIFMT_RIGHT_J already programs at 16-bit width
		 * above. DSP_A and DSP_B are indistinguishable at this
		 * hardware frame (the set_fmt() of sa11x0-ssp.c treats them
		 * identically for the same reason), so both map to the same
		 * bit.
		 */
		if (params_width(params) != 16) {
			dev_err(dai->dev, "DSP_A/DSP_B is only wired at 16-bit width\n");
			return -EINVAL;
		}
		val |= BIT(1);
		break;
	default:
		dev_err(dai->dev, "unsupported DAI format\n");
		return -EINVAL;
	}

	return regmap_update_bits(uda1341->regmap, UDA1341_STATUS0,
				   UDA1341_STATUS0_SC_MASK | UDA1341_STATUS0_DAIFMT_MASK, val);
}

static int uda1341_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				  unsigned int freq, int dir)
{
	struct uda1341 *uda1341 = snd_soc_component_get_drvdata(dai->component);

	/* 256fs*8kHz .. 512fs*48kHz: the codec's slave-mode operating range */
	if (freq < (256 * 8000) || freq > (512 * 48000)) {
		dev_err(dai->dev, "unsupported sysclk %u Hz\n", freq);
		return -EINVAL;
	}

	uda1341->sysclk = freq;

	return 0;
}

static int uda1341_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct uda1341 *uda1341 = snd_soc_component_get_drvdata(dai->component);

	/*
	 * BCK and WS are inputs of the UDA1341TS: the part consumes the bit
	 * clock and the frame and never drives either.
	 */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC) {
		dev_err(dai->dev, "unsupported clocking mode\n");
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
		dev_err(dai->dev, "unsupported clock inversion\n");
		return -EINVAL;
	}

	/* Word length is not known yet, so the STATUS0 write waits for hw_params() */
	uda1341->dai_fmt = fmt;

	return 0;
}

static int uda1341_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct uda1341 *uda1341 = snd_soc_component_get_drvdata(dai->component);

	return regmap_update_bits(uda1341->regmap, UDA1341_DATA010,
				   UDA1341_DATA010_MUTE, mute ? UDA1341_DATA010_MUTE : 0);
}

static const struct snd_soc_dai_ops uda1341_dai_ops = {
	.startup	 = uda1341_startup,
	.shutdown	 = uda1341_shutdown,
	.hw_params	 = uda1341_hw_params,
	.mute_stream	 = uda1341_mute_stream,
	.set_sysclk	 = uda1341_set_dai_sysclk,
	.set_fmt	 = uda1341_set_dai_fmt,
	.no_capture_mute = 1,
};

#define UDA1341_RATES SNDRV_PCM_RATE_8000_48000
#define UDA1341_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
			  SNDRV_PCM_FMTBIT_S20_3LE)

static struct snd_soc_dai_driver uda1341_dai = {
	.name = "uda1341-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = UDA1341_RATES,
		.formats = UDA1341_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = UDA1341_RATES,
		.formats = UDA1341_FORMATS,
	},
	.ops = &uda1341_dai_ops,
};

/*
 * There is no hardware reset or mute line on the h3800, but the codec has
 * a software reset of its own in STATUS0 bit 6: setting it, holding it for
 * 1 ms and clearing it returns the whole part to its power-on state. That
 * pulse runs first and is then followed, not replaced, by replaying every
 * register's value over L3, which is what regcache_sync() does against this
 * regmap and what actually loads the register set that this driver
 * believes the part is holding.
 *
 * This codec node carries no power-supply GPIO of its own, only the L3
 * DATA/MODE/CLOCK lines: AUD_PWR_ON is ASIC1 GPIO 14, owned by the machine
 * driver. Whatever brings the bias level of this component up out of
 * SND_SOC_BIAS_OFF for the first time MUST NOT do so until AUD_PWR_ON has
 * been raised and mdelay(1) has elapsed, because writing the register set
 * into a part that has not finished powering up silently discards it.
 */
static int uda1341_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	struct uda1341 *uda1341 = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(component);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_dapm_get_bias_level(dapm) == SND_SOC_BIAS_OFF) {
			regmap_update_bits(uda1341->regmap, UDA1341_STATUS0,
					   UDA1341_STATUS0_RST, UDA1341_STATUS0_RST);
			fsleep(1000);
			regmap_update_bits(uda1341->regmap, UDA1341_STATUS0,
					   UDA1341_STATUS0_RST, 0);
			regcache_sync(uda1341->regmap);
		}
		break;
	case SND_SOC_BIAS_OFF:
		regcache_mark_dirty(uda1341->regmap);
		break;
	}

	return 0;
}

static const struct snd_soc_component_driver soc_component_dev_uda1341 = {
	.set_bias_level		= uda1341_set_bias_level,
	.controls		= uda1341_snd_controls,
	.num_controls		= ARRAY_SIZE(uda1341_snd_controls),
	.dapm_widgets		= uda1341_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(uda1341_dapm_widgets),
	.dapm_routes		= uda1341_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(uda1341_dapm_routes),
	/*
	 * No .idle_bias_on: that would move this component OFF->STANDBY
	 * (running the STATUS0 reset pulse and the regcache_sync() replay
	 * above) as soon as the card registers, before any stream has called
	 * hw_params(), that is before the machine driver has raised
	 * AUD_PWR_ON even once. The reset pulse and the whole 15-register
	 * replay would go out over L3 to an unpowered part and be silently
	 * discarded. Leaving idle bias at OFF defers the first
	 * OFF->STANDBY transition to the normal DAPM stream power-up path,
	 * which runs after ipaq_h3800_hw_params() has raised AUD_PWR_ON.
	 */
	.suspend_bias_off	= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct of_device_id uda1341_of_match[] = {
	{ .compatible = "nxp,uda1341ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, uda1341_of_match);

static int uda1341_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uda1341 *uda1341;

	uda1341 = devm_kzalloc(dev, sizeof(*uda1341), GFP_KERNEL);
	if (!uda1341)
		return -ENOMEM;

	platform_set_drvdata(pdev, uda1341);

	/*
	 * GPIOD_OUT_HIGH drives the requested level before the SA-1100
	 * gpiochip switches the pin to output (sa1100_direction_output()
	 * programs GPSR/GPCR ahead of GPDR), which is the L3 init order this
	 * bus needs: lines high, then outputs.
	 *
	 * Nothing here touches GAFR, because gpio-sa1100.c exposes no
	 * alternate-function control. These three pins carry no competing
	 * alternate function, unlike GPIO19 (SSCR1_ECS), which is set by the
	 * SSP driver.
	 */
	uda1341->data = devm_gpiod_get(dev, "l3-data", GPIOD_OUT_HIGH);
	if (IS_ERR(uda1341->data))
		return dev_err_probe(dev, PTR_ERR(uda1341->data), "failed to get l3-data-gpios\n");

	uda1341->mode = devm_gpiod_get(dev, "l3-mode", GPIOD_OUT_HIGH);
	if (IS_ERR(uda1341->mode))
		return dev_err_probe(dev, PTR_ERR(uda1341->mode), "failed to get l3-mode-gpios\n");

	uda1341->clock = devm_gpiod_get(dev, "l3-clock", GPIOD_OUT_HIGH);
	if (IS_ERR(uda1341->clock))
		return dev_err_probe(dev, PTR_ERR(uda1341->clock),
				      "failed to get l3-clock-gpios\n");

	uda1341->regmap = devm_regmap_init(dev, NULL, uda1341, &uda1341_regmap_config);
	if (IS_ERR(uda1341->regmap))
		return PTR_ERR(uda1341->regmap);

	/*
	 * regcache_sync(), called from the STANDBY case of
	 * uda1341_set_bias_level(), is what loads the register set of this
	 * codec; the STATUS0 reset pulse before it only returns the part to its
	 * power-on defaults. It returns immediately unless cache_dirty is set,
	 * and nothing sets that on a freshly allocated regmap. Without this, the first bias-level
	 * transition on a cold boot replays nothing over L3 and the codec
	 * never receives its register set at all.
	 */
	regcache_mark_dirty(uda1341->regmap);

	return devm_snd_soc_register_component(dev, &soc_component_dev_uda1341, &uda1341_dai, 1);
}

static struct platform_driver uda1341_codec_driver = {
	.driver = {
		.name = "uda1341-codec",
		.of_match_table = uda1341_of_match,
	},
	.probe = uda1341_probe,
};
module_platform_driver(uda1341_codec_driver);

MODULE_DESCRIPTION("NXP/Philips UDA1341TS ASoC codec driver");
MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_LICENSE("GPL");
