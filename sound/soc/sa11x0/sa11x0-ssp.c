// SPDX-License-Identifier: GPL-2.0-only
/*
 * sa11x0-ssp.c -- ASoC CPU DAI driver for the SA-1110 Ser4 SSP
 *
 * Wired on the HP/Compaq iPAQ h38xx family to a UDA1341TS codec, with a
 * 16-bit TI-format frame. The SSP drives both the bit clock and the frame;
 * it divides the bit clock from a reference clock on GPIO19 (SSCR1_ECS).
 *
 * Capture is not implemented: the SSP is frame master, so RX never
 * advances unless a TX DMA stream is also running, and that dummy-transmit
 * path is left as a follow-up.
 *
 * Copyright (C) 2026 Koen Kooi
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* Ser4SSCR0/1, Ser4SSDR, Ser4SSSR (mach-sa1100/include/mach/SA-1100.h) */
#define SSP_SSCR0		0x00
#define SSP_SSCR1		0x04
#define SSP_SSDR		0x0c
#define SSP_SSSR		0x14

#define SSCR0_DSS_16		0x0f		/* Fld(4,0): DataSize(16) = (16-1) << 0 */
#define SSCR0_FRF_TI		BIT(4)		/* Fld(2,4) = 1: TI synchronous serial */
#define SSCR0_SCR_MASK		GENMASK(15, 8)	/* Fld(8,8) */
#define SSCR0_SERCLKDIV(div)	((((div) - 2) / 2) << 8)
#define SSCR0_SSE		BIT(7)

#define SSCR1_ECS		BIT(5)		/* External Clock (GPIO[19]) */

struct sa11x0_ssp {
	struct device *dev;
	void __iomem *base;
	phys_addr_t phys_base;
	unsigned int sysclk;
	struct snd_dmaengine_dai_dma_data dma_playback;
};

/*
 * The rates this hardware supports, from the four fixed audio-clock taps of
 * ASIC2. 10985 Hz is the real rate behind the usual "11025" label. It fits
 * no standard SNDRV_PCM_RATE_* bit, so SNDRV_PCM_RATE_KNOT plus this
 * constraint list is needed to express it.
 */
static const unsigned int sa11x0_ssp_rates[] = {
	8000, 16000, 32000, 48000, 10985, 22050, 44100,
};

static const struct snd_pcm_hw_constraint_list sa11x0_ssp_rate_list = {
	.count	= ARRAY_SIZE(sa11x0_ssp_rates),
	.list	= sa11x0_ssp_rates,
};

static int sa11x0_ssp_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &sa11x0_ssp_rate_list);
	if (ret < 0)
		return ret;

	return 0;
}

static void sa11x0_ssp_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct sa11x0_ssp *priv = dev_get_drvdata(dai->dev);

	writel_relaxed(0, priv->base + SSP_SSCR0);
}

static int sa11x0_ssp_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/*
	 * The SSP always drives the bit clock and the frame: SSCR1_ECS only
	 * selects the reference that the bit clock is divided from. DSP_A and
	 * DSP_B both map onto the same fixed SSCR0_FRF_TI hardware frame, so
	 * accept either.
	 */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BP_FP)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		return 0;
	default:
		return -EINVAL;
	}
}

static int sa11x0_ssp_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	struct sa11x0_ssp *priv = dev_get_drvdata(dai->dev);

	/*
	 * freq is the ASIC2 audio-clock tap rate feeding both the SYSCLK of
	 * the codec and the GPIO19 reference that this SSP divides its bit
	 * clock from, which are the same physical signal. It is always an input as far as this
	 * DAI is concerned.
	 */
	if (dir != SND_SOC_CLOCK_IN)
		return -EINVAL;

	priv->sysclk = freq;

	return 0;
}

static int sa11x0_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct sa11x0_ssp *priv = dev_get_drvdata(dai->dev);
	unsigned int rate = params_rate(params);
	u32 sscr0, div;

	if (params_format(params) != SNDRV_PCM_FORMAT_S16_LE || params_channels(params) != 2)
		return -EINVAL;

	if (!priv->sysclk) {
		dev_err(priv->dev,
			"no SYSCLK set, call set_sysclk() with the ASIC2 audio-clock tap rate first\n");
		return -EINVAL;
	}

	/*
	 * SerClkDiv(8)/(12)/(16) for 256/384/512 fs all yield the same 32 x fs
	 * bit clock. The machine driver must keep this call and the SC field
	 * of the codec in lock-step to preserve that.
	 */
	switch (priv->sysclk / rate) {
	case 256:
		div = SSCR0_SERCLKDIV(8);
		break;
	case 384:
		div = SSCR0_SERCLKDIV(12);
		break;
	case 512:
		div = SSCR0_SERCLKDIV(16);
		break;
	default:
		dev_err(priv->dev, "SYSCLK %u Hz is not 256/384/512x the %u Hz rate\n",
			priv->sysclk, rate);
		return -EINVAL;
	}

	sscr0 = readl_relaxed(priv->base + SSP_SSCR0);
	if (sscr0 & SSCR0_SSE) {
		/*
		 * Live rate change: SSE off, one frame period at the slowest
		 * supported rate, then the new divisor and SSE back on in a
		 * single write. Never combine the divisor store with clearing
		 * SSE.
		 *
		 * udelay(), not usleep_range(): this is a hardware-timed
		 * minimum before the divisor is safe to change, and 125us is
		 * short enough that a busy-wait is acceptable.
		 */
		writel_relaxed(sscr0 & ~SSCR0_SSE, priv->base + SSP_SSCR0);
		udelay(125);
		writel_relaxed((sscr0 & ~SSCR0_SCR_MASK) | div | SSCR0_SSE,
			       priv->base + SSP_SSCR0);
	} else {
		/*
		 * First configuration since open: SSCR0 must be 0 before any
		 * field is programmed, and SSE is its own separate write, last,
		 * never combined with the configuration store.
		 */
		writel_relaxed(0, priv->base + SSP_SSCR0);
		writel_relaxed(SSCR0_DSS_16 | SSCR0_FRF_TI | div, priv->base + SSP_SSCR0);
		writel_relaxed(SSCR1_ECS, priv->base + SSP_SSCR1);
		writel_relaxed(SSCR0_DSS_16 | SSCR0_FRF_TI | div | SSCR0_SSE,
			       priv->base + SSP_SSCR0);

		/*
		 * One write of a zero frame flips the word-select latch so that
		 * left and right are not swapped. It must happen after SSE is
		 * set, not before.
		 */
		writel_relaxed(0, priv->base + SSP_SSDR);
	}

	return 0;
}

/*
 * The dmaengine PCM reads the DMA data when it opens the substream, which
 * ASoC does before it calls the DAI's startup(), so it has to be in place
 * from the DAI probe on.
 */
static int sa11x0_ssp_dai_probe(struct snd_soc_dai *dai)
{
	struct sa11x0_ssp *priv = dev_get_drvdata(dai->dev);

	snd_soc_dai_init_dma_data(dai, &priv->dma_playback, NULL);

	return 0;
}

static const struct snd_soc_dai_ops sa11x0_ssp_dai_ops = {
	.probe		= sa11x0_ssp_dai_probe,
	.startup	= sa11x0_ssp_startup,
	.shutdown	= sa11x0_ssp_shutdown,
	.hw_params	= sa11x0_ssp_hw_params,
	.set_fmt	= sa11x0_ssp_set_fmt,
	.set_sysclk	= sa11x0_ssp_set_sysclk,
};

static struct snd_soc_dai_driver sa11x0_ssp_dai = {
	.name = "sa11x0-ssp",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_KNOT,
		.rate_min	= 8000,
		.rate_max	= 48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &sa11x0_ssp_dai_ops,
};

static const struct snd_soc_component_driver sa11x0_ssp_component = {
	.name = "sa11x0-ssp",
};

static const struct snd_dmaengine_pcm_config sa11x0_ssp_pcm_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
};

static const struct of_device_id sa11x0_ssp_of_match[] = {
	{ .compatible = "intel,sa1110-ssp" },
	{ }
};
MODULE_DEVICE_TABLE(of, sa11x0_ssp_of_match);

static int sa11x0_ssp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sa11x0_ssp *priv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	priv->phys_base = res->start;
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->dma_playback.addr = priv->phys_base + SSP_SSDR;
	priv->dma_playback.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	priv->dma_playback.maxburst = 8;
	priv->dma_playback.chan_name = "tx";

	dev_set_drvdata(dev, priv);

	ret = devm_snd_dmaengine_pcm_register(dev, &sa11x0_ssp_pcm_config, 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register PCM\n");

	return devm_snd_soc_register_component(dev, &sa11x0_ssp_component, &sa11x0_ssp_dai, 1);
}

static struct platform_driver sa11x0_ssp_driver = {
	.driver = {
		.name = "sa11x0-ssp",
		.of_match_table = sa11x0_ssp_of_match,
	},
	.probe = sa11x0_ssp_probe,
};
module_platform_driver(sa11x0_ssp_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("SA-1110 SSP SoC Audio Interface");
MODULE_LICENSE("GPL");
