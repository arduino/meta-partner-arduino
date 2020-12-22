/*
 * ANX7625 Audio transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 * Copyright (c) 2016, Linaro Limited
 */

#include <sound/core.h>
#include <sound/hdmi-codec.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/of_graph.h>

#include "anx7625.h"

#define DEBUG 1

#ifdef DEBUG
#ifdef DRM_DEV_DEBUG_DRIVER
#undef DRM_DEV_DEBUG_DRIVER
#endif
#define DRM_DEV_DEBUG_DRIVER(dev, fmt, ...) dev_printk(KERN_ERR, dev, fmt, ##__VA_ARGS__)

#ifdef DRM_DEV_ERROR
#undef DRM_DEV_ERROR
#endif
#define DRM_DEV_ERROR(dev, fmt, ...) dev_printk(KERN_ERR, dev, fmt, ##__VA_ARGS__)
#endif

int anx7625_hdmi_hw_params(struct device *dev, void *data,
			   struct hdmi_codec_daifmt *fmt,
			   struct hdmi_codec_params *hparms)
{
	struct anx7625_data *ctx = dev_get_drvdata(dev);
//	unsigned int audio_source, i2s_format = 0;
//	unsigned int invert_clock;
	unsigned int rate;
	unsigned int len;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "anx7625_hdmi_hw_params sample rate %d, width %d.\n",
			     hparms->sample_rate, hparms->sample_width);

	return 0;	switch (hparms->sample_rate) {
	case 32000:
		rate = AUDIO_FS_32K;
		break;
	case 44100:
		rate = AUDIO_FS_441K;
		break;
	case 48000:
		rate = AUDIO_FS_48K;
		break;
	case 88200:
		rate = AUDIO_FS_882K;
		break;
	case 96000:
		rate = AUDIO_FS_96K;
		break;
	case 176400:
		rate = AUDIO_FS_1764K;
		break;
	case 192000:
		rate = AUDIO_FS_192K;
		break;
	default:
		return -EINVAL;
	}

	switch (hparms->sample_width) {
	case 16:
		len = AUDIO_W_LEN_16_20MAX;
		break;
	case 18:
		len = AUDIO_W_LEN_18_20MAX;
		break;
	case 20:
		len = AUDIO_W_LEN_20_20MAX;
		break;
	case 24:
		len = AUDIO_W_LEN_24_24MAX;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt->fmt) {
	case HDMI_I2S:
		//audio_source = ADV7511_AUDIO_SOURCE_I2S;
		//i2s_format = ADV7511_I2S_FORMAT_I2S;
		break;
	case HDMI_RIGHT_J:
	case HDMI_LEFT_J:
	case HDMI_DSP_A:
	case HDMI_DSP_B:
	case HDMI_AC97:
	case HDMI_SPDIF:
	default:
		return -EINVAL;
	}

//	invert_clock = fmt->bit_clk_inv;

	/* Channel num */
	ret = anx7625_reg_write(ctx, ctx->i2c.tx_p2_client,
				AUDIO_CHANNEL_STATUS_6, I2S_CH_2 << 5);

	/* FS */
	ret |= anx7625_write_and_or(ctx, ctx->i2c.tx_p2_client,
				    AUDIO_CHANNEL_STATUS_4, 0xF0, rate);

	/* Word length */
	ret |= anx7625_write_and_or(ctx, ctx->i2c.tx_p2_client,
				    AUDIO_CHANNEL_STATUS_5, 0xF0, len);
	
	/* Audio change flag */
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
				AP_AV_STATUS, AP_AUDIO_CHG);

	return ret? -EINVAL: 0;
}

static int audio_startup(struct device *dev, void *data)
{
	struct anx7625_data *ctx = dev_get_drvdata(dev);

DRM_DEV_DEBUG_DRIVER(dev, "audio_startup\n");
	return anx7625_config_audio_input(ctx);
}

static void audio_shutdown(struct device *dev, void *data)
{
}

static int anx7625_hdmi_i2s_get_dai_id(struct snd_soc_component *component,
					struct device_node *endpoint)
{
	struct of_endpoint of_ep;
	int ret;

	ret = of_graph_parse_endpoint(endpoint, &of_ep);
	if (ret < 0) {
		printk("anx7625_hdmi_i2s_get_dai_id: ERROR: endpoint not found\n");
		return ret;
	}
	/*
	 * HDMI sound should be located as reg = <3>
	 * Then, it is sound port 0
	 */
	if (of_ep.port == 3) {
		return 0;
	}
printk("anx7625_hdmi_i2s_get_dai_id: ERROR\n");
	return -EINVAL;
}

static const struct hdmi_codec_ops anx7625_codec_ops = {
	.hw_params	= anx7625_hdmi_hw_params,
	.audio_startup	= audio_startup,
	.audio_shutdown = audio_shutdown,
	.get_dai_id	= anx7625_hdmi_i2s_get_dai_id,
};

static const struct hdmi_codec_pdata codec_data = {
	.ops = &anx7625_codec_ops,
	.max_i2s_channels = 2,
	.i2s = 1,
};

int anx7625_audio_init(struct device *dev, struct anx7625_data *anx7625)
{
	anx7625->audio_pdev = platform_device_register_data(dev,
					HDMI_CODEC_DRV_NAME,
					PLATFORM_DEVID_AUTO,
					&codec_data,
					sizeof(codec_data));
DRM_DEV_DEBUG_DRIVER(dev, "anx7625_audio_init: audio_pdev %p\n", anx7625->audio_pdev);
	return PTR_ERR_OR_ZERO(anx7625->audio_pdev);
}

void anx7625_audio_exit(struct anx7625_data *anx7625)
{
	if (anx7625->audio_pdev) {
		platform_device_unregister(anx7625->audio_pdev);
		anx7625->audio_pdev = NULL;
	}
}
