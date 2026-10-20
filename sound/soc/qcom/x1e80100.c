// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/soundwire/sdw.h>
#include <linux/thermal.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "common.h"
#include "qdsp6/audioreach.h"
#include "qdsp6/q6afe.h"
#include "qdsp6/q6dsp-common.h"
#include "sdw.h"

struct x1e80100_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack dp_jack[8];
	bool jack_setup;
};

/*
 * Walk the merged WSA Playback link's codec DAIs for the VI-side wsa884x
 * DAIs, look up each codec's thermal zone (registered by wsa884x under
 * the SoundWire slave's dev_name), and publish them to audioreach in VI
 * channel order. The DT lists VI DAIs in channel order (left_woofer,
 * left_tweeter, right_woofer, right_tweeter), which already matches the
 * SP_VI module's channel map -- no explicit slot assignment needed.
 */
static void x1e80100_register_sp_speakers(struct snd_soc_pcm_runtime *rtd)
{
	struct thermal_zone_device *tzs[MAX_SP_VI_SPEAKERS];
	struct snd_soc_dai *codec_dai;
	unsigned int num = 0;
	int i;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		struct snd_soc_component *component = codec_dai->component;
		struct thermal_zone_device *tz;
		struct sdw_slave *slave;
		char tz_name[THERMAL_NAME_LENGTH];

		if (strcmp(codec_dai->name, "VI"))
			continue;
		if (!component || !component->dev)
			continue;
		if (num >= ARRAY_SIZE(tzs))
			break;

		/*
		 * Match the "wsa-<bus_id>-<unique_id>" naming the wsa884x
		 * codec uses when it registers its thermal zone -- shorter
		 * than dev_name() so it fits THERMAL_NAME_LENGTH, derivable
		 * from the SoundWire slave the codec DAI sits on.
		 */
		slave = dev_to_sdw_dev(component->dev);
		scnprintf(tz_name, sizeof(tz_name), "wsa-%d-%u",
			  slave->bus->id, slave->id.unique_id);
		tz = thermal_zone_get_zone_by_name(tz_name);
		if (IS_ERR(tz)) {
			dev_warn(rtd->dev,
				 "no thermal zone '%s' for %s (%ld); SP T0 kcontrol will read 0 for this slot\n",
				 tz_name, dev_name(component->dev),
				 PTR_ERR(tz));
			continue;
		}
		tzs[num++] = tz;
	}

	if (num)
		audioreach_sp_register_speaker_map(tzs, num);
}

static int x1e80100_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_jack *dp_jack = NULL;
	int dp_pcm_id = 0;

	switch (cpu_dai->id) {
	case WSA_CODEC_DMA_RX_0:
		x1e80100_register_sp_speakers(rtd);
		fallthrough;
	case WSA_CODEC_DMA_RX_1:
		/*
		 * Set limit of -3 dB on Digital Volume and 0 dB on PA Volume
		 * to reduce the risk of speaker damage until we have active
		 * speaker protection in place.
		 */
		snd_soc_limit_volume(card, "WSA WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "SpkrLeft PA Volume", 6);
		snd_soc_limit_volume(card, "SpkrRight PA Volume", 6);
		snd_soc_limit_volume(card, "WooferLeft PA Volume", 6);
		snd_soc_limit_volume(card, "TweeterLeft PA Volume", 6);
		snd_soc_limit_volume(card, "WooferRight PA Volume", 6);
		snd_soc_limit_volume(card, "TweeterRight PA Volume", 6);
		break;
	case DISPLAY_PORT_RX_0:
		dp_pcm_id = 0;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	case DISPLAY_PORT_RX_1 ... DISPLAY_PORT_RX_7:
		dp_pcm_id = cpu_dai->id - DISPLAY_PORT_RX_1 + 1;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	default:
		break;
	}

	if (dp_jack)
		return qcom_snd_dp_jack_setup(rtd, dp_jack, dp_pcm_id);

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

/*
 * Rewrite codec DAI params when the WSA merged BE link's playback-side
 * substream is reused for VI capture codec DAIs via multi_direction_codecs.
 * The VI capture chain (WSA codec VI, swr0 Pin12/13, wsa_macro_vifeedback)
 * runs at 8kHz/S32_LE/4ch regardless of what the RX side runs at.
 */
static int x1e80100_wsa_vi_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params,
					int stream)
{
	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt =
		hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	if (stream != SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	rate->min = rate->max = 8000;
	channels->min = channels->max = 4;
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

static int x1e80100_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
							  SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	if (cpu_dai->id == WSA_CODEC_DMA_TX_0) {
		rate->min = rate->max = 8000;
	} else {
		rate->min = rate->max = 48000;
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	}

	switch (cpu_dai->id) {
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
		channels->min = 1;
		break;
	default:
		break;
	}

	return 0;
}

static int x1e80100_snd_hw_map_channels(unsigned int *ch_map, int num)
{
	switch (num) {
	case 1:
		ch_map[0] = PCM_CHANNEL_FC;
		break;
	case 2:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_FR;
		break;
	case 3:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_FR;
		ch_map[2] = PCM_CHANNEL_FC;
		break;
	case 4:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_LB;
		ch_map[2] = PCM_CHANNEL_FR;
		ch_map[3] = PCM_CHANNEL_RB;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Light up the WSA macro's VI-feedback DAPM path for both WSA buses so
 * the LPASS WSA SPKR_PROT TX registers get programmed (via the WSA
 * macro's "WSA AIF_VI" AIF_OUT widget POST_PMU handler). The merged
 * RX_0 BE link brings up the WSA codec VI DAIs but never activates a
 * capture substream, so the AIF widget would otherwise stay off and
 * no VI samples would reach the DSP's SP_VI module.
 */
static int x1e80100_wsa_vi_dapm_power(struct snd_soc_pcm_runtime *rtd, bool enable)
{
	struct snd_soc_dai *codec_dai;
	int i, ret;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		struct snd_soc_component *component = codec_dai->component;
		struct snd_soc_dapm_context *dapm;
		char pin[64];

		if (strcmp(codec_dai->name, "wsa_macro_vifeedback"))
			continue;

		dapm = snd_soc_component_to_dapm(component);
		if (!dapm)
			continue;

		if (component->name_prefix)
			snprintf(pin, sizeof(pin), "%s WSA AIF_VI",
				 component->name_prefix);
		else
			snprintf(pin, sizeof(pin), "WSA AIF_VI");

		if (enable)
			ret = snd_soc_dapm_force_enable_pin(dapm, pin);
		else
			ret = snd_soc_dapm_disable_pin(dapm, pin);
		if (ret)
			continue;

		snd_soc_dapm_sync(dapm);
	}

	return 0;
}

static int x1e80100_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int channels = substream->runtime->channels;
	unsigned int rx_slot[4];
	int ret;

	switch (cpu_dai->id) {
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
		ret = x1e80100_snd_hw_map_channels(rx_slot, channels);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						  channels, rx_slot);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	ret = qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
	if (ret)
		return ret;

	if (cpu_dai->id == WSA_CODEC_DMA_RX_0 &&
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		x1e80100_wsa_vi_dapm_power(rtd, true);

	return 0;
}

static int x1e80100_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	if (cpu_dai->id == WSA_CODEC_DMA_RX_0 &&
	    substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		x1e80100_wsa_vi_dapm_power(rtd, false);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops x1e80100_be_ops = {
	.startup = qcom_snd_sdw_startup,
	.shutdown = qcom_snd_sdw_shutdown,
	.hw_free = x1e80100_snd_hw_free,
	.prepare = x1e80100_snd_prepare,
};

static void x1e80100_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = x1e80100_snd_init;
			link->be_hw_params_fixup = x1e80100_be_hw_params_fixup;
			link->ops = &x1e80100_be_ops;
		}

		/*
		 * The WSA speaker-protection playback link bundles both the
		 * SPKR playback-side codec DAIs and the VI capture-side
		 * codec DAIs (VI, swr0 Pin12/13, wsa_macro_vifeedback) so a
		 * single MM1 playback open brings up both chains. The
		 * SP<->SP_VI control-link in the tplg handles the DSP-side
		 * pairing.
		 */
		if (link->id == WSA_CODEC_DMA_RX_0) {
			link->multi_direction_codecs = 1;
			link->multi_direction_params_fixup =
				x1e80100_wsa_vi_params_fixup;
		}
	}
}

static int x1e80100_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct x1e80100_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->owner = THIS_MODULE;
	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	x1e80100_add_be_ops(card);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_x1e80100_dt_match[] = {
	{ .compatible = "qcom,x1e80100-sndcard", .data = "x1e80100" },
	{ .compatible = "qcom,glymur-sndcard", .data = "glymur" },
	{}
};
MODULE_DEVICE_TABLE(of, snd_x1e80100_dt_match);

static struct platform_driver snd_x1e80100_driver = {
	.probe  = x1e80100_platform_probe,
	.driver = {
		.name = "snd-x1e80100",
		.of_match_table = snd_x1e80100_dt_match,
	},
};
module_platform_driver(snd_x1e80100_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_AUTHOR("Krzysztof Kozlowski <krzysztof.kozlowski@linaro.org>");
MODULE_DESCRIPTION("Qualcomm X1E80100 ASoC Machine Driver");
MODULE_LICENSE("GPL");
