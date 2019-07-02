// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019, Linaro Limited

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/soundwire/sdw.h>
#include "common.h"
#include "qdsp6/q6afe.h"

#define DEFAULT_SAMPLE_RATE_48K		48000
#define DEFAULT_MCLK_RATE		24576000
#define MI2S_BCLK_RATE		1536000

#define SLIM_MAX_TX_PORTS 16
#define SLIM_MAX_RX_PORTS 16
#define WCD934X_DEFAULT_MCLK_RATE	9600000
#define WSA881X_MAX_SWR_PORTS   4
#define SWM_MAX_DAIS	16
#define DB845C_NUM_SPEAKERS	2

struct db845c_spkr_data {
	struct sdw_stream_config sconfig;
	struct sdw_slave *sdev;
	struct sdw_port_config port_config[WSA881X_MAX_SWR_PORTS];
	bool port_enable[WSA881X_MAX_SWR_PORTS];
	int active_ports;
};

struct db845c_data {
	struct sdw_stream_runtime *sruntime[SWM_MAX_DAIS];
	struct db845c_spkr_data lspkr_data;
	struct db845c_spkr_data rspkr_data;
};

struct sdw_port_config wsa_spkr_port_config[WSA881X_MAX_SWR_PORTS] = {
	{
		.num = 1,
		.ch_mask = 0x1,
	},{
		.num = 2,
		.ch_mask = 0xf,
	},{
		.num = 3,
		.ch_mask = 0x3,
	},{	/* IV feedback */
		.num = 4,
		.ch_mask = 0x3,
	},
};

static int db845_add_spkr_stream(struct db845c_spkr_data *data,
				 struct sdw_stream_runtime *sruntime)
{
	int i;

	data->active_ports = 0;
	for (i = 0; i < WSA881X_MAX_SWR_PORTS; i++) {
		if (!data->port_enable[i])
			continue;

		data->port_config[data->active_ports] = wsa_spkr_port_config[i];
		data->active_ports++;
	}

	return sdw_stream_add_slave(data->sdev, &data->sconfig,
				    data->port_config, data->active_ports,
				    sruntime);
}

static void db845_remove_spkr_stream(struct db845c_spkr_data *sdata,
				     struct sdw_stream_runtime *sruntime)
{
	memset(sdata->port_config, 0, WSA881X_MAX_SWR_PORTS *
	       sizeof(struct sdw_port_config));
	sdw_stream_remove_slave(sdata->sdev, sruntime);
}

static int db845c_snd_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct db845c_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	int ret = 0, i;

	switch (cpu_dai->id) {
	case QUATERNARY_MI2S_RX:
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		break;
	case SLIMBUS_0_RX...SLIMBUS_6_TX:
		for (i = 0 ; i < dai_link->num_codecs; i++) {
			ret = snd_soc_dai_get_channel_map(rtd->codec_dais[i],
					&tx_ch_cnt, tx_ch, &rx_ch_cnt, rx_ch);

			if (ret != 0 && ret != -ENOTSUPP) {
				pr_err("failed to get codec chan map, err:%d\n", ret);
				return ret;
			} else if (ret == -0) {
				if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
					ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
							  rx_ch_cnt, rx_ch);
				else {
					ret = snd_soc_dai_set_channel_map(cpu_dai, tx_ch_cnt, tx_ch,
										  0, NULL);
					pr_err("TX set codec chan map,       err:%d %d %d\n", ret, tx_ch_cnt,
					       tx_ch[0]);
				}
			} else if (ret == -ENOTSUPP) {
				/* Ignore unsupported */
				ret = 0;
			}

			/* If soundwire connected then do the slave config */
			data->sruntime[cpu_dai->id] =
					snd_soc_dai_get_sdw_stream(
							rtd->codec_dais[i], 0);
			if (!IS_ERR(data->sruntime[cpu_dai->id])) {
				db845_add_spkr_stream(&data->lspkr_data,
						      data->sruntime[cpu_dai->id]);
				db845_add_spkr_stream(&data->rspkr_data,
						      data->sruntime[cpu_dai->id]);

			}
		}

		break;
	default:
		pr_err("%s: invalid dai id 0x%x\n", __func__, cpu_dai->id);
		break;
	}

	return ret;
}

static int spkr_get_port(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct db845c_data *data = snd_soc_card_get_drvdata(comp->card);
	struct soc_mixer_control *mixer =
		(struct soc_mixer_control *)kcontrol->private_value;
	int portidx = mixer->reg - 1;
	struct db845c_spkr_data *spkr_data;

	if (mixer->shift)
		spkr_data = &data->rspkr_data;
	else
		spkr_data = &data->lspkr_data;

	ucontrol->value.integer.value[0] = spkr_data->port_enable[portidx];

	return 0;
}

static int spkr_set_port(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct db845c_data *data = snd_soc_card_get_drvdata(comp->card);
	struct soc_mixer_control *mixer =
		(struct soc_mixer_control *)kcontrol->private_value;
	int portidx = mixer->reg - 1;
	struct db845c_spkr_data *spkr_data;

	if (mixer->shift)
		spkr_data = &data->rspkr_data;
	else
		spkr_data = &data->lspkr_data;

	if (ucontrol->value.integer.value[0])
		spkr_data->port_enable[portidx] = true;
	else
		spkr_data->port_enable[portidx] = false;

	return 0;
}

static const struct snd_kcontrol_new spkr_left_controls[] = {
	SOC_SINGLE_EXT("COMP Switch", 1, 0, 1, 0,
		spkr_get_port, spkr_set_port),
	SOC_SINGLE_EXT("BOOST Switch", 2, 0, 1, 0,
		spkr_get_port, spkr_set_port),
	SOC_SINGLE_EXT("VISENSE Switch", 3, 0, 1, 0,
		spkr_get_port, spkr_set_port),
};

static const struct snd_kcontrol_new spkr_right_controls[] = {
	SOC_SINGLE_EXT("COMP Switch", 1, 1, 1, 0,
		spkr_get_port, spkr_set_port),
	SOC_SINGLE_EXT("BOOST Switch", 2, 1, 1, 0,
		spkr_get_port, spkr_set_port),
	SOC_SINGLE_EXT("VISENSE Switch", 3, 1, 1, 0,
		spkr_get_port, spkr_set_port),
};

static int db845c_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct db845c_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	switch (cpu_dai->id) {
	case SLIMBUS_0_RX...SLIMBUS_6_TX:
		if (!IS_ERR(data->sruntime[cpu_dai->id])) {
			//sdw_stream_remove_master(data->rspkr_data.sdev->bus, data->sruntime[cpu_dai->id]);
			db845_remove_spkr_stream(&data->lspkr_data,
						 data->sruntime[cpu_dai->id]);
			db845_remove_spkr_stream(&data->rspkr_data,
						 data->sruntime[cpu_dai->id]);
			data->sruntime[cpu_dai->id] = NULL;
		}

		break;
	default:
		break;
	}
	return 0;
}

static int db845c_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	switch (cpu_dai->id) {
	case QUATERNARY_MI2S_RX:
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_MCLK_4,
			DEFAULT_MCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBS_CFS);


		break;

	default:
		pr_err("%s: invalid dai id 0x%x\n", __func__, cpu_dai->id);
		break;
	}
	return 0;
}
static const struct snd_soc_ops db845c_be_ops = {
	.hw_params = db845c_snd_hw_params,
	.hw_free = db845c_snd_hw_free,
	.startup = db845c_snd_startup,
};

static int db845c_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = DEFAULT_SAMPLE_RATE_48K;
	channels->min = channels->max = 2;
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static int db845c_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	/*
	 * Codec SLIMBUS configuration
	 * RX1, RX2, RX3, RX4, RX5, RX6, RX7, RX8, RX9, RX10, RX11, RX12, RX13
	 * TX1, TX2, TX3, TX4, TX5, TX6, TX7, TX8, TX9, TX10, TX11, TX12, TX13
	 * TX14, TX15, TX16
	 */
	unsigned int rx_ch[SLIM_MAX_RX_PORTS] = {144, 145, 146, 147, 148, 149,
					150, 151, 152, 153, 154, 155, 156};
	unsigned int tx_ch[SLIM_MAX_TX_PORTS] = {128, 129, 130, 131, 132, 133,
					    134, 135, 136, 137, 138, 139,
					    140, 141, 142, 143};
	int ret, i;

	switch (cpu_dai->id) {
	case SLIMBUS_0_RX...SLIMBUS_6_TX:
		for (i = 0 ; i < dai_link->num_codecs; i++) {
			ret = snd_soc_dai_set_channel_map(rtd->codec_dais[i],
							  ARRAY_SIZE(tx_ch),
							  tx_ch,
							  ARRAY_SIZE(rx_ch),
							  rx_ch);
			if (ret != 0 && ret != -ENOTSUPP) {
				return ret;
			} else if (!ret) {
				snd_soc_dai_set_sysclk(rtd->codec_dais[i], 0,
						       WCD934X_DEFAULT_MCLK_RATE,
						       SNDRV_PCM_STREAM_PLAYBACK);
			}
		}
		break;
	default:
		break;
	}

	return 0;
}

static void db845c_add_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->ops = &db845c_be_ops;
			link->init = db845c_init;
			link->be_hw_params_fixup = db845c_be_hw_params_fixup;
		}
	}
}

static int db845c_spkr_left_init(struct snd_soc_component *component)
{
	struct db845c_data *data = snd_soc_card_get_drvdata(component->card);

	data->lspkr_data.sdev = dev_to_sdw_dev(component->dev);
	data->lspkr_data.sconfig.ch_count = 1;
	data->lspkr_data.sconfig.bps = 1;
	data->lspkr_data.sconfig.frame_rate = 48000;
	data->lspkr_data.sconfig.direction = SDW_DATA_DIR_RX;
	data->lspkr_data.sconfig.type = SDW_STREAM_PDM;
	snd_soc_add_component_controls(component, spkr_left_controls,
				       ARRAY_SIZE(spkr_left_controls));

	return 0;
}

static int db845c_spkr_right_init(struct snd_soc_component *component)
{
	struct db845c_data *data = snd_soc_card_get_drvdata(component->card);

	data->rspkr_data.sdev = dev_to_sdw_dev(component->dev);
	data->rspkr_data.sconfig.ch_count = 1;
	data->rspkr_data.sconfig.bps = 1;
	data->rspkr_data.sconfig.frame_rate = 48000;
	data->rspkr_data.sconfig.direction = SDW_DATA_DIR_RX;
	data->rspkr_data.sconfig.type = SDW_STREAM_PDM;
	snd_soc_add_component_controls(component, spkr_right_controls,
				       ARRAY_SIZE(spkr_right_controls));

	return 0;
}

static struct snd_soc_aux_dev wsa_aux_dev[] = {
        {
                .name = "SpkrLeft",
                .codec_name = "sdw:0:217:2010:0:1",
		.init = db845c_spkr_left_init,
        },
        {
                .name = "SpkrRight",
                .codec_name = "sdw:0:217:2010:0:2",
		.init = db845c_spkr_right_init,
        },
};

static struct snd_soc_codec_conf wsa_codec_conf[] = {
        {
                .dev_name = "sdw:0:217:2010:0:1",
                .name_prefix = "SpkrLeft",
        },
        {
                .dev_name = "sdw:0:217:2010:0:2",
                .name_prefix = "SpkrRight",
        },
};

static int db845c_snd_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;
	struct db845c_data *data;
	int ret;
	card = kzalloc( sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret) {
		dev_err(dev, "Error parsing OF data\n");
		goto parse_dt_fail;
	}
	card->aux_dev = wsa_aux_dev;
	card->num_aux_devs = ARRAY_SIZE(wsa_aux_dev);
	card->codec_conf = wsa_codec_conf;
        card->num_configs = ARRAY_SIZE(wsa_codec_conf);

	db845c_add_ops(card);
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(dev, "Sound card registration failed\n");
		goto register_card_fail;
	}
	return ret;

register_card_fail:
	kfree(card->dai_link);
parse_dt_fail:
	kfree(card);
	return ret;
}

static int db845c_snd_platform_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_card(card);
	kfree(card->dai_link);
	kfree(card);
	return 0;
}

static const struct of_device_id db845c_snd_device_id[]  = {
	{ .compatible = "qcom,db845c-sndcard" },
	{},
};
MODULE_DEVICE_TABLE(of, db845c_snd_device_id);

static struct platform_driver db845c_snd_driver = {
	.probe = db845c_snd_platform_probe,
	.remove = db845c_snd_platform_remove,
	.driver = {
		.name = "msm-snd-db845c",
		.of_match_table = db845c_snd_device_id,
	},
};
module_platform_driver(db845c_snd_driver);

MODULE_DESCRIPTION("db845c ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
