/* audio support for HiFiBerry based DAC+ADC Pro based boards
 *
 * Copyright 2022 HiFiBerry
 * Author Joerg Schambacher <joerg@hifiberry.com>
 *
 * ONLY MASTER MODE SUPPORTED FOR NOW !!!
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>
#include "fsl_sai.h"
#include "../codecs/pcm512x.h"
#include "../codecs/pcm186x.h"

#define DAC_CLK_EXT_44K 22579200UL
#define DAC_CLK_EXT_48K 24576000UL

struct imx_dacplusadc_data {
	struct snd_soc_dai_link dai_link[3];
	struct snd_soc_card card;
	unsigned int slots;
	unsigned int slot_width;
	unsigned int daifmt;
	bool dac_sclk;
	bool dac_pluspro;
	bool dac_led_status;
	bool dac_gain_limit;
	bool one2one_ratio;
	bool tdm_mode;
	bool dac_run;
	int dac_format;
	bool adc_run;
	int adc_format;
	bool dac_slave;
};

enum ext_osc {
	DAC_CLK_INT,
	DAC_CLK_EXT_44EN,
	DAC_CLK_EXT_48EN,
};

static const struct imx_pcm512x_fs_map {
	unsigned int rmin;
	unsigned int rmax;
	unsigned int wmin;
	unsigned int wmax;
}
fs_map[] = {
	/* Normal, < 32kHz */
	{
		.rmin = 8000, .rmax = 24000, .wmin = 1024, .wmax = 1024,
	},
	/* Normal, 32kHz */
	{
		.rmin = 32000,
		.rmax = 32000,
		.wmin = 256,
		.wmax = 1024,
	},
	/* Normal */
	{
		.rmin = 44100,
		.rmax = 48000,
		.wmin = 256,
		.wmax = 768,
	},
	/* Double */
	{
		.rmin = 88200,
		.rmax = 96000,
		.wmin = 256,
		.wmax = 512,
	},
	/* Quad */
	{
		.rmin = 176400,
		.rmax = 192000,
		.wmin = 128,
		.wmax = 256,
	},
};

static const u32 dacplusadc_rates[] = {
	8000,
	11025,
	16000,
	22050,
	32000,
	44100,
	48000,
	64000,
	88200,
	96000,
	176400,
	192000,
	352800,
	384000,
};

static const unsigned int pcm186x_adc_input_channel_sel_value[] = {
	0x00,
	0x01,
	0x02,
	0x03,
	0x10
};

static const char *const pcm186x_adcl_input_channel_sel_text[] = {
		"No Select",
		"VINL1[SE]",
		/* Default for ADCL */
		"VINL2[SE]",
		"VINL2[SE] + VINL1[SE]",
		"{VIN1P, VIN1M}[DIFF]"
	};

static const char *const pcm186x_adcr_input_channel_sel_text[] = {
		"No Select",
		"VINR1[SE]",
		/* Default for ADCR */
		"VINR2[SE]",
		"VINR2[SE] + VINR1[SE]",
		"{VIN2P, VIN2M}[DIFF]"
	};

static const struct soc_enum pcm186x_adc_input_channel_sel[] = {
		SOC_VALUE_ENUM_SINGLE(PCM186X_ADC1_INPUT_SEL_L, 0,
		PCM186X_ADC_INPUT_SEL_MASK,
		ARRAY_SIZE(pcm186x_adcl_input_channel_sel_text),
		pcm186x_adcl_input_channel_sel_text,
		pcm186x_adc_input_channel_sel_value),
		SOC_VALUE_ENUM_SINGLE(PCM186X_ADC1_INPUT_SEL_R, 0,
		PCM186X_ADC_INPUT_SEL_MASK,
		ARRAY_SIZE(pcm186x_adcr_input_channel_sel_text),
		pcm186x_adcr_input_channel_sel_text,
		pcm186x_adc_input_channel_sel_value),
};

static const unsigned int pcm186x_mic_bias_sel_value[] = {
	0x00,
	0x01,
	0x11
};

static const char *const pcm186x_mic_bias_sel_text[] = {
		"Mic Bias off",
		"Mic Bias on",
		"Mic Bias with Bypass Resistor"
	};

static const struct soc_enum pcm186x_mic_bias_sel[] = {
		SOC_VALUE_ENUM_SINGLE(PCM186X_MIC_BIAS_CTRL, 0,
		GENMASK(4, 0),
		ARRAY_SIZE(pcm186x_mic_bias_sel_text),
		pcm186x_mic_bias_sel_text,
		pcm186x_mic_bias_sel_value),
};

static const unsigned int pcm186x_gain_sel_value[] = {
	0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
	0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50
};

static const char *const pcm186x_gain_sel_text[] = {
	"-12.0dB", "-11.5dB", "-11.0dB", "-10.5dB", "-10.0dB", "-9.5dB",
	"-9.0dB", "-8.5dB", "-8.0dB", "-7.5dB", "-7.0dB", "-6.5dB",
	"-6.0dB", "-5.5dB", "-5.0dB", "-4.5dB", "-4.0dB", "-3.5dB",
	"-3.0dB", "-2.5dB", "-2.0dB", "-1.5dB", "-1.0dB", "-0.5dB",
	"0.0dB", "0.5dB", "1.0dB", "1.5dB", "2.0dB", "2.5dB",
	"3.0dB", "3.5dB", "4.0dB", "4.5dB", "5.0dB", "5.5dB",
	"6.0dB", "6.5dB", "7.0dB", "7.5dB", "8.0dB", "8.5dB",
	"9.0dB", "9.5dB", "10.0dB", "10.5dB", "11.0dB", "11.5dB",
	"12.0dB", "12.5dB", "13.0dB", "13.5dB", "14.0dB", "14.5dB",
	"15.0dB", "15.5dB", "16.0dB", "16.5dB", "17.0dB", "17.5dB",
	"18.0dB", "18.5dB", "19.0dB", "19.5dB", "20.0dB", "20.5dB",
	"21.0dB", "21.5dB", "22.0dB", "22.5dB", "23.0dB", "23.5dB",
	"24.0dB", "24.5dB", "25.0dB", "25.5dB", "26.0dB", "26.5dB",
	"27.0dB", "27.5dB", "28.0dB", "28.5dB", "29.0dB", "29.5dB",
	"30.0dB", "30.5dB", "31.0dB", "31.5dB", "32.0dB", "32.5dB",
	"33.0dB", "33.5dB", "34.0dB", "34.5dB", "35.0dB", "35.5dB",
	"36.0dB", "36.5dB", "37.0dB", "37.5dB", "38.0dB", "38.5dB",
	"39.0dB", "39.5dB", "40.0dB"};

static const struct soc_enum pcm186x_gain_sel[] = {
	SOC_VALUE_ENUM_SINGLE(PCM186X_PGA_VAL_CH1_L, 0,
		0xff,
		ARRAY_SIZE(pcm186x_gain_sel_text),
		pcm186x_gain_sel_text,
		pcm186x_gain_sel_value),
	SOC_VALUE_ENUM_SINGLE(PCM186X_PGA_VAL_CH1_R, 0,
		0xff,
		ARRAY_SIZE(pcm186x_gain_sel_text),
		pcm186x_gain_sel_text,
		pcm186x_gain_sel_value),
};

static const struct snd_kcontrol_new pcm1863_snd_controls_card[] = {
	SOC_ENUM("ADC Left Input", pcm186x_adc_input_channel_sel[0]),
	SOC_ENUM("ADC Right Input", pcm186x_adc_input_channel_sel[1]),
	SOC_ENUM("ADC Mic Bias", pcm186x_mic_bias_sel),
	SOC_ENUM("PGA Gain Left", pcm186x_gain_sel[0]),
	SOC_ENUM("PGA Gain Right", pcm186x_gain_sel[1]),
};

static int pcm1863_add_controls(struct snd_soc_component *component)
{
	snd_soc_add_component_controls(component,
		pcm1863_snd_controls_card,
		ARRAY_SIZE(pcm1863_snd_controls_card));
	return 0;
}

static int imx_dacplusadc_select_ext_clk(struct snd_soc_component *comp, int ext_osc)
{
	switch (ext_osc) {
	case DAC_CLK_INT:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x00);
		break;
	case DAC_CLK_EXT_44EN:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x20);
		break;
	case DAC_CLK_EXT_48EN:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x04);
		break;
	}
	usleep_range(3000, 4000);

	return 0;
}

static bool imx_dacplusadc_is_sclk(struct snd_soc_component *comp)
{
	unsigned int sclk;

	snd_soc_component_read(comp, PCM512x_RATE_DET_4, &sclk);

	return (!(sclk & 0x40));
}

static bool imx_dacplusadc_is_sclk_sleep(struct snd_soc_component *comp)
{
	msleep(2);
	return imx_dacplusadc_is_sclk(comp);
}

static int imx_dacplusadcpro_configure_gpios(struct snd_soc_component *dac,
		struct snd_soc_component *adc)
{
	/* ADC LED */
	snd_soc_component_write(adc, PCM186X_GPIO3_2_CTRL, 0x00);
	snd_soc_component_write(adc, PCM186X_GPIO3_2_DIR_CTRL, 0x04);

	/* DAC LED */
	snd_soc_component_update_bits(dac, PCM512x_GPIO_OUTPUT_4, 0x0f, 0x02);
	snd_soc_component_update_bits(dac, PCM512x_GPIO_EN, 0x08, 0x08);

	/* Oscillator control */
	snd_soc_component_update_bits(dac, PCM512x_GPIO_EN, 0x24, 0x24);
	snd_soc_component_update_bits(dac, PCM512x_GPIO_OUTPUT_3, 0x0f, 0x02);
	snd_soc_component_update_bits(dac, PCM512x_GPIO_OUTPUT_6, 0x0f, 0x02);

	return 0;
}

static bool imx_dacplusadc_is_pro_card(struct snd_soc_component *dac,
		struct snd_soc_component *adc)
{
	bool isClk44EN, isClk48En, isNoClk;

	imx_dacplusadc_select_ext_clk(dac, DAC_CLK_EXT_44EN);
	isClk44EN = imx_dacplusadc_is_sclk_sleep(dac);

	imx_dacplusadc_select_ext_clk(dac, DAC_CLK_INT);
	isNoClk = imx_dacplusadc_is_sclk_sleep(dac);

	imx_dacplusadc_select_ext_clk(dac, DAC_CLK_EXT_48EN);
	isClk48En = imx_dacplusadc_is_sclk_sleep(dac);

	return (isClk44EN && isClk48En && !isNoClk);
}

static int imx_dacplusadc_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *dac = rtd->codec_dais[0]->component;
	struct snd_soc_component *adc = rtd->codec_dais[1]->component;
	struct snd_soc_card *card = rtd->card;
	struct imx_dacplusadc_data *data = snd_soc_card_get_drvdata(card);
	int ret;
	struct snd_soc_dai_driver *adc_driver = rtd->codec_dais[1]->driver;

	if (data->dac_gain_limit) {
		ret = snd_soc_limit_volume(card, "Digital Playback Volume", 207);
		if (ret)
			dev_warn(card->dev, "fail to set volume limit");
	}

	/* set ADC DAI configuration; ADC is always 'slave' */
	ret = snd_soc_dai_set_fmt(rtd->codec_dais[1],
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	adc_driver->capture.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE;

	ret = pcm1863_add_controls(adc);
	if (ret < 0)
		dev_warn(rtd->dev, "Failed to add pcm1863 controls: %d\n",
			ret);

	imx_dacplusadcpro_configure_gpios(dac, adc);

	if (!data->dac_slave)
		data->dac_pluspro = imx_dacplusadc_is_pro_card(dac, adc);
	else
		data->dac_pluspro = false;

	/* set GPIO2 to output, GPIO3 input */
	snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x40);

	snd_soc_component_update_bits(dac, PCM512x_GPIO_CONTROL_1, 0x08, 0x08);

	return 0;
}

static unsigned long dacplusadc_get_mclk_rate(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_dacplusadc_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int width = data->slots * data->slot_width;
	unsigned int rate = params_rate(params);
	int i;

	for (i = 0; i < ARRAY_SIZE(fs_map); i++) {
		if (rate >= fs_map[i].rmin && rate <= fs_map[i].rmax) {
			width = max(width, fs_map[i].wmin);
			width = min(width, fs_map[i].wmax);
			/*Adjust SAI bclk:mclk ratio */
			width *= data->one2one_ratio ? 1 : 2;

			return rate * width;
		}
	}

	return 0;
}

static int imx_dacplusadc_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *dac_dai = rtd->codec_dais[0];
	struct snd_soc_dai *adc_dai = rtd->codec_dais[1];
	struct snd_soc_component *dac = dac_dai->component;
	struct snd_soc_card *card = rtd->card;
	struct imx_dacplusadc_data *data = snd_soc_card_get_drvdata(card);
	unsigned int channels = params_channels(params);
	unsigned int sample_rate = params_rate(params);
	unsigned long mclk_freq;
	int ret;
	const struct snd_soc_dai_ops *dac_ops = dac_dai->driver->ops;

	data->slots = 2;
	data->slot_width = params_physical_width(params);
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM /*data->daifmt*/);
	if (ret) {
		dev_err(card->dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai,
		BIT(channels) - 1, BIT(channels) - 1,
		data->slots, data->slot_width);
	if (ret) {
		dev_err(card->dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, data->slots * data->slot_width);
	if (ret) {
		dev_err(card->dev, "failed to set cpu dai bclk ratio\n");
		return ret;
	}

	if (data->dac_slave)
		ret = snd_soc_dai_set_fmt(dac_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	else
		ret = snd_soc_dai_set_fmt(dac_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		dev_err(card->dev, "failed to set DAC dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(adc_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	if (ret) {
		dev_err(card->dev, "failed to set ADC dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_bclk_ratio(dac_dai,
		data->slots * data->slot_width);

	if (ret) {
		dev_err(card->dev, "failed to set DAC dai bclk ratio\n");
		return ret;
	}

	if (dac_ops->hw_params)
		ret = dac_ops->hw_params(substream, params, dac_dai);

	if (ret) {
		dev_err(card->dev, "failed to set DAC hw_params\n");
		return ret;
	}

	/* set MCLK freq */
	if (data->dac_pluspro && data->dac_sclk) {
		if (do_div(sample_rate, 8000)) {
			mclk_freq = DAC_CLK_EXT_44K;
			imx_dacplusadc_select_ext_clk(dac, DAC_CLK_EXT_44EN);
			ret = snd_soc_dai_set_sysclk(dac_dai,
				PCM512x_SYSCLK_MCLK1, mclk_freq, SND_SOC_CLOCK_IN);
		} else {
			mclk_freq = DAC_CLK_EXT_48K;
			imx_dacplusadc_select_ext_clk(dac, DAC_CLK_EXT_48EN);
			ret = snd_soc_dai_set_sysclk(dac_dai,
				PCM512x_SYSCLK_MCLK2, mclk_freq, SND_SOC_CLOCK_IN);
		}
		if (ret < 0)
			dev_err(card->dev, "failed to set cpu dai mclk rate (%lu): %d\n",
				mclk_freq, ret);
	} else {
		mclk_freq = dacplusadc_get_mclk_rate(substream, params);
		ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1,
			mclk_freq, SND_SOC_CLOCK_OUT);
		if (ret < 0)
			dev_err(card->dev, "failed to set cpu dai mclk1 rate (%lu): %d\n",
				mclk_freq, ret);
	}

	return ret;
}

static int imx_dacplusadc_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	static struct snd_pcm_hw_constraint_list constraint_rates;
	struct snd_soc_component *dac = rtd->codec_dais[0]->component;
	struct snd_soc_component *adc = rtd->codec_dais[1]->component;
	int ret;

	constraint_rates.list = dacplusadc_rates;
	constraint_rates.count = ARRAY_SIZE(dacplusadc_rates);

	ret = snd_pcm_hw_constraint_list(runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &constraint_rates);
	if (ret)
		return ret;

	/* switch on respective LED */
	if (!substream->stream)
		snd_soc_component_update_bits(dac, PCM512x_GPIO_CONTROL_1, 0x08, 0x08);
	else
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x40);

	return 0;
}

static void imx_dacplusadc_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *dac_dai = rtd->codec_dais[0];
	struct snd_soc_dai *adc_dai = rtd->codec_dais[1];
	struct snd_soc_component *dac = dac_dai->component;
	struct snd_soc_component *adc = adc_dai->component;

	/* switch off respective LED */
	if (!substream->stream)
		snd_soc_component_update_bits(dac, PCM512x_GPIO_CONTROL_1, 0x08, 0x00);
	else
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x00);
}

static struct snd_soc_ops imx_dacplusadc_ops = {
	.hw_params = imx_dacplusadc_hw_params,
	.startup = imx_dacplusadc_startup,
	.shutdown = imx_dacplusadc_shutdown,
};

SND_SOC_DAILINK_DEFS(hifi,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(
		COMP_CODEC(NULL, "pcm512x-hifi"),
		COMP_CODEC(NULL, "pcm1863-aif")
	),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link imx_dacplusadc_dai[] = {
	{
		.name = "HiFiBerry DAC+ADC PRO",
			.stream_name = "HiFiBerry DAC+ADC PRO HiFi",
			.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
			.ops = &imx_dacplusadc_ops,
			.init = imx_dacplusadc_dai_init,
			.ignore_pmdown_time = 1,
			SND_SOC_DAILINK_REG(hifi),
	},
};

static int imx_dacplusadc_probe(struct platform_device *pdev)
{
	struct device_node *bitclkmaster, *framemaster = NULL;
	struct device_node *cpu_np, *codec_np_dac = NULL, *codec_np_adc = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct platform_device *cpu_pdev = NULL;
	struct snd_soc_dai_link_component *comp;
	struct imx_dacplusadc_data *data;
	struct i2c_client *codec_dev_dac, *codec_dev_adc;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	comp = devm_kzalloc(&pdev->dev, 4 * sizeof(*comp), GFP_KERNEL);
	if (!comp)
		return -ENOMEM;

	cpu_np = of_parse_phandle(np, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "audio dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np_dac = of_parse_phandle(np, "audio-codec", 0);
	if (!codec_np_dac) {
		dev_err(&pdev->dev, "audio DAC codec phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np_adc = of_parse_phandle(np, "audio-codec", 1);
	if (!codec_np_adc) {
		dev_err(&pdev->dev, "audio ADC codec phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_dev_dac = of_find_i2c_device_by_node(codec_np_dac);
	if (!codec_dev_dac || !codec_dev_dac->dev.driver) {
		dev_err(&pdev->dev, "failed to find DAC device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}

	codec_dev_adc = of_find_i2c_device_by_node(codec_np_adc);
	if (!codec_dev_adc || !codec_dev_adc->dev.driver) {
		dev_err(&pdev->dev, "failed to find ADC device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}

	data->dac_gain_limit = of_property_read_bool(np, "dac,24db_digital_gain");
	data->dac_led_status = of_property_read_bool(np, "dac,led_status");
	data->dac_slave = of_property_read_bool(np, "dac,slave");

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	memcpy(data->dai_link, imx_dacplusadc_dai,
		sizeof(struct snd_soc_dai_link) * ARRAY_SIZE(data->dai_link));

	data->card.owner = THIS_MODULE;
	data->card.dev = &pdev->dev;
	data->card.dai_link = data->dai_link;
	data->card.num_links = 1;

	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret) {
		dev_err(&pdev->dev, "failed to find card model name\n");
		goto fail;
	}

	if (of_property_read_bool(np, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
		if (ret) {
			dev_err(&pdev->dev, "failed to parse audio-routing\n");
			goto fail;
		}
	}

	if (of_property_read_bool(np, "audio-widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(&data->card, "audio-widgets");
		if (ret) {
			dev_err(&pdev->dev, "failed to parse audio-widgets\n");
			goto fail;
		}
	}

	data->daifmt = snd_soc_of_parse_daifmt(np, NULL, &bitclkmaster, &framemaster);
	data->daifmt &= ~SND_SOC_DAIFMT_MASTER_MASK;
	if (codec_np_dac == bitclkmaster)
		       data->daifmt |= (codec_np_dac == framemaster) ?
			       SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBM_CFS;
	else
		       data->daifmt |= (codec_np_dac == framemaster) ?
			       SND_SOC_DAIFMT_CBS_CFM : SND_SOC_DAIFMT_CBS_CFS;

	/* ONLY MASTER MODE SUPPORTED FOR NOW !!!
	 * DAI needs to be initialized as clock slave till we detect a
	 * sysclk connected to the DAC. The DAIs will be configured at runtime
	 */
	data->daifmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS;

	if (!bitclkmaster)
		of_node_put(bitclkmaster);
	if (!framemaster)
		of_node_put(framemaster);
	if (of_property_read_bool(codec_np_dac, "clocks"))
		data->dac_sclk = true;

	data->dac_sclk = true;
	data->dac_run = false;
	data->dac_format = 0;
	data->adc_run = false;
	data->adc_format = 0;

	data->dai_link[0].cpus = &comp[0];
	data->dai_link[0].num_cpus = 1;
	data->dai_link[0].codecs = &comp[1];
	data->dai_link[0].num_codecs = 2;
	data->dai_link[0].platforms = &comp[3];
	data->dai_link[0].num_platforms = 1;

	data->dai_link[0].cpus->of_node = cpu_np;
	data->dai_link[0].cpus->dai_name = NULL;
	data->dai_link[0].platforms->name = NULL;
	data->dai_link[0].platforms->of_node = cpu_np;
	data->dai_link[0].codecs[0].of_node = codec_np_dac;
	data->dai_link[0].codecs[1].of_node = codec_np_adc;
	data->dai_link[0].codecs[0].dai_name = "pcm512x-hifi";
	data->dai_link[0].codecs[1].dai_name = "pcm1863-aif";
	data->dai_link[0].dai_fmt = data->daifmt;

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}
	ret = 0;

fail:
		if (cpu_np)
			of_node_put(cpu_np);
	if (codec_np_dac)
		of_node_put(codec_np_dac);
	if (codec_np_adc)
		of_node_put(codec_np_adc);

	return ret;
}

static int imx_dacplusadc_remove(struct platform_device *pdev)
{
	struct imx_dacplusadc_data *data = platform_get_drvdata(pdev);

	return snd_soc_unregister_card(&data->card);
}

static const struct of_device_id imx_dacplusadcpro_dt_ids[] = {
	{
		.compatible = "hifiberry,imx-audio-dacplusadcpro",
	},
	{},
};
MODULE_DEVICE_TABLE(of, imx_dacplusadcpro_dt_ids);

static struct platform_driver imx_dacplusadcpro_driver = {
	.driver = {
		.name = "imx-dacplusadcpro",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_dacplusadcpro_dt_ids,
	},
	.probe = imx_dacplusadc_probe,
	.remove = imx_dacplusadc_remove,
};
module_platform_driver(imx_dacplusadcpro_driver);

MODULE_DESCRIPTION("i.MX HiFiBerry dacplusadcpro ASoC machine driver");
MODULE_AUTHOR("Joerg Schambacher <joerg@hifiberry.com>");
MODULE_ALIAS("platform:imx-dacplusadcpro");
MODULE_LICENSE("GPL");
