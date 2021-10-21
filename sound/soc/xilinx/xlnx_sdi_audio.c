// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx SDI embed and extract audio support
 *
 * Copyright (c) 2018 Xilinx Pvt., Ltd
 *
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <drm/drm_modes.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define DRIVER_NAME "xlnx-sdi-audio"

#define XSDIAUD_CNTRL_REG_OFFSET		0x00
#define XSDIAUD_SOFT_RST_REG_OFFSET		0x04
#define XSDIAUD_VER_REG_OFFSET			0x08
#define XSDIAUD_INT_EN_REG_OFFSET		0x0C
#define XSDIAUD_INT_STS_REG_OFFSET		0x10
#define XSDIAUD_EMB_VID_CNTRL_REG_OFFSET	0X14
#define XSDIAUD_AUD_CNTRL_REG_OFFSET		0x18
#define XSDIAUD_AXIS_CHCOUNT_REG_OFFSET		0x1C
#define XSDIAUD_MUX1_OR_DMUX1_CNTRL_REG_OFFSET	0x20
#define XSDIAUD_DMUX1_CNTRL_REG_OFFSET		0x20
#define XSDIAUD_GRP_PRES_REG_OFFSET		0X40
#define XSDIAUD_EXT_CNTRL_PKTSTAT_REG_OFFSET	0X44
#define XSDIAUD_EXT_CH_STAT0_REG_OFFSET		0X48
#define XSDIAUD_GUI_PARAM_REG_OFFSET		0XFC

#define XSDIAUD_CNTRL_EN_MASK		BIT(0)
#define XSDIAUD_SOFT_RST_ACLK_MASK	BIT(0)
#define XSDIAUD_SOFT_RST_SCLK_MASK	BIT(1)
#define XSDIAUD_VER_MASK		GENMASK(7, 0)

#define XSDIAUD_EMB_VID_CNT_STD_MASK	GENMASK(4, 0)
#define XSDIAUD_EMB_VID_CNT_ELE_SHIFT	(5)
#define XSDIAUD_EMB_VID_CNT_ELE_MASK	BIT(5)
#define XSDIAUD_EMB_AUD_CNT_SR_MASK	GENMASK(2, 0)
#define XSDIAUD_EMB_AUD_CNT_SS_SHIFT	(3)
#define XSDIAUD_EMB_AUD_CNT_SS_MASK	BIT(3)
#define XSDIAUD_EMB_AXIS_CHCOUNT_MASK	GENMASK(4, 0)
#define XSDIAUD_EMD_MUX_CNT_GS_MASK	GENMASK(1, 0)

#define XSDIAUD_GRP_PRESNT_MASK		GENMASK(3, 0)
#define XSDIAUD_GRP_PRESNTV_MASK	BIT(4)

#define XSDIAUD_INT_EN_GRP_CHG_MASK	BIT(0)
#define XSDIAUD_EXT_INT_EN_PKT_CHG_MASK	BIT(1)
#define XSDIAUD_EXT_INT_EN_STS_CHG_MASK	BIT(2)
#define XSDIAUD_EXT_INT_EN_FIFO_OF_MASK	BIT(3)
#define XSDIAUD_EXT_INT_EN_PERR_MASK	BIT(4)
#define XSDIAUD_EXT_INT_EN_CERR_MASK	BIT(5)
#define XSDIAUD_INT_ST_GRP_CHG_MASK	BIT(0)
#define XSDIAUD_EXT_INT_ST_PKT_CHG_MASK	BIT(1)
#define XSDIAUD_EXT_INT_ST_STS_CHG_MASK	BIT(2)
#define XSDIAUD_EXT_INT_ST_FIFO_OF_MASK	BIT(3)
#define XSDIAUD_EXT_INT_ST_PERR_MASK	BIT(4)
#define XSDIAUD_EXT_INT_ST_CERR_MASK	BIT(5)
#define XSDIAUD_EXT_AUD_CNT_CP_EN_MASK	BIT(0)
#define XSDIAUD_EXT_AXIS_CHCOUNT_MASK	GENMASK(4, 0)
#define XSDIAUD_EXT_DMUX_GRPS_MASK	GENMASK(1, 0)
#define XSDIAUD_EXT_DMUX_MUTEALL_MASK	GENMASK(5, 2)
#define XSDIAUD_EXT_DMUX_MUTE1_MASK	BIT(2)
#define XSDIAUD_EXT_DMUX_MUTE2_MASK	BIT(3)
#define XSDIAUD_EXT_PKTST_AC_MASK	GENMASK(27, 12)

/* audio params macros */
#define PROF_SAMPLERATE_MASK		GENMASK(7, 6)
#define PROF_SAMPLERATE_SHIFT		6
#define PROF_CHANNEL_COUNT_MASK		GENMASK(11, 8)
#define PROF_CHANNEL_COUNT_SHIFT	8
#define PROF_MAX_BITDEPTH_MASK		GENMASK(18, 16)
#define PROF_MAX_BITDEPTH_SHIFT		16
#define PROF_BITDEPTH_MASK		GENMASK(21, 19)
#define PROF_BITDEPTH_SHIFT		19

#define AES_FORMAT_MASK			BIT(0)
#define PROF_SAMPLERATE_UNDEFINED	0
#define PROF_SAMPLERATE_44100		1
#define PROF_SAMPLERATE_48000		2
#define PROF_SAMPLERATE_32000		3
#define PROF_CHANNELS_UNDEFINED		0
#define PROF_TWO_CHANNELS		8
#define PROF_STEREO_CHANNELS		2
#define PROF_MAX_BITDEPTH_UNDEFINED	0
#define PROF_MAX_BITDEPTH_20		2
#define PROF_MAX_BITDEPTH_24		4

#define CON_SAMPLE_RATE_MASK		GENMASK(27, 24)
#define CON_SAMPLE_RATE_SHIFT		24
#define CON_CHANNEL_COUNT_MASK		GENMASK(23, 20)
#define CON_CHANNEL_COUNT_SHIFT		20
#define CON_MAX_BITDEPTH_MASK		BIT(1)
#define CON_BITDEPTH_MASK		GENMASK(3, 1)
#define CON_BITDEPTH_SHIFT		0x1

#define CON_SAMPLERATE_44100		0
#define CON_SAMPLERATE_48000		2
#define CON_SAMPLERATE_32000		3

enum IP_MODE {
	EMBED,
	EXTRACT,
};

/**
 * enum sdi_audio_samplerate - audio sampling rate
 * @XSDIAUD_SAMPRATE0:	48 KHz
 * @XSDIAUD_SAMPRATE1:	44.1 KHz
 * @XSDIAUD_SAMPRATE2:	32 KHz
 */
enum sdi_audio_samplerate {
	XSDIAUD_SAMPRATE0,
	XSDIAUD_SAMPRATE1,
	XSDIAUD_SAMPRATE2
};

/**
 * enum sdi_audio_samplesize - bits per sample
 * @XSDIAUD_SAMPSIZE0:	20 Bit Audio Sample
 * @XSDIAUD_SAMPSIZE1:	24 Bit Audio Sample
 */
enum sdi_audio_samplesize {
	XSDIAUD_SAMPSIZE0,
	XSDIAUD_SAMPSIZE1
};

enum audio_group {
	XSDIAUD_GROUP_0 = 0,
	XSDIAUD_GROUP_1,
	XSDIAUD_GROUP_2,
	XSDIAUD_GROUP_1_2,
	XSDIAUD_GROUP_3,
	XSDIAUD_GROUP_1_3,
	XSDIAUD_GROUP_2_3,
	XSDIAUD_GROUP_1_2_3,
	XSDIAUD_GROUP_4,
	XSDIAUD_GROUP_1_4,
	XSDIAUD_GROUP_2_4,
	XSDIAUD_GROUP_1_2_4,
	XSDIAUD_GROUP_3_4,
	XSDIAUD_GROUP_1_3_4,
	XSDIAUD_GROUP_2_3_4,
	XSDIAUD_GROUP_ALL,
	XSDIAUD_NUM_CHANNELS
};

enum audio_group_num {
	XSDIAUD_GROUP1 = 1,
	XSDIAUD_GROUP2,
	XSDIAUD_GROUP3,
	XSDIAUD_GROUP4
};

/**
 * struct audio_params - audio stream parameters
 * @srate: sampling rate
 * @sig_bits: significant bits in container
 * @channels: number of channels
 */
struct audio_params {
	u32 srate;
	u32 sig_bits;
	u32 channels;
};

struct dev_ctx {
	enum IP_MODE mode;
	void __iomem *base;
	struct device *dev;
	struct audio_params *params;
	struct drm_display_mode *video_mode;
	struct snd_pcm_substream *stream;
};

/**
 * struct xsdi_aud_videostd - video properties
 * @vdisplay: resolution(vertical)
 * @vrefresh: refresh rate
 */
struct xsdi_aud_videostd {
	u32 vdisplay;
	u32 vrefresh[5];
};

/*
 * programmable values for a given vdisplay and refresh combination
 * Ex: To send/embed 48khz audio on 1080p@60 (1920x1080p => 1125 lines):
 * number of audio samples = 48000 / (60 * 1125). Audio embed block maps
 * video properties to index valus in the below table to program audio block.
 */
static const struct xsdi_aud_videostd xsdi_aud_videostd_table[] = {
	/* vdisplay 24 25 30 50 60 */
	{720,  {12, 11, 10, 9, 7} },
	{1080, {6, 5, 4, 14, 13} },
	{2160, {19, 20, 22, 24, 26} },
};

static void audio_enable(void __iomem *aud_base)
{
	u32 val;

	val = readl(aud_base + XSDIAUD_CNTRL_REG_OFFSET);
	val |= XSDIAUD_CNTRL_EN_MASK;
	writel(val, aud_base + XSDIAUD_CNTRL_REG_OFFSET);
}

static void audio_disable(void __iomem *aud_base)
{
	u32 val;

	val = readl(aud_base + XSDIAUD_CNTRL_REG_OFFSET);
	val &= ~XSDIAUD_CNTRL_EN_MASK;
	writel(val, aud_base + XSDIAUD_CNTRL_REG_OFFSET);
}

static void audio_reset_core(void __iomem *aud_base, bool reset)
{
	u32 val;

	if (reset) {
		/* reset the core */
		val = readl(aud_base + XSDIAUD_SOFT_RST_REG_OFFSET);
		val |= XSDIAUD_SOFT_RST_SCLK_MASK;
		writel(val, aud_base + XSDIAUD_SOFT_RST_REG_OFFSET);
	} else {
		/* bring the core out of reset */
		val = readl(aud_base + XSDIAUD_SOFT_RST_REG_OFFSET);
		val &= ~XSDIAUD_SOFT_RST_SCLK_MASK;
		writel(val, aud_base + XSDIAUD_SOFT_RST_REG_OFFSET);
	}
}

static int vdisplay_to_index(u32 vrefresh)
{
	int idx;

	switch (vrefresh) {
	case 24:
		idx = 0;
		break;
	case 25:
		idx = 1;
		break;
	case 30:
		idx = 2;
		break;
	case 50:
		idx = 3;
		break;
	case 60:
		idx = 4;
		break;
	default:
		idx = -1;
		break;
	}

	return idx;
}

static void audio_rx_irq_enable(void __iomem *aud_base, bool enable)
{
	u32 val;

	val = readl(aud_base + XSDIAUD_INT_EN_REG_OFFSET);
	if (enable)
		val |= XSDIAUD_INT_EN_GRP_CHG_MASK;
	else
		val &= ~XSDIAUD_INT_EN_GRP_CHG_MASK;

	writel(val, aud_base + XSDIAUD_INT_EN_REG_OFFSET);
}

/*
 * Audio channels is received in groups. Each group can hold max 4 channels.
 * Number of channels and the group in which they are present in Rx stream,
 * is detected earlier to this call. This function need to mark the group
 * again with the number of active channels. If channels exceed 4,
 * next consecutive group is marked.
 */
static void audio_set_channels(void __iomem *aud_base,
			       enum audio_group group_num, u16 num_ch)
{
	u32 i, offset, val, num_grp;

	writel(num_ch, aud_base + XSDIAUD_AXIS_CHCOUNT_REG_OFFSET);

	num_grp = num_ch / 4;
	group_num = group_num - 1;

	for (i = 0; i < num_grp; i++) {
		offset = XSDIAUD_MUX1_OR_DMUX1_CNTRL_REG_OFFSET + (4 * i);
		val = readl(aud_base + offset);
		val &= ~XSDIAUD_EMD_MUX_CNT_GS_MASK;
		val |= group_num;
		group_num = group_num + 1;
		writel(val, aud_base + offset);
	}
}

static irqreturn_t xtract_irq_handler(int irq, void *dev_id)
{
	u16 num_ch;
	u32 val, audio_groups;

	struct dev_ctx *ctx = dev_id;

	val = readl(ctx->base + XSDIAUD_INT_STS_REG_OFFSET);
	val &= XSDIAUD_INT_EN_GRP_CHG_MASK;
	if (!val)
		return IRQ_NONE;

	/* TODO: handle other interrupt types */
	writel(XSDIAUD_INT_EN_GRP_CHG_MASK,
	       ctx->base + XSDIAUD_INT_STS_REG_OFFSET);

	audio_reset_core(ctx->base, true);

	val = readl(ctx->base + XSDIAUD_EXT_CNTRL_PKTSTAT_REG_OFFSET);
	val = val & XSDIAUD_EXT_PKTST_AC_MASK;
	num_ch = hweight32(val);

	val = readl(ctx->base + XSDIAUD_AUD_CNTRL_REG_OFFSET);
	val |= XSDIAUD_EXT_AUD_CNT_CP_EN_MASK;
	writel(val, ctx->base + XSDIAUD_AUD_CNTRL_REG_OFFSET);

	audio_groups = readl(ctx->base + XSDIAUD_GRP_PRES_REG_OFFSET);
	audio_groups &= XSDIAUD_GRP_PRESNT_MASK;

	audio_reset_core(ctx->base, false);
	dev_info(ctx->dev, "detected audio groups = %d num channels = %d\n",
		 audio_groups, num_ch);
	if (num_ch > 2)
		dev_info(ctx->dev,
			 "Receiving more channels, but only 2 are extracted\n");

	/* TODO: support more channels later, currently only 2 */
	audio_set_channels(ctx->base, XSDIAUD_GROUP1, 2);
	return IRQ_HANDLED;
}

static struct audio_params *parse_professional_format(u32 reg1_val,
						      u32 reg2_val)
{
	u32 padded, val;
	struct audio_params *params;

	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params)
		return NULL;

	val = (reg1_val & PROF_SAMPLERATE_MASK) >> PROF_SAMPLERATE_SHIFT;
	switch (val) {
	case PROF_SAMPLERATE_44100:
		params->srate = 44100;
		break;
	case PROF_SAMPLERATE_48000:
		params->srate = 48000;
		break;
	case PROF_SAMPLERATE_32000:
		params->srate = 32000;
		break;
	case PROF_SAMPLERATE_UNDEFINED:
	default:
		/* not indicated */
		kfree(params);
		return NULL;
	}

	val = (reg1_val & PROF_CHANNEL_COUNT_MASK) >> PROF_CHANNEL_COUNT_SHIFT;
	switch (val) {
	case PROF_CHANNELS_UNDEFINED:
	case PROF_STEREO_CHANNELS:
	case PROF_TWO_CHANNELS:
		params->channels = 2;
		break;
	default:
		/* TODO: handle more channels in future*/
		kfree(params);
		return NULL;
	}

	val = (reg1_val & PROF_MAX_BITDEPTH_MASK) >> PROF_MAX_BITDEPTH_SHIFT;
	switch (val) {
	case PROF_MAX_BITDEPTH_UNDEFINED:
	case PROF_MAX_BITDEPTH_20:
		padded = 0;
		break;
	case PROF_MAX_BITDEPTH_24:
		padded = 4;
		break;
	default:
		/* user defined values are not supported */
		kfree(params);
		return NULL;
	}

	val = (reg1_val & PROF_BITDEPTH_MASK) >> PROF_BITDEPTH_SHIFT;
	switch (val) {
	case 1:
		params->sig_bits = 16 + padded;
		break;
	case 2:
		params->sig_bits = 18 + padded;
		break;
	case 4:
		params->sig_bits = 19 + padded;
		break;
	case 5:
		params->sig_bits = 20 + padded;
		break;
	case 6:
		params->sig_bits = 17 + padded;
		break;
	case 0:
	default:
		kfree(params);
		return NULL;
	}

	return params;
}

static struct audio_params *parse_consumer_format(u32 reg1_val, u32 reg2_val)
{
	u32 padded, val;
	struct audio_params *params;

	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params)
		return NULL;

	val = (reg1_val & CON_SAMPLE_RATE_MASK) >> CON_SAMPLE_RATE_SHIFT;
	switch (val) {
	case CON_SAMPLERATE_44100:
		params->srate = 44100;
		break;
	case CON_SAMPLERATE_48000:
		params->srate = 48000;
		break;
	case CON_SAMPLERATE_32000:
		params->srate = 32000;
		break;
	default:
		kfree(params);
		return NULL;
	}

	val = (reg1_val & CON_CHANNEL_COUNT_MASK) >> CON_CHANNEL_COUNT_SHIFT;
	params->channels = val;

	if (reg2_val & CON_MAX_BITDEPTH_MASK)
		padded = 4;
	else
		padded = 0;

	val = (reg2_val & CON_BITDEPTH_MASK) >> CON_BITDEPTH_SHIFT;
	switch (val) {
	case 1:
		params->sig_bits = 16 + padded;
		break;
	case 2:
		params->sig_bits = 18 + padded;
		break;
	case 4:
		params->sig_bits = 19 + padded;
		break;
	case 5:
		params->sig_bits = 20 + padded;
		break;
	case 6:
		params->sig_bits = 17 + padded;
		break;
	case 0:
	default:
		kfree(params);
		return NULL;
	}

	return params;
}

static int xlnx_sdi_rx_pcm_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	int err;
	u32 reg1_val, reg2_val;

	struct snd_pcm_runtime *rtd = substream->runtime;
	struct dev_ctx *ctx = dev_get_drvdata(dai->dev);
	void __iomem *base = ctx->base;

	reg1_val = readl(base + XSDIAUD_EXT_CH_STAT0_REG_OFFSET);
	reg2_val = readl(base + XSDIAUD_EXT_CH_STAT0_REG_OFFSET + 4);
	if (reg1_val & AES_FORMAT_MASK)
		ctx->params = parse_professional_format(reg1_val, reg2_val);
	else
		ctx->params = parse_consumer_format(reg1_val, reg2_val);

	if (!ctx->params)
		return -EINVAL;

	dev_info(ctx->dev,
		 "Audio properties: srate %d sig_bits = %d channels = %d\n",
		ctx->params->srate, ctx->params->sig_bits,
		ctx->params->channels);

	err = snd_pcm_hw_constraint_minmax(rtd, SNDRV_PCM_HW_PARAM_RATE,
					   ctx->params->srate,
					   ctx->params->srate);

	if (err < 0) {
		dev_err(ctx->dev, "failed to constrain samplerate to %dHz\n",
			ctx->params->srate);
		kfree(ctx->params);
		return err;
	}

	/*
	 * During record, after AES bits(8) are removed, pcm is at max 24bits.
	 * Out of 24 bits, sig_bits represent valid number of audio bits from
	 * input stream.
	 */
	err = snd_pcm_hw_constraint_msbits(rtd, 0, 24, ctx->params->sig_bits);

	if (err < 0) {
		dev_err(ctx->dev,
			"failed to constrain 'bits per sample' %d bits\n",
			ctx->params->sig_bits);
		kfree(ctx->params);
		return err;
	}

	err = snd_pcm_hw_constraint_minmax(rtd, SNDRV_PCM_HW_PARAM_CHANNELS,
					   ctx->params->channels,
					   ctx->params->channels);
	if (err < 0) {
		dev_err(ctx->dev,
			"failed to constrain channel count to %d\n",
			ctx->params->channels);
		kfree(ctx->params);
		return err;
	}

	dev_info(ctx->dev, " sdi rx audio enabled\n");
	return 0;
}

static void xlnx_sdi_rx_pcm_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct dev_ctx *ctx = dev_get_drvdata(dai->dev);

	kfree(ctx->params);
	dev_info(dai->dev, " sdi rx audio disabled\n");
}

static int xlnx_sdi_tx_pcm_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct dev_ctx *ctx = dev_get_drvdata(dai->dev);
	void __iomem *base = ctx->base;

	audio_enable(base);
	ctx->stream = substream;

	dev_info(ctx->dev, " sdi tx audio enabled\n");
	return 0;
}

static int xlnx_sdi_tx_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	int i;
	u32 num_channels, sample_rate, sig_bits, sample_size, srate;
	u32 val, vid_table_size, idx;
	struct xsdi_aud_videostd const *item;
	u32 vid_std = 0;

	struct dev_ctx *ctx = dev_get_drvdata(dai->dev);
	void __iomem *base = ctx->base;

	/* video mode properties needed by audio driver are shared to audio
	 * driver through a pointer in platform data. This is used here in
	 * audio driver. The solution may be needed to modify/extend to avoid
	 * probable error scenarios
	 */
	if (!ctx->video_mode || !ctx->video_mode->vdisplay ||
	    !ctx->video_mode->vrefresh) {
		dev_err(ctx->dev, "couldn't find video display properties\n");
		return -EINVAL;
	}

	/* map video properties to properties in audio ip */
	vid_table_size = ARRAY_SIZE(xsdi_aud_videostd_table);
	for (i = 0; i < vid_table_size; i++) {
		item = &xsdi_aud_videostd_table[i];
		if (item->vdisplay == ctx->video_mode->vdisplay) {
			idx = vdisplay_to_index(ctx->video_mode->vrefresh);
			if (idx >= 0)
				vid_std = item->vrefresh[idx];
			break;
		}
	}

	if (!vid_std) {
		dev_err(ctx->dev, "couldn't map video properties to audio\n");
		return -EINVAL;
	}

	val = readl(base + XSDIAUD_EMB_VID_CNTRL_REG_OFFSET);
	val &= ~XSDIAUD_EMB_VID_CNT_STD_MASK;
	val |= vid_std;
	writel(val, base + XSDIAUD_EMB_VID_CNTRL_REG_OFFSET);

	num_channels = params_channels(params);
	sample_rate = params_rate(params);
	sig_bits = snd_pcm_format_width(params_format(params));

	dev_info(ctx->dev,
		 "stream params: channels = %d sample_rate = %d bits = %d\n",
		 num_channels, sample_rate, sig_bits);

	switch (sample_rate) {
	case 44100:
		srate = XSDIAUD_SAMPRATE1;
		break;
	case 32000:
		srate = XSDIAUD_SAMPRATE2;
		break;
	case 48000:
	default:
		srate = XSDIAUD_SAMPRATE0;
		break;
	}

	/* TODO: support more channels; currently only 2 */
	audio_set_channels(base, XSDIAUD_GROUP1, num_channels);

	val = readl(base +  XSDIAUD_AUD_CNTRL_REG_OFFSET);
	val &= ~XSDIAUD_EMB_AUD_CNT_SR_MASK;
	val |= srate;
	writel(val, base + XSDIAUD_AUD_CNTRL_REG_OFFSET);

	if (sig_bits == 24)
		sample_size = XSDIAUD_SAMPSIZE1;
	else
		sample_size = XSDIAUD_SAMPSIZE0;

	val = readl(base +  XSDIAUD_AUD_CNTRL_REG_OFFSET);
	val &= ~XSDIAUD_EMB_AUD_CNT_SS_MASK;
	sample_size = sample_size << XSDIAUD_EMB_AUD_CNT_SS_SHIFT;
	val |= sample_size;
	writel(val, base + XSDIAUD_AUD_CNTRL_REG_OFFSET);

	val = readl(base + XSDIAUD_EMB_VID_CNTRL_REG_OFFSET);
	val |= XSDIAUD_EMB_VID_CNT_ELE_MASK;
	writel(val, base + XSDIAUD_EMB_VID_CNTRL_REG_OFFSET);

	return 0;
}

static void xlnx_sdi_tx_pcm_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct dev_ctx *ctx = dev_get_drvdata(dai->dev);
	void __iomem *base = ctx->base;

	audio_disable(base);
	ctx->stream = NULL;

	dev_info(ctx->dev, " sdi tx audio disabled\n");
}

static const struct snd_soc_component_driver xlnx_sdi_component = {
	.name = "xlnx-sdi-dai-component",
};

static const struct snd_soc_dai_ops xlnx_sdi_rx_dai_ops = {
	.startup = xlnx_sdi_rx_pcm_startup,
	.shutdown = xlnx_sdi_rx_pcm_shutdown,
};

static struct snd_soc_dai_driver xlnx_sdi_rx_dai = {
	.name = "xlnx_sdi_rx",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &xlnx_sdi_rx_dai_ops,
};

static const struct snd_soc_dai_ops xlnx_sdi_tx_dai_ops = {
	.startup =	xlnx_sdi_tx_pcm_startup,
	.hw_params =	xlnx_sdi_tx_hw_params,
	.shutdown =	xlnx_sdi_tx_pcm_shutdown,
};

static struct snd_soc_dai_driver xlnx_sdi_tx_dai = {
	.name = "xlnx_sdi_tx",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &xlnx_sdi_tx_dai_ops,
};

static int xlnx_sdi_audio_probe(struct platform_device *pdev)
{
	u32 val;
	int ret;
	struct dev_ctx *ctx;
	struct resource *res;
	struct device *video_dev;
	struct device_node *video_node;
	struct platform_device *video_pdev;
	struct snd_soc_dai_driver *snd_dai;

	ctx = devm_kzalloc(&pdev->dev, sizeof(struct dev_ctx), GFP_KERNEL);
	if (!ctx)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No IO MEM resource found\n");
		return -ENODEV;
	}

	ctx->base = devm_ioremap_resource(&pdev->dev, res);
	if (!ctx->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -EADDRNOTAVAIL;
	}

	ctx->dev = &pdev->dev;

	val = readl(ctx->base + XSDIAUD_GUI_PARAM_REG_OFFSET);
	if (val & BIT(6)) {
		ctx->mode = EXTRACT;
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (!res) {
			dev_err(&pdev->dev, "No IRQ resource found\n");
			return -ENODEV;
		}
		ret = devm_request_irq(&pdev->dev, res->start,
				       xtract_irq_handler,
				       0, "XLNX_SDI_AUDIO_XTRACT", ctx);
		if (ret) {
			dev_err(&pdev->dev, "extract irq request failed\n");
			return -ENODEV;
		}

		snd_dai = &xlnx_sdi_rx_dai;
	} else {
		ctx->mode = EMBED;
		video_node = of_parse_phandle(pdev->dev.of_node,
					      "xlnx,sdi-tx-video", 0);
		if (!video_node) {
			dev_err(ctx->dev, "video_node not found\n");
			of_node_put(video_node);
			return -ENODEV;
		}

		video_pdev = of_find_device_by_node(video_node);
		if (!video_pdev) {
			of_node_put(video_node);
			return -ENODEV;
		}

		video_dev = &video_pdev->dev;
		ctx->video_mode =
			(struct drm_display_mode *)video_dev->platform_data;
		/* invalid 'platform_data' implies video driver is not loaded */
		if (!ctx->video_mode) {
			of_node_put(video_node);
			return -EPROBE_DEFER;
		}

		snd_dai = &xlnx_sdi_tx_dai;
		of_node_put(video_node);
	}

	ret = devm_snd_soc_register_component(&pdev->dev, &xlnx_sdi_component,
					      snd_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "couldn't register codec DAI\n");
		return ret;
	}

	dev_set_drvdata(&pdev->dev, ctx);

	audio_reset_core(ctx->base, true);
	audio_reset_core(ctx->base, false);
	audio_enable(ctx->base);

	if (ctx->mode == EXTRACT)
		audio_rx_irq_enable(ctx->base, true);

	dev_info(&pdev->dev, "xlnx sdi codec dai component registered\n");
	return 0;
}

static int xlnx_sdi_audio_remove(struct platform_device *pdev)
{
	struct dev_ctx *ctx = dev_get_drvdata(&pdev->dev);

	audio_disable(ctx->base);
	audio_reset_core(ctx->base, true);

	return 0;
}

static const struct of_device_id xlnx_sdi_audio_of_match[] = {
	{ .compatible = "xlnx,v-uhdsdi-audio-1.0"},
	{ }
};
MODULE_DEVICE_TABLE(of, xlnx_sdi_audio_of_match);

static struct platform_driver xlnx_sdi_audio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = xlnx_sdi_audio_of_match,
	},
	.probe = xlnx_sdi_audio_probe,
	.remove = xlnx_sdi_audio_remove,
};

module_platform_driver(xlnx_sdi_audio_driver);

MODULE_DESCRIPTION("xilinx sdi audio codec driver");
MODULE_AUTHOR("Maruthi Srinivas Bayyavarapu");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
