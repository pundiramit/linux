// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019, Linaro Limited

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_clk.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/slimbus.h>
#include <linux/kernel.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "wcd934x.h"
#include "wcd-clsh-v2.h"

#define WCD934X_RATES_MASK (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			    SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)
/* Fractional Rates */
#define WCD934X_FRAC_RATES_MASK (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
				 SNDRV_PCM_RATE_176400)
#define WCD934X_FORMATS_S16_S24_LE (SNDRV_PCM_FMTBIT_S16_LE | \
				    SNDRV_PCM_FMTBIT_S24_LE)

/* slave port water mark level
 *   (0: 6bytes, 1: 9bytes, 2: 12 bytes, 3: 15 bytes)
 */
#define SLAVE_PORT_WATER_MARK_6BYTES  0
#define SLAVE_PORT_WATER_MARK_9BYTES  1
#define SLAVE_PORT_WATER_MARK_12BYTES 2
#define SLAVE_PORT_WATER_MARK_15BYTES 3
#define SLAVE_PORT_WATER_MARK_SHIFT 1
#define SLAVE_PORT_ENABLE           1
#define SLAVE_PORT_DISABLE          0
#define WCD934X_SLIM_WATER_MARK_VAL \
	((SLAVE_PORT_WATER_MARK_12BYTES << SLAVE_PORT_WATER_MARK_SHIFT) | \
	 (SLAVE_PORT_ENABLE))

#define WCD934X_SLIM_NUM_PORT_REG 3
#define WCD934X_SLIM_PGD_PORT_INT_TX_EN0 (WCD934X_SLIM_PGD_PORT_INT_EN0 + 2)
#define WCD934X_SLIM_IRQ_OVERFLOW BIT(0)
#define WCD934X_SLIM_IRQ_UNDERFLOW BIT(1)
#define WCD934X_SLIM_IRQ_PORT_CLOSED BIT(2)

#define WCD934X_MCLK_CLK_12P288MHZ	12288000
#define WCD934X_MCLK_CLK_9P6MHZ		9600000

/* Only valid for 9.6 MHz mclk */
#define WCD9XXX_DMIC_SAMPLE_RATE_2P4MHZ 2400000
#define WCD9XXX_DMIC_SAMPLE_RATE_4P8MHZ 4800000

/* Only valid for 12.288 MHz mclk */
#define WCD9XXX_DMIC_SAMPLE_RATE_4P096MHZ 4096000

#define WCD934X_DMIC_CLK_DIV_2  0x0
#define WCD934X_DMIC_CLK_DIV_3  0x1
#define WCD934X_DMIC_CLK_DIV_4  0x2
#define WCD934X_DMIC_CLK_DIV_6  0x3
#define WCD934X_DMIC_CLK_DIV_8  0x4
#define WCD934X_DMIC_CLK_DIV_16  0x5
#define WCD934X_DMIC_CLK_DRIVE_DEFAULT 0x02

#define TX_HPF_CUT_OFF_FREQ_MASK	0x60
#define CF_MIN_3DB_4HZ			0x0
#define CF_MIN_3DB_75HZ		0x1
#define CF_MIN_3DB_150HZ		0x2

#define WCD934X_RX_START	16
#define WCD934X_NUM_INTERPOLATORS 9
#define WCD934X_RX_PATH_CTL_OFFSET 20
#define WCD934X_MAX_VALID_ADC_MUX  13
#define WCD934X_INVALID_ADC_MUX 9

#define WCD934X_SLIM_RX_CH(p) \
	{.port = p + WCD934X_RX_START, .shift = p,}

#define WCD934X_SLIM_TX_CH(p) \
	{.port = p, .shift = p,}

/* Feature masks to distinguish codec version */
#define DSD_DISABLED_MASK   0
#define SLNQ_DISABLED_MASK  1

#define DSD_DISABLED   BIT(DSD_DISABLED_MASK)
#define SLNQ_DISABLED  BIT(SLNQ_DISABLED_MASK)

/* As fine version info cannot be retrieved before wcd probe.
 * Define three coarse versions for possible future use before wcd probe.
 */
#define WCD_VERSION_1_0             0
#define WCD_VERSION_1_1             1
#define WCD_VERSION_WCD9340_1_0     2
#define WCD_VERSION_WCD9341_1_0     3
#define WCD_VERSION_WCD9340_1_1     4
#define WCD_VERSION_WCD9341_1_1     5
#define WCD_IS_1_0(wcd) \
	((wcd->type == WCD934X) ? \
	 ((wcd->version == WCD_VERSION_1_0 || \
	   wcd->version == WCD_VERSION_WCD9340_1_0 || \
	   wcd->version == WCD_VERSION_WCD9341_1_0) ? 1 : 0) : 0)
#define WCD_IS_1_1(wcd) \
	((wcd->type == WCD934X) ? \
	 ((wcd->version == WCD_VERSION_1_1 || \
	   wcd->version == WCD_VERSION_WCD9340_1_1 || \
	   wcd->version == WCD_VERSION_WCD9341_1_1) ? 1 : 0) : 0)
#define WCD_IS_WCD9340_1_0(wcd) \
	((wcd->type == WCD934X) ? \
	 ((wcd->version == WCD_VERSION_WCD9340_1_0) ? 1 : 0) : 0)
#define WCD_IS_WCD9341_1_0(wcd) \
	((wcd->type == WCD934X) ? \
	 ((wcd->version == WCD_VERSION_WCD9341_1_0) ? 1 : 0) : 0)
#define WCD_IS_WCD9340_1_1(wcd) \
	((wcd->type == WCD934X) ? \
	 ((wcd->version == WCD_VERSION_WCD9340_1_1) ? 1 : 0) : 0)
#define WCD_IS_WCD9341_1_1(wcd) \
	((wcd->type == WCD934X) ? \
	 ((wcd->version == WCD_VERSION_WCD9341_1_1) ? 1 : 0) : 0)

#define WCD934X_AMIC_PWR_LEVEL_LP 0
#define WCD934X_AMIC_PWR_LEVEL_DEFAULT 1
#define WCD934X_AMIC_PWR_LEVEL_HP 2
#define WCD934X_AMIC_PWR_LEVEL_HYBRID 3
#define WCD934X_AMIC_PWR_LVL_MASK 0x60
#define WCD934X_AMIC_PWR_LVL_SHIFT 0x5

#define WCD934X_DEC_PWR_LVL_MASK 0x06
#define WCD934X_DEC_PWR_LVL_LP 0x02
#define WCD934X_DEC_PWR_LVL_HP 0x04
#define WCD934X_DEC_PWR_LVL_DF 0x00
#define WCD934X_DEC_PWR_LVL_HYBRID WCD934X_DEC_PWR_LVL_DF

#define WCD934X_MAX_MICBIAS 4
#define WCD934X_DEF_MICBIAS_MV	1800
#define WCD934X_MAX_MICBIAS_MV	2850

#define WCD_IIR_FILTER_SIZE	(sizeof(u32) * BAND_MAX)

#define WCD_IIR_FILTER_CTL(xname, iidx, bidx) \
{ \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = wcd934x_iir_filter_info, \
	.get = wcd934x_get_iir_band_audio_mixer, \
	.put = wcd934x_put_iir_band_audio_mixer, \
	.private_value = (unsigned long)&(struct wcd_iir_filter_ctl) { \
		.iir_idx = iidx, \
		.band_idx = bidx, \
		.bytes_ext = {.max = WCD_IIR_FILTER_SIZE, }, \
	} \
}

enum {
	SIDO_SOURCE_INTERNAL,
	SIDO_SOURCE_RCO_BG,
};

enum {
	INTERP_EAR = 0,
	INTERP_HPHL,
	INTERP_HPHR,
	INTERP_LO1,
	INTERP_LO2,
	INTERP_LO3_NA, /* LO3 not avalible in Tavil */
	INTERP_LO4_NA,
	INTERP_SPKR1, /*INT7 WSA Speakers via soundwire */
	INTERP_SPKR2, /*INT8 WSA Speakers via soundwire */
	INTERP_MAX,
};

enum {
	WCD934X_RX0 = 0,
	WCD934X_RX1,
	WCD934X_RX2,
	WCD934X_RX3,
	WCD934X_RX4,
	WCD934X_RX5,
	WCD934X_RX6,
	WCD934X_RX7,
	WCD934X_RX8,
	WCD934X_RX9,
	WCD934X_RX10,
	WCD934X_RX11,
	WCD934X_RX12,
	WCD934X_RX_MAX,
};

enum {
	WCD934X_TX0 = 0,
	WCD934X_TX1,
	WCD934X_TX2,
	WCD934X_TX3,
	WCD934X_TX4,
	WCD934X_TX5,
	WCD934X_TX6,
	WCD934X_TX7,
	WCD934X_TX8,
	WCD934X_TX9,
	WCD934X_TX10,
	WCD934X_TX11,
	WCD934X_TX12,
	WCD934X_TX13,
	WCD934X_TX14,
	WCD934X_TX15,
	WCD934X_TX_MAX,
};

struct wcd934x_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

struct wcd934x_slim_ch {
	u32 ch_num;
	u16 port;
	u16 shift;
	struct list_head list;
};

static const struct wcd934x_slim_ch wcd934x_tx_chs[WCD934X_TX_MAX] = {
	WCD934X_SLIM_TX_CH(0),
	WCD934X_SLIM_TX_CH(1),
	WCD934X_SLIM_TX_CH(2),
	WCD934X_SLIM_TX_CH(3),
	WCD934X_SLIM_TX_CH(4),
	WCD934X_SLIM_TX_CH(5),
	WCD934X_SLIM_TX_CH(6),
	WCD934X_SLIM_TX_CH(7),
	WCD934X_SLIM_TX_CH(8),
	WCD934X_SLIM_TX_CH(9),
	WCD934X_SLIM_TX_CH(10),
	WCD934X_SLIM_TX_CH(11),
	WCD934X_SLIM_TX_CH(12),
	WCD934X_SLIM_TX_CH(13),
	WCD934X_SLIM_TX_CH(14),
	WCD934X_SLIM_TX_CH(15),
};

static const struct wcd934x_slim_ch wcd934x_rx_chs[WCD934X_RX_MAX] = {
	WCD934X_SLIM_RX_CH(0),	 /* 16 */
	WCD934X_SLIM_RX_CH(1),	 /* 17 */
	WCD934X_SLIM_RX_CH(2),
	WCD934X_SLIM_RX_CH(3),
	WCD934X_SLIM_RX_CH(4),
	WCD934X_SLIM_RX_CH(5),
	WCD934X_SLIM_RX_CH(6),
	WCD934X_SLIM_RX_CH(7),
	WCD934X_SLIM_RX_CH(8),
	WCD934X_SLIM_RX_CH(9),
	WCD934X_SLIM_RX_CH(10),
	WCD934X_SLIM_RX_CH(11),
	WCD934X_SLIM_RX_CH(12),
};

/* Codec supports 2 IIR filters */
enum {
	IIR0 = 0,
	IIR1,
	IIR_MAX,
};

/* Each IIR has 5 Filter Stages */
enum {
	BAND1 = 0,
	BAND2,
	BAND3,
	BAND4,
	BAND5,
	BAND_MAX,
};

enum {
	COMPANDER_1, /* HPH_L */
	COMPANDER_2, /* HPH_R */
	COMPANDER_3, /* LO1_DIFF */
	COMPANDER_4, /* LO2_DIFF */
	COMPANDER_5, /* LO3_SE - not used in Tavil */
	COMPANDER_6, /* LO4_SE - not used in Tavil */
	COMPANDER_7, /* SWR SPK CH1 */
	COMPANDER_8, /* SWR SPK CH2 */
	COMPANDER_MAX,
};

enum {
	AIF1_PB = 0,
	AIF1_CAP,
	AIF2_PB,
	AIF2_CAP,
	AIF3_PB,
	AIF3_CAP,
	AIF4_PB,
	AIF4_VIFEED,
	AIF4_MAD_TX,
	NUM_CODEC_DAIS,
};

enum {
	INTn_1_INP_SEL_ZERO = 0,
	INTn_1_INP_SEL_DEC0,
	INTn_1_INP_SEL_DEC1,
	INTn_1_INP_SEL_IIR0,
	INTn_1_INP_SEL_IIR1,
	INTn_1_INP_SEL_RX0,
	INTn_1_INP_SEL_RX1,
	INTn_1_INP_SEL_RX2,
	INTn_1_INP_SEL_RX3,
	INTn_1_INP_SEL_RX4,
	INTn_1_INP_SEL_RX5,
	INTn_1_INP_SEL_RX6,
	INTn_1_INP_SEL_RX7,
};

enum {
	INTn_2_INP_SEL_ZERO = 0,
	INTn_2_INP_SEL_RX0,
	INTn_2_INP_SEL_RX1,
	INTn_2_INP_SEL_RX2,
	INTn_2_INP_SEL_RX3,
	INTn_2_INP_SEL_RX4,
	INTn_2_INP_SEL_RX5,
	INTn_2_INP_SEL_RX6,
	INTn_2_INP_SEL_RX7,
	INTn_2_INP_SEL_PROXIMITY,
};

enum {
	INTERP_MAIN_PATH,
	INTERP_MIX_PATH,
};

struct interp_sample_rate {
	int sample_rate;
	int rate_val;
};

static struct interp_sample_rate sr_val_tbl[] = {
	{8000, 0x0},
	{16000, 0x1},
	{32000, 0x3},
	{48000, 0x4},
	{96000, 0x5},
	{192000, 0x6},
	{384000, 0x7},
	{44100, 0x9},
	{88200, 0xA},
	{176400, 0xB},
	{352800, 0xC},
};

struct wcd_slim_codec_dai_data {
	struct list_head slim_ch_list;
	struct slim_stream_config sconfig;
	struct slim_stream_runtime *sruntime;
};

#define WCD934X_MAX_SUPPLY	5
struct wcd934x_codec {
	struct device *dev;
	struct clk_hw hw;
	struct clk *extclk;
	int clk_mclk_users;
	int master_bias_users;
	int rate;
	int irq;
	int reset_gpio;
	struct regmap *regmap;
	struct regmap *if_regmap;
	struct regmap_irq_chip_data *irq_data;

	struct slim_device *slim;
	struct slim_device *slim_ifc_dev;
	struct wcd934x_slim_ch rx_chs[WCD934X_RX_MAX];
	struct wcd934x_slim_ch tx_chs[WCD934X_TX_MAX];
	struct regulator_bulk_data supplies[WCD934X_MAX_SUPPLY];
	u32 num_rx_port;
	u32 num_tx_port;
	struct snd_soc_component *component;
	struct wcd_slim_codec_dai_data dai[NUM_CODEC_DAIS];
	struct wcd_clsh_ctrl *clsh_ctrl;
	u32 hph_mode;
	u32 rx_bias_count;
	u32 version;
	int comp_enabled[COMPANDER_MAX];
	int main_clk_users[WCD934X_NUM_INTERPOLATORS];
	unsigned int rx_port_value;
	unsigned int tx_port_value;

	/*TX*/
	int micb_ref[WCD934X_MAX_MICBIAS];
	int pullup_ref[WCD934X_MAX_MICBIAS];
	int dmic_0_1_clk_cnt;
	int dmic_2_3_clk_cnt;
	int dmic_4_5_clk_cnt;
	int dmic_sample_rate;
	int mad_dmic_sample_rate;

	int power_active_ref;
	int cur_power_state;
	int sido_input_src;
	int clk_type;
};

#define to_wcd934x_codec(_hw) container_of(_hw, struct wcd934x_codec, hw)

enum wcd_clock_type {
	WCD_CLK_OFF,
	WCD_CLK_RCO,
	WCD_CLK_MCLK,
};

enum codec_power_states {
	WCD_REGION_POWER_COLLAPSE_REMOVE,
	WCD_REGION_POWER_COLLAPSE_BEGIN,
	WCD_REGION_POWER_DOWN,
};

struct wcd_iir_filter_ctl {
	unsigned int iir_idx;
	unsigned int band_idx;
	struct soc_bytes_ext bytes_ext;
};

static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 1, 0);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);
static const DECLARE_TLV_DB_SCALE(ear_pa_gain, 0, 150, 0);

/* Cutoff frequency for high pass filter */
static const char * const cf_text[] = {
	"CF_NEG_3DB_4HZ", "CF_NEG_3DB_75HZ", "CF_NEG_3DB_150HZ"
};

static const char * const rx_cf_text[] = {
	"CF_NEG_3DB_4HZ", "CF_NEG_3DB_75HZ", "CF_NEG_3DB_150HZ",
	"CF_NEG_3DB_0P48HZ"
};

static const char * const rx_hph_mode_mux_text[] = {
	"Class H Invalid", "Class-H Hi-Fi", "Class-H Low Power", "Class-AB",
	"Class-H Hi-Fi Low Power"
};

static const struct soc_enum cf_dec0_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX0_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX1_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec2_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX2_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec3_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX3_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec4_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX4_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec5_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX5_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec6_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX6_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec7_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX7_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_dec8_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_TX8_TX_PATH_CFG0, 5, 3, cf_text);

static const struct soc_enum cf_int0_1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_RX0_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int0_2_enum, WCD934X_CDC_RX0_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int1_1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_RX1_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int1_2_enum, WCD934X_CDC_RX1_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int2_1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_RX2_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int2_2_enum, WCD934X_CDC_RX2_RX_PATH_MIX_CFG, 2,
		     rx_cf_text);

static const struct soc_enum cf_int3_1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_RX3_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int3_2_enum, WCD934X_CDC_RX3_RX_PATH_MIX_CFG, 2,
			    rx_cf_text);

static const struct soc_enum cf_int4_1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_RX4_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int4_2_enum, WCD934X_CDC_RX4_RX_PATH_MIX_CFG, 2,
			    rx_cf_text);

static const struct soc_enum cf_int7_1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_RX7_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int7_2_enum, WCD934X_CDC_RX7_RX_PATH_MIX_CFG, 2,
			    rx_cf_text);

static const struct soc_enum cf_int8_1_enum =
	SOC_ENUM_SINGLE(WCD934X_CDC_RX8_RX_PATH_CFG2, 0, 4, rx_cf_text);

static SOC_ENUM_SINGLE_DECL(cf_int8_2_enum, WCD934X_CDC_RX8_RX_PATH_MIX_CFG, 2,
			    rx_cf_text);

static const struct soc_enum rx_hph_mode_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rx_hph_mode_mux_text),
			    rx_hph_mode_mux_text);
static void wcd934x_update_reg_defaults(struct wcd934x_codec *wcd)
{
	struct regmap *rm = wcd->regmap;

	regmap_update_bits(rm, WCD934X_BIAS_VBG_FINE_ADJ, 0xFF, 0x75);
	regmap_update_bits(rm, WCD934X_CODEC_CPR_SVS_CX_VDD, 0xFF, 0x7C);
	regmap_update_bits(rm, WCD934X_CODEC_CPR_SVS2_CX_VDD, 0xFF, 0x58);
	regmap_update_bits(rm, WCD934X_CDC_RX0_RX_PATH_DSMDEM_CTL, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_RX1_RX_PATH_DSMDEM_CTL, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_RX2_RX_PATH_DSMDEM_CTL, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_RX3_RX_PATH_DSMDEM_CTL, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_RX4_RX_PATH_DSMDEM_CTL, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_RX7_RX_PATH_DSMDEM_CTL, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_RX8_RX_PATH_DSMDEM_CTL, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_COMPANDER8_CTL7, 0x1E, 0x18);
	regmap_update_bits(rm, WCD934X_CDC_COMPANDER7_CTL7, 0x1E, 0x18);
	regmap_update_bits(rm, WCD934X_CDC_RX0_RX_PATH_SEC0, 0x08, 0x0);
	regmap_update_bits(rm, WCD934X_CDC_CLSH_DECAY_CTRL, 0x03, 0x0);
	regmap_update_bits(rm, WCD934X_MICB1_TEST_CTL_2, 0x07, 0x01);
	regmap_update_bits(rm, WCD934X_CDC_BOOST0_BOOST_CFG1, 0x3F, 0x12);
	regmap_update_bits(rm, WCD934X_CDC_BOOST0_BOOST_CFG2, 0x1C, 0x08);
	regmap_update_bits(rm, WCD934X_CDC_BOOST1_BOOST_CFG1, 0x3F, 0x12);
	regmap_update_bits(rm, WCD934X_CDC_BOOST1_BOOST_CFG2, 0x1C, 0x08);
	regmap_update_bits(rm, WCD934X_CPE_SS_CPARMAD_BUFRDY_INT_PERIOD,
			   0x1F, 0x09);
	regmap_update_bits(rm, WCD934X_CDC_TX0_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX1_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX2_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX3_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX4_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX5_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX6_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX7_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_CDC_TX8_TX_PATH_CFG1, 0x01, 0x00);
	regmap_update_bits(rm, WCD934X_RX_OCP_CTL, 0x0F, 0x02);
	regmap_update_bits(rm, WCD934X_HPH_OCP_CTL, 0xFF, 0x3A);
	regmap_update_bits(rm, WCD934X_HPH_L_TEST, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_HPH_R_TEST, 0x01, 0x01);
	regmap_update_bits(rm, WCD934X_CPE_FLL_CONFIG_CTL_2, 0xFF, 0x20);
	regmap_update_bits(rm, WCD934X_MBHC_NEW_CTL_2, 0x0C, 0x00);
}

static int wcd934x_dig_core_remove_power_collapse(struct wcd934x_codec *data)
{
	data->power_active_ref++;
	if (data->power_active_ref == 1 &&
	    data->cur_power_state == WCD_REGION_POWER_DOWN) {
		regmap_write(data->regmap,
			     WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x05);
		regmap_write(data->regmap,
			     WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x07);
		regmap_update_bits(data->regmap,
				   WCD934X_CODEC_RPM_RST_CTL, 0x02, 0x00);
		regmap_update_bits(data->regmap,
				   WCD934X_CODEC_RPM_RST_CTL, 0x02, 0x02);
		regmap_write(data->regmap,
			     WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x03);

		data->cur_power_state = WCD_REGION_POWER_COLLAPSE_REMOVE;
	}
	return 0;
}

static void wcd934x_codec_power_gate_digital_core(struct wcd934x_codec *data)
{
	data->power_active_ref--;

	if (data->power_active_ref > 0) {
		data->cur_power_state = WCD_REGION_POWER_COLLAPSE_BEGIN;
		regmap_update_bits(data->regmap,
			   WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x04, 0x04);
		regmap_update_bits(data->regmap,
			   WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x01, 0x00);
		regmap_update_bits(data->regmap,
			   WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x02, 0x00);
		data->cur_power_state = WCD_REGION_POWER_DOWN;
	}
}

static int wcd934x_enable_master_bias(struct wcd934x_codec *data)
{
	if (++data->master_bias_users != 1)
		return 0;

	regmap_update_bits(data->regmap, WCD934X_ANA_BIAS,   0x80, 0x80);
	regmap_update_bits(data->regmap, WCD934X_ANA_BIAS,   0x40, 0x40);
	/*
	 * 1ms delay is required after pre-charge is enabled
	 * as per HW requirement
	 */
	usleep_range(1000, 1100);
	regmap_update_bits(data->regmap, WCD934X_ANA_BIAS, 0x40, 0x00);
	regmap_update_bits(data->regmap,  WCD934X_ANA_BIAS, 0x20, 0x00);

	return 0;
}

static int wcd934x_disable_master_bias(struct wcd934x_codec *data)
{
	if (data->master_bias_users <= 0)
		return 0;

	data->master_bias_users--;

	if (data->master_bias_users == 0) {
		regmap_update_bits(data->regmap, WCD934X_ANA_BIAS,
				   0x80, 0x00);
		regmap_update_bits(data->regmap,
				   WCD934X_ANA_BIAS, 0x20, 0x00);
	}

	return 0;
}

static int wcd934x_enable_clk_mclk(struct wcd934x_codec *data)
{
	if (++data->clk_mclk_users == 1) {
		/*
		 * In data clock contrl register is changed
		 * to CLK_SYS_MCLK_PRG
		 */
		regmap_update_bits(data->regmap, WCD934X_CLK_SYS_MCLK_PRG,
				   0x80, 0x80);
		regmap_update_bits(data->regmap, WCD934X_CLK_SYS_MCLK_PRG,
				   0x30, 0x10);
		regmap_update_bits(data->regmap, WCD934X_CLK_SYS_MCLK_PRG,
				   0x02, 0x00);
		regmap_update_bits(data->regmap, WCD934X_CLK_SYS_MCLK_PRG,
				   0x01, 0x01);
		regmap_update_bits(data->regmap, WCD934X_CLK_SYS_MCLK_PRG,
				   0x02, 0x00);
		regmap_update_bits(data->regmap,
				   WCD934X_CDC_CLK_RST_CTRL_FS_CNT_CONTROL,
				   0x01, 0x01);
		regmap_update_bits(data->regmap,
				   WCD934X_CDC_CLK_RST_CTRL_MCLK_CONTROL,
				   0x01, 0x01);
		regmap_update_bits(data->regmap,
				   WCD934X_CDC_CLK_RST_CTRL_MCLK_CONTROL,
				   0x01, 0x01);
		regmap_update_bits(data->regmap, WCD934X_CODEC_RPM_CLK_GATE,
				   0x03, 0x00);
		/*
		 * 10us sleep is required after clock is enabled
		 * as per HW requirement
		 */
		usleep_range(10, 15);
	}

	data->clk_type = WCD_CLK_MCLK;

	return 0;
}

static void wcd934x_set_sido_input_src(struct wcd934x_codec *data,
				       int sido_src)
{
	if (sido_src == data->sido_input_src)
		return;

	if (sido_src == SIDO_SOURCE_INTERNAL) {
		regmap_update_bits(data->regmap, WCD934X_ANA_BUCK_CTL,
				   0x04, 0x00);
		usleep_range(100, 110);
		regmap_update_bits(data->regmap, WCD934X_ANA_BUCK_CTL,
				   0x03, 0x00);
		usleep_range(100, 110);
		regmap_update_bits(data->regmap, WCD934X_ANA_RCO,
				   0x80, 0x00);
		usleep_range(100, 110);
	} else if (sido_src == SIDO_SOURCE_RCO_BG) {
		regmap_update_bits(data->regmap, WCD934X_ANA_RCO,
				   0x80, 0x80);
		usleep_range(100, 110);
		regmap_update_bits(data->regmap, WCD934X_ANA_BUCK_CTL,
				   0x02, 0x02);
		usleep_range(100, 110);
		regmap_update_bits(data->regmap, WCD934X_ANA_BUCK_CTL,
				   0x01, 0x01);
		usleep_range(100, 110);
		regmap_update_bits(data->regmap, WCD934X_ANA_BUCK_CTL,
				   0x04, 0x04);
		usleep_range(100, 110);
	}
	data->sido_input_src = sido_src;
}

static int wcd934x_disable_clk_mclk(struct wcd934x_codec *data)
{
	if (--data->clk_mclk_users == 0) {
		regmap_update_bits(data->regmap,
				   WCD934X_CLK_SYS_MCLK_PRG, 0x81, 0x00);
		data->clk_type = WCD_CLK_OFF;
		wcd934x_set_sido_input_src(data, SIDO_SOURCE_INTERNAL);
	}

	return 0;
}

static int wcd934x_cdc_req_mclk_enable(struct wcd934x_codec *data,
				       bool enable)
{
	int ret = 0;

	if (enable) {
		ret = clk_prepare_enable(data->extclk);
		if (ret) {
			dev_err(data->dev, "%s: ext clk enable failed\n",
				__func__);
			goto done;
		}
		wcd934x_enable_master_bias(data);
		wcd934x_enable_clk_mclk(data);
	} else {
		wcd934x_disable_clk_mclk(data);
		wcd934x_disable_master_bias(data);
		clk_disable_unprepare(data->extclk);
	}

done:
	return ret;
}

static int __wcd934x_cdc_mclk_enable(struct wcd934x_codec *wcd,
				     bool enable)
{
	int ret = 0;

	if (enable) {
		wcd934x_dig_core_remove_power_collapse(wcd);
		ret = wcd934x_cdc_req_mclk_enable(wcd, true);
		if (ret)
			goto done;
	} else {
		int val;

		regmap_read(wcd->regmap, WCD934X_CDC_CLK_RST_CTRL_SWR_CONTROL,
			    &val);

		/* Don't disable clock if the soundwire/others using it.*/
		if (val & 0x1)
			return 0;

		wcd934x_cdc_req_mclk_enable(wcd, false);
		wcd934x_codec_power_gate_digital_core(wcd);
	}

done:
	return ret;
}

static void wcd934x_get_version(struct wcd934x_codec *wcd)
{
	int val1, val2, version, ret;
	struct regmap *regmap;
	u16 id_minor;
	u32 version_mask = 0;

	regmap = wcd->regmap;
	version = 0;

	ret = regmap_bulk_read(regmap, WCD934X_CHIP_TIER_CTRL_CHIP_ID_BYTE0,
			       (u8 *)&id_minor, sizeof(u16));

	if (ret)
		return;

	regmap_read(regmap, WCD934X_CHIP_TIER_CTRL_EFUSE_VAL_OUT14, &val1);
	regmap_read(regmap, WCD934X_CHIP_TIER_CTRL_EFUSE_VAL_OUT15, &val2);

	dev_info(wcd->dev, "%s: chip version :0x%x 0x:%x\n",
		 __func__, val1, val2);

	version_mask |= (!!((u8)val1 & 0x80)) << DSD_DISABLED_MASK;
	version_mask |= (!!((u8)val2 & 0x01)) << SLNQ_DISABLED_MASK;

	switch (version_mask) {
	case DSD_DISABLED | SLNQ_DISABLED:
		if (id_minor == 0)
			version = WCD_VERSION_WCD9340_1_0;
		else if (id_minor == 0x01)
			version = WCD_VERSION_WCD9340_1_1;
		break;
	case SLNQ_DISABLED:
		if (id_minor == 0)
			version = WCD_VERSION_WCD9341_1_0;
		else if (id_minor == 0x01)
			version = WCD_VERSION_WCD9341_1_1;
		break;
	}

	wcd->version = version;
}

static void wcd934x_enable_efuse_sensing(struct wcd934x_codec *data)
{
	int rc, val;

	__wcd934x_cdc_mclk_enable(data, true);

	regmap_update_bits(data->regmap,
			   WCD934X_CHIP_TIER_CTRL_EFUSE_CTL, 0x1E, 0x10);
	regmap_update_bits(data->regmap,
			   WCD934X_CHIP_TIER_CTRL_EFUSE_CTL, 0x01, 0x01);
	/*
	 * 5ms sleep required after enabling efuse control
	 * before checking the status.
	 */
	usleep_range(5000, 5500);
	wcd934x_set_sido_input_src(data, SIDO_SOURCE_RCO_BG);

	rc = regmap_read(data->regmap,
			 WCD934X_CHIP_TIER_CTRL_EFUSE_STATUS, &val);
	if (rc || (!(val & 0x01)))
		WARN(1, "%s: Efuse sense is not complete val=%x, ret=%d\n",
		     __func__, val, rc);

	__wcd934x_cdc_mclk_enable(data, false);
}

struct wcd934x_cpr_reg_defaults {
	int wr_data;
	int wr_addr;
};

static const struct wcd934x_cpr_reg_defaults cpr_defaults[] = {
	{ 0x00000820, 0x00000094 },
	{ 0x00000fC0, 0x00000048 },
	{ 0x0000f000, 0x00000044 },
	{ 0x0000bb80, 0xC0000178 },
	{ 0x00000000, 0x00000160 },
	{ 0x10854522, 0x00000060 },
	{ 0x10854509, 0x00000064 },
	{ 0x108544dd, 0x00000068 },
	{ 0x108544ad, 0x0000006C },
	{ 0x0000077E, 0x00000070 },
	{ 0x000007da, 0x00000074 },
	{ 0x00000000, 0x00000078 },
	{ 0x00000000, 0x0000007C },
	{ 0x00042029, 0x00000080 },
	{ 0x4002002A, 0x00000090 },
	{ 0x4002002B, 0x00000090 },
};

static void wcd934x_update_cpr_defaults(struct wcd934x_codec *data)
{
	int i;

	__wcd934x_cdc_mclk_enable(data, true);

	wcd934x_set_sido_input_src(data, SIDO_SOURCE_RCO_BG);
	regmap_write(data->regmap, WCD934X_CODEC_CPR_SVS2_MIN_CX_VDD, 0x2C);
	regmap_update_bits(data->regmap, WCD934X_CODEC_RPM_CLK_GATE,
			   0x10, 0x00);

	for (i = 0; i < ARRAY_SIZE(cpr_defaults); i++) {
		regmap_bulk_write(data->regmap,
				  WCD934X_CODEC_CPR_WR_DATA_0,
				(u8 *)&cpr_defaults[i].wr_data, 4);
		regmap_bulk_write(data->regmap,
				  WCD934X_CODEC_CPR_WR_ADDR_0,
				(u8 *)&cpr_defaults[i].wr_addr, 4);
	}

	__wcd934x_cdc_mclk_enable(data, false);
}

static int wcd934x_swrm_clock(struct wcd934x_codec *data, bool enable)
{
	if (enable) {
		regmap_update_bits(data->regmap,
				   WCD934X_TEST_DEBUG_NPL_DLY_TEST_1,
				   0x10, 0x00);
		__wcd934x_cdc_mclk_enable(data, true);
		regmap_update_bits(data->regmap,
				   WCD934X_CDC_CLK_RST_CTRL_SWR_CONTROL,
				   0x01, 0x01);
	} else {
		regmap_update_bits(data->regmap,
				   WCD934X_CDC_CLK_RST_CTRL_SWR_CONTROL,
				   0x01, 0x00);
		__wcd934x_cdc_mclk_enable(data, false);
		regmap_update_bits(data->regmap,
				   WCD934X_TEST_DEBUG_NPL_DLY_TEST_1,
				   0x10, 0x10);
	}

	return 0;
}

static const struct wcd934x_reg_mask_val wcd934x_codec_reg_init_1_1_val[] = {
	{WCD934X_CDC_COMPANDER1_CTL7, 0x1E, 0x06},
	{WCD934X_CDC_COMPANDER2_CTL7, 0x1E, 0x06},
	{WCD934X_HPH_NEW_INT_RDAC_HD2_CTL_L, 0xFF, 0x84},
	{WCD934X_HPH_NEW_INT_RDAC_HD2_CTL_R, 0xFF, 0x84},
	{WCD934X_CDC_RX3_RX_PATH_SEC0, 0xFC, 0xF4},
	{WCD934X_CDC_RX4_RX_PATH_SEC0, 0xFC, 0xF4},
};

static const struct wcd934x_reg_mask_val wcd934x_codec_reg_init_common_val[] = {
	{WCD934X_CDC_CLSH_K2_MSB, 0x0F, 0x00},
	{WCD934X_CDC_CLSH_K2_LSB, 0xFF, 0x60},
	{WCD934X_CPE_SS_DMIC_CFG, 0x80, 0x00},
	{WCD934X_CDC_BOOST0_BOOST_CTL, 0x7C, 0x58},
	{WCD934X_CDC_BOOST1_BOOST_CTL, 0x7C, 0x58},
	{WCD934X_CDC_RX7_RX_PATH_CFG1, 0x08, 0x08},
	{WCD934X_CDC_RX8_RX_PATH_CFG1, 0x08, 0x08},
	{WCD934X_CDC_TOP_TOP_CFG1, 0x02, 0x02},
	{WCD934X_CDC_TOP_TOP_CFG1, 0x01, 0x01},
	{WCD934X_CDC_TX9_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD934X_CDC_TX10_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD934X_CDC_TX11_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD934X_CDC_TX12_SPKR_PROT_PATH_CFG0, 0x01, 0x01},
	{WCD934X_DATA_HUB_SB_TX11_INP_CFG, 0x01, 0x01},
	{WCD934X_CDC_CLK_RST_CTRL_FS_CNT_CONTROL, 0x01, 0x01},
	{WCD934X_CDC_COMPANDER7_CTL3, 0x80, 0x80},
	{WCD934X_CDC_COMPANDER8_CTL3, 0x80, 0x80},
	{WCD934X_CDC_COMPANDER7_CTL7, 0x01, 0x01},
	{WCD934X_CDC_COMPANDER8_CTL7, 0x01, 0x01},
	{WCD934X_CODEC_RPM_CLK_GATE, 0x08, 0x00},
	{WCD934X_TLMM_DMIC3_CLK_PINCFG, 0xFF, 0x0a},
	{WCD934X_TLMM_DMIC3_DATA_PINCFG, 0xFF, 0x0a},
	{WCD934X_CPE_SS_SVA_CFG, 0x60, 0x00},
	{WCD934X_CPE_SS_CPAR_CFG, 0x10, 0x10},
	{WCD934X_MICB1_TEST_CTL_1, 0xff, 0xfa},
	{WCD934X_MICB2_TEST_CTL_1, 0xff, 0xfa},
	{WCD934X_MICB3_TEST_CTL_1, 0xff, 0xfa},
	{WCD934X_MICB4_TEST_CTL_1, 0xff, 0xfa},
};

static const struct wcd934x_reg_mask_val wcd934x_codec_mclk2_1_1_defaults[] = {
	{WCD934X_CLK_SYS_MCLK2_PRG1, 0x60, 0x20},
};

static void wcd934x_mclk2_reg_defaults(struct wcd934x_codec *data)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(wcd934x_codec_mclk2_1_1_defaults); i++)
		regmap_update_bits(data->regmap,
				   wcd934x_codec_mclk2_1_1_defaults[i].reg,
				    wcd934x_codec_mclk2_1_1_defaults[i].mask,
				    wcd934x_codec_mclk2_1_1_defaults[i].val);
}

static void wcd934x_codec_init_reg(struct wcd934x_codec *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wcd934x_codec_reg_init_common_val); i++)
		regmap_update_bits(data->regmap,
				   wcd934x_codec_reg_init_common_val[i].reg,
				    wcd934x_codec_reg_init_common_val[i].mask,
				    wcd934x_codec_reg_init_common_val[i].val);

	for (i = 0; i < ARRAY_SIZE(wcd934x_codec_reg_init_1_1_val); i++)
		regmap_update_bits(data->regmap,
				   wcd934x_codec_reg_init_1_1_val[i].reg,
				   wcd934x_codec_reg_init_1_1_val[i].mask,
				   wcd934x_codec_reg_init_1_1_val[i].val);
}

static int wcd934x_set_prim_interpolator_rate(struct snd_soc_dai *dai,
					      u8 rate_val,
					      u32 rate)
{
	struct snd_soc_component *comp = dai->component;
	struct wcd934x_codec *wcd = dev_get_drvdata(comp->dev);
	struct wcd934x_slim_ch *ch;
	u8 cfg0, cfg1, inp0_sel, inp1_sel, inp2_sel;
	int inp, j;

	list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list) {
		inp = ch->shift + INTn_1_INP_SEL_RX0;
		/*
		 * Loop through all interpolator MUX inputs and find out
		 * to which interpolator input, the slim rx port
		 * is connected
		 */
		for (j = 0; j < WCD934X_NUM_INTERPOLATORS; j++) {
			/* Interpolators 5 and 6 are not aviliable in Tavil */
			if (j == INTERP_LO3_NA || j == INTERP_LO4_NA)
				continue;

			cfg0 = snd_soc_component_read32(comp,
					WCD934X_CDC_RX_INP_MUX_RX_INT_CFG0(j));
			cfg1 = snd_soc_component_read32(comp,
					WCD934X_CDC_RX_INP_MUX_RX_INT_CFG1(j));

			inp0_sel = cfg0 &
				 WCD934X_CDC_RX_INP_MUX_RX_INT_SEL_MASK;
			inp1_sel = (cfg0 >> 4) &
				 WCD934X_CDC_RX_INP_MUX_RX_INT_SEL_MASK;
			inp2_sel = (cfg1 >> 4) &
				 WCD934X_CDC_RX_INP_MUX_RX_INT_SEL_MASK;

			if ((inp0_sel == inp) ||  (inp1_sel == inp) ||
			    (inp2_sel == inp)) {
				/* rate is in Hz */
				/*
				 * Ear and speaker primary path does not support
				 * native sample rates
				 */
				if ((j == INTERP_EAR || j == INTERP_SPKR1 ||
				     j == INTERP_SPKR2) && rate == 44100)
					dev_err(wcd->dev,
						"Cannot set 44.1KHz on INT%d\n",
						j);
				else
					snd_soc_component_update_bits(comp,
					      WCD934X_CDC_RX_PATH_CTL(j),
					      WCD934X_CDC_MIX_PCM_RATE_MASK,
					      rate_val);
			}
		}
	}

	return 0;
}

static int wcd934x_set_mix_interpolator_rate(struct snd_soc_dai *dai,
					     int rate_val,
					     u32 rate)
{
	struct snd_soc_component *component = dai->component;
	struct wcd934x_codec *wcd = dev_get_drvdata(component->dev);
	struct wcd934x_slim_ch *ch;
	int val, j;

	list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list) {
		for (j = 0; j < WCD934X_NUM_INTERPOLATORS; j++) {
			/* Interpolators 5 and 6 are not aviliable in Tavil */
			if (j == INTERP_LO3_NA || j == INTERP_LO4_NA)
				continue;
			val = snd_soc_component_read32(component,
					WCD934X_CDC_RX_INP_MUX_RX_INT_CFG1(j)) &
					WCD934X_CDC_RX_INP_MUX_RX_INT_SEL_MASK;

			if (val == (ch->shift + INTn_2_INP_SEL_RX0)) {
				/*
				 * Ear mix path supports only 48, 96, 192,
				 * 384KHz only
				 */
				if ((j == INTERP_EAR) &&
				    (rate_val < 0x4 ||
				     rate_val > 0x7)) {
					dev_err(component->dev,
						"Invalid rate for AIF_PB DAI(%d)\n",
						dai->id);
					return -EINVAL;
				}

				snd_soc_component_update_bits(component,
					      WCD934X_CDC_RX_PATH_MIX_CTL(j),
					      WCD934X_CDC_MIX_PCM_RATE_MASK,
					      rate_val);
			}
		}
	}

	return 0;
}

static int wcd934x_set_interpolator_rate(struct snd_soc_dai *dai,
					 u32 sample_rate)
{
	int rate_val = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(sr_val_tbl); i++) {
		if (sample_rate == sr_val_tbl[i].sample_rate) {
			rate_val = sr_val_tbl[i].rate_val;
			break;
		}
	}
	if ((i == ARRAY_SIZE(sr_val_tbl)) || (rate_val < 0)) {
		pr_err("%s: Unsupported sample rate: %d\n",
		       __func__, sample_rate);
		return -EINVAL;
	}

	ret = wcd934x_set_prim_interpolator_rate(dai, (u8)rate_val,
						 sample_rate);
	if (ret)
		return ret;
	ret = wcd934x_set_mix_interpolator_rate(dai, (u8)rate_val,
						sample_rate);
	if (ret)
		return ret;

	return ret;
}

static int wcd934x_set_decimator_rate(struct snd_soc_dai *dai,
				      u8 rate_val, u32 rate)
{
	struct snd_soc_component *comp = dai->component;
	struct wcd934x_codec *wcd = snd_soc_component_get_drvdata(comp);
	u8 shift = 0, shift_val = 0, tx_mux_sel;
	struct wcd934x_slim_ch *ch;
	int tx_port, tx_port_reg;
	int decimator = -1;

	list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list) {
		tx_port = ch->port;
		if ((tx_port == 12) || (tx_port >= 14)) {
			dev_err(wcd->dev, "Invalid SLIM TX%u port DAI ID:%d\n",
				tx_port, dai->id);
			return -EINVAL;
		}
		/* Find the SB TX MUX input - which decimator is connected */
		if (tx_port < 4) {
			tx_port_reg = WCD934X_CDC_IF_ROUTER_TX_MUX_CFG0;
			shift = (tx_port << 1);
			shift_val = 0x03;
		} else if ((tx_port >= 4) && (tx_port < 8)) {
			tx_port_reg = WCD934X_CDC_IF_ROUTER_TX_MUX_CFG1;
			shift = ((tx_port - 4) << 1);
			shift_val = 0x03;
		} else if ((tx_port >= 8) && (tx_port < 11)) {
			tx_port_reg = WCD934X_CDC_IF_ROUTER_TX_MUX_CFG2;
			shift = ((tx_port - 8) << 1);
			shift_val = 0x03;
		} else if (tx_port == 11) {
			tx_port_reg = WCD934X_CDC_IF_ROUTER_TX_MUX_CFG3;
			shift = 0;
			shift_val = 0x0F;
		} else if (tx_port == 13) {
			tx_port_reg = WCD934X_CDC_IF_ROUTER_TX_MUX_CFG3;
			shift = 4;
			shift_val = 0x03;
		} else {
			return -EINVAL;
		}

		tx_mux_sel = snd_soc_component_read32(comp, tx_port_reg) &
						      (shift_val << shift);

		tx_mux_sel = tx_mux_sel >> shift;
		if (tx_port <= 8) {
			if ((tx_mux_sel == 0x2) || (tx_mux_sel == 0x3))
				decimator = tx_port;
		} else if (tx_port <= 10) {
			if ((tx_mux_sel == 0x1) || (tx_mux_sel == 0x2))
				decimator = ((tx_port == 9) ? 7 : 6);
		} else if (tx_port == 11) {
			if ((tx_mux_sel >= 1) && (tx_mux_sel < 7))
				decimator = tx_mux_sel - 1;
		} else if (tx_port == 13) {
			if ((tx_mux_sel == 0x1) || (tx_mux_sel == 0x2))
				decimator = 5;
		}

		if (decimator >= 0) {
			snd_soc_component_update_bits(comp,
				      WCD934X_CDC_TX_PATH_CTL(decimator),
				      WCD934X_CDC_TX_PATH_CTL_PCM_RATE_MASK,
					rate_val);
		} else if ((tx_port <= 8) && (tx_mux_sel == 0x01)) {
			/* Check if the TX Mux input is RX MIX TXn */
			dev_err(wcd->dev, "RX_MIX_TX%u going to SLIM TX%u\n",
				tx_port, tx_port);
		} else {
			dev_err(wcd->dev, "ERROR: Invalid decimator: %d\n",
				decimator);
			return -EINVAL;
		}
	}

	return 0;
}

static int wcd934x_slim_set_hw_params(struct wcd934x_codec *wcd,
				      struct wcd_slim_codec_dai_data *dai_data,
				      int direction)
{
	struct list_head *slim_ch_list = &dai_data->slim_ch_list;
	struct slim_stream_config *cfg = &dai_data->sconfig;
	struct wcd934x_slim_ch *ch;
	u16 payload = 0;
	int ret, i;

	cfg->ch_count = 0;
	cfg->direction = direction;
	cfg->port_mask = 0;

	/* Configure slave interface device */
	list_for_each_entry(ch, slim_ch_list, list) {
		cfg->ch_count++;
		payload |= 1 << ch->shift;
		cfg->port_mask |= BIT(ch->port);
	}

	cfg->chs = kcalloc(cfg->ch_count, sizeof(unsigned int), GFP_KERNEL);
	if (!cfg->chs)
		return -ENOMEM;

	i = 0;
	list_for_each_entry(ch, slim_ch_list, list) {
		cfg->chs[i++] = ch->ch_num;
		if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
			/* write to interface device */
			ret = regmap_write(wcd->if_regmap,
			   WCD934X_SLIM_PGD_RX_PORT_MULTI_CHNL_0(ch->port),
			   payload);

			if (ret < 0)
				goto err;

			/* configure the slave port for water mark and enable*/
			ret = regmap_write(wcd->if_regmap,
					WCD934X_SLIM_PGD_RX_PORT_CFG(ch->port),
					WCD934X_SLIM_WATER_MARK_VAL);
			if (ret < 0)
				goto err;
		} else {
			ret = regmap_write(wcd->if_regmap,
				WCD934X_SLIM_PGD_TX_PORT_MULTI_CHNL_0(ch->port),
				payload & 0x00FF);
			if (ret < 0)
				goto err;

			/* ports 8,9 */
			ret = regmap_write(wcd->if_regmap,
				WCD934X_SLIM_PGD_TX_PORT_MULTI_CHNL_1(ch->port),
				(payload & 0xFF00) >> 8);
			if (ret < 0)
				goto err;

			/* configure the slave port for water mark and enable*/
			ret = regmap_write(wcd->if_regmap,
					WCD934X_SLIM_PGD_TX_PORT_CFG(ch->port),
					WCD934X_SLIM_WATER_MARK_VAL);

			if (ret < 0)
				goto err;
		}
	}

	dai_data->sruntime = slim_stream_allocate(wcd->slim, "WCD934x-SLIM");

	return 0;

err:
	dev_err(wcd->dev, "Error Setting slim hw params\n");
	kfree(cfg->chs);
	cfg->chs = NULL;

	return ret;
}

static int wcd934x_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct wcd934x_codec *wcd;
	int ret, tx_fs_rate = 0;

	wcd = snd_soc_component_get_drvdata(dai->component);

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		ret = wcd934x_set_interpolator_rate(dai, params_rate(params));
		if (ret) {
			dev_err(wcd->dev, "cannot set sample rate: %u\n",
				params_rate(params));
			return ret;
		}
		switch (params_width(params)) {
		case 16 ... 24:
			wcd->dai[dai->id].sconfig.bps = params_width(params);
			break;
		default:
			dev_err(wcd->dev, "%s: Invalid format 0x%x\n",
				__func__, params_width(params));
			return -EINVAL;
		}
		break;

	case SNDRV_PCM_STREAM_CAPTURE:
		switch (params_rate(params)) {
		case 8000:
			tx_fs_rate = 0;
			break;
		case 16000:
			tx_fs_rate = 1;
			break;
		case 32000:
			tx_fs_rate = 3;
			break;
		case 48000:
			tx_fs_rate = 4;
			break;
		case 96000:
			tx_fs_rate = 5;
			break;
		case 192000:
			tx_fs_rate = 6;
			break;
		case 384000:
			tx_fs_rate = 7;
			break;
		default:
			dev_err(wcd->dev, "%s: Invalid TX sample rate: %d\n",
				__func__, params_rate(params));
			return -EINVAL;

		};

		ret = wcd934x_set_decimator_rate(dai, tx_fs_rate,
						 params_rate(params));
		if (ret < 0) {
			dev_err(wcd->dev, "Cannot set TX Decimator rate\n");
			return ret;
		}
		switch (params_width(params)) {
		case 16 ... 32:
			wcd->dai[dai->id].sconfig.bps = params_width(params);
			break;
		default:
			dev_err(wcd->dev, "%s: Invalid format 0x%x\n",
				__func__, params_width(params));
			return -EINVAL;
		};
		break;
	default:
		dev_err(wcd->dev, "Invalid stream type %d\n",
			substream->stream);
		return -EINVAL;
	};

	wcd->dai[dai->id].sconfig.rate = params_rate(params);
	wcd934x_slim_set_hw_params(wcd, &wcd->dai[dai->id], substream->stream);

	return 0;
}

static int wcd934x_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct wcd_slim_codec_dai_data *dai_data;
	struct wcd934x_codec *wcd;
	struct slim_stream_config *cfg;

	wcd = snd_soc_component_get_drvdata(dai->component);

	dai_data = &wcd->dai[dai->id];

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		cfg = &dai_data->sconfig;
		slim_stream_prepare(dai_data->sruntime, cfg);
		slim_stream_enable(dai_data->sruntime);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		slim_stream_unprepare(dai_data->sruntime);
		slim_stream_disable(dai_data->sruntime);
		break;
	default:
		break;
	}

	return 0;
}

static int wcd934x_set_channel_map(struct snd_soc_dai *dai,
				   unsigned int tx_num, unsigned int *tx_slot,
				   unsigned int rx_num, unsigned int *rx_slot)
{
	struct wcd934x_codec *wcd;
	int i;

	wcd = snd_soc_component_get_drvdata(dai->component);

	if (!tx_slot || !rx_slot) {
		dev_err(wcd->dev, "Invalid tx_slot=%p, rx_slot=%p\n",
			tx_slot, rx_slot);
		return -EINVAL;
	}

	if (wcd->rx_chs) {
		wcd->num_rx_port = rx_num;
		for (i = 0; i < rx_num; i++) {
			wcd->rx_chs[i].ch_num = rx_slot[i];
			INIT_LIST_HEAD(&wcd->rx_chs[i].list);
		}
	}

	if (wcd->tx_chs) {
		wcd->num_tx_port = tx_num;
		for (i = 0; i < tx_num; i++) {
			wcd->tx_chs[i].ch_num = tx_slot[i];
			INIT_LIST_HEAD(&wcd->tx_chs[i].list);
		}
	}

	return 0;
}

static int wcd934x_get_channel_map(struct snd_soc_dai *dai,
				   unsigned int *tx_num, unsigned int *tx_slot,
				   unsigned int *rx_num, unsigned int *rx_slot)
{
	struct wcd934x_slim_ch *ch;
	struct wcd934x_codec *wcd;
	int i = 0;

	wcd = snd_soc_component_get_drvdata(dai->component);

	switch (dai->id) {
	case AIF1_PB:
	case AIF2_PB:
	case AIF3_PB:
	case AIF4_PB:
		if (!rx_slot || !rx_num) {
			dev_err(wcd->dev, "Invalid rx_slot %p or rx_num %p\n",
				rx_slot, rx_num);
			return -EINVAL;
		}

		list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list)
			rx_slot[i++] = ch->ch_num;

		*rx_num = i;
		break;
	case AIF1_CAP:
	case AIF2_CAP:
	case AIF3_CAP:
		if (!tx_slot || !tx_num) {
			dev_err(wcd->dev, "Invalid tx_slot %p or tx_num %p\n",
				tx_slot, tx_num);
			return -EINVAL;
		}

		list_for_each_entry(ch, &wcd->dai[dai->id].slim_ch_list, list)
			tx_slot[i++] = ch->ch_num;

		*tx_num = i;
		break;
	default:
		dev_err(wcd->dev, "Invalid DAI ID %x\n", dai->id);
		break;
	}

	return 0;
}

static struct snd_soc_dai_ops wcd934x_dai_ops = {
	.hw_params = wcd934x_hw_params,
	.trigger = wcd934x_trigger,
	.set_channel_map = wcd934x_set_channel_map,
	.get_channel_map = wcd934x_get_channel_map,
};

static struct snd_soc_dai_driver wcd934x_slim_dais[] = {
	[0] = {
		.name = "wcd934x_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD934X_RATES_MASK | WCD934X_FRAC_RATES_MASK,
			.formats = WCD934X_FORMATS_S16_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd934x_dai_ops,
	},
	[1] = {
		.name = "wcd934x_tx1",
		.id = AIF1_CAP,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = WCD934X_RATES_MASK,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd934x_dai_ops,
	},
	[2] = {
		.name = "wcd934x_rx2",
		.id = AIF2_PB,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = WCD934X_RATES_MASK | WCD934X_FRAC_RATES_MASK,
			.formats = WCD934X_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd934x_dai_ops,
	},
	[3] = {
		.name = "wcd934x_tx2",
		.id = AIF2_CAP,
		.capture = {
			.stream_name = "AIF2 Capture",
			.rates = WCD934X_RATES_MASK,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd934x_dai_ops,
	},
	[4] = {
		.name = "wcd934x_rx3",
		.id = AIF3_PB,
		.playback = {
			.stream_name = "AIF3 Playback",
			.rates = WCD934X_RATES_MASK | WCD934X_FRAC_RATES_MASK,
			.formats = WCD934X_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd934x_dai_ops,
	},
	[5] = {
		.name = "wcd934x_tx3",
		.id = AIF3_CAP,
		.capture = {
			.stream_name = "AIF3 Capture",
			.rates = WCD934X_RATES_MASK,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd934x_dai_ops,
	},
	[6] = {
		.name = "wcd934x_rx4",
		.id = AIF4_PB,
		.playback = {
			.stream_name = "AIF4 Playback",
			.rates = WCD934X_RATES_MASK | WCD934X_FRAC_RATES_MASK,
			.formats = WCD934X_FORMATS_S16_S24_LE,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcd934x_dai_ops,
	},
};

static int swclk_gate_enable(struct clk_hw *hw)
{
	return wcd934x_swrm_clock(to_wcd934x_codec(hw), true);
}

static void swclk_gate_disable(struct clk_hw *hw)
{
	wcd934x_swrm_clock(to_wcd934x_codec(hw), false);
}

static int swclk_gate_is_enabled(struct clk_hw *hw)
{
	struct wcd934x_codec *wcd = to_wcd934x_codec(hw);
	int ret, val;

	regmap_read(wcd->regmap, WCD934X_CDC_CLK_RST_CTRL_SWR_CONTROL, &val);
	ret = val & 0x01;

	return ret;
}

static unsigned long swclk_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	return parent_rate / 2;
}

static const struct clk_ops swclk_gate_ops = {
	.prepare = swclk_gate_enable,
	.unprepare = swclk_gate_disable,
	.is_enabled = swclk_gate_is_enabled,
	.recalc_rate = swclk_recalc_rate,

};

static struct clk *wcd934x_register_mclk_output(struct wcd934x_codec *wcd)
{
	struct clk *parent = wcd->extclk;
	struct device *dev = wcd->dev;
	struct device_node *np = dev->of_node;
	const char *parent_clk_name = NULL;
	const char *clk_name = "mclk";
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	wcd->extclk = parent;

	if (of_property_read_u32(np, "clock-frequency", &wcd->rate))
		return NULL;

	parent_clk_name = __clk_get_name(parent);

	of_property_read_string(np, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = &swclk_gate_ops;
	init.flags = 0;
	init.parent_names = &parent_clk_name;
	init.num_parents = 1;
	wcd->hw.init = &init;

	hw = &wcd->hw;
	ret = clk_hw_register(wcd->dev, hw);
	if (ret)
		return ERR_PTR(ret);

	of_clk_add_provider(np, of_clk_src_simple_get, hw->clk);

	return NULL;
}

static int wcd934x_get_micbias_val(struct device *dev, const char *micbias)
{
	int mv;

	if (of_property_read_u32(dev->of_node, micbias, &mv))
		mv = WCD934X_DEF_MICBIAS_MV;

	if (mv < 1000 || mv > 2850)
		mv = WCD934X_DEF_MICBIAS_MV;

	return (mv - 1000) / 50;
}

static int wcd934x_init_dmic(struct snd_soc_component *comp)
{
	int vout_ctl_1, vout_ctl_2, vout_ctl_3, vout_ctl_4;
	struct wcd934x_codec *wcd = dev_get_drvdata(comp->dev);
	u32 def_dmic_rate, dmic_clk_drv;

	vout_ctl_1 = wcd934x_get_micbias_val(comp->dev, "qcom,mibias1-lvl");
	vout_ctl_2 = wcd934x_get_micbias_val(comp->dev, "qcom,mibias2-lvl");
	vout_ctl_3 = wcd934x_get_micbias_val(comp->dev, "qcom,mibias3-lvl");
	vout_ctl_4 = wcd934x_get_micbias_val(comp->dev, "qcom,mibias4-lvl");

	snd_soc_component_update_bits(comp, WCD934X_ANA_MICB1,
				      WCD934X_MICB_VAL_MASK, vout_ctl_1);
	snd_soc_component_update_bits(comp, WCD934X_ANA_MICB2,
				      WCD934X_MICB_VAL_MASK, vout_ctl_2);
	snd_soc_component_update_bits(comp, WCD934X_ANA_MICB3,
				      WCD934X_MICB_VAL_MASK, vout_ctl_3);
	snd_soc_component_update_bits(comp, WCD934X_ANA_MICB4,
				      WCD934X_MICB_VAL_MASK, vout_ctl_4);

	if (wcd->rate == WCD934X_MCLK_CLK_9P6MHZ)
		def_dmic_rate = WCD9XXX_DMIC_SAMPLE_RATE_4P8MHZ;
	else
		def_dmic_rate = WCD9XXX_DMIC_SAMPLE_RATE_4P096MHZ;

	wcd->dmic_sample_rate = def_dmic_rate;

	dmic_clk_drv = 0;
	snd_soc_component_update_bits(comp, WCD934X_TEST_DEBUG_PAD_DRVCTL_0,
				      0x0C, dmic_clk_drv << 2);

	return 0;
}

static int wcd934x_comp_init(struct snd_soc_component *component)
{
	struct wcd934x_codec *wcd = dev_get_drvdata(component->dev);

	wcd934x_update_reg_defaults(wcd);
	wcd934x_enable_efuse_sensing(wcd);
	wcd934x_get_version(wcd);
	wcd934x_update_cpr_defaults(wcd);
	wcd934x_dig_core_remove_power_collapse(wcd);
	wcd934x_codec_init_reg(wcd);
	return 0;
}

static irqreturn_t wcd934x_slim_irq_handler(int irq, void *data)
{
	struct wcd934x_codec *wcd = data;
	unsigned long status = 0;
	int i, j, port_id;
	unsigned int val, int_val = 0;
	irqreturn_t ret = IRQ_NONE;
	bool tx;
	unsigned short reg = 0;

	for (i = WCD934X_SLIM_PGD_PORT_INT_STATUS_RX_0, j = 0;
	     i <= WCD934X_SLIM_PGD_PORT_INT_STATUS_TX_1; i++, j++) {
		regmap_read(wcd->if_regmap, i, &val);
		status |= ((u32)val << (8 * j));
	}

	for_each_set_bit(j, &status, 32) {
		tx = (j >= 16 ? true : false);
		port_id = (tx ? j - 16 : j);
		regmap_read(wcd->if_regmap,
			    WCD934X_SLIM_PGD_PORT_INT_RX_SOURCE0 + j, &val);
		if (val) {
			if (!tx)
				reg = WCD934X_SLIM_PGD_PORT_INT_EN0 +
					(port_id / 8);
			else
				reg = WCD934X_SLIM_PGD_PORT_INT_TX_EN0 +
					(port_id / 8);
			regmap_read(
				wcd->if_regmap, reg, &int_val);
			/*
			 * Ignore interrupts for ports for which the
			 * interrupts are not specifically enabled.
			 */
			if (!(int_val & (1 << (port_id % 8))))
				continue;
		}

		if (val & WCD934X_SLIM_IRQ_OVERFLOW)
			dev_err_ratelimited(wcd->dev,
					    "%s: overflow error on %s port %d, value %x\n",
			   __func__, (tx ? "TX" : "RX"), port_id, val);

		if (val & WCD934X_SLIM_IRQ_UNDERFLOW)
			dev_err_ratelimited(wcd->dev,
					    "%s: underflow error on %s port %d, value %x\n",
			   __func__, (tx ? "TX" : "RX"), port_id, val);

		if ((val & WCD934X_SLIM_IRQ_OVERFLOW) ||
		    (val & WCD934X_SLIM_IRQ_UNDERFLOW)) {
			if (!tx)
				reg = WCD934X_SLIM_PGD_PORT_INT_EN0 +
					(port_id / 8);
			else
				reg = WCD934X_SLIM_PGD_PORT_INT_TX_EN0 +
					(port_id / 8);
			regmap_read(
				wcd->if_regmap, reg, &int_val);
			if (int_val & (1 << (port_id % 8))) {
				int_val = int_val ^ (1 << (port_id % 8));
				regmap_write(wcd->if_regmap,
					     reg, int_val);
			}
		}

		if (val & WCD934X_SLIM_IRQ_PORT_CLOSED)
			dev_err_ratelimited(wcd->dev,
					    "%s: Port Closed %s port %d, value %x\n",
			   __func__, (tx ? "TX" : "RX"), port_id, val);

		regmap_write(wcd->if_regmap,
			     WCD934X_SLIM_PGD_PORT_INT_CLR_RX_0 + (j / 8),
				BIT(j % 8));
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int wcd934x_comp_probe(struct snd_soc_component *component)
{
	struct wcd934x_codec *wcd = dev_get_drvdata(component->dev);
	int i;

	snd_soc_component_init_regmap(component, wcd->regmap);
	wcd->component = component;

	/* Class-H Init*/
	wcd->clsh_ctrl = wcd_clsh_ctrl_alloc(component, wcd->version);
	if (IS_ERR(wcd->clsh_ctrl))
		return PTR_ERR(wcd->clsh_ctrl);

	/* Default HPH Mode to Class-H Low HiFi */
	wcd->hph_mode = CLS_H_LOHIFI;

	wcd934x_comp_init(component);

	for (i = 0; i < NUM_CODEC_DAIS; i++)
		INIT_LIST_HEAD(&wcd->dai[i].slim_ch_list);

	wcd934x_mclk2_reg_defaults(wcd);
	wcd934x_init_dmic(component);
	return 0;
}

static void wcd934x_comp_remove(struct snd_soc_component *comp)
{
	struct wcd934x_codec *wcd = dev_get_drvdata(comp->dev);

	wcd_clsh_ctrl_free(wcd->clsh_ctrl);
}

static int wcd934x_comp_set_sysclk(struct snd_soc_component *comp,
				   int clk_id, int source,
				    unsigned int freq, int dir)
{
	struct wcd934x_codec *wcd = dev_get_drvdata(comp->dev);
	int val = WCD934X_CODEC_RPM_CLK_MCLK_CFG_9P6MHZ;

	wcd->rate = freq;

	if (wcd->rate == WCD934X_MCLK_CLK_12P288MHZ)
		val = WCD934X_CODEC_RPM_CLK_MCLK_CFG_12P288MHZ;

	snd_soc_component_update_bits(comp, WCD934X_CODEC_RPM_CLK_MCLK_CFG,
				      WCD934X_CODEC_RPM_CLK_MCLK_CFG_MCLK_MASK,
				      val);

	return clk_set_rate(wcd->extclk, freq);
}

static uint32_t get_iir_band_coeff(struct snd_soc_component *component,
				   int iir_idx, int band_idx,
				   int coeff_idx)
{
	u32 value = 0;
	int reg, b2_reg;

	/* Address does not automatically update if reading */
	reg = WCD934X_CDC_SIDETONE_IIR0_IIR_COEF_B1_CTL + 16 * iir_idx;
	b2_reg = WCD934X_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL + 16 * iir_idx;

	snd_soc_component_write(component, reg,
				((band_idx * BAND_MAX + coeff_idx) *
				 sizeof(uint32_t)) & 0x7F);

	value |= snd_soc_component_read32(component, b2_reg);
	snd_soc_component_write(component, reg,
				((band_idx * BAND_MAX + coeff_idx)
				 * sizeof(uint32_t) + 1) & 0x7F);

	value |= (snd_soc_component_read32(component, b2_reg) << 8);
	snd_soc_component_write(component, reg,
				((band_idx * BAND_MAX + coeff_idx)
				 * sizeof(uint32_t) + 2) & 0x7F);

	value |= (snd_soc_component_read32(component, b2_reg) << 16);
	snd_soc_component_write(component, reg,
		((band_idx * BAND_MAX + coeff_idx)
		* sizeof(uint32_t) + 3) & 0x7F);

	/* Mask bits top 2 bits since they are reserved */
	value |= (snd_soc_component_read32(component, b2_reg) << 24);
	return value;
}

static void set_iir_band_coeff(struct snd_soc_component *component,
			       int iir_idx, int band_idx,
				uint32_t value)
{
	int reg = WCD934X_CDC_SIDETONE_IIR0_IIR_COEF_B2_CTL + 16 * iir_idx;

	snd_soc_component_write(component, reg, (value & 0xFF));
	snd_soc_component_write(component, reg, (value >> 8) & 0xFF);
	snd_soc_component_write(component, reg, (value >> 16) & 0xFF);
	/* Mask top 2 bits, 7-8 are reserved */
	snd_soc_component_write(component, reg,  (value >> 24) & 0x3F);
}

static int wcd934x_put_iir_band_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wcd_iir_filter_ctl *ctl =
			(struct wcd_iir_filter_ctl *)kcontrol->private_value;
	struct soc_bytes_ext *params = &ctl->bytes_ext;
	int iir_idx = ctl->iir_idx;
	int band_idx = ctl->band_idx;
	u32 coeff[BAND_MAX];
	int reg = WCD934X_CDC_SIDETONE_IIR0_IIR_COEF_B1_CTL + 16 * iir_idx;

	memcpy(&coeff[0], ucontrol->value.bytes.data, params->max);

	/* Mask top bit it is reserved */
	/* Updates addr automatically for each B2 write */
	snd_soc_component_write(component, reg, (band_idx * BAND_MAX *
						 sizeof(uint32_t)) & 0x7F);

	set_iir_band_coeff(component, iir_idx, band_idx, coeff[0]);
	set_iir_band_coeff(component, iir_idx, band_idx, coeff[1]);
	set_iir_band_coeff(component, iir_idx, band_idx, coeff[2]);
	set_iir_band_coeff(component, iir_idx, band_idx, coeff[3]);
	set_iir_band_coeff(component, iir_idx, band_idx, coeff[4]);

	return 0;
}

static int wcd934x_get_iir_band_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wcd_iir_filter_ctl *ctl =
			(struct wcd_iir_filter_ctl *)kcontrol->private_value;
	struct soc_bytes_ext *params = &ctl->bytes_ext;
	int iir_idx = ctl->iir_idx;
	int band_idx = ctl->band_idx;
	u32 coeff[BAND_MAX];

	coeff[0] = get_iir_band_coeff(component, iir_idx, band_idx, 0);
	coeff[1] = get_iir_band_coeff(component, iir_idx, band_idx, 1);
	coeff[2] = get_iir_band_coeff(component, iir_idx, band_idx, 2);
	coeff[3] = get_iir_band_coeff(component, iir_idx, band_idx, 3);
	coeff[4] = get_iir_band_coeff(component, iir_idx, band_idx, 4);

	memcpy(ucontrol->value.bytes.data, &coeff[0], params->max);

	return 0;
}

static int wcd934x_iir_filter_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *ucontrol)
{
	struct wcd_iir_filter_ctl *ctl =
		(struct wcd_iir_filter_ctl *)kcontrol->private_value;
	struct soc_bytes_ext *params = &ctl->bytes_ext;

	ucontrol->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	ucontrol->count = params->max;

	return 0;
}

static int wcd934x_compander_get(struct snd_kcontrol *kc,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	int comp = ((struct soc_mixer_control *)kc->private_value)->shift;
	struct wcd934x_codec *wcd = dev_get_drvdata(component->dev);

	ucontrol->value.integer.value[0] = wcd->comp_enabled[comp];

	return 0;
}

static int wcd934x_compander_set(struct snd_kcontrol *kc,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	struct wcd934x_codec *wcd = dev_get_drvdata(component->dev);
	int comp = ((struct soc_mixer_control *)kc->private_value)->shift;
	int value = ucontrol->value.integer.value[0];
	int sel;

	wcd->comp_enabled[comp] = value;
	sel = value ? WCD934X_HPH_GAIN_SRC_SEL_COMPANDER :
		WCD934X_HPH_GAIN_SRC_SEL_REGISTER;

	/* Any specific register configuration for compander */
	switch (comp) {
	case COMPANDER_1:
		/* Set Gain Source Select based on compander enable/disable */
		snd_soc_component_update_bits(component, WCD934X_HPH_L_EN,
					      WCD934X_HPH_GAIN_SRC_SEL_MASK,
					      sel);
		break;
	case COMPANDER_2:
		snd_soc_component_update_bits(component, WCD934X_HPH_R_EN,
					      WCD934X_HPH_GAIN_SRC_SEL_MASK,
					      sel);
		break;
	case COMPANDER_3:
	case COMPANDER_4:
	case COMPANDER_7:
	case COMPANDER_8:
		break;
	default:
		break;
	};

	return 0;
}

static int wcd934x_rx_hph_mode_get(struct snd_kcontrol *kc,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	struct wcd934x_codec *wcd = dev_get_drvdata(component->dev);

	ucontrol->value.enumerated.item[0] = wcd->hph_mode;

	return 0;
}

static int wcd934x_rx_hph_mode_put(struct snd_kcontrol *kc,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kc);
	struct wcd934x_codec *wcd = dev_get_drvdata(component->dev);
	u32 mode_val;

	mode_val = ucontrol->value.enumerated.item[0];

	if (mode_val == 0) {
		dev_err(wcd->dev, "Invalid HPH Mode, default to ClSH HiFi\n");
		mode_val = CLS_H_LOHIFI;
	}
	wcd->hph_mode = mode_val;

	return 0;
}

static const struct snd_kcontrol_new wcd934x_snd_controls[] = {
	/* Gain Controls */
	SOC_SINGLE_TLV("EAR PA Volume", WCD934X_ANA_EAR, 4, 4, 1, ear_pa_gain),
	SOC_SINGLE_TLV("HPHL Volume", WCD934X_HPH_L_EN, 0, 24, 1, line_gain),
	SOC_SINGLE_TLV("HPHR Volume", WCD934X_HPH_R_EN, 0, 24, 1, line_gain),
	SOC_SINGLE_TLV("LINEOUT1 Volume", WCD934X_DIFF_LO_LO1_COMPANDER,
		       3, 16, 1, line_gain),
	SOC_SINGLE_TLV("LINEOUT2 Volume", WCD934X_DIFF_LO_LO2_COMPANDER,
		       3, 16, 1, line_gain),

	SOC_SINGLE_TLV("ADC1 Volume", WCD934X_ANA_AMIC1, 0, 20, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", WCD934X_ANA_AMIC2, 0, 20, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", WCD934X_ANA_AMIC3, 0, 20, 0, analog_gain),
	SOC_SINGLE_TLV("ADC4 Volume", WCD934X_ANA_AMIC4, 0, 20, 0, analog_gain),

	SOC_SINGLE_SX_TLV("RX0 Digital Volume", WCD934X_CDC_RX0_RX_VOL_CTL,
			  0, -84, 40, digital_gain), /* -84dB min - 40dB max */
	SOC_SINGLE_SX_TLV("RX1 Digital Volume", WCD934X_CDC_RX1_RX_VOL_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX2 Digital Volume", WCD934X_CDC_RX2_RX_VOL_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX3 Digital Volume", WCD934X_CDC_RX3_RX_VOL_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX4 Digital Volume", WCD934X_CDC_RX4_RX_VOL_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX7 Digital Volume", WCD934X_CDC_RX7_RX_VOL_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX8 Digital Volume", WCD934X_CDC_RX8_RX_VOL_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX0 Mix Digital Volume",
			  WCD934X_CDC_RX0_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX1 Mix Digital Volume",
			  WCD934X_CDC_RX1_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX2 Mix Digital Volume",
			  WCD934X_CDC_RX2_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX3 Mix Digital Volume",
			  WCD934X_CDC_RX3_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX4 Mix Digital Volume",
			  WCD934X_CDC_RX4_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX7 Mix Digital Volume",
			  WCD934X_CDC_RX7_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("RX8 Mix Digital Volume",
			  WCD934X_CDC_RX8_RX_VOL_MIX_CTL,
			  0, -84, 40, digital_gain),

	SOC_SINGLE_SX_TLV("DEC0 Volume", WCD934X_CDC_TX0_TX_VOL_CTL,
			  0, -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC1 Volume", WCD934X_CDC_TX1_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC2 Volume", WCD934X_CDC_TX2_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC3 Volume", WCD934X_CDC_TX3_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC4 Volume", WCD934X_CDC_TX4_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC5 Volume", WCD934X_CDC_TX5_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC6 Volume", WCD934X_CDC_TX6_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC7 Volume", WCD934X_CDC_TX7_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),
	SOC_SINGLE_SX_TLV("DEC8 Volume", WCD934X_CDC_TX8_TX_VOL_CTL, 0,
			  -84, 40, digital_gain),

	SOC_SINGLE_SX_TLV("IIR0 INP0 Volume",
			  WCD934X_CDC_SIDETONE_IIR0_IIR_GAIN_B1_CTL, 0, -84, 40,
			  digital_gain),
	SOC_SINGLE_SX_TLV("IIR0 INP1 Volume",
			  WCD934X_CDC_SIDETONE_IIR0_IIR_GAIN_B2_CTL, 0, -84, 40,
			  digital_gain),
	SOC_SINGLE_SX_TLV("IIR0 INP2 Volume",
			  WCD934X_CDC_SIDETONE_IIR0_IIR_GAIN_B3_CTL, 0, -84, 40,
			  digital_gain),
	SOC_SINGLE_SX_TLV("IIR0 INP3 Volume",
			  WCD934X_CDC_SIDETONE_IIR0_IIR_GAIN_B4_CTL, 0, -84, 40,
			  digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP0 Volume",
			  WCD934X_CDC_SIDETONE_IIR1_IIR_GAIN_B1_CTL, 0, -84, 40,
			  digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP1 Volume",
			  WCD934X_CDC_SIDETONE_IIR1_IIR_GAIN_B2_CTL, 0, -84, 40,
			  digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP2 Volume",
			  WCD934X_CDC_SIDETONE_IIR1_IIR_GAIN_B3_CTL, 0, -84, 40,
			  digital_gain),
	SOC_SINGLE_SX_TLV("IIR1 INP3 Volume",
			  WCD934X_CDC_SIDETONE_IIR1_IIR_GAIN_B4_CTL, 0, -84, 40,
			  digital_gain),

	SOC_ENUM("TX0 HPF cut off", cf_dec0_enum),
	SOC_ENUM("TX1 HPF cut off", cf_dec1_enum),
	SOC_ENUM("TX2 HPF cut off", cf_dec2_enum),
	SOC_ENUM("TX3 HPF cut off", cf_dec3_enum),
	SOC_ENUM("TX4 HPF cut off", cf_dec4_enum),
	SOC_ENUM("TX5 HPF cut off", cf_dec5_enum),
	SOC_ENUM("TX6 HPF cut off", cf_dec6_enum),
	SOC_ENUM("TX7 HPF cut off", cf_dec7_enum),
	SOC_ENUM("TX8 HPF cut off", cf_dec8_enum),

	SOC_ENUM("RX INT0_1 HPF cut off", cf_int0_1_enum),
	SOC_ENUM("RX INT0_2 HPF cut off", cf_int0_2_enum),
	SOC_ENUM("RX INT1_1 HPF cut off", cf_int1_1_enum),
	SOC_ENUM("RX INT1_2 HPF cut off", cf_int1_2_enum),
	SOC_ENUM("RX INT2_1 HPF cut off", cf_int2_1_enum),
	SOC_ENUM("RX INT2_2 HPF cut off", cf_int2_2_enum),
	SOC_ENUM("RX INT3_1 HPF cut off", cf_int3_1_enum),
	SOC_ENUM("RX INT3_2 HPF cut off", cf_int3_2_enum),
	SOC_ENUM("RX INT4_1 HPF cut off", cf_int4_1_enum),
	SOC_ENUM("RX INT4_2 HPF cut off", cf_int4_2_enum),
	SOC_ENUM("RX INT7_1 HPF cut off", cf_int7_1_enum),
	SOC_ENUM("RX INT7_2 HPF cut off", cf_int7_2_enum),
	SOC_ENUM("RX INT8_1 HPF cut off", cf_int8_1_enum),
	SOC_ENUM("RX INT8_2 HPF cut off", cf_int8_2_enum),

	SOC_ENUM_EXT("RX HPH Mode", rx_hph_mode_mux_enum,
		     wcd934x_rx_hph_mode_get, wcd934x_rx_hph_mode_put),

	SOC_SINGLE("IIR1 Band1 Switch", WCD934X_CDC_SIDETONE_IIR0_IIR_CTL,
		   0, 1, 0),
	SOC_SINGLE("IIR1 Band2 Switch", WCD934X_CDC_SIDETONE_IIR0_IIR_CTL,
		   1, 1, 0),
	SOC_SINGLE("IIR1 Band3 Switch", WCD934X_CDC_SIDETONE_IIR0_IIR_CTL,
		   2, 1, 0),
	SOC_SINGLE("IIR1 Band4 Switch", WCD934X_CDC_SIDETONE_IIR0_IIR_CTL,
		   3, 1, 0),
	SOC_SINGLE("IIR1 Band5 Switch", WCD934X_CDC_SIDETONE_IIR0_IIR_CTL,
		   4, 1, 0),
	SOC_SINGLE("IIR2 Band1 Switch", WCD934X_CDC_SIDETONE_IIR1_IIR_CTL,
		   0, 1, 0),
	SOC_SINGLE("IIR2 Band2 Switch", WCD934X_CDC_SIDETONE_IIR1_IIR_CTL,
		   1, 1, 0),
	SOC_SINGLE("IIR2 Band3 Switch", WCD934X_CDC_SIDETONE_IIR1_IIR_CTL,
		   2, 1, 0),
	SOC_SINGLE("IIR2 Band4 Switch", WCD934X_CDC_SIDETONE_IIR1_IIR_CTL,
		   3, 1, 0),
	SOC_SINGLE("IIR2 Band5 Switch", WCD934X_CDC_SIDETONE_IIR1_IIR_CTL,
		   4, 1, 0),
	WCD_IIR_FILTER_CTL("IIR0 Band1", IIR0, BAND1),
	WCD_IIR_FILTER_CTL("IIR0 Band2", IIR0, BAND2),
	WCD_IIR_FILTER_CTL("IIR0 Band3", IIR0, BAND3),
	WCD_IIR_FILTER_CTL("IIR0 Band4", IIR0, BAND4),
	WCD_IIR_FILTER_CTL("IIR0 Band5", IIR0, BAND5),

	WCD_IIR_FILTER_CTL("IIR1 Band1", IIR1, BAND1),
	WCD_IIR_FILTER_CTL("IIR1 Band2", IIR1, BAND2),
	WCD_IIR_FILTER_CTL("IIR1 Band3", IIR1, BAND3),
	WCD_IIR_FILTER_CTL("IIR1 Band4", IIR1, BAND4),
	WCD_IIR_FILTER_CTL("IIR1 Band5", IIR1, BAND5),

	SOC_SINGLE_EXT("COMP1 Switch", SND_SOC_NOPM, COMPANDER_1, 1, 0,
		       wcd934x_compander_get, wcd934x_compander_set),
	SOC_SINGLE_EXT("COMP2 Switch", SND_SOC_NOPM, COMPANDER_2, 1, 0,
		       wcd934x_compander_get, wcd934x_compander_set),
	SOC_SINGLE_EXT("COMP3 Switch", SND_SOC_NOPM, COMPANDER_3, 1, 0,
		       wcd934x_compander_get, wcd934x_compander_set),
	SOC_SINGLE_EXT("COMP4 Switch", SND_SOC_NOPM, COMPANDER_4, 1, 0,
		       wcd934x_compander_get, wcd934x_compander_set),
	SOC_SINGLE_EXT("COMP7 Switch", SND_SOC_NOPM, COMPANDER_7, 1, 0,
		       wcd934x_compander_get, wcd934x_compander_set),
	SOC_SINGLE_EXT("COMP8 Switch", SND_SOC_NOPM, COMPANDER_8, 1, 0,
		       wcd934x_compander_get, wcd934x_compander_set),
};

static const struct snd_soc_component_driver wcd934x_component_drv = {
	.probe = wcd934x_comp_probe,
	.remove = wcd934x_comp_remove,
	.set_sysclk = wcd934x_comp_set_sysclk,
	.controls = wcd934x_snd_controls,
	.num_controls = ARRAY_SIZE(wcd934x_snd_controls),
};

static int wcd934x_codec_probe(struct wcd934x_codec *wcd)
{
	struct device *dev = wcd->dev;
	int ret, irq;

	irq = regmap_irq_get_virq(wcd->irq_data, WCD934X_IRQ_SLIMBUS);
	if (irq < 0) {
		dev_err(wcd->dev, "Failed to get SLIM IRQ\n");
		return irq;
	}

	wcd->cur_power_state = WCD_REGION_POWER_COLLAPSE_REMOVE;

	/* set default rate 9P6MHz */
	regmap_update_bits(wcd->regmap, WCD934X_CODEC_RPM_CLK_MCLK_CFG,
			   WCD934X_CODEC_RPM_CLK_MCLK_CFG_MCLK_MASK,
			   WCD934X_CODEC_RPM_CLK_MCLK_CFG_9P6MHZ);
	memcpy(wcd->rx_chs, wcd934x_rx_chs, sizeof(wcd934x_rx_chs));
	memcpy(wcd->tx_chs, wcd934x_tx_chs, sizeof(wcd934x_tx_chs));

	ret = devm_request_threaded_irq(dev, irq, NULL,
					wcd934x_slim_irq_handler,
					IRQF_TRIGGER_RISING,
					"slim", wcd);
	if (ret) {
		dev_err(dev, "Failed to request slimbus irq\n");
		return ret;
	}
	wcd934x_register_mclk_output(wcd);
	of_property_read_u32(dev->of_node, "qcom,dmic-sample-rate",
			     &wcd->dmic_sample_rate);
	ret = devm_snd_soc_register_component(dev, &wcd934x_component_drv,
					      wcd934x_slim_dais,
					      ARRAY_SIZE(wcd934x_slim_dais));

	return of_platform_populate(wcd->dev->of_node, NULL, NULL, wcd->dev);
}

static const struct regmap_range_cfg wcd934x_ranges[] = {
	{	.name = "WCD934X",
		.range_min =  0x0,
		.range_max =  0xffff,
		.selector_reg = 0x800,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0x800,
		.window_len = 0x100,
	},
};

static bool wcd934x_is_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case WCD934X_INTR_PIN1_STATUS0...WCD934X_INTR_PIN2_CLEAR3:
	case WCD934X_SWR_AHB_BRIDGE_RD_DATA_0 ... WCD934X_SWR_AHB_BRIDGE_RD_DATA_3:
	case WCD934X_SWR_AHB_BRIDGE_ACCESS_STATUS:
	case WCD934X_ANA_MBHC_RESULT_3:
	case WCD934X_ANA_MBHC_RESULT_2:
	case WCD934X_ANA_MBHC_RESULT_1:
	case WCD934X_ANA_MBHC_MECH:
	case WCD934X_ANA_MBHC_ELECT:
	case WCD934X_ANA_MBHC_ZDET:
	case WCD934X_ANA_MICB2:
	case WCD934X_ANA_RCO:
	case WCD934X_ANA_BIAS:
		return true;
	default:
		return false;
	}

};

static const struct regmap_range_cfg wcd934x_ifc_ranges[] = {
	{
		.name = "WCD9335-IFC-DEV",
		.range_min =  0x0,
		.range_max = 0xffff,
		.selector_reg = 0x800,
		.selector_mask = 0xfff,
		.selector_shift = 0,
		.window_start = 0x800,
		.window_len = 0x400,
	},
};

static struct regmap_config wcd934x_ifc_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = wcd934x_ifc_ranges,
	.num_ranges = ARRAY_SIZE(wcd934x_ifc_ranges),
};

static struct regmap_config wcd934x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0xffff,
	.can_multi_write = true,
	.ranges = wcd934x_ranges,
	.num_ranges = ARRAY_SIZE(wcd934x_ranges),
	.volatile_reg = wcd934x_is_volatile_register,
};

static int wcd934x_parse_resources(struct wcd934x_codec *wcd)
{
	struct device *dev = wcd->dev;
	struct device_node *np = dev->of_node;
	int ret;
	/*
	 * INTR1 consists of all possible interrupt sources Ear OCP,
	 * HPH OCP, MBHC, MAD, VBAT, and SVA
	 * INTR2 is a subset of first interrupt sources MAD, VBAT, and SVA
	 */
	wcd->irq = of_irq_get_byname(wcd->dev->of_node, "intr1");
	if (wcd->irq < 0) {
		if (wcd->irq != -EPROBE_DEFER)
			dev_err(wcd->dev, "Unable to configure IRQ\n");

		return wcd->irq;
	}

	wcd->reset_gpio = of_get_named_gpio(np,	"reset-gpios", 0);
	if (wcd->reset_gpio < 0) {
		dev_err(dev, "Reset gpio missing in DT\n");
		return wcd->reset_gpio;
	}

	wcd->extclk = devm_clk_get(dev, "extclk");
	if (IS_ERR(wcd->extclk)) {
		dev_err(dev, "extclk not found\n");
		return PTR_ERR(wcd->extclk);
	}

	wcd->supplies[0].supply = "vdd-buck";
	wcd->supplies[1].supply = "vdd-buck-sido";
	wcd->supplies[2].supply = "vdd-tx";
	wcd->supplies[3].supply = "vdd-rx";
	wcd->supplies[4].supply = "vdd-io";

	ret = regulator_bulk_get(dev, WCD934X_MAX_SUPPLY, wcd->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	return 0;
}

static int wcd934x_power_on_reset(struct wcd934x_codec *wcd)
{
	struct device *dev = wcd->dev;
	int ret;

	ret = regulator_bulk_enable(WCD934X_MAX_SUPPLY, wcd->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	/*
	 * For WCD934X, it takes about 600us for the Vout_A and
	 * Vout_D to be ready after BUCK_SIDO is powered up.
	 * SYS_RST_N shouldn't be pulled high during this time
	 */
	usleep_range(600, 650);
	gpio_direction_output(wcd->reset_gpio, 0);
	msleep(20);
	gpio_set_value(wcd->reset_gpio, 1);
	msleep(20);

	return 0;
}

static int wcd934x_slim_probe(struct slim_device *slim)
{
	struct device *dev = &slim->dev;
	struct wcd934x_codec *wcd;
	int ret = 0;

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return	-ENOMEM;

	wcd->dev = dev;
	ret = wcd934x_parse_resources(wcd);
	if (ret) {
		dev_err(dev, "Error parsing DT (%d)\n", ret);
		return ret;
	}

	ret = wcd934x_power_on_reset(wcd);
	if (ret) {
		dev_err(dev, "Error Powering\n");
		return ret;
	}

	dev_set_drvdata(dev, wcd);
	wcd->slim = slim;

	return 0;
}

static int wcd934x_bring_up(struct wcd934x_codec *wcd)
{
	struct regmap *wcd_regmap = wcd->regmap;
	u16 id_minor, id_major;
	int ret;

	ret = regmap_bulk_read(wcd_regmap, WCD934X_CHIP_TIER_CTRL_CHIP_ID_BYTE0,
			      (u8 *)&id_minor, sizeof(u16));
	if (ret)
		return -EINVAL;

	ret = regmap_bulk_read(wcd_regmap, WCD934X_CHIP_TIER_CTRL_CHIP_ID_BYTE2,
			      (u8 *)&id_major, sizeof(u16));
	if (ret)
		return -EINVAL;

	dev_info(wcd->dev, "%s: wcd9xxx chip id major 0x%x, minor 0x%x\n",
		 __func__, id_major, id_minor);

	regmap_write(wcd_regmap, WCD934X_CODEC_RPM_RST_CTL, 0x01);
	regmap_write(wcd_regmap, WCD934X_SIDO_NEW_VOUT_A_STARTUP, 0x19);
	regmap_write(wcd_regmap, WCD934X_SIDO_NEW_VOUT_D_STARTUP, 0x15);
	/* Add 1msec delay for VOUT to settle */
	usleep_range(1000, 1100);
	regmap_write(wcd_regmap, WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x5);
	regmap_write(wcd_regmap, WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x7);
	regmap_write(wcd_regmap, WCD934X_CODEC_RPM_RST_CTL, 0x3);
	regmap_write(wcd_regmap, WCD934X_CODEC_RPM_RST_CTL, 0x7);
	regmap_write(wcd_regmap, WCD934X_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x3);

	return 0;

}

static const struct regmap_irq wcd934x_irqs[] = {
	/* INTR_REG 0 */
	[WCD934X_IRQ_SLIMBUS] = {
		.reg_offset = 0,
		.mask = BIT(0),
		.type = {
			.type_reg_offset = 0,
			.types_supported = IRQ_TYPE_EDGE_BOTH,
			.type_reg_mask  = BIT(0),
			.type_level_low_val = BIT(0),
			.type_level_high_val = BIT(0),
			.type_falling_val = 0,
			.type_rising_val = 0,
		},
	},
	[WCD934X_IRQ_SOUNDWIRE] = {
		.reg_offset = 2,
		.mask = BIT(4),
		.type = {
			.type_reg_offset = 2,
			.types_supported = IRQ_TYPE_EDGE_BOTH,
			.type_reg_mask  = BIT(4),
		},
	},
};

static const struct regmap_irq_chip wcd934x_regmap_irq_chip = {
	.name = "wcd934x_irq",
	.status_base = WCD934X_INTR_PIN1_STATUS0,
	.mask_base = WCD934X_INTR_PIN1_MASK0,
	.ack_base = WCD934X_INTR_PIN1_CLEAR0,
	.type_base = WCD934X_INTR_LEVEL0,
	.num_type_reg = 4,
	.type_in_mask = false,
	.num_regs = 4,
	.irqs = wcd934x_irqs,
	.num_irqs = ARRAY_SIZE(wcd934x_irqs),
};

static int wcd934x_slim_status(struct slim_device *sdev,
				 enum slim_device_status status)
{
	struct device *dev = &sdev->dev;
	struct device_node *ifc_dev_np;
	struct wcd934x_codec *wcd;
	int ret;

	wcd = dev_get_drvdata(dev);

	switch (status) {
	case SLIM_DEVICE_STATUS_UP:
		ifc_dev_np = of_parse_phandle(dev->of_node, "slim-ifc-dev", 0);
		if (!ifc_dev_np) {
			dev_err(dev, "No Interface device found\n");
			return -EINVAL;
		}
		wcd->slim = sdev;
		wcd->slim_ifc_dev = of_slim_get_device(sdev->ctrl, ifc_dev_np);
		if (!wcd->slim_ifc_dev) {
			dev_err(dev, "Unable to get SLIM Interface device\n");
			return -EINVAL;
		}

		slim_get_logical_addr(wcd->slim_ifc_dev);
		wcd->regmap = regmap_init_slimbus(sdev, &wcd934x_regmap_config);
		if (IS_ERR(wcd->regmap)) {
			dev_err(dev, "Failed to allocate slim register map\n");
			return PTR_ERR(wcd->regmap);
		}

		wcd->if_regmap = regmap_init_slimbus(wcd->slim_ifc_dev,
					  &wcd934x_ifc_regmap_config);
		if (IS_ERR(wcd->if_regmap)) {
			dev_err(dev, "Failed to allocate ifc register map\n");
			return PTR_ERR(wcd->if_regmap);
		}

		ret = wcd934x_bring_up(wcd);
		if (ret) {
			dev_err(dev, "Failed to bringup WCD934X\n");
			return ret;
		}

		ret = devm_regmap_add_irq_chip(wcd->dev, wcd->regmap,
					       wcd->irq,
					       IRQF_TRIGGER_HIGH, 0,
					       &wcd934x_regmap_irq_chip,
					       &wcd->irq_data);
		if (ret) {
			dev_err(wcd->dev, "Failed to register IRQ chip: %d\n",
				ret);
			return ret;
		}

		return wcd934x_codec_probe(wcd);
	case SLIM_DEVICE_STATUS_DOWN:
		of_platform_depopulate(wcd->dev);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct slim_device_id wcd934x_slim_id[] = {
	{SLIM_MANF_ID_QCOM, SLIM_PROD_CODE_WCD9340, 0x1, 0x0},
	{}
};

static struct slim_driver wcd934x_slim_driver = {
	.driver = {
		.name = "wcd934x-slim",
	},
	.probe = wcd934x_slim_probe,
	.device_status = wcd934x_slim_status,
	.id_table = wcd934x_slim_id,
};

module_slim_driver(wcd934x_slim_driver);
MODULE_DESCRIPTION("WCD934x codec driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("slim:217:250:*");
