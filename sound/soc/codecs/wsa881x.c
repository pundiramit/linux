// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2015-2017, The Linux Foundation.
// Copyright (c) 2019, Linaro Limited

#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soundwire/sdw.h>
#include <linux/soundwire/sdw_registers.h>
#include <linux/soundwire/sdw_type.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define WSA881X_DIGITAL_BASE		0x3000
#define WSA881X_ANALOG_BASE		0x3100

/* Digital register address space */
#define WSA881X_CHIP_ID0			(WSA881X_DIGITAL_BASE + 0x0000)
#define WSA881X_CHIP_ID1			(WSA881X_DIGITAL_BASE + 0x0001)
#define WSA881X_CHIP_ID2			(WSA881X_DIGITAL_BASE + 0x0002)
#define WSA881X_CHIP_ID3			(WSA881X_DIGITAL_BASE + 0x0003)
#define WSA881X_BUS_ID				(WSA881X_DIGITAL_BASE + 0x0004)
#define WSA881X_CDC_RST_CTL			(WSA881X_DIGITAL_BASE + 0x0005)
#define WSA881X_CDC_TOP_CLK_CTL			(WSA881X_DIGITAL_BASE + 0x0006)
#define WSA881X_CDC_ANA_CLK_CTL			(WSA881X_DIGITAL_BASE + 0x0007)
#define WSA881X_CDC_DIG_CLK_CTL			(WSA881X_DIGITAL_BASE + 0x0008)
#define WSA881X_CLOCK_CONFIG			(WSA881X_DIGITAL_BASE + 0x0009)
#define WSA881X_ANA_CTL				(WSA881X_DIGITAL_BASE + 0x000A)
#define WSA881X_SWR_RESET_EN			(WSA881X_DIGITAL_BASE + 0x000B)
#define WSA881X_RESET_CTL			(WSA881X_DIGITAL_BASE + 0x000C)
#define WSA881X_TADC_VALUE_CTL			(WSA881X_DIGITAL_BASE + 0x000F)
#define WSA881X_TEMP_DETECT_CTL			(WSA881X_DIGITAL_BASE + 0x0010)
#define WSA881X_TEMP_MSB			(WSA881X_DIGITAL_BASE + 0x0011)
#define WSA881X_TEMP_LSB			(WSA881X_DIGITAL_BASE + 0x0012)
#define WSA881X_TEMP_CONFIG0			(WSA881X_DIGITAL_BASE + 0x0013)
#define WSA881X_TEMP_CONFIG1			(WSA881X_DIGITAL_BASE + 0x0014)
#define WSA881X_CDC_CLIP_CTL			(WSA881X_DIGITAL_BASE + 0x0015)
#define WSA881X_SDM_PDM9_LSB			(WSA881X_DIGITAL_BASE + 0x0016)
#define WSA881X_SDM_PDM9_MSB			(WSA881X_DIGITAL_BASE + 0x0017)
#define WSA881X_CDC_RX_CTL			(WSA881X_DIGITAL_BASE + 0x0018)
#define WSA881X_DEM_BYPASS_DATA0		(WSA881X_DIGITAL_BASE + 0x0019)
#define WSA881X_DEM_BYPASS_DATA1		(WSA881X_DIGITAL_BASE + 0x001A)
#define WSA881X_DEM_BYPASS_DATA2		(WSA881X_DIGITAL_BASE + 0x001B)
#define WSA881X_DEM_BYPASS_DATA3		(WSA881X_DIGITAL_BASE + 0x001C)
#define WSA881X_OTP_CTRL0			(WSA881X_DIGITAL_BASE + 0x001D)
#define WSA881X_OTP_CTRL1			(WSA881X_DIGITAL_BASE + 0x001E)
#define WSA881X_HDRIVE_CTL_GROUP1		(WSA881X_DIGITAL_BASE + 0x001F)
#define WSA881X_INTR_MODE			(WSA881X_DIGITAL_BASE + 0x0020)
#define WSA881X_INTR_MASK			(WSA881X_DIGITAL_BASE + 0x0021)
#define WSA881X_INTR_STATUS			(WSA881X_DIGITAL_BASE + 0x0022)
#define WSA881X_INTR_CLEAR			(WSA881X_DIGITAL_BASE + 0x0023)
#define WSA881X_INTR_LEVEL			(WSA881X_DIGITAL_BASE + 0x0024)
#define WSA881X_INTR_SET			(WSA881X_DIGITAL_BASE + 0x0025)
#define WSA881X_INTR_TEST			(WSA881X_DIGITAL_BASE + 0x0026)
#define WSA881X_PDM_TEST_MODE			(WSA881X_DIGITAL_BASE + 0x0030)
#define WSA881X_ATE_TEST_MODE			(WSA881X_DIGITAL_BASE + 0x0031)
#define WSA881X_PIN_CTL_MODE			(WSA881X_DIGITAL_BASE + 0x0032)
#define WSA881X_PIN_CTL_OE			(WSA881X_DIGITAL_BASE + 0x0033)
#define WSA881X_PIN_WDATA_IOPAD			(WSA881X_DIGITAL_BASE + 0x0034)
#define WSA881X_PIN_STATUS			(WSA881X_DIGITAL_BASE + 0x0035)
#define WSA881X_DIG_DEBUG_MODE			(WSA881X_DIGITAL_BASE + 0x0037)
#define WSA881X_DIG_DEBUG_SEL			(WSA881X_DIGITAL_BASE + 0x0038)
#define WSA881X_DIG_DEBUG_EN			(WSA881X_DIGITAL_BASE + 0x0039)
#define WSA881X_SWR_HM_TEST1			(WSA881X_DIGITAL_BASE + 0x003B)
#define WSA881X_SWR_HM_TEST2			(WSA881X_DIGITAL_BASE + 0x003C)
#define WSA881X_TEMP_DETECT_DBG_CTL		(WSA881X_DIGITAL_BASE + 0x003D)
#define WSA881X_TEMP_DEBUG_MSB			(WSA881X_DIGITAL_BASE + 0x003E)
#define WSA881X_TEMP_DEBUG_LSB			(WSA881X_DIGITAL_BASE + 0x003F)
#define WSA881X_SAMPLE_EDGE_SEL			(WSA881X_DIGITAL_BASE + 0x0044)
#define WSA881X_IOPAD_CTL			(WSA881X_DIGITAL_BASE + 0x0045)
#define WSA881X_SPARE_0				(WSA881X_DIGITAL_BASE + 0x0050)
#define WSA881X_SPARE_1				(WSA881X_DIGITAL_BASE + 0x0051)
#define WSA881X_SPARE_2				(WSA881X_DIGITAL_BASE + 0x0052)
#define WSA881X_OTP_REG_0			(WSA881X_DIGITAL_BASE + 0x0080)
#define WSA881X_OTP_REG_1			(WSA881X_DIGITAL_BASE + 0x0081)
#define WSA881X_OTP_REG_2			(WSA881X_DIGITAL_BASE + 0x0082)
#define WSA881X_OTP_REG_3			(WSA881X_DIGITAL_BASE + 0x0083)
#define WSA881X_OTP_REG_4			(WSA881X_DIGITAL_BASE + 0x0084)
#define WSA881X_OTP_REG_5			(WSA881X_DIGITAL_BASE + 0x0085)
#define WSA881X_OTP_REG_6			(WSA881X_DIGITAL_BASE + 0x0086)
#define WSA881X_OTP_REG_7			(WSA881X_DIGITAL_BASE + 0x0087)
#define WSA881X_OTP_REG_8			(WSA881X_DIGITAL_BASE + 0x0088)
#define WSA881X_OTP_REG_9			(WSA881X_DIGITAL_BASE + 0x0089)
#define WSA881X_OTP_REG_10			(WSA881X_DIGITAL_BASE + 0x008A)
#define WSA881X_OTP_REG_11			(WSA881X_DIGITAL_BASE + 0x008B)
#define WSA881X_OTP_REG_12			(WSA881X_DIGITAL_BASE + 0x008C)
#define WSA881X_OTP_REG_13			(WSA881X_DIGITAL_BASE + 0x008D)
#define WSA881X_OTP_REG_14			(WSA881X_DIGITAL_BASE + 0x008E)
#define WSA881X_OTP_REG_15			(WSA881X_DIGITAL_BASE + 0x008F)
#define WSA881X_OTP_REG_16			(WSA881X_DIGITAL_BASE + 0x0090)
#define WSA881X_OTP_REG_17			(WSA881X_DIGITAL_BASE + 0x0091)
#define WSA881X_OTP_REG_18			(WSA881X_DIGITAL_BASE + 0x0092)
#define WSA881X_OTP_REG_19			(WSA881X_DIGITAL_BASE + 0x0093)
#define WSA881X_OTP_REG_20			(WSA881X_DIGITAL_BASE + 0x0094)
#define WSA881X_OTP_REG_21			(WSA881X_DIGITAL_BASE + 0x0095)
#define WSA881X_OTP_REG_22			(WSA881X_DIGITAL_BASE + 0x0096)
#define WSA881X_OTP_REG_23			(WSA881X_DIGITAL_BASE + 0x0097)
#define WSA881X_OTP_REG_24			(WSA881X_DIGITAL_BASE + 0x0098)
#define WSA881X_OTP_REG_25			(WSA881X_DIGITAL_BASE + 0x0099)
#define WSA881X_OTP_REG_26			(WSA881X_DIGITAL_BASE + 0x009A)
#define WSA881X_OTP_REG_27			(WSA881X_DIGITAL_BASE + 0x009B)
#define WSA881X_OTP_REG_28			(WSA881X_DIGITAL_BASE + 0x009C)
#define WSA881X_OTP_REG_29			(WSA881X_DIGITAL_BASE + 0x009D)
#define WSA881X_OTP_REG_30			(WSA881X_DIGITAL_BASE + 0x009E)
#define WSA881X_OTP_REG_31			(WSA881X_DIGITAL_BASE + 0x009F)
#define WSA881X_OTP_REG_63			(WSA881X_DIGITAL_BASE + 0x00BF)

/* Analog Register address space */
#define WSA881X_BIAS_REF_CTRL			(WSA881X_ANALOG_BASE + 0x0000)
#define WSA881X_BIAS_TEST			(WSA881X_ANALOG_BASE + 0x0001)
#define WSA881X_BIAS_BIAS			(WSA881X_ANALOG_BASE + 0x0002)
#define WSA881X_TEMP_OP				(WSA881X_ANALOG_BASE + 0x0003)
#define WSA881X_TEMP_IREF_CTRL			(WSA881X_ANALOG_BASE + 0x0004)
#define WSA881X_TEMP_ISENS_CTRL			(WSA881X_ANALOG_BASE + 0x0005)
#define WSA881X_TEMP_CLK_CTRL			(WSA881X_ANALOG_BASE + 0x0006)
#define WSA881X_TEMP_TEST			(WSA881X_ANALOG_BASE + 0x0007)
#define WSA881X_TEMP_BIAS			(WSA881X_ANALOG_BASE + 0x0008)
#define WSA881X_TEMP_ADC_CTRL			(WSA881X_ANALOG_BASE + 0x0009)
#define WSA881X_TEMP_DOUT_MSB			(WSA881X_ANALOG_BASE + 0x000A)
#define WSA881X_TEMP_DOUT_LSB			(WSA881X_ANALOG_BASE + 0x000B)
#define WSA881X_ADC_EN_MODU_V			(WSA881X_ANALOG_BASE + 0x0010)
#define WSA881X_ADC_EN_MODU_I			(WSA881X_ANALOG_BASE + 0x0011)
#define WSA881X_ADC_EN_DET_TEST_V		(WSA881X_ANALOG_BASE + 0x0012)
#define WSA881X_ADC_EN_DET_TEST_I		(WSA881X_ANALOG_BASE + 0x0013)
#define WSA881X_ADC_SEL_IBIAS			(WSA881X_ANALOG_BASE + 0x0014)
#define WSA881X_ADC_EN_SEL_IBAIS		(WSA881X_ANALOG_BASE + 0x0015)
#define WSA881X_SPKR_DRV_EN			(WSA881X_ANALOG_BASE + 0x001A)
#define WSA881X_SPKR_DRV_GAIN			(WSA881X_ANALOG_BASE + 0x001B)
#define WSA881X_SPKR_DAC_CTL			(WSA881X_ANALOG_BASE + 0x001C)
#define WSA881X_SPKR_DRV_DBG			(WSA881X_ANALOG_BASE + 0x001D)
#define WSA881X_SPKR_PWRSTG_DBG			(WSA881X_ANALOG_BASE + 0x001E)
#define WSA881X_SPKR_OCP_CTL			(WSA881X_ANALOG_BASE + 0x001F)
#define WSA881X_SPKR_CLIP_CTL			(WSA881X_ANALOG_BASE + 0x0020)
#define WSA881X_SPKR_BBM_CTL			(WSA881X_ANALOG_BASE + 0x0021)
#define WSA881X_SPKR_MISC_CTL1			(WSA881X_ANALOG_BASE + 0x0022)
#define WSA881X_SPKR_MISC_CTL2			(WSA881X_ANALOG_BASE + 0x0023)
#define WSA881X_SPKR_BIAS_INT			(WSA881X_ANALOG_BASE + 0x0024)
#define WSA881X_SPKR_PA_INT			(WSA881X_ANALOG_BASE + 0x0025)
#define WSA881X_SPKR_BIAS_CAL			(WSA881X_ANALOG_BASE + 0x0026)
#define WSA881X_SPKR_BIAS_PSRR			(WSA881X_ANALOG_BASE + 0x0027)
#define WSA881X_SPKR_STATUS1			(WSA881X_ANALOG_BASE + 0x0028)
#define WSA881X_SPKR_STATUS2			(WSA881X_ANALOG_BASE + 0x0029)
#define WSA881X_BOOST_EN_CTL			(WSA881X_ANALOG_BASE + 0x002A)
#define WSA881X_BOOST_CURRENT_LIMIT		(WSA881X_ANALOG_BASE + 0x002B)
#define WSA881X_BOOST_PS_CTL			(WSA881X_ANALOG_BASE + 0x002C)
#define WSA881X_BOOST_PRESET_OUT1		(WSA881X_ANALOG_BASE + 0x002D)
#define WSA881X_BOOST_PRESET_OUT2		(WSA881X_ANALOG_BASE + 0x002E)
#define WSA881X_BOOST_FORCE_OUT			(WSA881X_ANALOG_BASE + 0x002F)
#define WSA881X_BOOST_LDO_PROG			(WSA881X_ANALOG_BASE + 0x0030)
#define WSA881X_BOOST_SLOPE_COMP_ISENSE_FB	(WSA881X_ANALOG_BASE + 0x0031)
#define WSA881X_BOOST_RON_CTL			(WSA881X_ANALOG_BASE + 0x0032)
#define WSA881X_BOOST_LOOP_STABILITY		(WSA881X_ANALOG_BASE + 0x0033)
#define WSA881X_BOOST_ZX_CTL			(WSA881X_ANALOG_BASE + 0x0034)
#define WSA881X_BOOST_START_CTL			(WSA881X_ANALOG_BASE + 0x0035)
#define WSA881X_BOOST_MISC1_CTL			(WSA881X_ANALOG_BASE + 0x0036)
#define WSA881X_BOOST_MISC2_CTL			(WSA881X_ANALOG_BASE + 0x0037)
#define WSA881X_BOOST_MISC3_CTL			(WSA881X_ANALOG_BASE + 0x0038)
#define WSA881X_BOOST_ATEST_CTL			(WSA881X_ANALOG_BASE + 0x0039)
#define WSA881X_SPKR_PROT_FE_GAIN		(WSA881X_ANALOG_BASE + 0x003A)
#define WSA881X_SPKR_PROT_FE_CM_LDO_SET		(WSA881X_ANALOG_BASE + 0x003B)
#define WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET1	(WSA881X_ANALOG_BASE + 0x003C)
#define WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET2	(WSA881X_ANALOG_BASE + 0x003D)
#define WSA881X_SPKR_PROT_ATEST1		(WSA881X_ANALOG_BASE + 0x003E)
#define WSA881X_SPKR_PROT_ATEST2		(WSA881X_ANALOG_BASE + 0x003F)
#define WSA881X_SPKR_PROT_FE_VSENSE_VCM		(WSA881X_ANALOG_BASE + 0x0040)
#define WSA881X_SPKR_PROT_FE_VSENSE_BIAS_SET1	(WSA881X_ANALOG_BASE + 0x0041)
#define WSA881X_BONGO_RESRV_REG1		(WSA881X_ANALOG_BASE + 0x0042)
#define WSA881X_BONGO_RESRV_REG2		(WSA881X_ANALOG_BASE + 0x0043)
#define WSA881X_SPKR_PROT_SAR			(WSA881X_ANALOG_BASE + 0x0044)
#define WSA881X_SPKR_STATUS3			(WSA881X_ANALOG_BASE + 0x0045)

#define SWRS_SCP_FRAME_CTRL_BANK(m)		(0x60 + 0x10 * (m))
#define SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(m)	(0xE0 + 0x10 * (m))

#define WSA881X_NUM_REGISTERS			(WSA881X_SPKR_STATUS3 + 1)
#define WSA881X_MAX_REGISTER			(WSA881X_NUM_REGISTERS - 1)
#define WSA881X_CACHE_SIZE			WSA881X_NUM_REGISTERS
#define SWR_SLV_MAX_REG_ADDR	0x390
#define SWR_SLV_START_REG_ADDR	0x40
#define SWR_SLV_MAX_BUF_LEN	20
#define BYTES_PER_LINE		12
#define SWR_SLV_RD_BUF_LEN	8
#define SWR_SLV_WR_BUF_LEN	32
#define SWR_SLV_MAX_DEVICES	2
#define WSA881X_MAX_SWR_PORTS   4
#define WSA881X_VERSION_ENTRY_SIZE 27
#define WSA881X_OCP_CTL_TIMER_SEC 2
#define WSA881X_OCP_CTL_TEMP_CELSIUS 25
#define WSA881X_OCP_CTL_POLL_TIMER_SEC 60

#define WSA881X_PA_GAIN_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = wsa881x_put_pa_gain, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, max, invert, 0) }

static const u8 wsa881x_reg_readable[WSA881X_CACHE_SIZE] = {
	[WSA881X_CHIP_ID0] = 1,
	[WSA881X_CHIP_ID1] = 1,
	[WSA881X_CHIP_ID2] = 1,
	[WSA881X_CHIP_ID3] = 1,
	[WSA881X_BUS_ID] = 1,
	[WSA881X_CDC_RST_CTL] = 1,
	[WSA881X_CDC_TOP_CLK_CTL] = 1,
	[WSA881X_CDC_ANA_CLK_CTL] = 1,
	[WSA881X_CDC_DIG_CLK_CTL] = 1,
	[WSA881X_CLOCK_CONFIG] = 1,
	[WSA881X_ANA_CTL] = 1,
	[WSA881X_SWR_RESET_EN] = 1,
	[WSA881X_RESET_CTL] = 1,
	[WSA881X_TADC_VALUE_CTL] = 1,
	[WSA881X_TEMP_DETECT_CTL] = 1,
	[WSA881X_TEMP_MSB] = 1,
	[WSA881X_TEMP_LSB] = 1,
	[WSA881X_TEMP_CONFIG0] = 1,
	[WSA881X_TEMP_CONFIG1] = 1,
	[WSA881X_CDC_CLIP_CTL] = 1,
	[WSA881X_SDM_PDM9_LSB] = 1,
	[WSA881X_SDM_PDM9_MSB] = 1,
	[WSA881X_CDC_RX_CTL] = 1,
	[WSA881X_DEM_BYPASS_DATA0] = 1,
	[WSA881X_DEM_BYPASS_DATA1] = 1,
	[WSA881X_DEM_BYPASS_DATA2] = 1,
	[WSA881X_DEM_BYPASS_DATA3] = 1,
	[WSA881X_OTP_CTRL0] = 1,
	[WSA881X_OTP_CTRL1] = 1,
	[WSA881X_HDRIVE_CTL_GROUP1] = 1,
	[WSA881X_INTR_MODE] = 1,
	[WSA881X_INTR_MASK] = 1,
	[WSA881X_INTR_STATUS] = 1,
	[WSA881X_INTR_CLEAR] = 1,
	[WSA881X_INTR_LEVEL] = 1,
	[WSA881X_INTR_SET] = 1,
	[WSA881X_INTR_TEST] = 1,
	[WSA881X_PDM_TEST_MODE] = 1,
	[WSA881X_ATE_TEST_MODE] = 1,
	[WSA881X_PIN_CTL_MODE] = 1,
	[WSA881X_PIN_CTL_OE] = 1,
	[WSA881X_PIN_WDATA_IOPAD] = 1,
	[WSA881X_PIN_STATUS] = 1,
	[WSA881X_DIG_DEBUG_MODE] = 1,
	[WSA881X_DIG_DEBUG_SEL] = 1,
	[WSA881X_DIG_DEBUG_EN] = 1,
	[WSA881X_SWR_HM_TEST1] = 1,
	[WSA881X_SWR_HM_TEST2] = 1,
	[WSA881X_TEMP_DETECT_DBG_CTL] = 1,
	[WSA881X_TEMP_DEBUG_MSB] = 1,
	[WSA881X_TEMP_DEBUG_LSB] = 1,
	[WSA881X_SAMPLE_EDGE_SEL] = 1,
	[WSA881X_IOPAD_CTL] = 1,
	[WSA881X_SPARE_0] = 1,
	[WSA881X_SPARE_1] = 1,
	[WSA881X_SPARE_2] = 1,
	[WSA881X_OTP_REG_0] = 1,
	[WSA881X_OTP_REG_1] = 1,
	[WSA881X_OTP_REG_2] = 1,
	[WSA881X_OTP_REG_3] = 1,
	[WSA881X_OTP_REG_4] = 1,
	[WSA881X_OTP_REG_5] = 1,
	[WSA881X_OTP_REG_6] = 1,
	[WSA881X_OTP_REG_7] = 1,
	[WSA881X_OTP_REG_8] = 1,
	[WSA881X_OTP_REG_9] = 1,
	[WSA881X_OTP_REG_10] = 1,
	[WSA881X_OTP_REG_11] = 1,
	[WSA881X_OTP_REG_12] = 1,
	[WSA881X_OTP_REG_13] = 1,
	[WSA881X_OTP_REG_14] = 1,
	[WSA881X_OTP_REG_15] = 1,
	[WSA881X_OTP_REG_16] = 1,
	[WSA881X_OTP_REG_17] = 1,
	[WSA881X_OTP_REG_18] = 1,
	[WSA881X_OTP_REG_19] = 1,
	[WSA881X_OTP_REG_20] = 1,
	[WSA881X_OTP_REG_21] = 1,
	[WSA881X_OTP_REG_22] = 1,
	[WSA881X_OTP_REG_23] = 1,
	[WSA881X_OTP_REG_24] = 1,
	[WSA881X_OTP_REG_25] = 1,
	[WSA881X_OTP_REG_26] = 1,
	[WSA881X_OTP_REG_27] = 1,
	[WSA881X_OTP_REG_28] = 1,
	[WSA881X_OTP_REG_29] = 1,
	[WSA881X_OTP_REG_30] = 1,
	[WSA881X_OTP_REG_31] = 1,
	[WSA881X_OTP_REG_63] = 1,
	/* Analog Registers */
	[WSA881X_BIAS_REF_CTRL] = 1,
	[WSA881X_BIAS_TEST] = 1,
	[WSA881X_BIAS_BIAS] = 1,
	[WSA881X_TEMP_OP] = 1,
	[WSA881X_TEMP_IREF_CTRL] = 1,
	[WSA881X_TEMP_ISENS_CTRL] = 1,
	[WSA881X_TEMP_CLK_CTRL] = 1,
	[WSA881X_TEMP_TEST] = 1,
	[WSA881X_TEMP_BIAS] = 1,
	[WSA881X_TEMP_ADC_CTRL] = 1,
	[WSA881X_TEMP_DOUT_MSB] = 1,
	[WSA881X_TEMP_DOUT_LSB] = 1,
	[WSA881X_ADC_EN_MODU_V] = 1,
	[WSA881X_ADC_EN_MODU_I] = 1,
	[WSA881X_ADC_EN_DET_TEST_V] = 1,
	[WSA881X_ADC_EN_DET_TEST_I] = 1,
	[WSA881X_ADC_SEL_IBIAS] = 1,
	[WSA881X_ADC_EN_SEL_IBAIS] = 1,
	[WSA881X_SPKR_DRV_EN] = 1,
	[WSA881X_SPKR_DRV_GAIN] = 1,
	[WSA881X_SPKR_DAC_CTL] = 1,
	[WSA881X_SPKR_DRV_DBG] = 1,
	[WSA881X_SPKR_PWRSTG_DBG] = 1,
	[WSA881X_SPKR_OCP_CTL] = 1,
	[WSA881X_SPKR_CLIP_CTL] = 1,
	[WSA881X_SPKR_BBM_CTL] = 1,
	[WSA881X_SPKR_MISC_CTL1] = 1,
	[WSA881X_SPKR_MISC_CTL2] = 1,
	[WSA881X_SPKR_BIAS_INT] = 1,
	[WSA881X_SPKR_PA_INT] = 1,
	[WSA881X_SPKR_BIAS_CAL] = 1,
	[WSA881X_SPKR_BIAS_PSRR] = 1,
	[WSA881X_SPKR_STATUS1] = 1,
	[WSA881X_SPKR_STATUS2] = 1,
	[WSA881X_BOOST_EN_CTL] = 1,
	[WSA881X_BOOST_CURRENT_LIMIT] = 1,
	[WSA881X_BOOST_PS_CTL] = 1,
	[WSA881X_BOOST_PRESET_OUT1] = 1,
	[WSA881X_BOOST_PRESET_OUT2] = 1,
	[WSA881X_BOOST_FORCE_OUT] = 1,
	[WSA881X_BOOST_LDO_PROG] = 1,
	[WSA881X_BOOST_SLOPE_COMP_ISENSE_FB] = 1,
	[WSA881X_BOOST_RON_CTL] = 1,
	[WSA881X_BOOST_LOOP_STABILITY] = 1,
	[WSA881X_BOOST_ZX_CTL] = 1,
	[WSA881X_BOOST_START_CTL] = 1,
	[WSA881X_BOOST_MISC1_CTL] = 1,
	[WSA881X_BOOST_MISC2_CTL] = 1,
	[WSA881X_BOOST_MISC3_CTL] = 1,
	[WSA881X_BOOST_ATEST_CTL] = 1,
	[WSA881X_SPKR_PROT_FE_GAIN] = 1,
	[WSA881X_SPKR_PROT_FE_CM_LDO_SET] = 1,
	[WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET1] = 1,
	[WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET2] = 1,
	[WSA881X_SPKR_PROT_ATEST1] = 1,
	[WSA881X_SPKR_PROT_ATEST2] = 1,
	[WSA881X_SPKR_PROT_FE_VSENSE_VCM] = 1,
	[WSA881X_SPKR_PROT_FE_VSENSE_BIAS_SET1] = 1,
	[WSA881X_BONGO_RESRV_REG1] = 1,
	[WSA881X_BONGO_RESRV_REG2] = 1,
	[WSA881X_SPKR_PROT_SAR] = 1,
	[WSA881X_SPKR_STATUS3] = 1,
};

static struct reg_default wsa881x_defaults[] = {
	{WSA881X_CHIP_ID0, 0x00},
	{WSA881X_CHIP_ID1, 0x00},
	{WSA881X_CHIP_ID2, 0x00},
	{WSA881X_CHIP_ID3, 0x02},
	{WSA881X_BUS_ID, 0x00},
	{WSA881X_CDC_RST_CTL, 0x00},
	{WSA881X_CDC_TOP_CLK_CTL, 0x03},
	{WSA881X_CDC_ANA_CLK_CTL, 0x00},
	{WSA881X_CDC_DIG_CLK_CTL, 0x00},
	{WSA881X_CLOCK_CONFIG, 0x00},
	{WSA881X_ANA_CTL, 0x08},
	{WSA881X_SWR_RESET_EN, 0x00},
	{WSA881X_TEMP_DETECT_CTL, 0x01},
	{WSA881X_TEMP_MSB, 0x00},
	{WSA881X_TEMP_LSB, 0x00},
	{WSA881X_TEMP_CONFIG0, 0x00},
	{WSA881X_TEMP_CONFIG1, 0x00},
	{WSA881X_CDC_CLIP_CTL, 0x03},
	{WSA881X_SDM_PDM9_LSB, 0x00},
	{WSA881X_SDM_PDM9_MSB, 0x00},
	{WSA881X_CDC_RX_CTL, 0x7E},
	{WSA881X_DEM_BYPASS_DATA0, 0x00},
	{WSA881X_DEM_BYPASS_DATA1, 0x00},
	{WSA881X_DEM_BYPASS_DATA2, 0x00},
	{WSA881X_DEM_BYPASS_DATA3, 0x00},
	{WSA881X_OTP_CTRL0, 0x00},
	{WSA881X_OTP_CTRL1, 0x00},
	{WSA881X_HDRIVE_CTL_GROUP1, 0x00},
	{WSA881X_INTR_MODE, 0x00},
	{WSA881X_INTR_STATUS, 0x00},
	{WSA881X_INTR_CLEAR, 0x00},
	{WSA881X_INTR_LEVEL, 0x00},
	{WSA881X_INTR_SET, 0x00},
	{WSA881X_INTR_TEST, 0x00},
	{WSA881X_PDM_TEST_MODE, 0x00},
	{WSA881X_ATE_TEST_MODE, 0x00},
	{WSA881X_PIN_CTL_MODE, 0x00},
	{WSA881X_PIN_CTL_OE, 0x00},
	{WSA881X_PIN_WDATA_IOPAD, 0x00},
	{WSA881X_PIN_STATUS, 0x00},
	{WSA881X_DIG_DEBUG_MODE, 0x00},
	{WSA881X_DIG_DEBUG_SEL, 0x00},
	{WSA881X_DIG_DEBUG_EN, 0x00},
	{WSA881X_SWR_HM_TEST1, 0x08},
	{WSA881X_SWR_HM_TEST2, 0x00},
	{WSA881X_TEMP_DETECT_DBG_CTL, 0x00},
	{WSA881X_TEMP_DEBUG_MSB, 0x00},
	{WSA881X_TEMP_DEBUG_LSB, 0x00},
	{WSA881X_SAMPLE_EDGE_SEL, 0x0C},
	{WSA881X_SPARE_0, 0x00},
	{WSA881X_SPARE_1, 0x00},
	{WSA881X_SPARE_2, 0x00},
	{WSA881X_OTP_REG_0, 0x01},
	{WSA881X_OTP_REG_1, 0xFF},
	{WSA881X_OTP_REG_2, 0xC0},
	{WSA881X_OTP_REG_3, 0xFF},
	{WSA881X_OTP_REG_4, 0xC0},
	{WSA881X_OTP_REG_5, 0xFF},
	{WSA881X_OTP_REG_6, 0xFF},
	{WSA881X_OTP_REG_7, 0xFF},
	{WSA881X_OTP_REG_8, 0xFF},
	{WSA881X_OTP_REG_9, 0xFF},
	{WSA881X_OTP_REG_10, 0xFF},
	{WSA881X_OTP_REG_11, 0xFF},
	{WSA881X_OTP_REG_12, 0xFF},
	{WSA881X_OTP_REG_13, 0xFF},
	{WSA881X_OTP_REG_14, 0xFF},
	{WSA881X_OTP_REG_15, 0xFF},
	{WSA881X_OTP_REG_16, 0xFF},
	{WSA881X_OTP_REG_17, 0xFF},
	{WSA881X_OTP_REG_18, 0xFF},
	{WSA881X_OTP_REG_19, 0xFF},
	{WSA881X_OTP_REG_20, 0xFF},
	{WSA881X_OTP_REG_21, 0xFF},
	{WSA881X_OTP_REG_22, 0xFF},
	{WSA881X_OTP_REG_23, 0xFF},
	{WSA881X_OTP_REG_24, 0x03},
	{WSA881X_OTP_REG_25, 0x01},
	{WSA881X_OTP_REG_26, 0x03},
	{WSA881X_OTP_REG_27, 0x11},
	{WSA881X_OTP_REG_63, 0x40},
	/* WSA881x Analog registers */
	{WSA881X_BIAS_REF_CTRL, 0x6C},
	{WSA881X_BIAS_TEST, 0x16},
	{WSA881X_BIAS_BIAS, 0xF0},
	{WSA881X_TEMP_OP, 0x00},
	{WSA881X_TEMP_IREF_CTRL, 0x56},
	{WSA881X_TEMP_ISENS_CTRL, 0x47},
	{WSA881X_TEMP_CLK_CTRL, 0x87},
	{WSA881X_TEMP_TEST, 0x00},
	{WSA881X_TEMP_BIAS, 0x51},
	{WSA881X_TEMP_DOUT_MSB, 0x00},
	{WSA881X_TEMP_DOUT_LSB, 0x00},
	{WSA881X_ADC_EN_MODU_V, 0x00},
	{WSA881X_ADC_EN_MODU_I, 0x00},
	{WSA881X_ADC_EN_DET_TEST_V, 0x00},
	{WSA881X_ADC_EN_DET_TEST_I, 0x00},
	{WSA881X_ADC_EN_SEL_IBAIS, 0x10},
	{WSA881X_SPKR_DRV_EN, 0x74},
	{WSA881X_SPKR_DRV_DBG, 0x15},
	{WSA881X_SPKR_PWRSTG_DBG, 0x00},
	{WSA881X_SPKR_OCP_CTL, 0xD4},
	{WSA881X_SPKR_CLIP_CTL, 0x90},
	{WSA881X_SPKR_PA_INT, 0x54},
	{WSA881X_SPKR_BIAS_CAL, 0xAC},
	{WSA881X_SPKR_STATUS1, 0x00},
	{WSA881X_SPKR_STATUS2, 0x00},
	{WSA881X_BOOST_EN_CTL, 0x18},
	{WSA881X_BOOST_CURRENT_LIMIT, 0x7A},
	{WSA881X_BOOST_PRESET_OUT2, 0x70},
	{WSA881X_BOOST_FORCE_OUT, 0x0E},
	{WSA881X_BOOST_LDO_PROG, 0x16},
	{WSA881X_BOOST_SLOPE_COMP_ISENSE_FB, 0x71},
	{WSA881X_BOOST_RON_CTL, 0x0F},
	{WSA881X_BOOST_ZX_CTL, 0x34},
	{WSA881X_BOOST_START_CTL, 0x23},
	{WSA881X_BOOST_MISC1_CTL, 0x80},
	{WSA881X_BOOST_MISC2_CTL, 0x00},
	{WSA881X_BOOST_MISC3_CTL, 0x00},
	{WSA881X_BOOST_ATEST_CTL, 0x00},
	{WSA881X_SPKR_PROT_FE_GAIN, 0x46},
	{WSA881X_SPKR_PROT_FE_CM_LDO_SET, 0x3B},
	{WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET1, 0x8D},
	{WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET2, 0x8D},
	{WSA881X_SPKR_PROT_ATEST1, 0x01},
	{WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x8D},
	{WSA881X_SPKR_PROT_FE_VSENSE_BIAS_SET1, 0x4D},
	{WSA881X_SPKR_PROT_SAR, 0x00},
	{WSA881X_SPKR_STATUS3, 0x00},
};

static const struct reg_sequence wsa881x_pre_pmu_pa[] = {
	{WSA881X_SPKR_DRV_GAIN, 0x41, 0},
	{WSA881X_SPKR_MISC_CTL1, 0x01, 0},
	{WSA881X_ADC_EN_DET_TEST_I, 0x01, 0},
	{WSA881X_ADC_EN_MODU_V, 0x02, 0},
	{WSA881X_ADC_EN_DET_TEST_V, 0x10, 0},
	{WSA881X_SPKR_PWRSTG_DBG, 0xA0, 0},
};

static const struct reg_sequence wsa881x_pre_pmu_pa_2_0[] = {
	{WSA881X_SPKR_DRV_GAIN, 0x41, 0},
	{WSA881X_SPKR_MISC_CTL1, 0x87, 0},
};

static const struct reg_sequence wsa881x_post_pmu_pa[] = {
	{WSA881X_SPKR_PWRSTG_DBG, 0x00, 0},
	{WSA881X_ADC_EN_DET_TEST_V, 0x00, 0},
	{WSA881X_ADC_EN_MODU_V, 0x00, 0},
	{WSA881X_ADC_EN_DET_TEST_I, 0x00, 0},
};

static const struct reg_sequence wsa881x_vi_txfe_en[] = {
	{WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x85, 0},
	{WSA881X_SPKR_PROT_ATEST2, 0x0A, 0},
	{WSA881X_SPKR_PROT_FE_GAIN, 0xCF, 0},
};

static const struct reg_sequence wsa881x_vi_txfe_en_2_0[] = {
	{WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x85, 0},
	{WSA881X_SPKR_PROT_ATEST2, 0x0A, 0},
	{WSA881X_SPKR_PROT_FE_GAIN, 0x47, 0},
};

/* Default register reset values for WSA881x rev 2.0 */
static struct reg_sequence wsa881x_rev_2_0[] = {
	{WSA881X_RESET_CTL, 0x00, 0x00},
	{WSA881X_TADC_VALUE_CTL, 0x01, 0x00},
	{WSA881X_INTR_MASK, 0x1B, 0x00},
	{WSA881X_IOPAD_CTL, 0x00, 0x00},
	{WSA881X_OTP_REG_28, 0x3F, 0x00},
	{WSA881X_OTP_REG_29, 0x3F, 0x00},
	{WSA881X_OTP_REG_30, 0x01, 0x00},
	{WSA881X_OTP_REG_31, 0x01, 0x00},
	{WSA881X_TEMP_ADC_CTRL, 0x03, 0x00},
	{WSA881X_ADC_SEL_IBIAS, 0x45, 0x00},
	{WSA881X_SPKR_DRV_GAIN, 0xC1, 0x00},
	{WSA881X_SPKR_DAC_CTL, 0x42, 0x00},
	{WSA881X_SPKR_BBM_CTL, 0x02, 0x00},
	{WSA881X_SPKR_MISC_CTL1, 0x40, 0x00},
	{WSA881X_SPKR_MISC_CTL2, 0x07, 0x00},
	{WSA881X_SPKR_BIAS_INT, 0x5F, 0x00},
	{WSA881X_SPKR_BIAS_PSRR, 0x44, 0x00},
	{WSA881X_BOOST_PS_CTL, 0xA0, 0x00},
	{WSA881X_BOOST_PRESET_OUT1, 0xB7, 0x00},
	{WSA881X_BOOST_LOOP_STABILITY, 0x8D, 0x00},
	{WSA881X_SPKR_PROT_ATEST2, 0x02, 0x00},
	{WSA881X_BONGO_RESRV_REG1, 0x5E, 0x00},
	{WSA881X_BONGO_RESRV_REG2, 0x07, 0x00},
};

enum wsa_port_ids {
	WSA881X_PORT_DAC,
	WSA881X_PORT_COMP,
	WSA881X_PORT_BOOST,
	WSA881X_PORT_VISENSE,
};

/* 4 ports */
static struct sdw_dpn_prop wsa_sink_dpn_prop[WSA881X_MAX_SWR_PORTS] = {
	{
		/* DAC */
		.num = 1,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 8,
		.simple_ch_prep_sm = true,
	}, {
		/* COMP */
		.num = 2,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 8,
		.simple_ch_prep_sm = true,
	}, {
		/* BOOST */
		.num = 3,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 8,
		.simple_ch_prep_sm = true,
	}, {
		/* VISENSE */
		.num = 4,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 8,
		.simple_ch_prep_sm = true,
	}
};

static bool wsa881x_readable_register(struct device *dev, unsigned int reg)
{
	return wsa881x_reg_readable[reg];
}

static bool wsa881x_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case WSA881X_CHIP_ID0:
	case WSA881X_CHIP_ID1:
	case WSA881X_CHIP_ID2:
	case WSA881X_CHIP_ID3:
	case WSA881X_BUS_ID:
	case WSA881X_TEMP_MSB:
	case WSA881X_TEMP_LSB:
	case WSA881X_SDM_PDM9_LSB:
	case WSA881X_SDM_PDM9_MSB:
	case WSA881X_OTP_CTRL1:
	case WSA881X_INTR_STATUS:
	case WSA881X_ATE_TEST_MODE:
	case WSA881X_PIN_STATUS:
	case WSA881X_SWR_HM_TEST2:
	case WSA881X_SPKR_STATUS1:
	case WSA881X_SPKR_STATUS2:
	case WSA881X_SPKR_STATUS3:
	case WSA881X_OTP_REG_0:
	case WSA881X_OTP_REG_1:
	case WSA881X_OTP_REG_2:
	case WSA881X_OTP_REG_3:
	case WSA881X_OTP_REG_4:
	case WSA881X_OTP_REG_5:
	case WSA881X_OTP_REG_31:
	case WSA881X_TEMP_DOUT_MSB:
	case WSA881X_TEMP_DOUT_LSB:
	case WSA881X_TEMP_OP:
	case WSA881X_SPKR_PROT_SAR:
		return true;
	default:
		return false;
	}
}

static struct regmap_config wsa881x_regmap_config = {
	.reg_bits = 32,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = wsa881x_defaults,
	.num_reg_defaults = ARRAY_SIZE(wsa881x_defaults),
	.max_register = WSA881X_MAX_REGISTER,
	.volatile_reg = wsa881x_volatile_register,
	.readable_reg = wsa881x_readable_register,
	.reg_format_endian = REGMAP_ENDIAN_NATIVE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.can_multi_write = true,
};

enum {
	G_18DB = 0,
	G_16P5DB,
	G_15DB,
	G_13P5DB,
	G_12DB,
	G_10P5DB,
	G_9DB,
	G_7P5DB,
	G_6DB,
	G_4P5DB,
	G_3DB,
	G_1P5DB,
	G_0DB,
};

/*
 * Private data Structure for wsa881x. All parameters related to
 * WSA881X codec needs to be defined here.
 */
struct wsa881x_priv {
	struct regmap *regmap;
	struct device *dev;
	struct sdw_slave *slave;
	struct gpio_desc *sd_n;
	int bg_cnt;
	int clk_cnt;
	int version;
	/* bandgap lock */
	struct mutex bg_lock;
	/* clk resource lock */
	struct mutex res_lock;
	bool port_prepared[WSA881X_MAX_SWR_PORTS];
	u8 pa_gain;
};

static void wsa881x_init(struct snd_soc_component *comp)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	wsa881x->version = snd_soc_component_read32(comp, WSA881X_CHIP_ID1);
	regcache_cache_only(wsa881x->regmap, true);
	regmap_multi_reg_write(wsa881x->regmap, wsa881x_rev_2_0,
			       ARRAY_SIZE(wsa881x_rev_2_0));
	regcache_cache_only(wsa881x->regmap, false);
	/* Enable software reset output from soundwire slave */
	snd_soc_component_update_bits(comp, WSA881X_SWR_RESET_EN, 0x07, 0x07);
	/* Bring out of analog reset */
	snd_soc_component_update_bits(comp, WSA881X_CDC_RST_CTL, 0x02, 0x02);
	/* Bring out of digital reset */
	snd_soc_component_update_bits(comp, WSA881X_CDC_RST_CTL, 0x01, 0x01);

	snd_soc_component_update_bits(comp, WSA881X_CLOCK_CONFIG, 0x10, 0x10);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_OCP_CTL, 0x02, 0x02);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_MISC_CTL1, 0xC0, 0x80);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_MISC_CTL1, 0x06, 0x06);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_BIAS_INT, 0xFF, 0x00);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_PA_INT, 0xF0, 0x40);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_PA_INT, 0x0E, 0x0E);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_LOOP_STABILITY,
				      0x03, 0x03);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_MISC2_CTL,
				      0xFF, 0x14);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_START_CTL,
				      0x80, 0x80);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_START_CTL,
				      0x03, 0x00);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_SLOPE_COMP_ISENSE_FB,
				      0x0C, 0x04);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_SLOPE_COMP_ISENSE_FB,
				      0x03, 0x00);
	if (snd_soc_component_read32(comp, WSA881X_OTP_REG_0))
		snd_soc_component_update_bits(comp, WSA881X_BOOST_PRESET_OUT1,
					      0xF0, 0x70);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_PRESET_OUT2,
				      0xF0, 0x30);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_DRV_EN, 0x08, 0x08);
	snd_soc_component_update_bits(comp, WSA881X_BOOST_CURRENT_LIMIT,
				      0x0F, 0x08);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_OCP_CTL, 0x30, 0x30);
	snd_soc_component_update_bits(comp, WSA881X_SPKR_OCP_CTL, 0x0C, 0x00);
	snd_soc_component_update_bits(comp, WSA881X_OTP_REG_28, 0x3F, 0x3A);
	snd_soc_component_update_bits(comp, WSA881X_BONGO_RESRV_REG1,
				      0xFF, 0xB2);
	snd_soc_component_update_bits(comp, WSA881X_BONGO_RESRV_REG2,
				      0xFF, 0x05);
}

static int wsa881x_component_probe(struct snd_soc_component *comp)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	if (!wsa881x)
		return -EINVAL;

	snd_soc_component_init_regmap(comp, wsa881x->regmap);

	mutex_init(&wsa881x->bg_lock);
	mutex_init(&wsa881x->res_lock);
	wsa881x_init(comp);
	wsa881x->bg_cnt = 0;
	wsa881x->clk_cnt = 0;

	return 0;
}

static int wsa881x_put_pa_gain(struct snd_kcontrol *kc,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kc);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kc->private_value;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;

	/*
	 * program actual register just before compander enable and ensure hw
	 * sequence is followed
	 */
	wsa881x->pa_gain = (max - ucontrol->value.integer.value[0]) & mask;

	return 0;
}

static const char * const smart_boost_lvl_text[] = {
	"6.625 V", "6.750 V", "6.875 V", "7.000 V",
	"7.125 V", "7.250 V", "7.375 V", "7.500 V",
	"7.625 V", "7.750 V", "7.875 V", "8.000 V",
	"8.125 V", "8.250 V", "8.375 V", "8.500 V"
};

static const struct soc_enum smart_boost_lvl_enum =
	SOC_ENUM_SINGLE(WSA881X_BOOST_PRESET_OUT1, 0,
			ARRAY_SIZE(smart_boost_lvl_text),
			smart_boost_lvl_text);

static const DECLARE_TLV_DB_SCALE(pa_gain, 0, 150, 0);

static const struct snd_kcontrol_new wsa881x_snd_controls[] = {
	SOC_ENUM("Smart Boost Level", smart_boost_lvl_enum),
	WSA881X_PA_GAIN_TLV("PA Gain", WSA881X_SPKR_DRV_GAIN,
			    4, 0xC, 1, pa_gain),
	SOC_SINGLE("PA Mute Switch", WSA881X_SPKR_DRV_EN, 7, 1, 1),
};

static const struct snd_soc_dapm_route wsa881x_audio_map[] = {
	{"RDAC", NULL, "IN"},
	{"SPKR PGA", NULL, "RDAC"},
	{"SPKR", NULL, "SPKR PGA"},
};

static void wsa881x_clk_ctrl(struct snd_soc_component *comp, bool enable)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	mutex_lock(&wsa881x->res_lock);
	if (enable) {
		++wsa881x->clk_cnt;
		if (wsa881x->clk_cnt == 1) {
			snd_soc_component_write(comp,
						WSA881X_CDC_DIG_CLK_CTL,
						enable);
			snd_soc_component_write(comp,
						WSA881X_CDC_ANA_CLK_CTL,
						enable);
		}
	} else {
		--wsa881x->clk_cnt;
		if (wsa881x->clk_cnt <= 0) {
			WARN_ON(wsa881x->clk_cnt < 0);
			wsa881x->clk_cnt = 0;
			snd_soc_component_write(comp,
						WSA881X_CDC_DIG_CLK_CTL,
						enable);
			snd_soc_component_write(comp,
						WSA881X_CDC_ANA_CLK_CTL,
						enable);
		}
	}
	mutex_unlock(&wsa881x->res_lock);
}

static void wsa881x_bandgap_ctrl(struct snd_soc_component *comp, bool enable)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	mutex_lock(&wsa881x->bg_lock);
	if (enable) {
		++wsa881x->bg_cnt;
		if (wsa881x->bg_cnt == 1) {
			snd_soc_component_update_bits(comp, WSA881X_TEMP_OP,
						      0x08, 0x08);
			/* 400usec sleep is needed as per HW requirement */
			usleep_range(400, 410);
			snd_soc_component_update_bits(comp, WSA881X_TEMP_OP,
						      0x04, 0x04);
		}
	} else {
		--wsa881x->bg_cnt;
		if (wsa881x->bg_cnt <= 0) {
			WARN_ON(wsa881x->bg_cnt < 0);
			wsa881x->bg_cnt = 0;
			snd_soc_component_update_bits(comp, WSA881X_TEMP_OP,
						      0x04, 0x00);
			snd_soc_component_update_bits(comp, WSA881X_TEMP_OP,
						      0x08, 0x00);
		}
	}
	mutex_unlock(&wsa881x->bg_lock);
}

static int wsa881x_boost_ctrl(struct snd_soc_component *comp, bool enable)
{
	if (enable)
		snd_soc_component_update_bits(comp, WSA881X_BOOST_EN_CTL,
					      0x80, 0x80);
	else
		snd_soc_component_update_bits(comp, WSA881X_BOOST_EN_CTL,
					      0x80, 0x00);
	/*
	 * 1.5ms sleep is needed after boost enable/disable as per
	 * HW requirement
	 */
	usleep_range(1500, 1510);
	return 0;
}

static int32_t wsa881x_resource_acquire(struct snd_soc_component *comp,
					bool enable)
{
	wsa881x_clk_ctrl(comp, enable);
	wsa881x_bandgap_ctrl(comp, enable);

	return 0;
}

static int wsa881x_rdac_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wsa881x_resource_acquire(comp, true);
		wsa881x_boost_ctrl(comp, true);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wsa881x_boost_ctrl(comp, false);
		wsa881x_resource_acquire(comp, false);
		break;
	}
	return 0;
}

static int wsa881x_ramp_pa_gain(struct snd_soc_component *comp,
				int min_gain, int max_gain, int udelay)
{
	int val;

	for (val = min_gain; max_gain <= val; val--) {
		snd_soc_component_update_bits(comp, WSA881X_SPKR_DRV_GAIN,
					      0xF0, val << 4);
		/*
		 * 1ms delay is needed for every step change in gain as per
		 * HW requirement.
		 */
		usleep_range(udelay, udelay + 10);
	}
	return 0;
}

static int wsa881x_visense_txfe_ctrl(struct snd_soc_component *comp,
				     bool enable, u8 isense1_gain,
				     u8 isense2_gain,
				     u8 vsense_gain)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	if (enable) {
		regmap_multi_reg_write(wsa881x->regmap,
				       wsa881x_vi_txfe_en_2_0,
				ARRAY_SIZE(wsa881x_vi_txfe_en_2_0));
	} else {
		snd_soc_component_update_bits(comp,
					      WSA881X_SPKR_PROT_FE_VSENSE_VCM,
					      0x08, 0x08);
		/*
		 * 200us sleep is needed after visense txfe disable as per
		 * HW requirement.
		 */
		usleep_range(200, 210);
		snd_soc_component_update_bits(comp, WSA881X_SPKR_PROT_FE_GAIN,
					      0x01, 0x00);
	}
	return 0;
}

static int wsa881x_visense_adc_ctrl(struct snd_soc_component *comp,
				    bool enable)
{
	snd_soc_component_update_bits(comp, WSA881X_ADC_EN_MODU_V, (0x01 << 7),
				      (enable << 7));
	snd_soc_component_update_bits(comp, WSA881X_ADC_EN_MODU_I, (0x01 << 7),
				      (enable << 7));
	return 0;
}

static int wsa881x_spkr_pa_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);
	int min_gain, max_gain;

	dev_err(comp->dev, "%s: %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, WSA881X_SPKR_OCP_CTL,
					      0xC0, 0x80);
		regmap_multi_reg_write(wsa881x->regmap,
				       wsa881x_pre_pmu_pa_2_0,
				ARRAY_SIZE(wsa881x_pre_pmu_pa_2_0));

		/* Set register mode if compander is not enabled */
		if (!wsa881x->port_prepared[WSA881X_PORT_COMP])
			snd_soc_component_update_bits(comp,
						      WSA881X_SPKR_DRV_GAIN,
						      0x08, 0x08);
		else
			snd_soc_component_update_bits(comp,
						      WSA881X_SPKR_DRV_GAIN,
						      0x08, 0x00);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (!wsa881x->port_prepared[WSA881X_PORT_COMP]) {
			max_gain = wsa881x->pa_gain;
			/*
			 * Gain has to set incrementally in 4 steps
			 * as per HW sequence
			 */
			if (max_gain > G_4P5DB)
				min_gain = G_0DB;
			else
				min_gain = max_gain + 3;
			/*
			 * 1ms delay is needed before change in gain
			 * as per HW requirement.
			 */
			usleep_range(1000, 1010);
			wsa881x_ramp_pa_gain(comp, min_gain, max_gain, 1000);
		}

		if (wsa881x->port_prepared[WSA881X_PORT_VISENSE]) {
			wsa881x_visense_txfe_ctrl(comp, true,
						  0x00, 0x03, 0x01);
			snd_soc_component_update_bits(comp,
						      WSA881X_ADC_EN_SEL_IBAIS,
						      0x07, 0x01);
			wsa881x_visense_adc_ctrl(comp, true);
		}

		break;
	case SND_SOC_DAPM_POST_PMD:
		if (wsa881x->port_prepared[WSA881X_PORT_VISENSE]) {
			wsa881x_visense_adc_ctrl(comp, false);
			wsa881x_visense_txfe_ctrl(comp, false,
						  0x00, 0x01, 0x01);
		}

		snd_soc_component_update_bits(comp, WSA881X_SPKR_OCP_CTL,
					      0xC0, 0xC0);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget wsa881x_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_DAC_E("RDAC", NULL, WSA881X_SPKR_DAC_CTL, 7, 0,
			   wsa881x_rdac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("SPKR PGA", WSA881X_SPKR_DRV_EN, 7, 0, NULL, 0,
			   wsa881x_spkr_pa_event, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("SPKR"),
};

static void wsa881x_component_remove(struct snd_soc_component *comp)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	mutex_destroy(&wsa881x->bg_lock);
	mutex_destroy(&wsa881x->res_lock);
}

static const struct snd_soc_component_driver wsa881x_component_drv = {
	.probe = wsa881x_component_probe,
	.controls = wsa881x_snd_controls,
	.num_controls = ARRAY_SIZE(wsa881x_snd_controls),
	.remove = wsa881x_component_remove,
	.dapm_widgets = wsa881x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wsa881x_dapm_widgets),
	.dapm_routes = wsa881x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wsa881x_audio_map),
};

static int wsa881x_update_status(struct sdw_slave *slave,
				 enum sdw_slave_status status)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(&slave->dev);

	if (status == SDW_SLAVE_ATTACHED) {
		if (!wsa881x->regmap) {
			wsa881x->regmap = devm_regmap_init_sdw(slave,
						       &wsa881x_regmap_config);
			if (IS_ERR(wsa881x->regmap)) {
				dev_err(&slave->dev, "regmap_init failed\n");
				return PTR_ERR(wsa881x->regmap);
			}
		}

		return snd_soc_register_component(&slave->dev,
						  &wsa881x_component_drv,
						  NULL, 0);
	} else if (status == SDW_SLAVE_UNATTACHED) {
		snd_soc_unregister_component(&slave->dev);
	}

	return 0;
}

static int wsa881x_port_prep(struct sdw_slave *slave,
			     struct sdw_prepare_ch *prepare_ch,
			     enum sdw_port_prep_ops state)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(&slave->dev);

	if (state == SDW_OPS_PORT_POST_PREP)
		wsa881x->port_prepared[prepare_ch->num - 1] = true;
	else
		wsa881x->port_prepared[prepare_ch->num - 1] = false;

	return 0;
}

static int wsa881x_bus_config(struct sdw_slave *slave,
			      struct sdw_bus_params *params)
{
	sdw_write(slave, SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(params->next_bank),
		  0x01);

	return 0;
}

static struct sdw_slave_ops wsa881x_slave_ops = {
	.update_status = wsa881x_update_status,
	.bus_config = wsa881x_bus_config,
	.port_prep = wsa881x_port_prep,
};

static int wsa881x_probe(struct sdw_slave *pdev,
			 const struct sdw_device_id *id)
{
	struct wsa881x_priv *wsa881x;

	wsa881x = devm_kzalloc(&pdev->dev, sizeof(*wsa881x), GFP_KERNEL);
	if (!wsa881x)
		return -ENOMEM;

	wsa881x->sd_n = devm_gpiod_get_optional(&pdev->dev, "pd",
						GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(wsa881x->sd_n)) {
		dev_err(&pdev->dev, "Shutdown Control GPIO not found\n");
		return PTR_ERR(wsa881x->sd_n);
	}

	dev_set_drvdata(&pdev->dev, wsa881x);
	wsa881x->slave = pdev;
	wsa881x->dev = &pdev->dev;
	pdev->prop.sink_ports = GENMASK(WSA881X_MAX_SWR_PORTS, 0);
	pdev->prop.sink_dpn_prop = wsa_sink_dpn_prop;
	gpiod_set_value(wsa881x->sd_n, 1);

	return 0;
}

static int wsa881x_remove(struct sdw_slave *sdw)
{
	return 0;
}

static const struct sdw_device_id wsa881x_slave_id[] = {
	SDW_SLAVE_ENTRY(0x0217, 0x2010, 0),
	{},
};
MODULE_DEVICE_TABLE(sdw, wsa881x_slave_id);

static struct sdw_driver wsa881x_codec_driver = {
	.probe	= wsa881x_probe,
	.remove = wsa881x_remove,
	.ops = &wsa881x_slave_ops,
	.id_table = wsa881x_slave_id,
	.driver = {
		.name	= "wsa881x-codec",
	}
};
module_sdw_driver(wsa881x_codec_driver);

MODULE_DESCRIPTION("WSA881x codec driver");
MODULE_LICENSE("GPL v2");
