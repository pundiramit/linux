/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 20544 $
 * $Date: 2017-12-20 11:08:15 +0800 (週三, 20 十二月 2017) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>

#include <linux/debugfs.h>

#include "nt36xxx.h"

struct nvt_ts_data *ts;

static struct workqueue_struct *nvt_wq;

#define INPUT_EVENT_START			0
#define INPUT_EVENT_SENSITIVE_MODE_OFF		0
#define INPUT_EVENT_SENSITIVE_MODE_ON		1
#define INPUT_EVENT_STYLUS_MODE_OFF		2
#define INPUT_EVENT_STYLUS_MODE_ON		3
#define INPUT_EVENT_WAKUP_MODE_OFF		4
#define INPUT_EVENT_WAKUP_MODE_ON		5
#define INPUT_EVENT_COVER_MODE_OFF		6
#define INPUT_EVENT_COVER_MODE_ON		7
#define INPUT_EVENT_SLIDE_FOR_VOLUME		8
#define INPUT_EVENT_DOUBLE_TAP_FOR_VOLUME		9
#define INPUT_EVENT_SINGLE_TAP_FOR_VOLUME		10
#define INPUT_EVENT_LONG_SINGLE_TAP_FOR_VOLUME		11
#define INPUT_EVENT_PALM_OFF		12
#define INPUT_EVENT_PALM_ON		13
#define INPUT_EVENT_END				13

/*******************************************************
Description:
	Novatek touchscreen i2c read function.
return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = &buf[1];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)	break;
		retries++;
		msleep(20);
		NVT_ERR("error, retry=%d\n", retries);
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.
return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = !I2C_M_RD;
//	msg.flags = 0;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)	break;
		retries++;
		msleep(20);
		NVT_ERR("error, retry=%d\n", retries);
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}


/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.
return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4] = {0};

	/*---write i2c cmds to reset idle---*/
	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.
return:
	n.a.
*******************************************************/
int nvt_bootloader_reset(void)
{
	int ret = 0;
	uint8_t buf[8] = {0};

	//HERE

	/*write i2c cmds to reset*/
	buf[0] = 0x00;
	buf[1] = 0x69;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	/*need 35ms delay after bootloader reset*/
	msleep(35);

	printk(KERN_DEBUG "used");
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.
return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		/*---read reset state---*/
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if (unlikely(retry > 100)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
	}
        printk(KERN_DEBUG "used");

	return ret;
}

#if 0
/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.
return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	/*---set xdata index to EVENT BUF ADDR---*/
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	/*---read project id---*/
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	NVT_LOG("PID=%04X\n", ts->nvt_pid);
        printk(KERN_DEBUG "used");

	return ret;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.
return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	/*---set xdata index to EVENT BUF ADDR---*/
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	/*---read fw info---*/
	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];

	/*---clear x_num, y_num if fw info is broken---*/
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts->max_button_num = TOUCH_KEY_NUM;

		if (retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, \
					abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

        printk(KERN_DEBUG "used");

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.
return:
	n.a.
*******************************************************/
static int nvt_parse_dt(struct device *dev)
{
	struct device_node *temp, *np = dev->of_node;
	struct nvt_config_info *config_info;
	int retval;
	u32 temp_val;
	const char *name;

	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, NULL);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);

	ts->reset_tddi = of_get_named_gpio_flags(np, "novatek,reset-tddi", 0, NULL);
	NVT_LOG("novatek,reset-tddi=%d\n", ts->reset_tddi);

	retval = of_property_read_string(np, "novatek,vddio-reg-name", &name);
	if (retval == -EINVAL)
		ts->vddio_reg_name = NULL;
	else if (retval < 0)
		return -EINVAL;
	else {
		ts->vddio_reg_name = name;
		NVT_LOG("vddio_reg_name = %s\n", name);
	}

	retval = of_property_read_string(np, "novatek,lab-reg-name", &name);
	if (retval == -EINVAL)
		ts->lab_reg_name = NULL;
	else if (retval < 0)
		return -EINVAL;
	else {
		ts->lab_reg_name = name;
		NVT_LOG("lab_reg_name = %s\n", name);
	}

	retval = of_property_read_string(np, "novatek,ibb-reg-name", &name);
	if (retval == -EINVAL)
		ts->ibb_reg_name = NULL;
	else if (retval < 0)
		return -EINVAL;
	else {
		ts->ibb_reg_name = name;
		NVT_LOG("ibb_reg_name = %s\n", name);
	}

	retval = of_property_read_u32(np, "novatek,config-array-size",
				 (u32 *) & ts->config_array_size);
	if (retval) {
		NVT_LOG("Unable to get array size\n");
		return retval;
	}

	ts->config_array = devm_kzalloc(dev, ts->config_array_size *
					   sizeof(struct nvt_config_info),
					   GFP_KERNEL);

	if (!ts->config_array) {
		NVT_LOG("Unable to allocate memory\n");
		return -ENOMEM;
	}

	config_info = ts->config_array;
	for_each_child_of_node(np, temp) {
		retval = of_property_read_u32(temp, "novatek,tp-vendor", &temp_val);

		if (retval) {
			NVT_LOG("Unable to read tp vendor\n");
		} else {
			config_info->tp_vendor = (u8) temp_val;
			NVT_LOG("tp vendor: %u", config_info->tp_vendor);
		}

		retval = of_property_read_u32(temp, "novatek,hw-version", &temp_val);

		if (retval) {
			NVT_LOG("Unable to read tp hw version\n");
		} else {
			config_info->tp_hw_version = (u8) temp_val;
			NVT_LOG("tp hw version: %u", config_info->tp_hw_version);
		}

		retval = of_property_read_string(temp, "novatek,fw-name",
						 &config_info->nvt_cfg_name);

		if (retval && (retval != -EINVAL)) {
			NVT_LOG("Unable to read cfg name\n");
		} else {
			NVT_LOG("fw_name: %s", config_info->nvt_cfg_name);
		}
		retval = of_property_read_string(temp, "novatek,limit-name",
						 &config_info->nvt_limit_name);

		if (retval && (retval != -EINVAL)) {
			NVT_LOG("Unable to read limit name\n");
		} else {
			NVT_LOG("limit_name: %s", config_info->nvt_limit_name);
		}
		config_info++;
	}

	return 0;
}

static int nvt_get_reg(struct nvt_ts_data *ts, bool get) //REGULATORS
{
	int retval;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if ((ts->vddio_reg_name != NULL) && (*ts->vddio_reg_name != 0)) {
		ts->vddio_reg = regulator_get(&ts->client->dev,
				ts->vddio_reg_name);
		if (IS_ERR(ts->vddio_reg)) {
			NVT_ERR("Failed to get power regulator\n");
			retval = PTR_ERR(ts->vddio_reg);
			goto regulator_put;
		}
	}

	if ((ts->lab_reg_name != NULL) && (*ts->lab_reg_name != 0)) {
		ts->lab_reg = regulator_get(&ts->client->dev,
				ts->lab_reg_name);
		if (IS_ERR(ts->lab_reg)) {
			NVT_ERR("Failed to get lab regulator\n");
			retval = PTR_ERR(ts->lab_reg);
			goto regulator_put;
		}
	}

	if ((ts->ibb_reg_name != NULL) && (*ts->ibb_reg_name != 0)) {
		ts->ibb_reg = regulator_get(&ts->client->dev,
				ts->ibb_reg_name);
		if (IS_ERR(ts->ibb_reg)) {
			NVT_ERR("Failed to get ibb regulator\n");
			retval = PTR_ERR(ts->ibb_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (ts->vddio_reg) {
		regulator_put(ts->vddio_reg);
		ts->vddio_reg = NULL;
	}
	if (ts->lab_reg) {
		regulator_put(ts->lab_reg);
		ts->lab_reg = NULL;
	}
	if (ts->ibb_reg) {
		regulator_put(ts->ibb_reg);
		ts->ibb_reg = NULL;
	}

	return retval;
}

static int nvt_enable_reg(struct nvt_ts_data *ts, bool enable)
{
	int retval;

	if (!enable) {
		retval = 0;
		goto disable_ibb_reg;
	}

	if (ts->vddio_reg) {
		retval = regulator_enable(ts->vddio_reg);
		if (retval < 0) {
			NVT_ERR("Failed to enable vddio regulator\n");
			goto exit;
		}
	}

	if (ts->lab_reg && ts->lab_reg) {
		retval = regulator_enable(ts->lab_reg);
		if (retval < 0) {
			NVT_ERR("Failed to enable lab regulator\n");
			goto disable_vddio_reg;
		}
	}

	if (ts->ibb_reg) {
		retval = regulator_enable(ts->ibb_reg);
		if (retval < 0) {
			NVT_ERR("Failed to enable ibb regulator\n");
			goto disable_lab_reg;
		}
	}

	return 0;

disable_ibb_reg:
	if (ts->ibb_reg)
		regulator_disable(ts->ibb_reg);

disable_lab_reg:
	if (ts->lab_reg)
		regulator_disable(ts->lab_reg);

disable_vddio_reg:
	if (ts->vddio_reg)
		regulator_disable(ts->vddio_reg);

exit:
	return retval;
}

/*******************************************************
Description:
	Novatek touchscreen config and request gpio
return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-reset");
		if (ret) {
			NVT_ERR("Failed to request reset-int GPIO\n");
			goto err_request_reset_gpio;
		}
		gpio_direction_output(ts->reset_gpio, 1);
	}

	return ret;

err_request_reset_gpio:
err_request_irq_gpio:
	return ret;
}

#define POINT_DATA_LEN 65
/*******************************************************
Description:
	Novatek touchscreen work function.
return:
	n.a.
*******************************************************/
static void nvt_ts_work_func(struct work_struct *work)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
	int32_t i = 0;
	int32_t finger_cnt = 0;

	mutex_lock(&ts->lock);

	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}

	finger_cnt = 0;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	/*finger down (enter & moving)*/
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
				continue;
			input_w = (uint32_t)(point_data[position + 4]);
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			} else {
				input_p = (uint32_t)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);

			input_mt_sync(ts->input_dev);

			finger_cnt++;
		}
	}

	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}

	input_sync(ts->input_dev);

XFER_ERROR:
	enable_irq(ts->client->irq);

	mutex_unlock(&ts->lock);
}

/*******************************************************
Description:
	External interrupt service routine.
return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	disable_irq_nosync(ts->client->irq);
	queue_work(nvt_wq, &ts->nvt_work);

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.
return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

//HERE
	ret = nvt_bootloader_reset(); /* NOT in retry loop */
	if (ret < 0) {
		NVT_ERR("Can't reset the nvt ic\n");
		goto out;
	}
	/*---Check for 5 times---*/
	for (retry = 5; retry > 0; retry--) {
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		/* compare read chip id on supported list */
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			/* compare each byte */
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].carrier_system;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

static int nvt_pinctrl_init(struct nvt_ts_data *nvt_data)
{
	int retval = 0;
	/* Get pinctrl if target uses pinctrl */
	nvt_data->ts_pinctrl = devm_pinctrl_get(&nvt_data->client->dev);

	if (IS_ERR_OR_NULL(nvt_data->ts_pinctrl)) {
		retval = PTR_ERR(nvt_data->ts_pinctrl);
		NVT_ERR("Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	nvt_data->pinctrl_state_active
		= pinctrl_lookup_state(nvt_data->ts_pinctrl, PINCTRL_STATE_ACTIVE);

	if (IS_ERR_OR_NULL(nvt_data->pinctrl_state_active)) {
		retval = PTR_ERR(nvt_data->pinctrl_state_active);
		NVT_ERR("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	nvt_data->pinctrl_state_suspend
		= pinctrl_lookup_state(nvt_data->ts_pinctrl, PINCTRL_STATE_SUSPEND);

	if (IS_ERR_OR_NULL(nvt_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(nvt_data->pinctrl_state_suspend);
		NVT_ERR("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	return 0;
err_pinctrl_lookup:
	devm_pinctrl_put(nvt_data->ts_pinctrl);
err_pinctrl_get:
	nvt_data->ts_pinctrl = NULL;
	return retval;
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.
return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
	char *tp_maker = NULL;

	NVT_LOG("start\n");

	ts = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	/*---parse dts---*/
	nvt_parse_dt(&client->dev);

	ret = nvt_pinctrl_init(ts);
	if (!ret && ts->ts_pinctrl) {
		ret = pinctrl_select_state(ts->ts_pinctrl, ts->pinctrl_state_active);

		if (ret < 0) {
			NVT_ERR("Failed to select %s pinstate %d\n",
				PINCTRL_STATE_ACTIVE, ret);
		}
	} else {
		NVT_ERR("Failed to init pinctrl\n");
	}

	/*---request and config GPIOs---*/
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

       /*--- request regulator---*/
        nvt_get_reg(ts, true); //REGULATORS

        /* we should enable the reg for lpwg mode */
        nvt_enable_reg(ts, true);

	/*---check i2c func.---*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NVT_ERR("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	/* need 10ms delay after POR(power on reset) */
	msleep(10);

	//HERE
	/*---check chip version trim---*/
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	mutex_init(&ts->mdata_lock);
	mutex_init(&ts->lock);

	mutex_lock(&ts->lock);
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();
	mutex_unlock(&ts->lock);

	/*---create workqueue---*/
	nvt_wq = create_workqueue("nvt_wq");
	if (!nvt_wq) {
		NVT_ERR("nvt_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_wq_failed;
	}
	INIT_WORK(&ts->nvt_work, nvt_ts_work_func);
	/*---allocate input device---*/
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

	ts->int_trigger_type = INT_TRIGGER_TYPE;

	/*---set input device info.---*/
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    /*pressure = TOUCH_FORCE_NUM*/

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	  /*area = 255*/

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif /*TOUCH_MAX_FINGER_NUM > 1*/

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	input_set_drvdata(ts->input_dev, ts);

	/*---register input device---*/
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	/*---set int-pin & request irq---*/
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);

		ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type, client->name, ts);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			disable_irq(client->irq);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

	ts->fw_name = BOOT_UPDATE_FIRMWARE_NAME;

	tp_maker = kzalloc(20, GFP_KERNEL);
	if (tp_maker == NULL)
		NVT_ERR("fail to alloc vendor name memory\n");
	else {
		kfree(tp_maker);
		tp_maker = NULL;
	}

	enable_irq(client->irq);

	return 0;

	free_irq(client->irq, ts);
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_create_nvt_wq_failed:
	mutex_destroy(&ts->lock);
err_chipvertrim_failed:
err_check_functionality_failed:
	gpio_free(ts->irq_gpio);
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
err_gpio_config_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	nvt_get_reg(ts, false);

	nvt_enable_reg(ts, false);

	mutex_destroy(&ts->lock);

	NVT_LOG("Removing driver...\n");

	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nvt_ts_id);

static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts",},
	{ },
};
MODULE_DEVICE_TABLE(of, nvt_match_table);

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = nvt_match_table,
	},
};
module_i2c_driver(nvt_i2c_driver);


MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
