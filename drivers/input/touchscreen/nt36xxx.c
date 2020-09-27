// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 * Copyright (C) 2020 AngeloGioacchino Del Regno <kholk11@gmail.com>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>

#include "nt36xxx.h"

static const struct nt36xxx_mem_map nt36xxx_memory_maps[] = {
	[NT36525_IC]  = { 0x11a00, 0x10000, 0x12000, 0x14000, 0x14002 },
	[NT36672A_IC] = { 0x21c00, 0x20000, 0x23000, 0x24000, 0x24002 },
	[NT36676F_IC] = { 0x11a00, 0x10000, 0x12000, 0x14000, 0x14002 },
	[NT36772_IC]  = { 0x11e00, 0x10000, 0x12000, 0x14000, 0x14002 },
	[NT36870_IC]  = { 0x25000, 0x20000, 0x23000, 0x24000, 0x24002 },
};

static const struct nt36xxx_trim_table trim_id_table[] = {
	{
	 .id = { 0x0A, 0xFF, 0xFF, 0x72, 0x66, 0x03 },
	 .mask = { 1, 0, 0, 1, 1, 1 },
	 .mapid = NT36672A_IC,
	},
	{
	 .id = { 0x55, 0x00, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0x55, 0x72, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xAA, 0x00, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xAA, 0x72, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x72, 0x67, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x70, 0x66, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x70, 0x67, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x72, 0x66, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x25, 0x65, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x70, 0x68, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x76, 0x66, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36676F_IC,
	},
};

static int nt36xxx_read(struct i2c_client *client, uint16_t address,
			u8 *buf, uint16_t len)
{
	int ret, retry = NT36XXX_MAX_RETRIES;
	struct i2c_msg msg[] = {
		/* Write slave position to i2c devices */
		{
			.addr = address,
			.len = 1,
			.buf = &buf[0]
		},
		/* Read data from position */
		{
			.addr = address,
			.flags = I2C_M_RD,
			.len = len - 1,
			.buf = &buf[1]
		}
	};

	do {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (likely(ret == ARRAY_SIZE(msg)))
			return 0;
	} while (--retry);

	return ret < 0 ? ret : -EIO;
}

static int nt36xxx_write(struct i2c_client *client, uint16_t address,
			 u8 *buf, uint16_t len)
{
	int ret, retry = NT36XXX_MAX_RETRIES;
	struct i2c_msg msg[] = {
		{
			.addr = address,
			.flags = 0,
			.len = len,
			.buf = buf,
		},
	};

	do {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (likely(ret == ARRAY_SIZE(msg)))
			return 0;

		usleep_range(100, 200);
	} while (--retry);

	return ret < 0 ? ret : -EIO;
}

static int nt36xxx_set_page(struct nt36xxx_i2c *ts, uint32_t pageaddr)
{
	u8 buf[3];

	buf[0] = NT36XXX_CMD_SET_PAGE;
	buf[1] = (pageaddr >> 16) & 0xFF;
	buf[2] = (pageaddr >> 8) & 0xFF;

	return nt36xxx_write(ts->client, NT36XXX_BLDR_ADDR, buf, 3);
}

static int nt36xxx_sw_reset_idle(struct nt36xxx_i2c *ts)
{
	u8 buf[] = { 0x00, NT36XXX_CMD_SW_RESET };
	int ret;

	ret = nt36xxx_write(ts->client, ts->client->addr,
			    buf, ARRAY_SIZE(buf));

	usleep_range(15000, 16000);
	return ret;
}

static int nt36xxx_bootloader_reset(struct nt36xxx_i2c *ts)
{
	u8 buf[] = { 0x00, NT36XXX_CMD_BOOTLOADER_RESET };
	int ret;

	ret = nt36xxx_write(ts->client, ts->client->addr,
			    buf, ARRAY_SIZE(buf));

	msleep(35);
	return ret;
}

static int nt36xxx_check_reset_state(struct nt36xxx_i2c *ts,
				     enum nt36xxx_fw_state fw_state)
{
	u8 buf[6] = { NT36XXX_EVT_RESET_COMPLETE, 0x00, 0x00,
			   0x00, 0x00, 0x00 };
	int ret, retry = NT36XXX_MAX_FW_RST_RETRY;

	do {
		ret = nt36xxx_read(ts->client, NT36XXX_BLDR_ADDR, buf, 6);
		if (likely(ret != -EIO) &&
		    (buf[1] >= fw_state) &&
		    (buf[1] <= NT36XXX_STATE_MAX)) {
			ret = 0;
			break;
		}
		usleep_range(10000, 11000);
	} while (--retry);

	if (!retry) {
		dev_err(&ts->client->dev, "Firmware reset failed.\n");
		ret = -EBUSY;
	}

	return ret;
}

static int nt36xxx_read_pid(struct nt36xxx_i2c *ts)
{
	u8 buf[] = { NT36XXX_EVT_PROJECTID, 0x00, 0x00 };
	int ret = 0;

	ret = nt36xxx_set_page(ts, ts->mmap->evtbuf_addr);
	if (unlikely(ret < 0))
		return ret;

	/* Read project id */
	ret = nt36xxx_read(ts->client, NT36XXX_BLDR_ADDR,
			   buf, ARRAY_SIZE(buf));
	if (unlikely(ret < 0))
		return ret;

	ts->fw_info.nvt_pid = (buf[2] << 8) + buf[1];

	return 0;
}

static int __nt36xxx_get_fw_info(struct nt36xxx_i2c *ts)
{
	struct nt36xxx_fw_info *fwi = &ts->fw_info;
	u8 buf[17] = { 0 };
	int ret = 0;

	ret = nt36xxx_set_page(ts, ts->mmap->evtbuf_addr);
	if (unlikely(ret < 0))
		return ret;

	/* Read fw info */
	buf[0] = NT36XXX_EVT_FWINFO;
	nt36xxx_read(ts->client, NT36XXX_BLDR_ADDR, buf, 17);
	fwi->fw_ver = buf[1];
	fwi->x_num = buf[3];
	fwi->y_num = buf[4];
	fwi->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	fwi->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	fwi->max_buttons = buf[11];

	/* Check fw info integrity and clear x_num, y_num if broken */
	if ((buf[1] + buf[2]) != 0xFF) {
		dev_dbg(&ts->client->dev,
			"FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n",
			buf[1], buf[2]);
		fwi->fw_ver = 0;
		fwi->x_num = 18;
		fwi->y_num = 32;
		fwi->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		fwi->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		fwi->max_buttons = 0;
		return -EINVAL;
	}

	/* Get Novatek ProjectID */
	return nt36xxx_read_pid(ts);
}

static int nt36xxx_get_fw_info(struct nt36xxx_i2c *ts)
{
	struct nt36xxx_fw_info *fwi = &ts->fw_info;
	int i, ret = 0;

	for (i = 0; i < NT36XXX_MAX_RETRIES; i++) {
		ret = __nt36xxx_get_fw_info(ts);
		if (ret == 0)
			break;
	}

	dev_dbg(&ts->client->dev,
		"Firmware Info: ver=0x%x res=%ux%u max=%ux%u buttons=%u",
		fwi->fw_ver, fwi->x_num, fwi->y_num,
		fwi->abs_x_max, fwi->abs_y_max, fwi->max_buttons);

	return ret;
}

static void nt36xxx_i2c_work_func(struct work_struct *work)
{
	struct nt36xxx_i2c *ts = container_of(work, struct nt36xxx_i2c, ts_work);
	struct nt36xxx_abs_object *obj = &ts->abs_obj;
	struct input_dev *input = ts->input;
	u8 input_id = 0;
	u8 press_id[TOUCH_MAX_FINGER_NUM] = { 0 };
	u8 point[POINT_DATA_LEN + 1] = { 0 };
	unsigned int ppos = 0;
	int i, ret, finger_cnt = 0;

	mutex_lock(&ts->lock);

	ret = nt36xxx_read(ts->client, NT36XXX_BLDR_ADDR, point,
			   POINT_DATA_LEN + 1);
	if (unlikely(ret < 0)) {
		dev_dbg(&ts->client->dev,
			"Cannot read touch point data: %d\n", ret);
		goto xfer_error;
	}

	for (i = 0; i < ts->max_fingers; i++) {
		ppos = 1 + 6 * i;
		input_id = point[ppos + 0] >> 3;
		if ((input_id == 0) || (input_id > ts->max_fingers))
			continue;

		if (((point[ppos] & 0x07) == 0x01) ||
		    ((point[ppos] & 0x07) == 0x02)) {
			obj->x = (point[ppos + 1] << 4) +
				 (point[ppos + 3] >> 4);
			obj->y = (point[ppos + 2] << 4) +
				 (point[ppos + 3] & 0xf);
			if ((obj->x > ts->fw_info.abs_x_max) ||
			    (obj->y > ts->fw_info.abs_y_max))
				continue;

			obj->tm = point[ppos + 4];
			if (obj->tm == 0)
				obj->tm = 1;

			obj->z = point[ppos + 5];
			if (i < 2) {
				obj->z += point[i + 63] << 8;
				if (obj->z > TOUCH_MAX_PRESSURE)
					obj->z = TOUCH_MAX_PRESSURE;
			}

			if (obj->z == 0)
				obj->z = 1;

			press_id[input_id - 1] = 1;
			input_mt_slot(input, input_id - 1);
			input_mt_report_slot_state(input,
						   MT_TOOL_FINGER, true);

			input_report_abs(input, ABS_MT_POSITION_X, obj->x);
			input_report_abs(input, ABS_MT_POSITION_Y, obj->y);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, obj->tm);
			input_report_abs(input, ABS_MT_PRESSURE, obj->z);

			finger_cnt++;
		}
	}

	for (i = 0; i < ts->max_fingers; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(input, i);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(input, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(input,
						   MT_TOOL_FINGER, false);
		}
	}

	input_report_key(input, BTN_TOUCH, (finger_cnt > 0));

	input_sync(input);

xfer_error:
	enable_irq(ts->client->irq);

	mutex_unlock(&ts->lock);
}

static irqreturn_t nt36xxx_i2c_irq_handler(int irq, void *dev_id)
{
	struct nt36xxx_i2c *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->ts_workq, &ts->ts_work);

	return IRQ_HANDLED;
}

static int nvt_stop_crc_reboot(struct nt36xxx_i2c *ts)
{
	u8 buf[4] = { 0 };
	int ret, retry = NT36XXX_MAX_RETRIES;

	/* Read dummy buffer to check CRC fail reboot is happening or not */

	/* Change I2C index to prevent geting 0xFF, but not 0xFC */
	ret = nt36xxx_set_page(ts, NT36XXX_PAGE_CHIP_INFO);
	if (ret < 0) {
		dev_dbg(&ts->client->dev,
			"CRC reset failed: Cannot select page.\n");
		return ret;
	}

	/* Read to check if buf is 0xFC which means IC is in CRC reboot */
	buf[0] = 0x4E;
	ret = nt36xxx_read(ts->client, NT36XXX_BLDR_ADDR, buf, 3);
	if (ret < 0)
		return ret;

	if (((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
	    ((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
		/* IC is in CRC fail reboot loop, needs to be stopped! */
		do {
			/* Write i2c cmds to reset idle - part #1 */
			buf[0] = 0x00;
			buf[1] = 0xA5;
			nt36xxx_write(ts->client, ts->client->addr,
				      buf, 2);

			/* Write i2c cmds to reset idle - part #2 */
			buf[0] = 0x00;
			buf[1] = 0xA5;
			nt36xxx_write(ts->client, ts->client->addr,
				      buf, 2);
			usleep_range(1000, 1100);

			/* Clear CRC_ERR_FLAG */
			ret = nt36xxx_set_page(ts, NT36XXX_PAGE_CRC);
			if (ret < 0)
				continue;

			buf[0] = 0x35;
			buf[1] = 0xA5;
			nt36xxx_write(ts->client, NT36XXX_BLDR_ADDR, buf, 2);

			/* Check CRC_ERR_FLAG */
			ret = nt36xxx_set_page(ts, NT36XXX_PAGE_CRC);
			if (ret < 0)
				continue;

			buf[0] = 0x35;
			buf[1] = 0x00;
			nt36xxx_read(ts->client, NT36XXX_BLDR_ADDR, buf, 2);

			if (buf[1] == 0xA5)
				break;
		} while (--retry);

		if (retry == 0) {
			dev_err(&ts->client->dev,
				"CRC reset failed: buf[1]=0x%02X\n", buf[1]);
		}
	}

	return 0;
}

static int nt36xxx_i2c_chip_version_init(struct nt36xxx_i2c *ts)
{
	u8 buf[7] = { 0 };
	int retry = NT36XXX_MAX_RETRIES;
	int sz = sizeof(trim_id_table) / sizeof(struct nt36xxx_trim_table);
	int i, list, mapid, ret;

	ret = nt36xxx_bootloader_reset(ts);
	if (ret < 0) {
		dev_err(&ts->client->dev, "Can't reset the nvt IC\n");
		return ret;
	}

	do {
		ret = nt36xxx_sw_reset_idle(ts);
		if (ret < 0)
			continue;

		buf[0] = 0x00;
		buf[1] = NT36XXX_CMD_UNLOCK;
		nt36xxx_write(ts->client, ts->client->addr, buf, 2);
		usleep_range(10000, 11000);

		ret = nt36xxx_set_page(ts, NT36XXX_PAGE_CHIP_INFO);
		if (ret < 0)
			continue;

		memset(buf, 0, ARRAY_SIZE(buf));
		buf[0] = NT36XXX_EVT_CHIPID;
		nt36xxx_read(ts->client, NT36XXX_BLDR_ADDR, buf, 7);

		/* Compare read chip id with trim list */
		for (list = 0; list < sz; list++) {
			/* Compare each not masked byte */
			for (i = 0; i < NT36XXX_ID_LEN_MAX; i++) {
				if (trim_id_table[list].mask[i] &&
				    buf[i + 1] != trim_id_table[list].id[i])
					break;
			}

			if (i == NT36XXX_ID_LEN_MAX) {
				mapid = trim_id_table[list].mapid;
				ts->mmap = &nt36xxx_memory_maps[mapid];
				return 0;
			}

			ts->mmap = NULL;
			ret = -ENOENT;
		}

		/* Stop CRC check to prevent IC auto reboot */
		if (((buf[1] == 0xFC) && (buf[2] == 0xFC) &&
		     (buf[3] == 0xFC)) ||
		    ((buf[1] == 0xFF) && (buf[2] == 0xFF) &&
		     (buf[3] == 0xFF))) {
			ret = nvt_stop_crc_reboot(ts);
			if (ret < 0)
				continue;
		}

		usleep_range(10000, 11000);
	} while (--retry);

	return ret;
}

static int nt36xxx_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct nt36xxx_i2c *ts;
	struct input_dev *input;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	if (!client->irq) {
		dev_err(&client->dev, "No irq specified\n");
		return -EINVAL;
	}

	ts = devm_kzalloc(&client->dev, sizeof(struct nt36xxx_i2c), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->supplies = devm_kcalloc(&client->dev,
				    NT36XXX_NUM_SUPPLIES,
				    sizeof(struct regulator_bulk_data),
				    GFP_KERNEL);
	if (!ts->supplies)
		return -ENOMEM;

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return -ENOMEM;

	ts->client = client;
	ts->input = input;
	i2c_set_clientdata(client, ts);

	ts->ts_workq = alloc_ordered_workqueue("nt36xxx", 0);
	if (!ts->ts_workq)
		return -EINVAL;

	ts->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return PTR_ERR(ts->reset_gpio);
	if (ts->reset_gpio)
		gpiod_set_consumer_name(ts->reset_gpio, "nt36xxx reset");

	/* These supplies are optional */
	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "vio";
	ret = devm_regulator_bulk_get(&client->dev,
				      NT36XXX_NUM_SUPPLIES,
				      ts->supplies);
	if (ret != 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev, "Cannot get supplies: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(NT36XXX_NUM_SUPPLIES, ts->supplies);
	if (ret)
		return ret;

	usleep_range(10000, 11000);

	mutex_init(&ts->lock);

	ret = nt36xxx_i2c_chip_version_init(ts);
	if (ret) {
		dev_err(&client->dev, "Failed to check chip version\n");
		goto error;
	}

	mutex_lock(&ts->lock);
	ret = nt36xxx_bootloader_reset(ts);
	ret += nt36xxx_check_reset_state(ts, NT36XXX_STATE_INIT);
	ret += nt36xxx_get_fw_info(ts);
	mutex_unlock(&ts->lock);
	if (unlikely(ret < 0))
		goto error;

	ts->max_fingers = TOUCH_MAX_FINGER_NUM;
	INIT_WORK(&ts->ts_work, nt36xxx_i2c_work_func);

	input->phys = devm_kasprintf(&client->dev, GFP_KERNEL,
				     "%s/input0", dev_name(&client->dev));

	input->name = NT36XXX_INPUT_DEVICE_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->evbit[0] =
		BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_mt_init_slots(input, ts->max_fingers, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			     TOUCH_MAX_PRESSURE, 0, 0);

	if (ts->max_fingers > 1) {
		input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

		input_set_abs_params(input, ABS_MT_POSITION_X, 0,
				     ts->fw_info.abs_x_max - 1, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
				     ts->fw_info.abs_y_max - 1, 0, 0);
	}

	input_set_drvdata(input, ts);

	ret = input_register_device(ts->input);
	if (ret) {
		dev_err(&client->dev, "Failed to register input device: %d\n",
			ret);
		goto error;
	}

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					nt36xxx_i2c_irq_handler, IRQF_ONESHOT,
					client->name, ts);
	if (ret != 0) {
		dev_err(&client->dev, "request irq failed: %d\n", ret);
		goto intr_error;
	}

	disable_irq(client->irq);
	device_init_wakeup(&client->dev, 1);
	enable_irq(client->irq);

	return 0;

intr_error:
	input_unregister_device(ts->input);
error:
	regulator_bulk_disable(NT36XXX_NUM_SUPPLIES, ts->supplies);
	return ret;
}

static int nt36xxx_i2c_remove(struct i2c_client *client)
{
	struct nt36xxx_i2c *ts = i2c_get_clientdata(client);

	free_irq(client->irq, ts);
	regulator_bulk_disable(NT36XXX_NUM_SUPPLIES, ts->supplies);
	mutex_destroy(&ts->lock);
	input_unregister_device(ts->input);

	return 0;
}

static int __maybe_unused nt36xxx_i2c_suspend(struct device *dev)
{
	struct nt36xxx_i2c *ts = i2c_get_clientdata(to_i2c_client(dev));
	u8 buf[] = { NT36XXX_EVT_HOST_CMD, NT36XXX_CMD_ENTER_SLEEP };
	int i, ret;

	disable_irq(ts->client->irq);

	ret = nt36xxx_write(ts->client, NT36XXX_BLDR_ADDR,
			    buf, ARRAY_SIZE(buf));
	if (unlikely(ret < 0)) {
		dev_err(&ts->client->dev, "Cannot enter suspend!!\n");
		return ret;
	}

	/* Release all touches */
	for (i = 0; i < ts->max_fingers; i++) {
		input_mt_slot(ts->input, i);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
	}
	input_sync(ts->input);

	return 0;
}

static int __maybe_unused nt36xxx_i2c_resume(struct device *dev)
{
	struct nt36xxx_i2c *ts = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	mutex_lock(&ts->lock);

	if (ts->reset_gpio)
		gpiod_set_value(ts->reset_gpio, 1);

	ret = nt36xxx_bootloader_reset(ts);
	if (ret < 0)
		goto end;

	ret = nt36xxx_check_reset_state(ts, NT36XXX_STATE_REK);
	if (ret < 0)
		goto end;

	enable_irq(ts->client->irq);
end:
	mutex_unlock(&ts->lock);
	return ret;
}

static SIMPLE_DEV_PM_OPS(nt36xxx_i2c_pm,
			 nt36xxx_i2c_suspend, nt36xxx_i2c_resume);

#ifdef CONFIG_OF
static const struct of_device_id nt36xxx_of_match[] = {
	{ .compatible = "novatek,nt36xxx" },
	{ }
};
MODULE_DEVICE_TABLE(of, nt36xxx_of_match);
#endif

static const struct i2c_device_id nt36xxx_i2c_ts_id[] = {
	{ "NVT-ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nt36xxx_i2c_ts_id);

static struct i2c_driver nt36xxx_i2c_ts_driver = {
	.driver = {
		.name	= "nt36xxx_ts",
		.of_match_table = of_match_ptr(nt36xxx_of_match),
		.pm	= &nt36xxx_i2c_pm,
	},
	.probe		= nt36xxx_i2c_probe,
	.remove		= nt36xxx_i2c_remove,
	.id_table	= nt36xxx_i2c_ts_id,
};
module_i2c_driver(nt36xxx_i2c_ts_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Novatek NT36XXX Touchscreen Driver");

