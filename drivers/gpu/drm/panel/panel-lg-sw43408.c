// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Linaro Ltd
 * Author: Sumit Semwal <sumit.semwal@linaro.org>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#include <video/mipi_display.h>

/*
to figure out:
 - do we need pinctrl for panel, or does the dsi_host one suffice?
*/

struct panel_cmd {
	size_t len;
	const char *data;
};

#define _INIT_CMD(...) { \
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

static const char * const regulator_names[] = {
	"vddi",
	"vpnl",
	"lab_reg", // Validate the name
};

static unsigned long const regulator_enable_loads[] = {
	62000,
	857000,
	100000,	// Confirm this value
};

static unsigned long const regulator_disable_loads[] = {
	80,
	0,
	100,	// confirm this value
};

struct panel_desc {
	const struct drm_display_mode *display_mode;
	const char *panel_name;

	unsigned int width_mm;
	unsigned int height_mm;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;

	const struct panel_cmd *on_cmds;
};

struct panel_info {
	struct drm_panel base;
	struct mipi_dsi_device *link;
	const struct panel_desc *desc;

	struct backlight_device *backlight;

	struct regulator_bulk_data supplies[ARRAY_SIZE(regulator_names)];

	struct gpio_desc *reset_gpio;

	bool prepared;
	bool enabled;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, base);
}

static int send_mipi_cmds(struct drm_panel *panel, const struct panel_cmd *cmds)
{
	struct panel_info *pinfo = to_panel_info(panel);
	unsigned int i = 0;
	int err;

	if (!cmds)
		return -EFAULT;

	for (i = 0; cmds[i].len != 0; i++) {
		const struct panel_cmd *cmd = &cmds[i];

		if (cmd->len == 2)
			err = mipi_dsi_dcs_write(pinfo->link,
						    cmd->data[1], NULL, 0);
		else
			err = mipi_dsi_dcs_write(pinfo->link,
						    cmd->data[1], cmd->data + 2,
						    cmd->len - 2);

		if (err < 0)
			return err;

		usleep_range((cmd->data[0]) * 1000,
			    (1 + cmd->data[0]) * 1000);
	}

	return 0;
}

static int lg_panel_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	backlight_disable(pinfo->backlight);

	pinfo->enabled = false;

	return 0;
}

static int lg_panel_power_off(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int i, ret = 0;

	gpiod_set_value(pinfo->reset_gpio, 0);

	for (i = 0; i < ARRAY_SIZE(pinfo->supplies); i++) {
		ret = regulator_set_load(pinfo->supplies[i].consumer,
				regulator_disable_loads[i]);
		if (ret) {
			DRM_DEV_ERROR(panel->dev,
				"regulator_set_load failed %d\n", ret);
			return ret;
		}
	}

	ret = regulator_bulk_disable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
	if (ret) {
		DRM_DEV_ERROR(panel->dev,
			"regulator_bulk_disable failed %d\n", ret);
	}
	return ret;
}

static int lg_panel_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (!pinfo->prepared)
		return 0;

	ret = mipi_dsi_dcs_write(pinfo->link, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0);
	if (ret < 0) {
		DRM_DEV_ERROR(panel->dev,
			"set_display_off cmd failed ret = %d\n",
			ret);
	}

	/* 120ms delay required here as per DCS spec */
	msleep(120);

	ret = mipi_dsi_dcs_write(pinfo->link, MIPI_DCS_ENTER_SLEEP_MODE, NULL, 0);
	if (ret < 0) {
		DRM_DEV_ERROR(panel->dev,
			"enter_sleep cmd failed ret = %d\n", ret);
	}

	ret = lg_panel_power_off(panel);
	if (ret < 0)
		DRM_DEV_ERROR(panel->dev, "power_off failed ret = %d\n", ret);

	pinfo->prepared = false;

	return ret;

}

static int lg_panel_power_on(struct panel_info *pinfo)
{
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(pinfo->supplies); i++) {
		ret = regulator_set_load(pinfo->supplies[i].consumer,
					regulator_enable_loads[i]);
		if (ret)
			return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(pinfo->supplies), pinfo->supplies);
	if (ret < 0)
		return ret;

	/*
	 * Reset sequence of LG sw43408 panel requires the panel to be
	 * out of reset for 9ms, followed by being held in reset
	 * for 1ms and then out again
	 */
	gpiod_set_value(pinfo->reset_gpio, 1);
	usleep_range(9000, 10000);
	gpiod_set_value(pinfo->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value(pinfo->reset_gpio, 1);
	usleep_range(9000, 10000);

	return 0;
}

static int lg_panel_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int err;

	if (pinfo->prepared)
		return 0;

	err = lg_panel_power_on(pinfo);
	if (err < 0)
		goto poweroff;

	/* send init code */
	err = send_mipi_cmds(panel, pinfo->desc->on_cmds);
	if (err < 0) {
		DRM_DEV_ERROR(panel->dev,
				"failed to send DCS Init Code: %d\n", err);
		goto poweroff;
	}

	pinfo->prepared = true;

	return 0;

poweroff:
	gpiod_set_value(pinfo->reset_gpio, 1);
	return err;
}

static int lg_panel_enable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (pinfo->enabled)
		return 0;

	ret = backlight_enable(pinfo->backlight);
	if (ret) {
		DRM_DEV_ERROR(panel->drm->dev,
				"Failed to enable backlight %d\n", ret);
		return ret;
	}

	pinfo->enabled = true;

	return 0;
}

static int lg_panel_get_modes(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	const struct drm_display_mode *m = pinfo->desc->display_mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, m);
	if (!mode) {
		DRM_DEV_ERROR(panel->drm->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, m->vrefresh);
		return -ENOMEM;
	}

	panel->connector->display_info.width_mm = pinfo->desc->width_mm;
	panel->connector->display_info.height_mm = pinfo->desc->height_mm;

	drm_mode_set_name(mode);
	drm_mode_probed_add(panel->connector, mode);

	return 1;
}

static const struct drm_panel_funcs panel_funcs = {
	.disable = lg_panel_disable,
	.unprepare = lg_panel_unprepare,
	.prepare = lg_panel_prepare,
	.enable = lg_panel_enable,
	.get_modes = lg_panel_get_modes,
};

static const struct panel_cmd lg_sw43408_on_cmds[] = {
	_INIT_CMD(0x00, 0x26, 0x02),	// MIPI_DCS_SET_GAMMA_CURVE, 0x02
	_INIT_CMD(0x00, 0x35, 0x00),	// MIPI_DCS_SET_TEAR_ON
	_INIT_CMD(0x00, 0x53, 0x0C, 0x30),
	_INIT_CMD(0x00, 0x55, 0x00, 0x70, 0xDF, 0x00, 0x70, 0xDF),
	_INIT_CMD(0x00, 0xF7, 0x01, 0x49, 0x0C),
	_INIT_CMD(0x00, 0x11),	// MIPI_DCS_EXIT_SLEEP_MODE
	_INIT_CMD(0x00, 0x11),	// repetition, but what does a 07 DTYPE mean?
	_INIT_CMD(0x00, 0xB0, 0xAC),
	_INIT_CMD(0x00, 0xCD,
			0x00, 0x00, 0x00, 0x19, 0x19, 0x19, 0x19, 0x19,
			0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
			0x16, 0x16),
	_INIT_CMD(0x00, 0xCB, 0x80, 0x5C, 0x07, 0x03, 0x28),
	_INIT_CMD(0x00, 0xC0, 0x02, 0x02, 0x0F),
	_INIT_CMD(0x00, 0xE5, 0x00, 0x3A, 0x00, 0x3A, 0x00, 0x0E, 0x10),
	_INIT_CMD(0x00, 0xB5,
			0x75, 0x60, 0x2D, 0x5D, 0x80, 0x00, 0x0A, 0x0B,
			0x00, 0x05, 0x0B, 0x00, 0x80, 0x0D, 0x0E, 0x40,
			0x00, 0x0C, 0x00, 0x16, 0x00, 0xB8, 0x00, 0x80,
			0x0D, 0x0E, 0x40, 0x00, 0x0C, 0x00, 0x16, 0x00,
			0xB8, 0x00, 0x81, 0x00, 0x03, 0x03, 0x03, 0x01,
			0x01),
	_INIT_CMD(0x00, 0x55, 0x04, 0x61, 0xDB, 0x04, 0x70, 0xDB),
	_INIT_CMD(0x00, 0xB0, 0xCA),
	_INIT_CMD(0x00, 0x29),

	{},
};

static const struct drm_display_mode lg_panel_default_mode = {
	.clock		= 152340,

	.hdisplay	= 1080,
	.hsync_start	= 1080 + 20,
	.hsync_end	= 1080 + 20 + 32,
	.htotal		= 1080 + 20 + 32 + 20,

	.vdisplay	= 2160,
	.vsync_start	= 2160 + 20,
	.vsync_end	= 2160 + 20 + 4,
	.vtotal		= 2160 + 20 + 4 + 20,
	.vrefresh	= 60,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct panel_desc lg_panel_desc = {
	.display_mode = &lg_panel_default_mode,

	.width_mm = 62,
	.height_mm = 124,

	.mode_flags = MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
	.on_cmds = lg_sw43408_on_cmds,
};


static const struct of_device_id panel_of_match[] = {
	{ .compatible = "lg,sw43408",
	  .data = &lg_panel_desc
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, panel_of_match);

static int panel_add(struct panel_info *pinfo)
{
	struct device *dev = &pinfo->link->dev;
	int i, ret;
pr_err("In sw43408 panel add\n");
	for (i = 0; i < ARRAY_SIZE(pinfo->supplies); i++)
		pinfo->supplies[i].supply = regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(pinfo->supplies),
				      pinfo->supplies);
	if (ret < 0)
		return ret;

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get reset gpio %ld\n",
			PTR_ERR(pinfo->reset_gpio));
		return PTR_ERR(pinfo->reset_gpio);
	}

	pinfo->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(pinfo->backlight))
		return PTR_ERR(pinfo->backlight);

	drm_panel_init(&pinfo->base);
pr_err("In sw43408 panel add: after drm_panel_init\n");
	pinfo->base.funcs = &panel_funcs;
	pinfo->base.dev = &pinfo->link->dev;

	ret = drm_panel_add(&pinfo->base);
	if (ret < 0)
		return ret;
pr_err("In sw43408 panel add: drm_panel_add returned %d\n", ret);
	return 0;
}

static void panel_del(struct panel_info *pinfo)
{
	if (pinfo->base.dev)
		drm_panel_remove(&pinfo->base);
}

static int panel_probe(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo;
	const struct panel_desc *desc;
	int err;

	pinfo = devm_kzalloc(&dsi->dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->mode_flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;
	pinfo->desc = desc;

	pinfo->link = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);
pr_err("In sw43408 panel probe\n");

	err = panel_add(pinfo);
	if (err < 0)
		return err;

	err = mipi_dsi_attach(dsi);
pr_err("In sw43408 panel probe: mipi_dsi_attach returned: %d\n", err);
	return err;	
}

static int panel_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int err;

	err = lg_panel_unprepare(&pinfo->base);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to unprepare panel: %d\n",
				err);

	err = lg_panel_disable(&pinfo->base);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to disable panel: %d\n", err);

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to detach from DSI host: %d\n",
				err);

	drm_panel_detach(&pinfo->base);
	panel_del(pinfo);

	return 0;
}

static void panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);

	lg_panel_disable(&pinfo->base);
	lg_panel_unprepare(&pinfo->base);
}

static struct mipi_dsi_driver panel_driver = {
	.driver = {
		.name = "panel-lg-sw43408",
		.of_match_table = panel_of_match,
	},
	.probe = panel_probe,
	.remove = panel_remove,
	.shutdown = panel_shutdown,
};
module_mipi_dsi_driver(panel_driver);

MODULE_AUTHOR("Sumit Semwal <sumit.semwal@linaro.org>");
MODULE_DESCRIPTION("LG SW436408 MIPI-DSI LED panel");
MODULE_LICENSE("GPL");
