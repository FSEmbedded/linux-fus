/*
 * i.MX drm driver - Novatek MIPI-DSI panel driver
 *
 * Copyright (C) 2019 F&S Ele
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#define MAX_DATA_LEN 20


typedef struct cmd_table {
    u8      cmd_type;
    size_t  payload_len;
    char    payload[MAX_DATA_LEN];
} cmd_table;

struct cmd_data {
	const struct cmd_table *cmd_list;
	size_t len;
};

static const struct cmd_table def_cmd_table_rg_t070qyh_13cp[] = {
    /* SET EXTC */
    {0x29, 0x05, {0xFF, 0xAA, 0x55, 0xA5, 0x80}},
    {0x29, 0x03, {0x6F, 0x11, 0x00}},
    {0x29, 0x03, {0xF7, 0x20, 0x00}},
    {0x23, 0x02, {0x6F, 0x06}},
    {0x23, 0x02, {0xF7, 0xA0}},
    {0x23, 0x02, {0x6F, 0x19}},
    {0x23, 0x02, {0xF7, 0x12}},
    {0x23, 0x02, {0xF4, 0x03}},
    {0x23, 0x02, {0x6F, 0x08}},
    {0x23, 0x02, {0xFA, 0x40}},
    {0x23, 0x02, {0x6F, 0x11}},
    {0x23, 0x02, {0xF3, 0x01}},

    /* page 0 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
    {0x23, 0x02, {0xC8, 0x80}},
    {0x29, 0x03, {0xB1, 0x6C, 0x01}},
    {0x23, 0x02, {0xB6, 0x08}},
    {0x23, 0x02, {0x6F, 0x02}},
    {0x23, 0x02, {0xB8, 0x08}},
    {0x29, 0x03, {0xBB, 0x54, 0x54}},
    {0x29, 0x03, {0xBC, 0x05, 0x05}},
    {0x23, 0x02, {0xC7, 0x01}},
    {0x29, 0x06, {0xBD, 0x02, 0xB0, 0x0C, 0x0A, 0x00}},

    /* page 1 */
    {0x29, 0x07, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}},
    {0x29, 0x03, {0xB0, 0x05, 0x05}},
    {0x29, 0x03, {0xB1, 0x05, 0x05}},
    {0x29, 0x03, {0xBC, 0x3A, 0x01}},
    {0x29, 0x03, {0xBD, 0x3E, 0x01}},
    {0x23, 0x02, {0xCA, 0x00}},
    {0x23, 0x02, {0xC0, 0x04}},
    {0x23, 0x02, {0xB2, 0x00}},
    {0x23, 0x02, {0xBE, 0x80}},
    {0x29, 0x03, {0xB3, 0x19, 0x19}},
    {0x29, 0x03, {0xB4, 0x12, 0x12}},
    {0x29, 0x03, {0xB9, 0x24, 0x24}},
    {0x29, 0x03, {0xBA, 0x14, 0x14}},

    /* page 2 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02}},
    {0x23, 0x02, {0xEE, 0x01}},
    {0x29, 0x05, {0xEF, 0x09, 0x06, 0x15, 0x18}},
    {0x29, 0x07, {0xB0, 0x00, 0x00, 0x00, 0x08, 0x00, 0x17}},
    {0x23, 0x02, {0x6F, 0x06}},
    {0x29, 0x07, {0xB0, 0x00, 0x25, 0x00, 0x30, 0x00, 0x45}},
    {0x23, 0x02, {0x6F, 0x0C}},
    {0x29, 0x05, {0xB0, 0x00, 0x56, 0x00, 0x7A}},
    {0x29, 0x07, {0xB1, 0x00, 0xA3, 0x00, 0xE7, 0x01, 0x20}},
    {0x23, 0x02, {0x6F, 0x06}},
    {0x29, 0x07, {0xB1, 0x01, 0x7A, 0x01, 0xC2, 0x01, 0xC5}},
    {0x23, 0x02, {0x6F, 0x0C}},
    {0x29, 0x05, {0xB1, 0x02, 0x06, 0x02, 0x5F}},
    {0x29, 0x07, {0xB2, 0x02, 0x92, 0x02, 0xD0, 0x02, 0xFC}},
    {0x23, 0x02, {0x6F, 0x06}},
    {0x29, 0x07, {0xB2, 0x03, 0x35, 0x03, 0x5D, 0x03, 0x8B}},
    {0x23, 0x02, {0x6F, 0x0C}},
    {0x29, 0x05, {0xB2, 0x03, 0xA2, 0x03, 0xBF}},
    {0x29, 0x05, {0xB3, 0x03, 0xD2, 0x03, 0xFF}},

    /* page 6 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x06}},
    {0x29, 0x03, {0xB0, 0x00, 0x17}},
    {0x29, 0x03, {0xB1, 0x16, 0x15}},
    {0x29, 0x03, {0xB2, 0x14, 0x13}},
    {0x29, 0x03, {0xB3, 0x12, 0x11}},
    {0x29, 0x03, {0xB4, 0x10, 0x2D}},
    {0x29, 0x03, {0xB5, 0x01, 0x08}},
    {0x29, 0x03, {0xB6, 0x09, 0x31}},
    {0x29, 0x03, {0xB7, 0x31, 0x31}},
    {0x29, 0x03, {0xB8, 0x31, 0x31}},
    {0x29, 0x03, {0xB9, 0x31, 0x31}},
    {0x29, 0x03, {0xBA, 0x31, 0x31}},
    {0x29, 0x03, {0xBB, 0x31, 0x31}},
    {0x29, 0x03, {0xBC, 0x31, 0x31}},
    {0x29, 0x03, {0xBD, 0x31, 0x09}},
    {0x29, 0x03, {0xBE, 0x08, 0x01}},
    {0x29, 0x03, {0xBF, 0x2D, 0x10}},

    {0x29, 0x03, {0xC0, 0x11, 0x12}},
    {0x29, 0x03, {0xC1, 0x13, 0x14}},
    {0x29, 0x03, {0xC2, 0x15, 0x16}},
    {0x29, 0x03, {0xC3, 0x17, 0x00}},

    {0x29, 0x03, {0xE5, 0x31, 0x31}},

    {0x29, 0x03, {0xC4, 0x00, 0x17}},
    {0x29, 0x03, {0xC5, 0x16, 0x15}},
    {0x29, 0x03, {0xC6, 0x14, 0x13}},
    {0x29, 0x03, {0xC7, 0x12, 0x11}},
    {0x29, 0x03, {0xC8, 0x10, 0x2D}},
    {0x29, 0x03, {0xC9, 0x01, 0x08}},
    {0x29, 0x03, {0xCA, 0x09, 0x31}},
    {0x29, 0x03, {0xCB, 0x31, 0x31}},
    {0x29, 0x03, {0xCC, 0x31, 0x31}},
    {0x29, 0x03, {0xCD, 0x31, 0x31}},
    {0x29, 0x03, {0xCE, 0x31, 0x31}},
    {0x29, 0x03, {0xCF, 0x31, 0x31}},

    {0x29, 0x03, {0xD0, 0x31, 0x31}},
    {0x29, 0x03, {0xD1, 0x31, 0x09}},
    {0x29, 0x03, {0xD2, 0x08, 0x01}},
    {0x29, 0x03, {0xD3, 0x2D, 0x10}},
    {0x29, 0x03, {0xD4, 0x11, 0x12}},
    {0x29, 0x03, {0xD5, 0x13, 0x14}},
    {0x29, 0x03, {0xD6, 0x15, 0x16}},
    {0x29, 0x03, {0xD7, 0x17, 0x00}},

    {0x29, 0x03, {0xE6, 0x31, 0x31}},

    {0x29, 0x06, {0xD8, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xD9, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x23, 0x02, {0xE7, 0x00}},

    /* page 3 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03}},
    {0x29, 0x03, {0xB0, 0x20, 0x00}},
    {0x29, 0x03, {0xB1, 0x20, 0x00}},
    {0x29, 0x06, {0xB2, 0x05, 0x00, 0x42, 0x00, 0x00}},

    {0x29, 0x06, {0xB6, 0x05, 0x00, 0x42, 0x00, 0x00}},

    {0x29, 0x06, {0xBA, 0x53, 0x00, 0x42, 0x00, 0x00}},
    {0x29, 0x06, {0xBB, 0x53, 0x00, 0x42, 0x00, 0x00}},

    {0x23, 0x02, {0xC4, 0x40}},

    /* page 5 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}},
    {0x29, 0x03, {0xB0, 0x17, 0x06}},

    {0x23, 0x02, {0xB8, 0x00}},

    {0x29, 0x06, {0xBD, 0x03, 0x01, 0x01, 0x00, 0x01}},

    {0x29, 0x03, {0xB1, 0x17, 0x06}},
    {0x29, 0x03, {0xB9, 0x00, 0x01}},
    {0x29, 0x03, {0xB2, 0x17, 0x06}},
    {0x29, 0x03, {0xBA, 0x00, 0x01}},
    {0x29, 0x03, {0xB3, 0x17, 0x06}},
    {0x29, 0x03, {0xBB, 0x0A, 0x00}},

    {0x29, 0x03, {0xB4, 0x17, 0x06}},
    {0x29, 0x03, {0xB5, 0x17, 0x06}},
    {0x29, 0x03, {0xB6, 0x14, 0x03}},
    {0x29, 0x03, {0xB7, 0x00, 0x00}},
    {0x29, 0x03, {0xBC, 0x02, 0x01}},
    {0x23, 0x02, {0xC0, 0x05}},

    {0x23, 0x02, {0xC4, 0xA5}},

    {0x29, 0x03, {0xC8, 0x03, 0x30}},
    {0x29, 0x03, {0xC9, 0x03, 0x51}},

    {0x29, 0x06, {0xD1, 0x00, 0x05, 0x03, 0x00, 0x00}},
    {0x29, 0x06, {0xD2, 0x00, 0x05, 0x09, 0x00, 0x00}},

    {0x23, 0x02, {0xE5, 0x02}},
    {0x23, 0x02, {0xE6, 0x02}},
    {0x23, 0x02, {0xE7, 0x02}},
    {0x23, 0x02, {0xE9, 0x02}},
    {0x23, 0x02, {0xED, 0x33}},
};

static const struct cmd_data config_table[] = {
	{
		.cmd_list = def_cmd_table_rg_t070qyh_13cp,
		.len = sizeof(def_cmd_table_rg_t070qyh_13cp) / sizeof(cmd_table)
	}
};

static const struct of_device_id nt_of_match[] = {
	{
		.compatible = "rongen,t070qyh-13cp",
	  	.data = &config_table
	}, {
		.compatible = "auo,g070tan01.0",
		.data = NULL
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, nt_of_match);

#define CMD_TABLE_LEN 2
typedef u8 cmd_set_table[CMD_TABLE_LEN];

static const u32 nt_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct nt35521 {
	struct device *dev;
	struct drm_panel panel;
	struct regulator_bulk_data supplies[2];

	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

static inline struct nt35521 *panel_to_nt35521(struct drm_panel *panel)
{
	return container_of(panel, struct nt35521, panel);
}

static int nt35521_push_cmd_list(struct mipi_dsi_device *dsi, const struct cmd_data *cmd_data_list)
{
	const struct cmd_table *cmd;
	const struct cmd_table *cmd_list = cmd_data_list->cmd_list;
	int ret = 0;
	size_t i, count = cmd_data_list->len;

	/* tx data */
	for (i = 0; i < count ; i++) {
		cmd = &cmd_list[i];

		switch(cmd->cmd_type)
		{
			case MIPI_DSI_DCS_SHORT_WRITE:
				ret = mipi_dsi_dcs_write(dsi, cmd->payload[0], 0, 0);
				break;
			case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
			case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
			case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
			case MIPI_DSI_GENERIC_LONG_WRITE:
			default:
				ret = mipi_dsi_generic_write(dsi, cmd->payload, cmd->payload_len);
        }
        if (ret < 0)
            return ret;
	}

	return ret;
};

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return 0x55;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return 0x66;
	case MIPI_DSI_FMT_RGB888:
		return 0x77;
	default:
		return 0x77; /* for backward compatibility */
	}
};

static int nt35521_power_on(struct nt35521 *nt)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(nt->supplies), nt->supplies);
	if (ret < 0) {
		dev_err(nt->dev, "unable to enable regulators\n");
		return ret;
	}

	/* Toggle RESET in accordance with datasheet page 370 */
	if (nt->reset_gpio) {
		gpiod_set_value(nt->reset_gpio, 1);
		/* Active min 10 us according to datasheet, let's say 20 */
		usleep_range(20, 1000);
		gpiod_set_value(nt->reset_gpio, 0);
		/*
		 * 5 ms during sleep mode, 120 ms during sleep out mode
		 * according to datasheet, let's use 120-140 ms.
		 */
		usleep_range(200000, 250000);
	}

	return 0;
}

static int nt35521_prepare(struct drm_panel *panel)
{
	struct nt35521 *nt = panel_to_nt35521(panel);
	int ret = 0;

	if (nt->prepared)
		return 0;

	ret = nt35521_power_on(nt);
	if (ret)
		return ret;

	nt->prepared = true;

	return 0;
}

static int nt35521_power_off(struct nt35521 *nt)
{
	int ret;

	ret = regulator_bulk_disable(ARRAY_SIZE(nt->supplies), nt->supplies);
	if (ret)
		return ret;

	if (nt->reset_gpio)
		gpiod_set_value(nt->reset_gpio, 1);

	return 0;
}

static int nt35521_unprepare(struct drm_panel *panel)
{
	struct nt35521 *nt = panel_to_nt35521(panel);
	int ret;

	if (!nt->prepared)
		return 0;

	if (nt->enabled) {
		dev_err(nt->dev, "Panel still enabled!\n");
		return -EPERM;
	}

	ret = nt35521_power_off(nt);
	if (ret)
		return ret;

	nt->prepared = false;

	return 0;
}

static int nt35521_enable(struct drm_panel *panel)
{
	struct nt35521 *nt = panel_to_nt35521(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(nt->dev);
	int color_format = color_format_from_dsi_format(dsi->format);
	const struct of_device_id *id;
	u16 brightness;
	int ret;

	if (nt->enabled)
		return 0;

	if (!nt->prepared) {
		dev_err(nt->dev, "Panel not prepared!\n");
		return -EPERM;
	}

	id = of_match_node(nt_of_match, nt->dev->of_node);
	if (!id)
		return -ENODEV;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		dev_err(nt->dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}

	msleep(120);

	if (id->data) {
		ret = nt35521_push_cmd_list(dsi, id->data);
		if (ret < 0) {
			dev_err(nt->dev, "Failed to send MCS (%d)\n", ret);
			goto fail;
		}
	}

	/* Set pixel format */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
	dev_dbg(nt->dev, "Interface color format set to 0x%x\n",
				color_format);
	if (ret < 0) {
		dev_err(nt->dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}

	/* Set display brightness */
	brightness = nt->backlight->props.brightness;
	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0) {
		dev_err(nt->dev, "Failed to set display brightness (%d)\n",
			      ret);
		goto fail;
	}

	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(nt->dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	msleep(800);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(nt->dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	msleep(40);

	nt->backlight->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(nt->backlight);

	nt->enabled = true;

	return 0;

fail:
	if (nt->reset_gpio)
		gpiod_set_value(nt->reset_gpio, 1);

	return ret;
}

static int nt35521_disable(struct drm_panel *panel)
{
	struct nt35521 *nt = panel_to_nt35521(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(nt->dev);
	int ret;

	if (!nt->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(nt->dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(nt->dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	usleep_range(10000, 15000);

	nt->backlight->props.power = FB_BLANK_POWERDOWN;
	backlight_update_status(nt->backlight);

	nt->enabled = false;

	return 0;
}

static int nt35521_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct nt35521 *nt = panel_to_nt35521(panel);
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		dev_err(nt->dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&nt->vm, mode);
	mode->width_mm = nt->width_mm;
	mode->height_mm = nt->height_mm;
	connector->display_info.width_mm = nt->width_mm;
	connector->display_info.height_mm = nt->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	if (nt->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (nt->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (nt->vm.flags & DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;
	if (nt->vm.flags & DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE )
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE ;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			nt_bus_formats, ARRAY_SIZE(nt_bus_formats));
	if (ret)
		return ret;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static int nt_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct nt35521 *nt = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	if (!nt->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int nt_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct nt35521 *nt = mipi_dsi_get_drvdata(dsi);
	//struct device *dev = &dsi->dev;
	int ret = 0;

	if (!nt->prepared)
		return 0;

	dev_dbg(nt->dev, "New brightness: %d\n", bl->props.brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops nt_bl_ops = {
	.update_status = nt_bl_update_status,
	.get_brightness = nt_bl_get_brightness,
};

static const struct drm_panel_funcs nt35521_funcs = {
	.prepare = nt35521_prepare,
	.unprepare = nt35521_unprepare,
	.enable = nt35521_enable,
	.disable = nt35521_disable,
	.get_modes = nt35521_get_modes,
};


/*
 * The clock might range from 66MHz (30Hz refresh rate)
 * to 132MHz (60Hz refresh rate)
 */
static const struct display_timing nt_default_timing = {
	.pixelclock = { 66000000, 132000000, 132000000 },
	.hactive = { 1080, 1080, 1080 },
	.hfront_porch = { 20, 20, 20 },
	.hsync_len = { 2, 2, 2 },
	.hback_porch = { 34, 34, 34 },
	.vactive = { 1920, 1920, 1920 },
	.vfront_porch = { 10, 10, 10 },
	.vsync_len = { 2, 2, 2 },
	.vback_porch = { 4, 4, 4 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
		 DISPLAY_FLAGS_VSYNC_LOW |
		 DISPLAY_FLAGS_DE_LOW |
		 DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int nt35521_parse_dt(struct nt35521 *nt, struct mipi_dsi_device *dsi)
{
	struct device *dev = nt->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
    /* use default burst mode */
	u32 video_mode = 0;
	int ret;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;

		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	/*
	 * 'display-timings' is optional, so verify if the node is present
	 * before calling of_get_videomode so we won't get console error
	 * messages
	 */
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
		of_node_put(timings);
		ret = of_get_videomode(np, &nt->vm, 0);
	} else {
		videomode_from_timing(&nt_default_timing, &nt->vm);
	}
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "panel-width-mm", &nt->width_mm);
	of_property_read_u32(np, "panel-height-mm", &nt->height_mm);

	return ret;
}

static int nt35521_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt35521 *nt;
	struct backlight_properties bl_props;
	int ret;

	nt = devm_kzalloc(&dsi->dev, sizeof(*nt), GFP_KERNEL);
	if (!nt)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, nt);
	nt->dev = dev;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO
                | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_BURST;

	ret = nt35521_parse_dt(nt, dsi);

	nt->supplies[0].supply = "vci"; /* 2.5-3.6 V */
	nt->supplies[1].supply = "vddi"; /* 1.65-3.6 V */
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(nt->supplies),
									nt->supplies);
	if (ret < 0)
		return ret;

	nt->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(nt->reset_gpio)) {
		dev_err(nt->dev,
	      "error getting RESET GPIO\n");
		return PTR_ERR(nt->reset_gpio);
	}

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	nt->backlight = devm_backlight_device_register(
				dev, dev_name(dev),
				dev, dsi,
				&nt_bl_ops, &bl_props);
	if (IS_ERR(nt->backlight)) {
		ret = PTR_ERR(nt->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&nt->panel, dev, &nt35521_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&nt->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&nt->panel);

	return ret;
}

static int nt35521_remove(struct mipi_dsi_device *dsi)
{
	struct nt35521 *nt = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(nt->dev, "Failed to detach from host (%d)\n",
			ret);

	ret = nt35521_power_off(nt);
	if (ret < 0)
		dev_err(nt->dev, "Failed to power off (%d)\n",
			ret);
	drm_panel_remove(&nt->panel);

	return ret;
}

static void nt35521_shutdown(struct mipi_dsi_device *dsi)
{
	struct nt35521 *nt = mipi_dsi_get_drvdata(dsi);

	nt35521_disable(&nt->panel);
	nt35521_unprepare(&nt->panel);
}

static struct mipi_dsi_driver nt35521_driver = {
	.driver = {
		.name = "panel-novatek-nt35521",
		.of_match_table = nt_of_match,
	},
	.probe = nt35521_probe,
	.remove = nt35521_remove,
	.shutdown = nt35521_shutdown,
};
module_mipi_dsi_driver(nt35521_driver);

MODULE_AUTHOR("F&S Elektronik Systeme GmbH");
MODULE_DESCRIPTION("Novatek NT35521");
MODULE_LICENSE("GPL v2");
