// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2022, US Micro Products
 * Modified: Daniel Wu
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/media-bus-format.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

// copy from rm67191
#include <linux/of.h>
#include <linux/of_platform.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_print.h>
//^^^^^^^^^^^^^^^^^^^^^

// Add for switch panel function
struct ek79202d_desc {
	const struct ek79202d_init_cmd *init;
	const size_t init_length;
	const struct drm_display_mode *mode;
};
//^^^^^^^^^^^^^^^^^^^^^

struct ek79202d {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
// Add for switch panel function
	const struct ek79202d_desc	*desc;
//^^^^^^^^^^^^^^^^^^^^

	struct backlight_device *backlight;
	struct regulator *vdd;
//	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
};

static const u32 fitipower_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 fitipower_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_SAMPLE_NEGEDGE;

static inline struct ek79202d *panel_to_ek79202d(struct drm_panel *panel)
{
	return container_of(panel, struct ek79202d, panel);
}

struct ek79202d_init_cmd {
	size_t len;
	const char *data;
};

#define EK79202D_INIT_CMD(...) { \
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

static const struct ek79202d_init_cmd ek97202d_leadership_init_cmds[] = {

	EK79202D_INIT_CMD(0xCD,0xAA),
	EK79202D_INIT_CMD(0x52,0x13,0x13,0x13,0x13,0x13,0x13,0x12,0x13,0x10,0x11,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x03,0x0C,0x13,0x13),
	EK79202D_INIT_CMD(0x59,0x13,0x13,0x13,0x13,0x13,0x13,0x12,0x13,0x10,0x11,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x03,0x0C,0x13,0x13),
	EK79202D_INIT_CMD(0x32,0x02), //BIST mode select: BIST_PIN=High with 0x32=0x02 normal display.
	EK79202D_INIT_CMD(0x34,0x7E),
	EK79202D_INIT_CMD(0x5F,0x38),
	EK79202D_INIT_CMD(0x2B,0x20),
	EK79202D_INIT_CMD(0x35,0x25), //25
	EK79202D_INIT_CMD(0x33,0x08), //ZIGZAG=1
	EK79202D_INIT_CMD(0x51,0x80),
	EK79202D_INIT_CMD(0x73,0xF0),
	EK79202D_INIT_CMD(0x74,0x91),
	EK79202D_INIT_CMD(0x75,0x03),
	EK79202D_INIT_CMD(0x71,0xC3),
	EK79202D_INIT_CMD(0x7A,0x17),
	EK79202D_INIT_CMD(0x3C,0x40),
	EK79202D_INIT_CMD(0x4A,0x02),
	EK79202D_INIT_CMD(0x18,0xFF),
	EK79202D_INIT_CMD(0x19,0x1F),
	EK79202D_INIT_CMD(0x1A,0xDC),
	EK79202D_INIT_CMD(0x4E,0x4A),
	EK79202D_INIT_CMD(0x4F,0x4C),
	EK79202D_INIT_CMD(0x53,0x37,0x2A,0x29,0x2A,0x2E,0x2F,0x22,0x0D,0x0E,0x0C,0x0E,0x0F,0x10),
	EK79202D_INIT_CMD(0x54,0x37,0x2A,0x29,0x2A,0x2E,0x2F,0x22,0x0D,0x0E,0x0C,0x0E,0x0F,0x10),
	EK79202D_INIT_CMD(0x55,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11),
	EK79202D_INIT_CMD(0x56,0x08),
	EK79202D_INIT_CMD(0x67,0x22),
	EK79202D_INIT_CMD(0x6F,0x01,0x01,0x01,0x0A,0x01,0x01),
	EK79202D_INIT_CMD(0x6D,0xA5),
	EK79202D_INIT_CMD(0x6C,0x08),
	EK79202D_INIT_CMD(0x0E,0x0A),

	EK79202D_INIT_CMD(0x5E,0x02),
	EK79202D_INIT_CMD(0x4A,0x04),
	EK79202D_INIT_CMD(0x7A,0x37),
	
};

static int ek79202d_prepare(struct drm_panel *panel)
{
	struct ek79202d *ctx = panel_to_ek79202d(panel);
	int ret;

	ret = regulator_enable(ctx->vdd);
	if (ret)
	return ret;

//	msleep(50)
//	Delay for x8h7 get ready	
	msleep(500);

//	gpiod_set_value_cansleep(ctx->reset_gpio, 1); // set to high
//	msleep(150);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0); // set to low
	msleep(5);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1); // set to high
	msleep(50);

	return 0;
}

static int ek79202d_enable(struct drm_panel *panel)
{
	struct ek79202d *ctx = panel_to_ek79202d(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned int i;
	int ret;

// Must change to Low Power Mode to send initialization code
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

// Must move following codes from prepare function to solve no crtc problem.
	for (i = 0; i < ctx->desc->init_length; i++) {
		const struct ek79202d_init_cmd *cmd = &ctx->desc->init[i];

		ret = mipi_dsi_generic_write(dsi, cmd->data, cmd->len);
		if (ret < 0)
		return ret;

//		msleep(50);
	}
//^^^^^^^^^^^^^^^^^

	msleep(240);
	
	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret)
		return ret;

	msleep(240);

	mipi_dsi_dcs_set_display_on(ctx->dsi);

	msleep(100);

//	For control PMIC EN pin
//	gpiod_set_value_cansleep(ctx->reset_gpio, 0); // set to low
//	msleep(250);

//	gpiod_set_value_cansleep(ctx->reset_gpio, 1); // set to high
//	msleep(50);

	return 0;
}

static int ek79202d_disable(struct drm_panel *panel)
{
	struct ek79202d *ctx = panel_to_ek79202d(panel);

	backlight_disable(ctx->backlight);
	return mipi_dsi_dcs_set_display_on(ctx->dsi);
}

static int ek79202d_unprepare(struct drm_panel *panel)
{
	struct ek79202d *ctx = panel_to_ek79202d(panel);
	int ret;

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
	dev_err(panel->dev, "failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	if (ret < 0)
	dev_err(panel->dev, "failed to enter sleep mode: %d\n", ret);

	msleep(100);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(50);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);

	regulator_disable(ctx->vdd);

	return 0;
}

static const struct drm_display_mode ek79202d_leadership_mode = {

	.clock 	= 70000,
//	.clock 	= 72400,
	
	.hdisplay 	= 1280,
	.hsync_start 	= 1280 + 10,
	.hsync_end 	= 1280 + 10 + 12,
	.htotal 	= 1280 + 10 + 12 + 88,
	
	.vdisplay 	= 800,
	.vsync_start 	= 800 + 2,
	.vsync_end 	= 800 + 2 + 1,
	.vtotal 	= 800 + 2 + 1 + 23,

	.width_mm 	= 143,
	.height_mm 	= 229,

};

static int ek79202d_get_modes(struct drm_panel *panel, 
				struct drm_connector *connector)
{
//	struct drm_connector *connector = panel->connector;
	struct ek79202d *ctx = panel_to_ek79202d(panel);
	struct drm_display_mode *mode;

// Add for switch panel function
	mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
	if (!mode) {
		dev_err(&ctx->dsi->dev, "failed to add mode %ux%ux@%u\n",
			ctx->desc->mode->hdisplay,
			ctx->desc->mode->vdisplay,
			drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}
//^^^^^^^^^^^^^^^^
	
	drm_mode_set_name(mode);
	
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	/* Add Bus Formats support */
	connector->display_info.bus_flags = fitipower_bus_flags;
	drm_display_info_set_bus_formats(&connector->display_info,
					 fitipower_bus_formats,
					 ARRAY_SIZE(fitipower_bus_formats));

	
	return 1;
}

static const struct drm_panel_funcs ek79202d_funcs = {
	.disable = ek79202d_disable,
	.unprepare = ek79202d_unprepare,
	.prepare = ek79202d_prepare,
	.enable = ek79202d_enable,
	.get_modes = ek79202d_get_modes,
};

static int ek79202d_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np;
	struct ek79202d *ctx;
	int ret;
	u32 video_mode, dsi_lanes;
	
	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		return -ENOMEM;
	}

// Add for switch panel function
	ctx->desc = of_device_get_match_data(&dsi->dev);
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	drm_panel_init(&ctx->panel, dev, &ek79202d_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = &dsi->dev;
	ctx->panel.funcs = &ek79202d_funcs;

	ctx->vdd = devm_regulator_get(&dsi->dev, "vdd");
	if (IS_ERR(ctx->vdd)) {
		dev_err(&dsi->dev, "Couldn't get vdd regulator\n");
		return PTR_ERR(ctx->vdd);
	}

/*	ctx->enable_gpio = devm_gpiod_get(&dsi->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio)) {
		dev_err(&dsi->dev, "Couldn't get our enable GPIO\n");
		return PTR_ERR(ctx->enable_gpio);
	}
*/
	ctx->reset_gpio = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(ctx->reset_gpio);
	}

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;

	/* get DSI Mode and Data Lanes from device tree */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO;
	ret = of_property_read_u32(dsi->dev.of_node, "video-mode", &video_mode);
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
			dev_warn(&dsi->dev, "invalid video mode %d\n", video_mode);
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		}
	}

	dsi->format = MIPI_DSI_FMT_RGB888;
	ret = of_property_read_u32(dsi->dev.of_node, "dsi-lanes", &dsi_lanes);
	dsi->lanes = dsi_lanes;
	
	return mipi_dsi_attach(dsi);
}

static void ek79202d_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct ek79202d *ctx = mipi_dsi_get_drvdata(dsi);
	
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return;
}

// Add for switch panel function

static const struct ek79202d_desc ek79202d_leadership_desc = {
	.init = ek97202d_leadership_init_cmds,
	.init_length = ARRAY_SIZE(ek97202d_leadership_init_cmds),
	.mode = &ek79202d_leadership_mode,
};
// ^^^^^^^^^^^^^^^^^^^^

static const struct of_device_id ek79202d_of_match[] = {
	{ .compatible = "usmp_leadership,ek79202d", .data = &ek79202d_leadership_desc},
	{ }
};
MODULE_DEVICE_TABLE(of, ek79202d_of_match);

static struct mipi_dsi_driver ek79202d_driver = {
	.probe = ek79202d_dsi_probe,
	.remove = ek79202d_dsi_remove,
	.driver = {
	.name = "ek79202d-dsi",
	.of_match_table = ek79202d_of_match,
	},
};
module_mipi_dsi_driver(ek79202d_driver);

MODULE_AUTHOR("Daniel Wu <dwu@xxxxxxxxxxxxxxxxxxxx>");
MODULE_DESCRIPTION("Fitipower EK79202D Controller Driver");
MODULE_LICENSE("GPL v2");
