// SPDX-License-Identifier: GPL-s2.0-only
/*
 * Copyright(c) 2021, Foundries.io.
 * Copyright(c) 2020, Analogix Semiconductor. All rights reserved.
 * Copyright(c) 2020, Foundries.io. All rights reserved.
 *
 */
#include <linux/gcd.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <video/display_timing.h>

#include <sound/core.h>
#include <sound/hdmi-codec.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <linux/timer.h>

#include "anx7625.h"

#define U_BOOT_EARLY_INIT
//#define DISABLE_PD
#define GPIO_VBUS_CONTROL
#define CABLE_DET_PIN_HAS_GLITCH
#define TIMER_CABLE_DET_POLL_DELAY (1 * HZ)
#define TIMEOUT_USB_DATA_ROLE 3

#define DEBUG

#ifdef DEBUG
#ifdef DRM_DEV_DEBUG_DRIVER
#undef DRM_DEV_DEBUG_DRIVER
#define DRM_DEV_DEBUG_DRIVER(dev, fmt, ...) \
dev_printk(KERN_ERR, dev, fmt, ##__VA_ARGS__)
#endif

#ifdef DRM_DEV_ERROR
#undef DRM_DEV_ERROR
#define DRM_DEV_ERROR(dev, fmt, ...) \
dev_printk(KERN_ERR, dev, fmt, ##__VA_ARGS__)
#endif
#endif

/* Global variables */
static int sys_sta_bak;
struct timer_list timer_cable_det;
struct anx7625_data *ctx_timer_cable_det;
#ifdef U_BOOT_EARLY_INIT
static int pending_video_work = 0;
#endif

/* Functions to report data role and power role changes to upper level */
static void anx7625_set_data_role(struct anx7625_data *ctx,
				    enum typec_data_role data_role)
{
	struct device *dev = &ctx->client->dev;
	enum usb_role usb_role = USB_ROLE_NONE;
	int ret = 0;

	if (data_role == TYPEC_HOST) {
		usb_role = USB_ROLE_HOST;
		DRM_DEV_DEBUG_DRIVER(dev, "Type C USB host role selected\n");
	}
	else {
		usb_role = USB_ROLE_DEVICE;
		DRM_DEV_DEBUG_DRIVER(dev, "Type C USB device role selected\n");
	}

	ret = usb_role_switch_set_role(ctx->usb_typec->role_sw, usb_role);
	if(ret)
		DRM_DEV_ERROR(dev, "Failed to perform usb role switch to usb peripheral: %d\n", ret);

	typec_set_data_role(ctx->usb_typec->port, data_role);
	ctx->usb_typec->usb_data_role_timeout = false;
}

static void anx7625_set_pwr_role(struct anx7625_data *ctx,
				    enum typec_role pwr_role,
				    enum typec_pwr_opmode opmode,
				    enum typec_role vconn_role)
{
	typec_set_pwr_role(ctx->usb_typec->port, pwr_role);
	typec_set_pwr_opmode(ctx->usb_typec->port, opmode);
	typec_set_vconn_role(ctx->usb_typec->port, vconn_role);
}

static int anx7625_get_fw_caps(struct anx7625_data *ctx,
				 struct fwnode_handle *fwnode)
{
	const char *cap_str;
	int ret;

	ctx->usb_typec->capability.fwnode = fwnode;
	ctx->usb_typec->capability.revision = 0x0120;	/* Type-C spec release 1.2 */
	ctx->usb_typec->capability.pd_revision = 0x0300;	/* USB-PD spec release 3.0 */

	/*
	 * Supported port type can be configured through device tree
	 */
	ret = fwnode_property_read_string(fwnode, "power-role", &cap_str);
	if (!ret) {
		ret = typec_find_port_power_role(cap_str);
		if (ret < 0)
			return ret;
		ctx->usb_typec->port_type = ret;
	}
	ctx->usb_typec->capability.type = ctx->usb_typec->port_type;

	/* Skip DRP/Source capabilities in case of Sink only */
	if (ctx->usb_typec->port_type == TYPEC_PORT_SNK)
		return 0;

	if (ctx->usb_typec->port_type == TYPEC_PORT_DRP)
		ctx->usb_typec->capability.prefer_role = TYPEC_SINK; /* @TODO: parse "try-power-role" in dts? */

	/*
	 * Supported power operation mode can be configured through device tree
	 * @TODO: need to parse "source-pdos", "sink-pdos"...etc
	 */
	ctx->usb_typec->pwr_opmode = TYPEC_PWR_MODE_PD;

	return 0;
}

static int anx7625_typec_connect(struct anx7625_data *ctx, bool usb_data_role_timeout)
{
	struct device *dev = &ctx->client->dev;
	struct typec_partner_desc desc;
	int ret = 0;

	DRM_DEV_DEBUG_DRIVER(dev, "Type C USB connected\n");

	// If POWER_ROLE == SOURCE
	// enable VBUS?
	ctx->usb_typec->vbus_on = true;

	desc.usb_pd = false;
	desc.accessory = TYPEC_ACCESSORY_NONE; /* @TODO: investigate correct enum to use in case of hub with alt mode */
	desc.identity = NULL;

	/* The partner is the usb port device attached to our port */
	ctx->usb_typec->partner = typec_register_partner(ctx->usb_typec->port, &desc);
	if (IS_ERR(ctx->usb_typec->partner)) {
		ret = PTR_ERR(ctx->usb_typec->partner);
		DRM_DEV_ERROR(dev, "Failed to register partner for typec port: %d\n", ret);
		goto vbus_disable;
	}

	/* Setup the flag to trigger default DEVICE usb data role */
	ctx->usb_typec->usb_data_role_timeout = usb_data_role_timeout;

	return 0;

vbus_disable:
	if (ctx->usb_typec->vbus_on) {
		// disable VBUS?
		ctx->usb_typec->vbus_on = false;
	}

	return ret;
}

static int anx7625_typec_disconnect(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "Type C USB disconnected\n");

	typec_unregister_partner(ctx->usb_typec->partner);
	ctx->usb_typec->partner = NULL;

	ctx->usb_typec->usb_data_role_timeout = false;

	return 0;
}

#ifdef CABLE_DET_PIN_HAS_GLITCH
/* There is a known issue with CABLE_DET signal, to be confirmed by Analogix. Cable detection is
 * done by analog circuitry using the CC pin voltage level. When you have PD data passing on CC pin, this
 * directly produces a noise on CABLE_DET signal. */
static unsigned char confirmed_cable_det(struct anx7625_data *ctx)
{
	unsigned int count = 10;
	unsigned int cable_det_count = 0;
	int val = 0;

	do
	{
		val = gpiod_get_value_cansleep(ctx->pdata.gpio_cbl_det);

		if (val > 0)
			cable_det_count++;
		usleep_range(1000, 1100);
	}
	while (count--);

	if (cable_det_count > 7)
		return 1;
	else
		return 0;
}
#endif

/*
 * There is a sync issue while access I2C register between AP(CPU) and
 * internal firmware(OCM), to avoid the race condition, AP should access
 * the reserved slave address before slave address occurs changes.
 */
static int i2c_access_workaround(struct anx7625_data *ctx,
				 struct i2c_client *client)
{
	u8 offset;
	struct device *dev = &client->dev;
	int ret;

	if (client == ctx->last_client)
		return 0;

	ctx->last_client = client;

	if (client == ctx->i2c.tcpc_client)
		offset = RSVD_00_ADDR;
	else if (client == ctx->i2c.tx_p0_client)
		offset = RSVD_D1_ADDR;
	else if (client == ctx->i2c.tx_p1_client)
		offset = RSVD_60_ADDR;
	else if (client == ctx->i2c.rx_p0_client)
		offset = RSVD_39_ADDR;
	else if (client == ctx->i2c.rx_p1_client)
		offset = RSVD_7F_ADDR;
	else
		offset = RSVD_00_ADDR;

	ret = i2c_smbus_write_byte_data(client, offset, 0x00);
	if (ret < 0)
		DRM_DEV_ERROR(dev,
			      "fail to access i2c id=%x\n:%x",
			      client->addr, offset);

	return ret;
}

static int anx7625_reg_read(struct anx7625_data *ctx,
			    struct i2c_client *client, u8 reg_addr)
{
	int ret;
	struct device *dev = &client->dev;

	i2c_access_workaround(ctx, client);

	ret = i2c_smbus_read_byte_data(client, reg_addr);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "read i2c fail id=%x:%x\n",
			      client->addr, reg_addr);

	return ret;
}

static int anx7625_reg_block_read(struct anx7625_data *ctx,
				  struct i2c_client *client,
				  u8 reg_addr, u8 len, u8 *buf)
{
	int ret;
	struct device *dev = &client->dev;

	i2c_access_workaround(ctx, client);

	ret = i2c_smbus_read_i2c_block_data(client, reg_addr, len, buf);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "read i2c block fail id=%x:%x\n",
			      client->addr, reg_addr);

	return ret;
}

static int anx7625_reg_write(struct anx7625_data *ctx,
			     struct i2c_client *client,
			     u8 reg_addr, u8 reg_val)
{
	int ret;
	struct device *dev = &client->dev;

	i2c_access_workaround(ctx, client);

	ret = i2c_smbus_write_byte_data(client, reg_addr, reg_val);

	if (ret < 0)
		DRM_DEV_ERROR(dev, "fail to write i2c id=%x\n:%x",
			      client->addr, reg_addr);

	return ret;
}

static int anx7625_write_or(struct anx7625_data *ctx,
			    struct i2c_client *client,
			    u8 offset, u8 mask)
{
	int val;

	val = anx7625_reg_read(ctx, client, offset);
	if (val < 0)
		return val;

	return anx7625_reg_write(ctx, client, offset, (val | (mask)));
}

static int anx7625_write_and(struct anx7625_data *ctx,
			     struct i2c_client *client,
			     u8 offset, u8 mask)
{
	int val;

	val = anx7625_reg_read(ctx, client, offset);
	if (val < 0)
		return val;

	return anx7625_reg_write(ctx, client, offset, (val & (mask)));
}

static int anx7625_write_and_or(struct anx7625_data *ctx,
				struct i2c_client *client,
				u8 offset, u8 and_mask, u8 or_mask)
{
	int val;

	val = anx7625_reg_read(ctx, client, offset);
	if (val < 0)
		return val;

	return anx7625_reg_write(ctx, client,
				 offset, (val & and_mask) | (or_mask));
}

static int anx7625_read_ctrl_status_p0(struct anx7625_data *ctx)
{
	return anx7625_reg_read(ctx, ctx->i2c.rx_p0_client, AP_AUX_CTRL_STATUS);
}

static int wait_aux_op_finish(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int val;
	int ret;

	ret = readx_poll_timeout(anx7625_read_ctrl_status_p0,
				 ctx, val,
				 (!(val & AP_AUX_CTRL_OP_EN) || (val < 0)),
				 2000,
				 2000 * 150);
	if (ret) {
		DRM_DEV_ERROR(dev, "aux operation fail!\n");
		return -EIO;
	}

	val = anx7625_read_ctrl_status_p0(ctx);
	if (val < 0 || (val & 0x0F)) {
		DRM_DEV_ERROR(dev, "aux status %02x\n", val);
		val = -EIO;
	}

	return val;
}

static int write_dpcd_addr(struct anx7625_data *ctx,
                           u8 addrh, u8 addrm, u8 addrl)
{
	int ret;

	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                        AP_AUX_ADDR_7_0,
	                        (u8)addrl);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                         AP_AUX_ADDR_15_8,
	                         (u8)addrm);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                         AP_AUX_ADDR_19_16,
	                         (u8)addrh);

	return ret;
}

static int sp_tx_aux_dpcdread_bytes(struct anx7625_data *ctx,
                                    u8 addrh, u8 addrm,
                                    u8 addrl, u8 count,
                                    u8 *buf)
{
	struct device *dev = &ctx->client->dev;
	int ret;
	u8 c;

	if (WARN_ON(count > MAX_DPCD_BUFFER_SIZE))
		return -E2BIG;

	c = ((count - 1) << 4) | 0x09;
	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                        AP_AUX_COMMAND,
	                        c);
	ret |= write_dpcd_addr(ctx, addrh, addrm, addrl);
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
	                        AP_AUX_CTRL_STATUS,
	                        AP_AUX_CTRL_OP_EN);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "aux enable failed.\n");
		return ret;
	}

	usleep_range(2000, 2100);
	ret = wait_aux_op_finish(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "aux read failed\n");
		return ret;
	}

	return anx7625_reg_block_read(ctx, ctx->i2c.rx_p0_client,
	                              AP_AUX_BUFF_START, count, buf);
}

static int anx7625_video_mute_control(struct anx7625_data *ctx,
				      u8 status)
{
	int ret;

	if (status) {
		/* Set mute on flag */
		ret = anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
				       AP_AV_STATUS, AP_MIPI_MUTE);
		/* Clear mipi RX en */
		ret |= anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
					 AP_AV_STATUS, (u8)~AP_MIPI_RX_EN);
	} else {
		/* Mute off flag */
		ret = anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
					AP_AV_STATUS, (u8)~AP_MIPI_MUTE);
		/* Set MIPI RX EN */
		ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
					AP_AV_STATUS, AP_MIPI_RX_EN);
	}

	return ret;
}

static int anx7625_config_audio_input(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int ret;

	/* Channel num */
	ret = anx7625_reg_write(ctx, ctx->i2c.tx_p2_client,
				AUDIO_CHANNEL_STATUS_6, I2S_CH_2 << 5);

	/* FS */
	ret |= anx7625_write_and_or(ctx, ctx->i2c.tx_p2_client,
				    AUDIO_CHANNEL_STATUS_4,
				    0xf0, AUDIO_FS_48K);
	/* Word length */
	ret |= anx7625_write_and_or(ctx, ctx->i2c.tx_p2_client,
				    AUDIO_CHANNEL_STATUS_5,
				    0xf0, AUDIO_W_LEN_24_24MAX);
	/* I2S */
	ret |= anx7625_write_or(ctx, ctx->i2c.tx_p2_client,
				AUDIO_CHANNEL_STATUS_6, I2S_SLAVE_MODE);
	ret |= anx7625_write_and(ctx, ctx->i2c.tx_p2_client,
				 AUDIO_CONTROL_REGISTER, ~TDM_TIMING_MODE);
	/* Audio change flag */
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
				AP_AV_STATUS, AP_AUDIO_CHG);

	if (ret < 0)
		DRM_DEV_ERROR(dev, "fail to config audio.\n");

	return ret;
}

/* Reduction of fraction a/b */
static void anx7625_reduction_of_a_fraction(unsigned long *a, unsigned long *b)
{
	unsigned long gcd_num;
	unsigned long tmp_a, tmp_b;
	u32 i = 1;

	gcd_num = gcd(*a, *b);
	*a /= gcd_num;
	*b /= gcd_num;

	tmp_a = *a;
	tmp_b = *b;

	while ((*a > MAX_UNSIGNED_24BIT) || (*b > MAX_UNSIGNED_24BIT)) {
		i++;
		*a = tmp_a / i;
		*b = tmp_b / i;
	}

	/*
	 * In the end, make a, b larger to have higher ODFC PLL
	 * output frequency accuracy
	 */
	while ((*a < MAX_UNSIGNED_24BIT) && (*b < MAX_UNSIGNED_24BIT)) {
		*a <<= 1;
		*b <<= 1;
	}

	*a >>= 1;
	*b >>= 1;
}

static int anx7625_calculate_m_n(u32 pixelclock,
				 unsigned long *m,
				 unsigned long *n,
				 u8 *post_divider)
{
	if (pixelclock > PLL_OUT_FREQ_ABS_MAX / POST_DIVIDER_MIN) {
		/* Pixel clock frequency is too high */
		DRM_ERROR("pixelclock too high, act(%d), maximum(%lu)\n",
			  pixelclock,
			  PLL_OUT_FREQ_ABS_MAX / POST_DIVIDER_MIN);
		return -EINVAL;
	}

	if (pixelclock < PLL_OUT_FREQ_ABS_MIN / POST_DIVIDER_MAX) {
		/* Pixel clock frequency is too low */
		DRM_ERROR("pixelclock too low, act(%d), maximum(%lu)\n",
			  pixelclock,
			  PLL_OUT_FREQ_ABS_MIN / POST_DIVIDER_MAX);
		return -EINVAL;
	}

	for (*post_divider = 1;
		pixelclock < (PLL_OUT_FREQ_MIN / (*post_divider));)
		*post_divider += 1;

	if (*post_divider > POST_DIVIDER_MAX) {
		for (*post_divider = 1;
			(pixelclock <
			 (PLL_OUT_FREQ_ABS_MIN / (*post_divider)));)
			*post_divider += 1;

		if (*post_divider > POST_DIVIDER_MAX) {
			DRM_ERROR("cannot find property post_divider(%d)\n",
				  *post_divider);
			return -EDOM;
		}
	}

	/* Patch to improve the accuracy */
	if (*post_divider == 7) {
		/* 27,000,000 is not divisible by 7 */
		*post_divider = 8;
	} else if (*post_divider == 11) {
		/* 27,000,000 is not divisible by 11 */
		*post_divider = 12;
	} else if ((*post_divider == 13) || (*post_divider == 14)) {
		/* 27,000,000 is not divisible by 13 or 14 */
		*post_divider = 15;
	}

	if (pixelclock * (*post_divider) > PLL_OUT_FREQ_ABS_MAX) {
		DRM_ERROR("act clock(%u) large than maximum(%lu)\n",
			  pixelclock * (*post_divider),
			  PLL_OUT_FREQ_ABS_MAX);
		return -EDOM;
	}

	*m = pixelclock;
	*n = XTAL_FRQ / (*post_divider);

	anx7625_reduction_of_a_fraction(m, n);

	return 0;
}

static int anx7625_odfc_config(struct anx7625_data *ctx,
			       u8 post_divider)
{
	int ret;
	struct device *dev = &ctx->client->dev;

	/* Config input reference clock frequency 27MHz/19.2MHz */
	ret = anx7625_write_and(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_16,
				~(REF_CLK_27000KHZ << MIPI_FREF_D_IND));
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_16,
				(REF_CLK_27000KHZ << MIPI_FREF_D_IND));
	/* Post divider */
	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p1_client,
				 MIPI_DIGITAL_PLL_8, 0x0f);
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_8,
				post_divider << 4);

	/* Add patch for MIS2-125 (5pcs ANX7625 fail ATE MBIST test) */
	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_7,
				 ~MIPI_PLL_VCO_TUNE_REG_VAL);

	/* Reset ODFC PLL */
	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_7,
				 ~MIPI_PLL_RESET_N);
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_7,
				MIPI_PLL_RESET_N);

	if (ret < 0)
		DRM_DEV_ERROR(dev, "IO error.\n");

	return ret;
}

static int anx7625_dsi_video_timing_config(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	unsigned long m, n;
	u16 htotal;
	int ret;
	u8 post_divider = 0;

	ret = anx7625_calculate_m_n(ctx->dt.pixelclock.min * 1000,
				    &m, &n, &post_divider);

	if (ret) {
		DRM_DEV_ERROR(dev, "cannot get property m n value.\n");
		return ret;
	}

	DRM_DEV_DEBUG_DRIVER(dev, "compute M(%lu), N(%lu), divider(%d).\n",
			     m, n, post_divider);

	/* Configure pixel clock */
	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client, PIXEL_CLOCK_L,
				(ctx->dt.pixelclock.min / 1000) & 0xFF);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client, PIXEL_CLOCK_H,
				 (ctx->dt.pixelclock.min / 1000) >> 8);
	/* Lane count */
	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p1_client,
			MIPI_LANE_CTRL_0, 0xfc);
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client,
				MIPI_LANE_CTRL_0, 3);

	/* Htotal */
	htotal = ctx->dt.hactive.min + ctx->dt.hfront_porch.min +
		ctx->dt.hback_porch.min + ctx->dt.hsync_len.min;
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_TOTAL_PIXELS_L, htotal & 0xFF);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_TOTAL_PIXELS_H, htotal >> 8);
	/* Hactive */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_ACTIVE_PIXELS_L, ctx->dt.hactive.min & 0xFF);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_ACTIVE_PIXELS_H, ctx->dt.hactive.min >> 8);
	/* HFP */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_FRONT_PORCH_L, ctx->dt.hfront_porch.min);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_FRONT_PORCH_H,
			ctx->dt.hfront_porch.min >> 8);
	/* HWS */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_SYNC_WIDTH_L, ctx->dt.hsync_len.min);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_SYNC_WIDTH_H, ctx->dt.hsync_len.min >> 8);
	/* HBP */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_BACK_PORCH_L, ctx->dt.hback_porch.min);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			HORIZONTAL_BACK_PORCH_H, ctx->dt.hback_porch.min >> 8);
	/* Vactive */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client, ACTIVE_LINES_L,
			ctx->dt.vactive.min);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client, ACTIVE_LINES_H,
			ctx->dt.vactive.min >> 8);
	/* VFP */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			VERTICAL_FRONT_PORCH, ctx->dt.vfront_porch.min);
	/* VWS */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			VERTICAL_SYNC_WIDTH, ctx->dt.vsync_len.min);
	/* VBP */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p2_client,
			VERTICAL_BACK_PORCH, ctx->dt.vback_porch.min);
	/* M value */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
			MIPI_PLL_M_NUM_23_16, (m >> 16) & 0xff);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
			MIPI_PLL_M_NUM_15_8, (m >> 8) & 0xff);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
			MIPI_PLL_M_NUM_7_0, (m & 0xff));
	/* N value */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
			MIPI_PLL_N_NUM_23_16, (n >> 16) & 0xff);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
			MIPI_PLL_N_NUM_15_8, (n >> 8) & 0xff);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client, MIPI_PLL_N_NUM_7_0,
			(n & 0xff));
	/* Diff */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
			MIPI_DIGITAL_ADJ_1, 0x37);

	ret |= anx7625_odfc_config(ctx, post_divider - 1);

	if (ret < 0)
		DRM_DEV_ERROR(dev, "mipi dsi setup IO error.\n");

	return ret;
}

static int anx7625_swap_dsi_lane3(struct anx7625_data *ctx)
{
	int val;
	struct device *dev = &ctx->client->dev;

	/* Swap MIPI-DSI data lane 3 P and N */
	val = anx7625_reg_read(ctx, ctx->i2c.rx_p1_client, MIPI_SWAP);
	if (val < 0) {
		DRM_DEV_ERROR(dev, "IO error : access MIPI_SWAP.\n");
		return -EIO;
	}

	val |= (1 << MIPI_SWAP_CH3);
	return anx7625_reg_write(ctx, ctx->i2c.rx_p1_client, MIPI_SWAP, val);
}

static int anx7625_api_dsi_config(struct anx7625_data *ctx)

{
	int val, ret;
	struct device *dev = &ctx->client->dev;

	/* Swap MIPI-DSI data lane 3 P and N */
	ret = anx7625_swap_dsi_lane3(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "IO error : swap dsi lane 3 fail.\n");
		return ret;
	}

	/* DSI clock settings */
	val = (0 << MIPI_HS_PWD_CLK)		|
		(0 << MIPI_HS_RT_CLK)		|
		(0 << MIPI_PD_CLK)		|
		(1 << MIPI_CLK_RT_MANUAL_PD_EN)	|
		(1 << MIPI_CLK_HS_MANUAL_PD_EN)	|
		(0 << MIPI_CLK_DET_DET_BYPASS)	|
		(0 << MIPI_CLK_MISS_CTRL)	|
		(0 << MIPI_PD_LPTX_CH_MANUAL_PD_EN);
	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
				MIPI_PHY_CONTROL_3, val);

	/*
	 * Decreased HS prepare timing delay from 160ns to 80ns work with
	 *     a) Dragon board 810 series (Qualcomm AP)
	 *     b) Moving Pixel DSI source (PG3A pattern generator +
	 *	P332 D-PHY Probe) default D-PHY timing
	 *	5ns/step
	 */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
				 MIPI_TIME_HS_PRPR, 0x10);

	/* Enable DSI mode*/
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_18,
				SELECT_DSI << MIPI_DPI_SELECT);

	ret |= anx7625_dsi_video_timing_config(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "dsi video timing config fail\n");
		return ret;
	}

	/* Toggle m, n ready */
	ret = anx7625_write_and(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_6,
				~(MIPI_M_NUM_READY | MIPI_N_NUM_READY));
	usleep_range(1000, 1100);
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client, MIPI_DIGITAL_PLL_6,
				MIPI_M_NUM_READY | MIPI_N_NUM_READY);

	/* Configure integer stable register */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
				 MIPI_VIDEO_STABLE_CNT, 0x02);
	/* Power on MIPI RX */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
				 MIPI_LANE_CTRL_10, 0x00);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p1_client,
				 MIPI_LANE_CTRL_10, 0x80);

	if (ret < 0)
		DRM_DEV_ERROR(dev, "IO error : mipi dsi enable init fail.\n");

	return ret;
}

static int anx7625_dsi_config(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "config dsi.\n");

	/* DSC disable */
	ret = anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
	                        R_DSC_CTRL_0,
	                        ~(u8)R_DSC_EN);
	ret |= anx7625_api_dsi_config(ctx);

	if (ret < 0) {
		DRM_DEV_ERROR(dev, "IO error : api dsi config error.\n");
		return ret;
	}

	/* Set MIPI RX EN */
	ret = anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
			       AP_AV_STATUS, AP_MIPI_RX_EN);
	/* Clear mute flag */
	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
	                         AP_AV_STATUS,
	                         ~(u8)AP_MIPI_MUTE);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "IO error : enable mipi rx fail.\n");
	else
		DRM_DEV_DEBUG_DRIVER(dev, "success to config DSI\n");

	return ret;
}

const u8 ANX_OUI[3] = { 0x00, 0x22, 0xB9 };
static int is_anx_dongle(struct anx7625_data *ctx)
{
	u8 buf[3];

	/* 0x0500~0x0502: BRANCH_IEEE_OUI */
	sp_tx_aux_dpcdread_bytes(ctx, 0x00, 0x05, 0x00, sizeof(buf), buf);
	if (memcmp(buf, ANX_OUI, sizeof(buf)) == 0)
		return 0;

	return -ENXIO;
}

static void sp_tx_get_rx_bw(struct anx7625_data *ctx, u8 *bw)
{
	u8 data_buf[4];
	/*
	 * When ANX dongle connected, if CHIP_ID = 0x7750, bandwidth is 6.75G,
	 * because ANX7750 DPCD 0x052x was not available.
	 */
	if (!is_anx_dongle(ctx)) {
		sp_tx_aux_dpcdread_bytes(ctx, 0x00, 0x05, 0x03, 0x04, data_buf);
		if (data_buf[0] == 0x37 &&
		    data_buf[1] == 0x37 &&
		    data_buf[2] == 0x35 &&
		    data_buf[3] == 0x30) {
			/* 0x19 : 6.75G */
			*bw = 0x19;
		} else {
			sp_tx_aux_dpcdread_bytes(ctx, 0x00, 0x05, 0x21, 1, bw);
			/*
			 * some ANX dongle read out value 0,
			 * retry standard register.
			 */
			if (((*bw) == 0) || ((*bw) == 0xff))
				sp_tx_aux_dpcdread_bytes(ctx, 0x00, 0x00,
				                         DPCD_MAX_LINK_RATE,
				                         1, bw);
		}
	} else {
		sp_tx_aux_dpcdread_bytes(ctx, 0x00, 0x00,
		                         DPCD_MAX_LINK_RATE, 1, bw);
	}
}

static int anx7625_hdmi_hw_params(struct device *dev, void *data,
                                  struct hdmi_codec_daifmt *fmt,
                                  struct hdmi_codec_params *hparms)
{
	DRM_DEV_DEBUG_DRIVER(dev, "anx7625_hdmi_hw_params: "
	                     " sample rate %d, width %d.\n",
	                     hparms->sample_rate, hparms->sample_width);

	DRM_DEV_DEBUG_DRIVER(dev, "anx7625_hdmi format: "
	                     " audio format 0x%x, invert clock %d.\n",
	                     fmt->fmt, fmt->bit_clk_inv);

	/*
	 * Signed 16 bit Little Endian, Rate 48000 Hz, Stereo [OK]
	 *
	 * TODO: needs handling of other use cases
	 */

	return 0;
}

static int anx7625_audio_startup(struct device *dev, void *data)
{
	struct anx7625_data *ctx = dev_get_drvdata(dev);
	return anx7625_config_audio_input(ctx);
}

static void anx7625_audio_shutdown(struct device *dev, void *data)
{
}

static int anx7625_hdmi_i2s_get_dai_id(struct snd_soc_component *component,
                                       struct device_node *endpoint)
{
	struct of_endpoint of_ep;
	int ret;

	ret = of_graph_parse_endpoint(endpoint, &of_ep);
	if (ret < 0)
		return ret;

	/* HDMI sound should be located as reg = <3> (sound port 0) */
	if (of_ep.port == 3)
		return 0;

	return -EINVAL;
}

static const struct hdmi_codec_ops anx7625_codec_ops = {
	.hw_params	= anx7625_hdmi_hw_params,
	.audio_startup	= anx7625_audio_startup,
	.audio_shutdown = anx7625_audio_shutdown,
	.get_dai_id	= anx7625_hdmi_i2s_get_dai_id,
};

static const struct hdmi_codec_pdata codec_data = {
	.ops = &anx7625_codec_ops,
	.max_i2s_channels = 2,
	.i2s = 1,
};

static int anx7625_audio_init(struct device *dev, struct anx7625_data *anx7625)
{
	anx7625->audio_pdev = platform_device_register_data(dev,
	                      HDMI_CODEC_DRV_NAME,
	                      PLATFORM_DEVID_AUTO,
	                      &codec_data,
	                      sizeof(codec_data));
	return PTR_ERR_OR_ZERO(anx7625->audio_pdev);
}

static void anx7625_audio_exit(struct anx7625_data *anx7625)
{
	if (anx7625->audio_pdev) {
		platform_device_unregister(anx7625->audio_pdev);
		anx7625->audio_pdev = NULL;
	}
}

static void anx7625_dp_start(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;

	if (!ctx->display_timing_valid) {
		DRM_DEV_ERROR(dev, "mipi not set display timing yet.\n");
		return;
	}

	if (anx7625_dsi_config(ctx) < 0)
		DRM_DEV_ERROR(dev, "MIPI phy setup error.\n");
}

static void anx7625_dp_stop(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "stop dp output\n");

	/*
	 * Video disable: 0x72:08 bit 7 = 0;
	 * Audio disable: 0x70:87 bit 0 = 0;
	 */
	ret = anx7625_write_and(ctx, ctx->i2c.tx_p0_client, 0x87, 0xfe);
	ret |= anx7625_write_and(ctx, ctx->i2c.tx_p2_client, VIDEO_CONTROL_0, 0x7f);

	ret |= anx7625_video_mute_control(ctx, 1);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "IO error : mute video fail\n");
}

static int sp_tx_rst_aux(struct anx7625_data *ctx)
{
	int ret;

	ret = anx7625_write_or(ctx, ctx->i2c.tx_p2_client, RST_CTRL2,
			       AUX_RST);
	ret |= anx7625_write_and(ctx, ctx->i2c.tx_p2_client, RST_CTRL2,
				 ~AUX_RST);
	return ret;
}

static int sp_tx_aux_wr(struct anx7625_data *ctx, u8 offset)
{
	int ret;

	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				AP_AUX_BUFF_START, offset);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				 AP_AUX_COMMAND, 0x04);
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
				AP_AUX_CTRL_STATUS, AP_AUX_CTRL_OP_EN);
	return (ret | wait_aux_op_finish(ctx));
}

static int sp_tx_aux_rd(struct anx7625_data *ctx, u8 len_cmd)
{
	int ret;

	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				AP_AUX_COMMAND, len_cmd);
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
				AP_AUX_CTRL_STATUS, AP_AUX_CTRL_OP_EN);
	return (ret | wait_aux_op_finish(ctx));
}

static int sp_tx_get_edid_block(struct anx7625_data *ctx)
{
	int c;
	struct device *dev = &ctx->client->dev;

	sp_tx_aux_wr(ctx, 0x7e);
	sp_tx_aux_rd(ctx, 0x01);
	c = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client, AP_AUX_BUFF_START);
	if (c < 0) {
		DRM_DEV_ERROR(dev, "IO error : access AUX BUFF.\n");
		return -EIO;
	}

	DRM_DEV_DEBUG_DRIVER(dev, " EDID Block = %d\n", c + 1);

	if (c > MAX_EDID_BLOCK)
		c = 1;

	return c;
}

static int edid_read(struct anx7625_data *ctx,
		     u8 offset, u8 *pblock_buf)
{
	int ret, cnt;
	struct device *dev = &ctx->client->dev;

	for (cnt = 0; cnt <= EDID_TRY_CNT; cnt++) {
		sp_tx_aux_wr(ctx, offset);
		/* Set I2C read com 0x01 mot = 0 and read 16 bytes */
		ret = sp_tx_aux_rd(ctx, 0xf1);

		if (ret) {
			sp_tx_rst_aux(ctx);
			DRM_DEV_DEBUG_DRIVER(dev, "edid read fail, reset!\n");
			cnt++;
		} else {
			ret = anx7625_reg_block_read(ctx, ctx->i2c.rx_p0_client,
						     AP_AUX_BUFF_START,
						     MAX_DPCD_BUFFER_SIZE,
						     pblock_buf);
			if (ret > 0)
				break;
		}
	}

	if (cnt > EDID_TRY_CNT)
		return -EIO;

	return 0;
}

static int segments_edid_read(struct anx7625_data *ctx,
			      u8 segment, u8 *buf, u8 offset)
{
	u8 cnt;
	int ret;
	struct device *dev = &ctx->client->dev;

	/* Write address only */
	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				AP_AUX_ADDR_7_0, 0x30);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				 AP_AUX_COMMAND, 0x04);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				 AP_AUX_CTRL_STATUS,
				 AP_AUX_CTRL_ADDRONLY | AP_AUX_CTRL_OP_EN);

	ret |= wait_aux_op_finish(ctx);
	/* Write segment address */
	ret |= sp_tx_aux_wr(ctx, segment);
	/* Data read */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				 AP_AUX_ADDR_7_0, 0x50);
	if (ret) {
		DRM_DEV_ERROR(dev, "IO error : aux initial fail.\n");
		return ret;
	}

	for (cnt = 0; cnt < EDID_TRY_CNT; cnt++) {
		sp_tx_aux_wr(ctx, offset);
		/* Set I2C read com 0x01 mot = 0 and read 16 bytes */
		ret = sp_tx_aux_rd(ctx, 0xf1);

		if (ret) {
			ret = sp_tx_rst_aux(ctx);
			DRM_DEV_ERROR(dev, "segment read fail, reset!\n");
			cnt++;
		} else {
			ret = anx7625_reg_block_read(ctx, ctx->i2c.rx_p0_client,
						     AP_AUX_BUFF_START,
						     MAX_DPCD_BUFFER_SIZE, buf);
			if (!ret)
				break;
		}
	}

	if (cnt == EDID_TRY_CNT)
		return -EIO;

	return 0;
}

static int sp_tx_edid_read(struct anx7625_data *ctx,
			   u8 *pedid_blocks_buf)
{
	u8 offset, edid_pos;
	int count, blocks_num;
	u8 pblock_buf[MAX_DPCD_BUFFER_SIZE];
	u8 i, j;
	u8 g_edid_break = 0;
	int ret;
	struct device *dev = &ctx->client->dev;

	/* Address initial */
	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				AP_AUX_ADDR_7_0, 0x50);
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
				 AP_AUX_ADDR_15_8, 0);
	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
				 AP_AUX_ADDR_19_16, 0xf0);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "access aux channel IO error.\n");
		return -EIO;
	}

	blocks_num = sp_tx_get_edid_block(ctx);
	if (blocks_num < 0)
		return blocks_num;

	count = 0;

	do {
		switch (count) {
		case 0:
		case 1:
			for (i = 0; i < 8; i++) {
				offset = (i + count * 8) * MAX_DPCD_BUFFER_SIZE;
				g_edid_break = edid_read(ctx, offset,
							 pblock_buf);

				if (g_edid_break)
					break;

				memcpy(&pedid_blocks_buf[offset],
				       pblock_buf,
				       MAX_DPCD_BUFFER_SIZE);
			}

			break;
		case 2:
			offset = 0x00;

			for (j = 0; j < 8; j++) {
				edid_pos = (j + count * 8) *
					MAX_DPCD_BUFFER_SIZE;

				if (g_edid_break == 1)
					break;

				segments_edid_read(ctx, count / 2,
						   pblock_buf, offset);
				memcpy(&pedid_blocks_buf[edid_pos],
				       pblock_buf,
				       MAX_DPCD_BUFFER_SIZE);
				offset = offset + 0x10;
			}

			break;
		case 3:
			offset = 0x80;

			for (j = 0; j < 8; j++) {
				edid_pos = (j + count * 8) *
					MAX_DPCD_BUFFER_SIZE;
				if (g_edid_break == 1)
					break;

				segments_edid_read(ctx, count / 2,
						   pblock_buf, offset);
				memcpy(&pedid_blocks_buf[edid_pos],
				       pblock_buf,
				       MAX_DPCD_BUFFER_SIZE);
				offset = offset + 0x10;
			}

			break;
		default:
			break;
		}

		count++;

	} while (blocks_num >= count);

	/* Check edid data */
	if (!drm_edid_is_valid((struct edid *)pedid_blocks_buf)) {
		DRM_DEV_ERROR(dev, "WARNING! edid check fail!\n");
		return -EINVAL;
	}

	/* Reset aux channel */
	sp_tx_rst_aux(ctx);

	return (blocks_num + 1);
}

static void anx7625_power_on(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;

	if (!ctx->pdata.low_power_mode) {
		DRM_DEV_DEBUG_DRIVER(dev, "not low power mode!\n");
		return;
	}

	/* 10ms: as per data sheet */
	gpiod_set_value(ctx->pdata.gpio_p_on, 1);
	usleep_range(10000, 10100);

	/* 10ms: as per data sheet */
	gpiod_set_value(ctx->pdata.gpio_reset, 1);
	usleep_range(10000, 10100);

	DRM_DEV_DEBUG_DRIVER(dev, "entering on state\n");
}

/* As a note: standby means 3.0v, 1.8v, 1.0v rails are on and
 * RESET_N = off
 * POWER_EN = off. So that anx7625 is still powered and able
 * to send interrupts to AP. */
static void anx7625_power_standby(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;

	if (!ctx->pdata.low_power_mode) {
		DRM_DEV_DEBUG_DRIVER(dev, "not low power mode!\n");
		return;
	}

	gpiod_set_value(ctx->pdata.gpio_reset, 0);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->pdata.gpio_p_on, 0);
	usleep_range(1000, 1100);

	DRM_DEV_DEBUG_DRIVER(dev, "standby mode on\n");
}

/* Basic configurations of ANX7625 */
static void anx7625_config(struct anx7625_data *ctx)
{
	anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
			  XTAL_FRQ_SEL, XTAL_FRQ_27M);
}

/* Auto rdo means auto pd negotiation */
static int anx7625_disable_auto_rdo(struct anx7625_data *ctx)
{
	int ret = 0;

	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
	                        AUTO_PD_MODE,
	                        ~AUTO_PD_ENABLE);
	return ret;
}

/* Auto rdo means auto pd negotiation */
static int anx7625_enable_auto_rdo(struct anx7625_data *ctx)
{
	int ret = 0;

	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
	                        AUTO_PD_MODE,
	                        AUTO_PD_ENABLE);
	return ret;
}

/* GOTO_SAFE5V_EN disable */
static int anx7625_disable_safe_5v_during_auto_rdo(struct anx7625_data *ctx)
{
	int ret = 0;

	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
	                        AUTO_PD_MODE,
	                        ~(u8)BIT(4));
	return ret;
}

static int anx7625_chip_register_init(struct anx7625_data *ctx)
{
	int ret = 0;

	/* interrupt vector mask bit as platform needed 0: enable 1: disable */
	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                        INTERFACE_CHANGE_INT_MASK,
	                        INT_MASK_OFF);

	/* AUTO RDO DISABLE */
	anx7625_disable_auto_rdo(ctx);

	/* Maximum Voltage in 100mV units: 5V */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                         MAX_VOLTAGE_SETTING,
	                         0x32);

	/* Maximum Power in 500mW units: 5W */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                         MAX_POWER_SETTING,
	                         0x0A);

	/* Minimum Power in 500mW units: 3.5W = 5V * 700mA */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                         MIN_POWER_SETTING,
	                         0x07);

	/* Try sink or source @TODO: need to read default policy from dts */
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
	                         AUTO_PD_MODE,
	                         TRYSNK_EN);
	/*ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
	                         AUTO_PD_MODE,
	                         TRYSRC_EN);*/

	/* Disable DRP */
	ret |= anx7625_write_and(ctx, ctx->i2c.tcpc_client,
	                         TCPC_ROLE_CONTROL,
	                         ~(u8)BIT(6));

	/* AUTO RDO ENABLE */
	anx7625_enable_auto_rdo(ctx);

	return ret;
}

/* Pd disable means you are not gonna using pd negotiation */
static void anx7625_disable_pd_protocol(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int ret = 0;

	/* When PD is disabled, OCM will stop sending cbl_det interrupts */

	/* reset main ocm */
	ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                        OCM_DEBUG_REG_8, OCM_MAIN_RESET);
	/* disable PD */
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p0_client,
	                         AP_AV_STATUS, AP_DISABLE_PD);
	/* release main ocm */
	ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                         OCM_DEBUG_REG_8, OCM_MAIN_RELEASE);

	if (ret < 0)
		DRM_DEV_DEBUG_DRIVER(dev, "disable PD feature fail.\n");
	else
		DRM_DEV_DEBUG_DRIVER(dev, "disable PD feature succeeded.\n");
}

/* Pd disable means you are not gonna using pd negotiation */
static int anx7625_enable_pd_protocol(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int ret = 0;

	/* When PD is enabled, OCM will begin sending cbl_det interrupts */

	/* reset main ocm */
	/*ret = anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                        OCM_DEBUG_REG_8, OCM_MAIN_RESET);*/
	/* enable PD */
	ret |= anx7625_write_and(ctx, ctx->i2c.rx_p0_client,
	                         AP_AV_STATUS, ~AP_DISABLE_PD);
	/* release main ocm */
	/*ret |= anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                         OCM_DEBUG_REG_8, OCM_MAIN_RELEASE);*/

	if (ret < 0)
		DRM_DEV_DEBUG_DRIVER(dev, "enable PD feature fail.\n");
	else
		DRM_DEV_DEBUG_DRIVER(dev, "enable PD feature succeeded.\n");

	return ret;
}

static int anx7625_ocm_loading_check(struct anx7625_data *ctx)
{
	int ret = -1;
	struct device *dev = &ctx->client->dev;

	/* Check interface workable */
	ret = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client,
			       FLASH_LOAD_STA);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "IO error : access flash load.\n");
		return ret;
	}

	if ((ret & FLASH_LOAD_STA_CHK) == FLASH_LOAD_STA_CHK)
		ret = 0;
	else
		ret = -ENODEV;

	return ret;
}

static void anx7625_power_on_init(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int retry_count, i, ret = 0;

	for (retry_count = 0; retry_count < 5; retry_count++) {
		anx7625_power_on(ctx);
		anx7625_config(ctx);

		for (i = 0; i < OCM_LOADING_TIME; i++) {
			/*Interface work? */
			if (anx7625_ocm_loading_check(ctx) == 0) {
#ifdef DISABLE_PD
				anx7625_disable_pd_protocol(ctx);
#endif
				ret = anx7625_chip_register_init(ctx);
				if (ret < 0)
					DRM_DEV_DEBUG_DRIVER(dev, "init registers failed.\n");
				else
					DRM_DEV_DEBUG_DRIVER(dev, "init registers succeeded.\n");

				DRM_DEV_DEBUG_DRIVER(dev, "Firmware ver %02x%02x,",
									 anx7625_reg_read(ctx,
													  ctx->i2c.rx_p0_client,
													  OCM_FW_VERSION),

									 anx7625_reg_read(ctx,
													  ctx->i2c.rx_p0_client,
													  OCM_FW_REVERSION));

				DRM_DEV_DEBUG_DRIVER(dev, "Chipid %02x%02x,",
									 anx7625_reg_read(ctx,
													  ctx->i2c.tcpc_client,
													  0x03),
									 anx7625_reg_read(ctx,
													  ctx->i2c.tcpc_client,
													  0x02));

				DRM_DEV_DEBUG_DRIVER(dev, "Driver version %s\n",
									 ANX7625_DRV_VERSION);
				return;
			}
			usleep_range(1000, 1100);
		}
		anx7625_power_standby(ctx);
	}
	atomic_set(&ctx->power_status, 0);
}

static void anx7625_chip_control(struct anx7625_data *ctx, int state)
{
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "before set, power_state(%d).\n",
			     atomic_read(&ctx->power_status));

	if (!ctx->pdata.low_power_mode)
		return;

	if (state) {
		if (atomic_inc_return(&ctx->power_status) == 1)
			anx7625_power_on_init(ctx);
	} else {
		if (atomic_read(&ctx->power_status)) {
			if (atomic_dec_and_test(&ctx->power_status))
				anx7625_power_standby(ctx);
		}
	}

	DRM_DEV_DEBUG_DRIVER(dev, "after set, power_state(%d).\n",
			     atomic_read(&ctx->power_status));
}

static void anx7625_init_gpio(struct anx7625_data *platform)
{
	struct device *dev = &platform->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "init gpio\n");

#ifndef U_BOOT_EARLY_INIT
#ifdef GPIO_VBUS_CONTROL
	platform->pdata.gpio_vbus_on = devm_gpiod_get_optional(dev, "usbc_pwr", GPIOD_OUT_HIGH); /* VBUS Off by default */
#endif
#endif
#ifdef U_BOOT_EARLY_INIT
	/* Gpio for chip power enable */
	platform->pdata.gpio_p_on =
		devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	/* Gpio for chip reset */
	platform->pdata.gpio_reset =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
#else
	/* Gpio for chip power enable */
	platform->pdata.gpio_p_on =
		devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	/* Gpio for chip reset */
	platform->pdata.gpio_reset =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
#endif

	platform->pdata.gpio_cbl_det = devm_gpiod_get_optional(dev, "cbl_det",
	                               GPIOD_IN);

	platform->pdata.gpio_intr_comm = devm_gpiod_get_optional(dev,
	                                 "intr_comm",
	                                 GPIOD_IN);

	if (platform->pdata.gpio_p_on && platform->pdata.gpio_reset) {
		platform->pdata.low_power_mode = 1;
#ifndef U_BOOT_EARLY_INIT
#ifdef GPIO_VBUS_CONTROL
		DRM_DEV_DEBUG_DRIVER(dev, "LOW POWER MODE enabled, "
		                     "VBUS_USBC(%d) = %d, "
		                     "POWER_EN(%d) = %d, "
		                     "RESET_N(%d) =  %d\n",
		                     desc_to_gpio(platform->pdata.gpio_vbus_on),
		                     gpiod_get_value_cansleep(platform->pdata.gpio_vbus_on),
		                     desc_to_gpio(platform->pdata.gpio_p_on),
		                     gpiod_get_value_cansleep(platform->pdata.gpio_p_on),
		                     desc_to_gpio(platform->pdata.gpio_reset),
		                     gpiod_get_value_cansleep(platform->pdata.gpio_reset));
#endif
#else
		DRM_DEV_DEBUG_DRIVER(dev, "LOW POWER MODE enabled, "
		                     "POWER_EN(%d) = %d, "
		                     "RESET_N(%d) =  %d\n",
		                     desc_to_gpio(platform->pdata.gpio_p_on),
		                     gpiod_get_value_cansleep(platform->pdata.gpio_p_on),
		                     desc_to_gpio(platform->pdata.gpio_reset),
		                     gpiod_get_value_cansleep(platform->pdata.gpio_reset));
#endif
	} else {
		/* @TODO: this could happens if no POWER_EN and RESET_N pins are provided, this means
		 * anx7625 stays always on. This is untested, probably needs some work. */
		platform->pdata.low_power_mode = 0;
		DRM_DEV_DEBUG_DRIVER(dev, "not low power mode.\n");
		atomic_set(&platform->power_status, 1);
	}
}

static void anx7625_stop_dp_work(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "anx: stop DP work\n");

	ctx->slimport_edid_p.edid_block_num = -1;
	ctx->hpd_status = 0;
	ctx->hpd_high_cnt = 0;
	ctx->display_timing_valid = 0;

	if (ctx->pdata.low_power_mode == 0)
		anx7625_disable_pd_protocol(ctx); /* @TODO: this is not done in Android driver in this point */
}

static void anx7625_start_dp_work(struct anx7625_data *ctx)
{
	u8 sp_tx_lane_count; /* link training lane count */
	u8 sp_tx_bw; /* linktraining banwidth */
	u8 buf[MAX_DPCD_BUFFER_SIZE];
	u8 hdcp_cap;
	int ret;
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "anx: start DP work\n");

	if (ctx->hpd_high_cnt >= 2) {
		DRM_DEV_DEBUG_DRIVER(dev, "filter useless HPD\n");
		return;
	}

	ctx->hpd_high_cnt++;
	sp_tx_get_rx_bw(ctx, &sp_tx_bw);
	sp_tx_aux_dpcdread_bytes(ctx, 0x00, 0x00, DPCD_MAX_LANE_COUNT, 1,
	                         &sp_tx_lane_count);

	sp_tx_lane_count = sp_tx_lane_count & 0x1f;
	sp_tx_aux_dpcdread_bytes(ctx, 0x06, 0x80, 0x28, 1, buf);/* read Bcap */
	hdcp_cap = buf[0] & 0x01;

	/* Not support HDCP */
	ret = anx7625_write_and(ctx, ctx->i2c.rx_p1_client, 0xee, 0x9f);
	if (hdcp_cap == 0x01)
		DRM_DEV_DEBUG_DRIVER(dev, "HDCP1.4\n");

	/* Try auth flag */
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client, 0xec, 0x10);
	/* Interrupt for DRM */
	ret |= anx7625_write_or(ctx, ctx->i2c.rx_p1_client, 0xff, 0x01);
	if (ret < 0)
		return;

	DRM_DEV_DEBUG_DRIVER(dev, "MAX BW=%02x, MAX Lane cnt=%02x, HDCP=%02x\n",
	                     (u32)sp_tx_bw,
	                     (u32)sp_tx_lane_count,
	                     (u32)hdcp_cap);

	ret = anx7625_reg_read(ctx, ctx->i2c.rx_p1_client, 0x86);
	if (ret < 0)
		return;

	DRM_DEV_DEBUG_DRIVER(dev, "Secure OCM version=%02x\n", ret);

	return;
}

static void dp_hpd_change_handler(struct anx7625_data *ctx, bool on)
{
	struct device *dev = &ctx->client->dev;

	/* HPD changed */
	DRM_DEV_DEBUG_DRIVER(dev, "dp_hpd_change_default_func: %d\n",
			     (u32)on);

	if (on == 0) {
		DRM_DEV_DEBUG_DRIVER(dev, " HPD low\n");
		anx7625_stop_dp_work(ctx);
	} else {
		ctx->hpd_status = 1;
		DRM_DEV_DEBUG_DRIVER(dev, " HPD high\n");
		anx7625_start_dp_work(ctx);
	}
}

static int anx7625_parse_dt(struct device *dev,
			    struct anx7625_platform_data *pdata)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	struct device_node *panel_node, *out_ep;

	pdata->connector_fwnode = device_get_named_child_node(dev, "connector");
	if (IS_ERR_OR_NULL(pdata->connector_fwnode)) {
		DRM_ERROR("Failed to get internal usb typec connector\n");
		return -ENODEV;
	}

	of_property_read_u32(dev->of_node, "panel_flags",
	                     &pdata->panel_flags);

	/* In this driver internal_panel is used to switch between panel
	 * always connected and a usbc dp that is modeled as a panel which
	 * can be present or not depending on the usbc cable (hpd_status) being
	 * connected or not. @TODO: need to handle hpd_status depending on DiscIdentity
	 * results. Hub may don't have dp port available. */
	if (pdata->panel_flags == 1)
		pdata->internal_panel = 1;

	pdata->mipi_host_node = of_graph_get_remote_node(np, 0, 0);
	of_node_put(pdata->mipi_host_node);
	DRM_DEV_DEBUG_DRIVER(dev, "found dsi host node.\n");

	pdata->panel_node = of_graph_get_port_by_id(np, 2);
	if (pdata->panel_node) {
		of_node_put(pdata->panel_node);
		out_ep = of_get_child_by_name(pdata->panel_node,
		                              "endpoint");
		if (!out_ep) {
			DRM_DEV_DEBUG_DRIVER(dev, "cannot get endpoint.\n");
			return -EPROBE_DEFER;
		}

		panel_node = of_graph_get_remote_port_parent(out_ep);
		of_node_put(out_ep);

		pdata->panel = of_drm_find_panel(panel_node);
		DRM_DEV_DEBUG_DRIVER(dev, "get panel node.\n");
		of_node_put(panel_node);
		if (IS_ERR_OR_NULL(pdata->panel))
			return -EPROBE_DEFER;
	} else {
		if (pdata->internal_panel) {
			DRM_ERROR("failed to get internal panel.\n");
			return -EPROBE_DEFER;
		}
	}

	DRM_DEV_DEBUG_DRIVER(dev, "%s internal panel\n",
	                     pdata->internal_panel ? "has" : "no");

	return 0;
#else
	return -ENODEV;
#endif
}

static inline struct anx7625_data *connector_to_anx7625(struct drm_connector *c)
{
	return container_of(c, struct anx7625_data, connector);
}

static inline struct anx7625_data *bridge_to_anx7625(struct drm_bridge *bridge)
{
	return container_of(bridge, struct anx7625_data, bridge);
}

static void anx7625_post_disable(struct drm_bridge *bridge)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device *dev = &ctx->client->dev;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "post disable\n");

	if (!ctx->pdata.panel)
		return;

	ret = drm_panel_unprepare(ctx->pdata.panel);
	if (ret)
		DRM_ERROR("failed to unprepare panel: %d\n", ret);
	else
		DRM_DEV_DEBUG_DRIVER(dev, "backlight unprepared.\n");

	atomic_set(&ctx->panel_power, 0);
}

static void anx7625_pre_enable(struct drm_bridge *bridge)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device *dev = &ctx->client->dev;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "pre enable\n");

	if (!ctx->pdata.panel)
		return;

	ret = drm_panel_prepare(ctx->pdata.panel);
	if (ret < 0)
		DRM_ERROR("failed to prepare panel: %d\n", ret);
	else
		DRM_DEV_DEBUG_DRIVER(dev, "backlight prepared.\n");

	atomic_set(&ctx->panel_power, 1);
}

static int anx7625_get_modes(struct drm_connector *connector)
{
	struct anx7625_data *ctx = connector_to_anx7625(connector);
	struct s_edid_data *p_edid = &ctx->slimport_edid_p;
	u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	struct device *dev = &ctx->client->dev;
	int err, num_modes = 0;
	int turn_off_flag = 0;

	DRM_DEV_DEBUG_DRIVER(dev, "drm get modes\n");

	if (ctx->slimport_edid_p.edid_block_num > 0)
		goto out;

	if (ctx->pdata.panel && atomic_read(&ctx->panel_power) == 0) {
		turn_off_flag = 1;
		anx7625_pre_enable(&ctx->bridge);
	}

	anx7625_chip_control(ctx, 1);
	p_edid->edid_block_num = sp_tx_edid_read(ctx, p_edid->edid_raw_data);
	anx7625_chip_control(ctx, 0);

	err = -EIO;
	if (p_edid->edid_block_num < 0) {
		DRM_ERROR("Failed to read EDID.\n");
		goto fail;
	}

	err = drm_connector_update_edid_property(connector,
	        (struct edid *)
	        &p_edid->edid_raw_data);
	if (err)
		DRM_ERROR("Failed to update EDID property: %d\n", err);
fail:
	if (ctx->pdata.panel && turn_off_flag == 1)
		anx7625_post_disable(&ctx->bridge);
	if (err)
		return err;
out:
	num_modes = drm_add_edid_modes(connector,
	                               (struct edid *)&p_edid->edid_raw_data);

	DRM_DEV_DEBUG_DRIVER(dev, "num_modes(%d)\n", num_modes);

	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_LOW |
	                                    DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;

	err = drm_display_info_set_bus_formats(&connector->display_info,
	                                       &bus_format, 1);
	if (err) {
		DRM_ERROR("Fail on drm_display_info_set_bus_formats\n");
		return err;
	}

	return num_modes;
}

static enum drm_mode_status
anx7625_connector_mode_valid(struct drm_connector *connector,
                             struct drm_display_mode *mode) {

	struct anx7625_data *ctx = connector_to_anx7625(connector);
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "%s: drm mode checking\n", __func__);

	/* Max 1200p at 5.4 Ghz, one lane, pixel clock 300M */
	if (mode->clock > SUPPORT_PIXEL_CLOCK) {
		DRM_DEV_DEBUG_DRIVER(dev,
				     "drm mode invalid, pixelclock too high.\n");
		return MODE_CLOCK_HIGH;
	}

	DRM_DEV_DEBUG_DRIVER(dev, "%s: drm mode valid.\n", __func__);

	return MODE_OK;
}

static struct drm_connector_helper_funcs anx7625_connector_helper_funcs = {
	.mode_valid = anx7625_connector_mode_valid,
	.get_modes = anx7625_get_modes,
};

static enum drm_connector_status anx7625_detect(struct drm_connector *connector,
        bool force)
{
	struct anx7625_data *ctx = connector_to_anx7625(connector);
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "drm detect\n");

	if (ctx->pdata.internal_panel) // a panel is always connected
		return connector_status_connected;

	if (!ctx->hpd_status) // a panel on dp usbc depends on the usbc cable being present or not
		return connector_status_disconnected;

	return connector_status_connected;
}

static const struct drm_connector_funcs anx7625_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = anx7625_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int anx7625_attach_dsi(struct anx7625_data *ctx)
{
	struct mipi_dsi_device *dsi;
	struct device *dev = &ctx->client->dev;
	struct mipi_dsi_host *host;
	const struct mipi_dsi_device_info info = {
		.type = "anx7625",
		.channel = 0,
		.node = NULL,
	};

	DRM_DEV_DEBUG_DRIVER(dev, "attach dsi\n");

	host = of_find_mipi_dsi_host_by_node(ctx->pdata.mipi_host_node);
	if (!host) {
		DRM_DEV_ERROR(dev, "fail to find dsi host.\n");
		return -EINVAL;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_DEV_ERROR(dev, "fail to create dsi device.\n");
		return -EINVAL;
	}

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO	|
		MIPI_DSI_MODE_VIDEO_SYNC_PULSE	|
		MIPI_DSI_MODE_EOT_PACKET	|
		MIPI_DSI_MODE_VIDEO_HSE;

	if (mipi_dsi_attach(dsi) < 0) {
		DRM_DEV_ERROR(dev, "fail to attach dsi to host.\n");
		mipi_dsi_device_unregister(dsi);
		return -EINVAL;
	}

	ctx->dsi = dsi;

	DRM_DEV_DEBUG_DRIVER(dev, "attach dsi succeeded.\n");

	return 0;
}

static void anx7625_bridge_detach(struct drm_bridge *bridge)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);

	if (ctx->dsi) {
		mipi_dsi_detach(ctx->dsi);
		mipi_dsi_device_unregister(ctx->dsi);
	}

	if (ctx->bridge_attached)
		drm_connector_unregister(&ctx->connector);
}

static int anx7625_bridge_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	int err;
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "drm attach\n");

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR) {
		DRM_DEV_ERROR(dev, "Fix bridge driver to make connector optional!");
		return -EINVAL;
	}

	if (!bridge->encoder) {
		DRM_DEV_ERROR(dev, "Parent encoder object not found");
		return -ENODEV;
	}

	err = drm_connector_init(bridge->dev, &ctx->connector,
	                         &anx7625_connector_funcs,
	                         DRM_MODE_CONNECTOR_DisplayPort);
	if (err) {
		DRM_ERROR("Failed to initialize connector: %d\n", err);
		return err;
	}

	drm_connector_helper_add(&ctx->connector,
	                         &anx7625_connector_helper_funcs);

	err = drm_connector_register(&ctx->connector);
	if (err) {
		DRM_ERROR("Failed to register connector: %d\n", err);
		return err;
	}

	ctx->connector.polled = DRM_CONNECTOR_POLL_HPD;

	err = drm_connector_attach_encoder(&ctx->connector, bridge->encoder);
	if (err) {
		DRM_ERROR("Failed to link up connector to encoder: %d\n", err);
		drm_connector_unregister(&ctx->connector);
		return err;
	}

	err = anx7625_attach_dsi(ctx);
	if (err) {
		DRM_DEV_ERROR(dev, "Fail to attach to dsi : %d\n", err);
		drm_connector_unregister(&ctx->connector);
		return err;
	}

	ctx->bridge_attached = 1;

	return 0;
}

static enum drm_mode_status
anx7625_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_info *info,
			  const struct drm_display_mode *mode)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "%s: drm mode checking\n", __func__);

	/* Max 1200p at 5.4 Ghz, one lane, pixel clock 300M */
	if (mode->clock > SUPPORT_PIXEL_CLOCK) {
		DRM_DEV_DEBUG_DRIVER(dev,
				     "drm mode invalid, pixelclock too high.\n");
		return MODE_CLOCK_HIGH;
	}

	DRM_DEV_DEBUG_DRIVER(dev, "%s: drm mode valid.\n", __func__);

	return MODE_OK;
}

static void anx7625_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *old_mode,
				    const struct drm_display_mode *mode)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device *dev = &ctx->client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "drm mode set\n");

	ctx->dt.pixelclock.min = mode->clock;
	ctx->dt.hactive.min = mode->hdisplay;
	ctx->dt.hsync_len.min = mode->hsync_end - mode->hsync_start;
	ctx->dt.hfront_porch.min = mode->hsync_start - mode->hdisplay;
	ctx->dt.hback_porch.min = mode->htotal - mode->hsync_end;
	ctx->dt.vactive.min = mode->vdisplay;
	ctx->dt.vsync_len.min = mode->vsync_end - mode->vsync_start;
	ctx->dt.vfront_porch.min = mode->vsync_start - mode->vdisplay;
	ctx->dt.vback_porch.min = mode->vtotal - mode->vsync_end;

	ctx->display_timing_valid = 1;

	DRM_DEV_DEBUG_DRIVER(dev, "pixelclock(%d).\n", ctx->dt.pixelclock.min);
	DRM_DEV_DEBUG_DRIVER(dev, "hactive(%d), hsync(%d), hfp(%d), hbp(%d)\n",
			     ctx->dt.hactive.min,
			     ctx->dt.hsync_len.min,
			     ctx->dt.hfront_porch.min,
			     ctx->dt.hback_porch.min);
	DRM_DEV_DEBUG_DRIVER(dev, "vactive(%d), vsync(%d), vfp(%d), vbp(%d)\n",
			     ctx->dt.vactive.min,
			     ctx->dt.vsync_len.min,
			     ctx->dt.vfront_porch.min,
			     ctx->dt.vback_porch.min);
	DRM_DEV_DEBUG_DRIVER(dev, "hdisplay(%d),hsync_start(%d).\n",
			     mode->hdisplay,
			     mode->hsync_start);
	DRM_DEV_DEBUG_DRIVER(dev, "hsync_end(%d),htotal(%d).\n",
			     mode->hsync_end,
			     mode->htotal);
	DRM_DEV_DEBUG_DRIVER(dev, "vdisplay(%d),vsync_start(%d).\n",
			     mode->vdisplay,
			     mode->vsync_start);
	DRM_DEV_DEBUG_DRIVER(dev, "vsync_end(%d),vtotal(%d).\n",
			     mode->vsync_end,
			     mode->vtotal);
}

static void anx7625_bridge_enable(struct drm_bridge *bridge)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device *dev = &ctx->client->dev;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "drm enable\n");

	anx7625_chip_control(ctx, 1);

	if (WARN_ON(!atomic_read(&ctx->power_status)))
		return;

	if (!ctx->pdata.panel)
		goto out;

	ret = drm_panel_enable(ctx->pdata.panel);
	if (ret < 0) {
		DRM_ERROR("failed to enable panel: %d.\n", ret);
		return;
	}
out:
	anx7625_dp_start(ctx);
}

static void anx7625_bridge_disable(struct drm_bridge *bridge)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device *dev = &ctx->client->dev;
	int ret;

	if (WARN_ON(!atomic_read(&ctx->power_status)))
		return;

	DRM_DEV_DEBUG_DRIVER(dev, "drm disable\n");

	if (!ctx->pdata.panel)
		goto out;

	ret = drm_panel_disable(ctx->pdata.panel);
	if (ret < 0)
		DRM_ERROR("failed to disable panel: %d.\n", ret);
out:
	anx7625_dp_stop(ctx);

	anx7625_chip_control(ctx, 0);
}

static const struct drm_bridge_funcs anx7625_bridge_funcs = {
	.attach = anx7625_bridge_attach,
	.detach = anx7625_bridge_detach,
	.disable = anx7625_bridge_disable,
	.post_disable = anx7625_post_disable,
	.pre_enable = anx7625_pre_enable,
	.mode_valid = anx7625_bridge_mode_valid,
	.mode_set = anx7625_bridge_mode_set,
	.enable = anx7625_bridge_enable,
};

static int anx7625_register_i2c_dummy_clients(struct anx7625_data *ctx,
					      struct i2c_client *client)
{
	ctx->i2c.tx_p0_client = i2c_new_dummy_device(client->adapter,
						     TX_P0_ADDR >> 1);
	if (!ctx->i2c.tx_p0_client)
		return -ENOMEM;

	ctx->i2c.tx_p1_client = i2c_new_dummy_device(client->adapter,
						     TX_P1_ADDR >> 1);
	if (!ctx->i2c.tx_p1_client)
		goto free_tx_p0;

	ctx->i2c.tx_p2_client = i2c_new_dummy_device(client->adapter,
						     TX_P2_ADDR >> 1);
	if (!ctx->i2c.tx_p2_client)
		goto free_tx_p1;

	ctx->i2c.rx_p0_client = i2c_new_dummy_device(client->adapter,
						     RX_P0_ADDR >> 1);
	if (!ctx->i2c.rx_p0_client)
		goto free_tx_p2;

	ctx->i2c.rx_p1_client = i2c_new_dummy_device(client->adapter,
						     RX_P1_ADDR >> 1);
	if (!ctx->i2c.rx_p1_client)
		goto free_rx_p0;

	ctx->i2c.rx_p2_client = i2c_new_dummy_device(client->adapter,
						     RX_P2_ADDR >> 1);
	if (!ctx->i2c.rx_p2_client)
		goto free_rx_p1;

	ctx->i2c.tcpc_client = i2c_new_dummy_device(client->adapter,
						    TCPC_INTERFACE_ADDR >> 1);
	if (!ctx->i2c.tcpc_client)
		goto free_rx_p2;

	return 0;

free_rx_p2:
	i2c_unregister_device(ctx->i2c.rx_p2_client);
free_rx_p1:
	i2c_unregister_device(ctx->i2c.rx_p1_client);
free_rx_p0:
	i2c_unregister_device(ctx->i2c.rx_p0_client);
free_tx_p2:
	i2c_unregister_device(ctx->i2c.tx_p2_client);
free_tx_p1:
	i2c_unregister_device(ctx->i2c.tx_p1_client);
free_tx_p0:
	i2c_unregister_device(ctx->i2c.tx_p0_client);

	return -ENOMEM;
}

static void anx7625_unregister_i2c_dummy_clients(struct anx7625_data *ctx)
{
	i2c_unregister_device(ctx->i2c.tx_p0_client);
	i2c_unregister_device(ctx->i2c.tx_p1_client);
	i2c_unregister_device(ctx->i2c.tx_p2_client);
	i2c_unregister_device(ctx->i2c.rx_p0_client);
	i2c_unregister_device(ctx->i2c.rx_p1_client);
	i2c_unregister_device(ctx->i2c.rx_p2_client);
	i2c_unregister_device(ctx->i2c.tcpc_client);
}

static void print_cc_status(struct anx7625_data *ctx, int cc_status)
{
	struct device *dev = &ctx->client->dev;

	switch (cc_status & 0x0F) {
	case 0:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC1: SRC.Open\n");
		break;
	case 1:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC1: SRC.Rd\n");
		break;
	case 2:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC1: SRC.Ra\n");
		break;
	case 4:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC1: SNK.default\n");
		break;
	case 8:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC1: SNK.power.1.5\n");
		break;
	case 12:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC1: SNK.power.3.0\n");
		break;
	default:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC1: Reserved\n");
	}

	switch ((cc_status >> 4) & 0x0F) {
	case 0:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC2: SRC.Open\n");
		break;
	case 1:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC2: SRC.Rd\n");
		break;
	case 2:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC2: SRC.Ra\n");
		break;
	case 4:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC2: SNK.default\n");
		break;
	case 8:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC2: SNK.power.1.5\n");
		break;
	case 12:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC2: SNK.power.3.0\n");
		break;
	default:
		DRM_DEV_DEBUG_DRIVER(dev, "anx: CC2: Reserved\n");
	}
}

static void print_sys_status(struct anx7625_data *ctx, int sys_status)
{
	struct device *dev = &ctx->client->dev;

	if (sys_status & BIT(0))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - Reserved\n");

	if (sys_status & BIT(1))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - Reserved \n");

	if (sys_status & BIT(2))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - VCONN status ON\n");
	if (!(sys_status & BIT(2)))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - VCONN status OFF\n");

	if (sys_status & BIT(3))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - VBUS power provider\n");
	if (!(sys_status & BIT(3)))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - VBUS power consumer\n");

	if (sys_status & BIT(5))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - Data role: DFP\n");
	if (!(sys_status & BIT(5)))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - Data role: UFP\n");

	if (sys_status & BIT(6))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - Reserved\n");

	if (sys_status & BIT(7))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - DP HPD high\n");
	if (!(sys_status & BIT(7)))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - DP HPD low\n");
}

static void print_ivector(struct anx7625_data *ctx, int ivector)
{
	struct device *dev = &ctx->client->dev;

	if (ivector & BIT(0))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - MSG INT\n");
	if (ivector & BIT(1))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - Reserved\n");
	if (ivector & BIT(2))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - VCONN change\n");
	if (ivector & BIT(3))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - VBUS change\n");
	if (ivector & BIT(4))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - CC status change\n");
	if (ivector & BIT(5))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - Data role change\n");
	if (ivector & BIT(6))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - As  power consumer, update the max of RDOs V and W after PD negotiation\n");
	if (ivector & BIT(7))
		DRM_DEV_DEBUG_DRIVER(dev, "anx: - DP HPD change\n");
}

/* Timer callback that periodically spawns the workqueue for CABLE_DET sampling and handling.
 * No strict timing requirement on CABLE_DET sampling is needed. */
static void timer_cable_det_poll(struct timer_list *unused)
{
	struct anx7625_data *ctx = (struct anx7625_data *)ctx_timer_cable_det;

	if (ctx->pdata.intp_irq)
		queue_work(ctx->workqueue, &ctx->work);

	mod_timer(&timer_cable_det, jiffies + TIMER_CABLE_DET_POLL_DELAY);
	return;
}

static int anx7625_handle_cable_det(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;

	if(atomic_read(&ctx->power_status)==0)
	{
		if(atomic_read(&ctx->cable_connected))
		{
			DRM_DEV_DEBUG_DRIVER(dev, "anx: %s power on\n", __func__);
			anx7625_typec_connect(ctx, true);
			usleep_range(10000, 10100); /* 10ms: as per data sheet */
			anx7625_chip_control(ctx, 1);
			return 0;
		}
	}
	else {
		if(atomic_read(&ctx->cable_connected)==0)
		{
			if (ctx->hpd_status) {
				anx7625_stop_dp_work(ctx);
			}
			DRM_DEV_DEBUG_DRIVER(dev, "anx: %s power standby\n", __func__);
			anx7625_power_standby(ctx);
			atomic_set(&ctx->power_status, 0);
			anx7625_typec_disconnect(ctx);
			sys_sta_bak = 0;
			return 0;
		}
	}

	return 0;
}

/* This function should notify to the system the default value for usb data role
 * n seconds after a usb type c cable has been plugged and if no data role change has been processed in the meantime.
 * This is done since anx doesn't set data role change bit in ivector (0x7e, 0x44) when dealing with a passive usb typec cable or with some hubs. On the other
 * hand defaulting to DEVICE in probe and then switching later on to HOST causes enumeration errors to
 * the system usb port. @TODO: investigate better solution.
 * NOTE: this function uses timer_cable_det_poll as a time basis. From tests data role is defined between 400-900ms since cable connection. I'm using
 * 2 seconds to have some room. */
static void anx7625_detect_usb_data_role_timeout(struct anx7625_data *ctx)
{
	static uint8_t counter = 0;
	struct device *dev = &ctx->client->dev;
	int sys_status = sys_sta_bak;

	if(ctx->usb_typec->usb_data_role_timeout==true) {
		counter++;
		if(counter == TIMEOUT_USB_DATA_ROLE) {
			counter = 0;
			ctx->usb_typec->usb_data_role_timeout = false;
			DRM_DEV_DEBUG_DRIVER(dev, "Timeout occurred for usb data role change, setting %s\n", (sys_status & BIT(5)) ? "DFP" : "UFP");
			if (sys_status & BIT(5))
				anx7625_set_data_role(ctx, TYPEC_HOST); /* DFP */
			else if (!(sys_status & BIT(5)))
				anx7625_set_data_role(ctx, TYPEC_DEVICE); /* UFP */
		}
	}
}

/* Worqueue for CABLE_DET handling */
static void anx7625_work_func(struct work_struct *work)
{
	struct anx7625_data *ctx = container_of(work,
						struct anx7625_data, work);
	struct device *dev = &ctx->client->dev;
	uint8_t cable_connected = 0;

	anx7625_detect_usb_data_role_timeout(ctx);

#ifdef U_BOOT_EARLY_INIT
	/* This is done once and is delayed work from anx7625_apply_pending_early_init_config */
	if(pending_video_work)
	{
		usleep_range(500000, 501000); /* 500ms */
		dp_hpd_change_handler(ctx, pending_video_work);
		pending_video_work = 0;

		anx7625_reg_write(ctx, ctx->i2c.tcpc_client,
	                  TCPC_INTR_ALERT_1,
	                  0xFF);
	}
#endif

#ifdef CABLE_DET_PIN_HAS_GLITCH
	cable_connected = confirmed_cable_det(ctx);
#else
	cable_connected = gpiod_get_value_cansleep(ctx->pdata.gpio_cbl_det);
#endif

	if((cable_connected==1 && (atomic_read(&ctx->power_status)>0)) || (cable_connected==0 && atomic_read(&ctx->power_status)==0))
	{
		return;
	}
	else
	{
		atomic_set(&ctx->cable_connected, cable_connected);
		DRM_DEV_DEBUG_DRIVER(dev, "anx: %s (%s)\n", __func__, atomic_read(&ctx->cable_connected) ? "PLUGGED" : "UNPLUGGED");
		DRM_DEV_DEBUG_DRIVER(dev, "anx: %s %d acquiring lock\n", __func__, __LINE__);
		mutex_lock(&ctx->lock);
		DRM_DEV_DEBUG_DRIVER(dev, "anx: %s %d lock acquired\n", __func__, __LINE__);
		anx7625_handle_cable_det(ctx); // @TODO: handle return value from this function?
		mutex_unlock(&ctx->lock);
		DRM_DEV_DEBUG_DRIVER(dev, "anx: %s %d lock released\n", __func__, __LINE__);
	}

	return;
}

/* 0 = On, 1 = Off */
static void vbus_control(struct anx7625_data *ctx, int value)
{
#ifdef GPIO_VBUS_CONTROL
	gpiod_set_value(ctx->pdata.gpio_vbus_on, value);
#endif
	return;
}

static irqreturn_t anx7625_comm_isr(int irq, void *data)
{
	struct anx7625_data *ctx = (struct anx7625_data *)data;
	struct device *dev = &ctx->client->dev;
	int sys_status, ivector, cc_status;

	if (atomic_read(&ctx->power_status)==0) {
		DRM_DEV_DEBUG_DRIVER(dev, "anx: comm isr NONE - no power, must be spurious\n");
		return IRQ_NONE;
	}

	if (atomic_read(&ctx->cable_connected)==0) {
		DRM_DEV_DEBUG_DRIVER(dev, "anx: comm isr NONE - cable not connected, "
		       "must be spurious\n");
		return IRQ_NONE;
	}

	DRM_DEV_DEBUG_DRIVER(dev, "anx: %s %d acquiring lock\n", __func__, __LINE__);
	mutex_lock(&ctx->lock);
	DRM_DEV_DEBUG_DRIVER(dev, "anx: %s %d lock acquired\n", __func__, __LINE__);

	ivector = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client,
	                           INTERFACE_CHANGE_INT);
	DRM_DEV_DEBUG_DRIVER(dev, "anx: comms - interrupt vector (0x44) 0x%x:\n", ivector);
	print_ivector(ctx, ivector);

	anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
	                  INTERFACE_CHANGE_INT,
	                  ivector &(~ivector));

	sys_status = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client, SYSTEM_STSTUS);
	DRM_DEV_DEBUG_DRIVER(dev, "anx: comms - system status (0x45) 0x%x:\n", sys_status);
	print_sys_status(ctx, sys_status);

	cc_status = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client, 0x46);
	DRM_DEV_DEBUG_DRIVER(dev, "anx: comms - CC status (0x46) c1 = 0x%x, c2 = 0x%x:\n",
	       cc_status & 0x0F, (cc_status >> 4) & 0x0F);
	print_cc_status(ctx, cc_status);

	/* Inform system of data role change */
	if (ivector & BIT(5)) { /* ivector == DATA ROLE CHANGE */
		if (sys_status & BIT(5))
			anx7625_set_data_role(ctx, TYPEC_HOST); /* DFP */
		else if (!(sys_status & BIT(5)))
			anx7625_set_data_role(ctx, TYPEC_DEVICE); /* UFP */
	}

	/* Inform system of power role changes */
	if (ivector & BIT(3)) { /* ivector == VBUS CHANGE */
		if (sys_status & BIT(3)) {
			typec_set_pwr_role(ctx->usb_typec->port, TYPEC_SOURCE); /* We're power provider */
			vbus_control(ctx, 0);
		}
		else if (!(sys_status & BIT(3))) {
			typec_set_pwr_role(ctx->usb_typec->port, TYPEC_SINK); /* We're power consumer */
			vbus_control(ctx, 1);
		}
	}

	/* Inform system of vconn changes */
	if (ivector & BIT(2)) { /* ivector == VCONN CHANGE */
		if (sys_status & BIT(2))
			typec_set_vconn_role(ctx->usb_typec->port, TYPEC_SOURCE); /* VCONN status OFF */
		else if (!(sys_status & BIT(2)))
			typec_set_vconn_role(ctx->usb_typec->port, TYPEC_SINK); /* VCONN status OFF @TODO: being sink requires Ra to be present, check */
	}

	if (ivector & HPD_STATUS_CHANGE)
		dp_hpd_change_handler(ctx, sys_status & HPD_STATUS);

	sys_sta_bak = sys_status;

	anx7625_reg_write(ctx, ctx->i2c.tcpc_client,
	                  TCPC_INTR_ALERT_1,
	                  0xFF);

	if (ctx->bridge_attached)
		drm_helper_hpd_irq_event(ctx->connector.dev);

	DRM_DEV_DEBUG_DRIVER(dev, "anx: comm isr done\n");
	mutex_unlock(&ctx->lock);
	DRM_DEV_DEBUG_DRIVER(dev, "anx: %s %d lock released\n", __func__, __LINE__);

	return IRQ_HANDLED;
}

#ifdef U_BOOT_EARLY_INIT
/* Purpose of following code is to
 * handle pending configuration from u-boot initialization and
 * reset comm interrupt pending. This has to be done once.
 */
static int anx7625_apply_pending_early_init_config(struct anx7625_data *ctx)
{
	struct device *dev = &ctx->client->dev;
	int sys_status, ivector, cc_status;

	DRM_DEV_DEBUG_DRIVER(dev, "%s: Start\n", __func__);

	DRM_DEV_DEBUG_DRIVER(dev, "%s: forcing ctx->power_status=1\n", __func__);
	atomic_set(&ctx->power_status, 1);

	if(confirmed_cable_det(ctx)) {
		atomic_set(&ctx->cable_connected, 1);
		DRM_DEV_DEBUG_DRIVER(dev, "anx: %s (%s)\n", __func__, atomic_read(&ctx->cable_connected) ? "PLUGGED" : "UNPLUGGED");
		anx7625_typec_connect(ctx, false); /* Usually it's done in cable_det */

		ivector = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client,
															 INTERFACE_CHANGE_INT);
		DRM_DEV_DEBUG_DRIVER(dev, "%s: interrupt vector (0x44) 0x%x:\n", __func__, ivector);
		print_ivector(ctx, ivector);

		anx7625_reg_write(ctx, ctx->i2c.rx_p0_client,
											INTERFACE_CHANGE_INT,
											ivector &(~ivector));

		sys_status = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client, SYSTEM_STSTUS);
		DRM_DEV_DEBUG_DRIVER(dev, "%s: system status (0x45) 0x%x:\n", __func__, sys_status);
		print_sys_status(ctx, sys_status);

		cc_status = anx7625_reg_read(ctx, ctx->i2c.rx_p0_client, 0x46);
		DRM_DEV_DEBUG_DRIVER(dev, "%s: CC status (0x46) c1 = 0x%x, c2 = 0x%x:\n", __func__,
					 cc_status & 0x0F, (cc_status >> 4) & 0x0F);
		print_cc_status(ctx, cc_status);

		/* Fetch data role config from what happened in u-boot */
		if (sys_status & BIT(5))
			anx7625_set_data_role(ctx, TYPEC_HOST); /* DFP */
		else if (!(sys_status & BIT(5)))
			anx7625_set_data_role(ctx, TYPEC_DEVICE); /* UFP */

		/* Fetch power role config from what happened in u-boot */
		if (sys_status & BIT(3)) {
			typec_set_pwr_role(ctx->usb_typec->port, TYPEC_SOURCE); /* We're power provider */
			ctx->pdata.gpio_vbus_on = devm_gpiod_get_optional(dev, "usbc_pwr", GPIOD_OUT_LOW);
			vbus_control(ctx, 0);
		}
		else if (!(sys_status & BIT(3))) {
			typec_set_pwr_role(ctx->usb_typec->port, TYPEC_SINK); /* We're power consumer */
			ctx->pdata.gpio_vbus_on = devm_gpiod_get_optional(dev, "usbc_pwr", GPIOD_OUT_HIGH);
			vbus_control(ctx, 1);
		}

		/* Fetch vconn config from what happened in u-boot */
		if (sys_status & BIT(2))
			typec_set_vconn_role(ctx->usb_typec->port, TYPEC_SOURCE); /* VCONN status OFF */
		else if (!(sys_status & BIT(2)))
			typec_set_vconn_role(ctx->usb_typec->port, TYPEC_SINK); /* VCONN status OFF @TODO: being sink requires Ra to be present, check */

		/* Fetch video cable presence from what happened in u-boot */
		pending_video_work = sys_status & HPD_STATUS;
	} else {
		/* Below lines are necessary since without cable anx7625 reports DFP role and power Source */
		typec_set_pwr_role(ctx->usb_typec->port, TYPEC_SINK); /* We're power consumer */
		ctx->pdata.gpio_vbus_on = devm_gpiod_get_optional(dev, "usbc_pwr", GPIOD_OUT_HIGH);
		vbus_control(ctx, 1);
	}

	DRM_DEV_DEBUG_DRIVER(dev, "%s: End\n", __func__);
	return 0;
}
#endif

static int anx7625_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct anx7625_data *platform;
	struct anx7625_platform_data *pdata;
	int ret = 0;
	struct device *dev = &client->dev;
	struct regulator *regulator;

	DRM_DEV_DEBUG_DRIVER(dev, "anx probing ...\n");

	regulator = devm_regulator_get(dev, "vdda");
	if (IS_ERR(regulator)) {
		if (PTR_ERR(regulator) == -EPROBE_DEFER) {
			/* wait for the 3V regulator */
			return PTR_ERR(regulator);
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	platform = kzalloc(sizeof(*platform), GFP_KERNEL);
	if (!platform) {
		DRM_DEV_ERROR(dev, "fail to allocate driver data\n");
		return -ENOMEM;
	}

	platform->usb_typec = kzalloc(sizeof(struct anx7625_usb_typec), GFP_KERNEL);
	if(IS_ERR_OR_NULL(platform->usb_typec)) {
		DRM_DEV_ERROR(dev, "Failed to allocate mem for platform->usb_typec\n");
		return -ENOMEM;
	}

	pdata = &platform->pdata;

	ret = anx7625_parse_dt(dev, pdata);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "fail to parse DT : %d\n", ret);
		goto free_platform;
	}

	platform->client = client;
	i2c_set_clientdata(client, platform);

	anx7625_init_gpio(platform);

	atomic_set(&platform->cable_connected, 0);

	mutex_init(&platform->lock);

	ret = anx7625_register_i2c_dummy_clients(platform, client);
	if (ret < 0)
		goto free_platform;

	if (IS_ENABLED(CONFIG_OF))
		platform->bridge.of_node = client->dev.of_node;

	platform->bridge.funcs = &anx7625_bridge_funcs;
	drm_bridge_add(&platform->bridge);

	ret = anx7625_audio_init(dev, platform);
	if (ret) {
		DRM_DEV_DEBUG_DRIVER(dev, "can't initialize audio\n");
		return ret;
	}

	/* Get usb typec configuration from device tree */
	ret = anx7625_get_fw_caps(platform, pdata->connector_fwnode);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to get connector caps: %d\n", ret);
		goto free_platform;
	}

	platform->usb_typec->port = typec_register_port(dev, &platform->usb_typec->capability);
	if (IS_ERR(platform->usb_typec->port)) {
		ret = PTR_ERR(platform->usb_typec->port);
		DRM_DEV_ERROR(dev, "Failed to register typec port: %d\n", ret);
		goto free_platform;
	}

	/* Here we're getting the role_sw reference which is used to
	 * propagate data role switch to the AP usb port attached to this usb typec connector */
	platform->usb_typec->role_sw = fwnode_usb_role_switch_get(pdata->connector_fwnode);
	if (IS_ERR(platform->usb_typec->role_sw)) {
		ret = PTR_ERR(platform->usb_typec->role_sw);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Failed to get usb role switch: %d\n", ret);
		goto free_platform;
	}

	/* Default power role and operation mode initialization: will be updated upon
	 * cable or comm interrupt
	 */
	anx7625_set_pwr_role(platform, TYPEC_SINK, platform->usb_typec->pwr_opmode, TYPEC_SINK);

	/* When a cable will be connected, a timeout for usb data role is configured */
	platform->usb_typec->usb_data_role_timeout = false;

	ret = gpiod_to_irq(platform->pdata.gpio_intr_comm);
	if (ret < 0)
		goto free_platform;
	client->irq = ret;
	platform->pdata.intp_irq = ret;

	if (platform->pdata.intp_irq) {
		INIT_WORK(&platform->work, anx7625_work_func);
		platform->workqueue = create_workqueue("anx7625_work");
		if (!platform->workqueue) {
			DRM_DEV_ERROR(dev, "fail to create work queue\n");
			ret = -ENOMEM;
			goto free_platform;
		}

		ret = devm_request_threaded_irq(dev, platform->pdata.intp_irq,
						NULL, anx7625_comm_isr,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						"anx7625-intp", platform);
		if (ret) {
			DRM_DEV_ERROR(dev, "fail to request irq\n");
			goto free_wq;
		}
	}

#ifdef U_BOOT_EARLY_INIT
	anx7625_apply_pending_early_init_config(platform);
#endif

	// Setup & start timer for cable det detection
	ctx_timer_cable_det = platform;
	timer_setup(&timer_cable_det, timer_cable_det_poll, 0);
	mod_timer(&timer_cable_det, jiffies + TIMER_CABLE_DET_POLL_DELAY);

	DRM_DEV_DEBUG_DRIVER(dev, "anx probed\n");

	return 0;

free_wq:
	if (platform->workqueue)
		destroy_workqueue(platform->workqueue);

free_platform:
	kfree(platform);
	anx7625_unregister_i2c_dummy_clients(platform);

	return ret;
}

static int anx7625_i2c_remove(struct i2c_client *client)
{
	struct anx7625_data *platform = i2c_get_clientdata(client);

	drm_bridge_remove(&platform->bridge);

	if (platform->pdata.intp_irq)
		destroy_workqueue(platform->workqueue);

	if (platform->usb_typec->partner) {
		typec_unregister_partner(platform->usb_typec->partner);
		platform->usb_typec->partner = NULL;
	}

	//if (platform->usb_typec->vbus_on) // @TODO: implement this
		//Disable VBUS

	if (platform->usb_typec->role_sw)
		usb_role_switch_put(platform->usb_typec->role_sw);

	typec_unregister_port(platform->usb_typec->port);

	anx7625_unregister_i2c_dummy_clients(platform);
	anx7625_audio_exit(platform);

	kfree(platform);
	return 0;
}

static const struct i2c_device_id anx7625_id[] = {
	{"anx7625", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, anx7625_id);

#ifdef CONFIG_OF
static const struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,anx7625",},
	{},
};
#endif

static struct i2c_driver anx7625_driver = {
	.driver = {
		.name = "anx7625",
#ifdef CONFIG_OF
		.of_match_table = anx_match_table,
#endif
	},
	.probe = anx7625_i2c_probe,
	.remove = anx7625_i2c_remove,

	.id_table = anx7625_id,
};

module_i2c_driver(anx7625_driver);

MODULE_DESCRIPTION("MIPI2DP anx7625 driver");
MODULE_AUTHOR("Xin Ji <xji@analogixsemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(ANX7625_DRV_VERSION);
