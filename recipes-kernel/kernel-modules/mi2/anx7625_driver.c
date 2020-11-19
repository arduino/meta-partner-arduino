/*
 * Copyright(c) 2016, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "anx7625_driver.h"
#include "anx7625_private_interface.h"
#include "anx7625_public_interface.h"
#include "anx7625_display.h"
#include "display.h"
#include "Flash.h"

#define ENABLE_DRM

#ifdef ENABLE_DRM
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/timer.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <video/display_timing.h>

#define SUPPORT_PIXEL_CLOCK  300000

#endif    // ENABLE_DRM


#define DBG_PRINT(fmt, ...) printk(KERN_ERR"%s:"fmt, __func__, ##__VA_ARGS__)
#define DBG_ERROR(fmt, ...) printk(KERN_ERR"ERROR:%s:"fmt, __func__, ##__VA_ARGS__)
#define IRQ_PRINT(fmt, ...) printk(KERN_ERR"IRQ:%s:"fmt, __func__, ##__VA_ARGS__)
#ifdef DRM_DEV_DEBUG_DRIVER
#undef DRM_DEV_DEBUG_DRIVER
#define DRM_DEV_DEBUG_DRIVER(dev, fmt, ...) dev_printk(KERN_ERR, dev, fmt, ##__VA_ARGS__)
#endif

#ifdef DRM_DEV_ERROR
#undef DRM_DEV_ERROR
#define DRM_DEV_ERROR(dev, fmt, ...) dev_printk(KERN_ERR, dev, fmt, ##__VA_ARGS__)
#endif

//#define DBG_I2C

static int create_sysfs_interfaces(struct device *dev);
static int destory_sysfs_interfaces(struct device *dev);

/* to access global platform data */
static struct anx7625_platform_data *g_pdata;

atomic_t anx7625_power_status;
unsigned char cable_connected;
unsigned char alert_arrived;
unsigned char vbus_en;

struct i2c_client *anx7625_client;

struct anx7625_platform_data {
	struct gpio_desc *gpio_p_on;
	struct gpio_desc *gpio_reset;
#ifndef DISABLE_PD
	struct gpio_desc *gpio_cbl_det;
	int               cbl_det_irq;
#endif
#ifdef SUP_INT_VECTOR
	struct gpio_desc *gpio_intr_comm;
#endif
#ifdef SUP_VBUS_CTL
	struct gpio_desc *gpio_vbus_ctrl;
#endif
	spinlock_t lock;
#ifdef ENABLE_DRM
	struct device_node   *mipi_dsi_host_node;
#endif
};

struct anx7625_data {
	struct anx7625_platform_data *pdata;
	struct delayed_work           work;
	struct workqueue_struct      *workqueue;
	struct mutex                  lock;
#ifdef USE_WAKE_LOCK
	struct wake_lock anx7625_lock;
#endif
#ifdef DYNAMIC_CONFIG_MIPI
	struct msm_dba_device_info    dev_info;
#endif
#ifdef ENABLE_DRM
	struct display_timing         dt;
	u8                            display_timing_valid;
	struct drm_bridge             bridge;
	u8                            bridge_attached;
	struct drm_connector          connector;
	struct mipi_dsi_device       *dsi;
	int                           init_done;
#endif
};

/* to access global platform data */
static struct anx7625_data *the_chip_anx7625;

unsigned char debug_on;
static unsigned char auto_start = 1; /* auto update OCM FW*/

unsigned char hpd_status;

static unsigned char default_dpi_config = 0xff; /*1:720p  3:1080p*/
static unsigned char default_dsi_config = RESOLUTION_QCOM820_1080P60_DSI;
static unsigned char default_audio_config = AUDIO_I2S_2CH_48K; /*I2S 2 channel*/

static unsigned char  last_read_DevAddr = 0xff;

#ifdef DYNAMIC_CONFIG_MIPI

struct timer_list mytimer;
int DBA_init_done = 0;

static void DelayDisplayFunc(unsigned long data)
{
	TRACE("Delay Display Triggered!\n");
	DBA_init_done = 1;
	/*downstream already inserted, trigger cable-det isr to check. */
#ifdef DISABLE_PD
	cable_connected = 1;
	anx7625_restart_work(10);
#else
	/*trigger chip restart*/
	anx7625_restart_work(10);
#endif
}

static int __init displaytimer_init(void)
{
	setup_timer(&mytimer, DelayDisplayFunc, (unsigned long)"Delay DBA Timer!");
	TRACE("Starting timer to fire in 20s (%ld)\n", jiffies);
	mod_timer(&mytimer, jiffies + msecs_to_jiffies(20000));
	return 0;
}

#endif  // DYNAMIC_CONFIG_MIPI

#ifdef ENABLE_DRM
/*
static void delay_drm_init(struct timer_list *ptmr);

DEFINE_TIMER(drm_timer, delay_drm_init);

static void delay_drm_init(struct timer_list *ptmr)
{
	TRACE("Delay Display Triggered!\n");
	the_chip_anx7625->init_done = 1;
	// trigger chip restart
	anx7625_restart_work(10);
}

static void drm_init(void)
{
	TRACE("drm_init: Starting timer to fire in 20s (%ld)\n", jiffies);
	mod_timer(&drm_timer, jiffies + msecs_to_jiffies(20000));
}
*/
#endif  // ENABLE_DRM

/* software workaround for silicon bug MIS2-124 */
static void Reg_Access_Conflict_Workaround(unsigned char DevAddr)
{
	unsigned char RegAddr;
	int ret = 0;

	if (DevAddr != last_read_DevAddr) {
		switch (DevAddr) {
		case  0x54:
		case  0x72:
		default:
			RegAddr = 0x00;
			break;

		case  0x58:
			RegAddr = 0x00;
			break;

		case  0x70:
			RegAddr = 0xD1;
			break;

		case  0x7A:
			RegAddr = 0x60;
			break;

		case  0x7E:
			RegAddr = 0x39;
			break;

		case  0x84:
			RegAddr = 0x7F;
			break;
		}

		anx7625_client->addr = (DevAddr >> 1);
		ret = i2c_smbus_write_byte_data(anx7625_client, RegAddr, 0x00);
#ifdef DBG_I2C
		printk("i2c wa: A:%02X, R:%02X, S:%d, D:%02X\n",
					anx7625_client->addr, RegAddr, (ret>=0)?0:ret, 0x00);
#endif
		if (ret < 0) {
			pr_err("%s %s: failed to write i2c addr=%x:%x ERROR %d\n",
				LOG_TAG, __func__, DevAddr, RegAddr, ret);
		}
		last_read_DevAddr = DevAddr;
	}
}

/* anx7625 power status, sync with interface and cable detection thread */
inline unsigned char ReadReg(unsigned char DevAddr, unsigned char RegAddr)
{
	int ret = 0;

	Reg_Access_Conflict_Workaround(DevAddr);

	anx7625_client->addr = DevAddr >> 1;
	ret = i2c_smbus_read_byte_data(anx7625_client, RegAddr);
#ifdef DBG_I2C
	printk("i2c rd: A:%02X, R:%02X, S:%d, D:%02X\n",
				anx7625_client->addr, RegAddr, (ret>=0)?0:ret, (u8)ret);
#endif
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x:%x ERROR %d\n", LOG_TAG,
			__func__, DevAddr, RegAddr, ret);
	}
	return (uint8_t) ret;
}

unsigned char GetRegVal(unsigned char DevAddr, unsigned char RegAddr)
{
	return ReadReg(DevAddr, RegAddr);
}

int Read_Reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;

	Reg_Access_Conflict_Workaround(slave_addr);

	anx7625_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx7625_client, offset);
#ifdef DBG_I2C
	printk("i2c rd: A:%02X, R:%02X, S:%d, D:%02X\n",
				anx7625_client->addr, offset, (ret>=0)?0:ret, (u8)ret);
#endif
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x:%x ERROR %d\n", LOG_TAG,
			__func__, slave_addr, offset, ret);
		return ret;
	}
	*buf = (uint8_t) ret;

	return 0;
}

inline int ReadBlockReg(unsigned char DevAddr, u8 RegAddr, u8 len, u8 *dat)
{
	int ret = 0;

	Reg_Access_Conflict_Workaround(DevAddr);

	anx7625_client->addr = (DevAddr >> 1);
	ret = i2c_smbus_read_i2c_block_data(anx7625_client, RegAddr, len, dat);
#ifdef DBG_I2C
	{
		char ds[32*3+1];
		int i, l;
		l = 0;
		for (i=0; i<len; i++) {
			l += sprintf(ds+l, " %02X", dat[i]);
		}
		printk("i2c rd: A:%02X, R:%02X, S:%d, L:%d, D:%s\n",
					anx7625_client->addr, RegAddr, (ret>=0)?0:ret, len, ds);
	}
#endif
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x:%x ERROR %d\n", LOG_TAG,
			__func__, DevAddr, RegAddr, ret);
		return -EPERM;
	}

	return (int)ret;
}

inline int WriteBlockReg(unsigned char DevAddr, u8 RegAddr, u8 len,
	const u8 *dat)
{
	int ret = 0;

	Reg_Access_Conflict_Workaround(DevAddr);

	anx7625_client->addr = (DevAddr >> 1);
	ret = i2c_smbus_write_i2c_block_data(anx7625_client, RegAddr, len, dat);
#ifdef DBG_I2C
	{
		char ds[32*3+1];
		int i, l;
		l = 0;
		for (i=0; i<len; i++) {
			l += sprintf(ds+l, " %02X", dat[i]);
		}
		printk("i2c wr: A:%02X, R:%02X, S:%d, L:%d, D:%s\n",
					anx7625_client->addr, RegAddr, ret, len, ds);
	}
#endif
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c block addr=%x:%x ERROR %d\n",
					LOG_TAG, __func__, DevAddr, RegAddr, ret);
		return -EPERM;
	}

	return (int)ret;
}

inline void WriteReg(unsigned char DevAddr, unsigned char RegAddr,
	unsigned char RegVal)
{
	int ret = 0;

	Reg_Access_Conflict_Workaround(DevAddr);

	anx7625_client->addr = (DevAddr >> 1);
	ret = i2c_smbus_write_byte_data(anx7625_client, RegAddr, RegVal);
#ifdef DBG_I2C
	printk("i2c wr: A:%02X, R:%02X, S:%d, D:%02X\n",
				anx7625_client->addr, RegAddr, ret, RegVal);
#endif
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x:%x ERROR %d\n",
					LOG_TAG, __func__, DevAddr, RegAddr, ret);
	}
}

void MI2_power_on(void)
{
#ifdef CONFIG_OF
	struct anx7625_platform_data *pdata = g_pdata;
#else
	struct anx7625_platform_data *pdata = anx7625_client->dev.platform_data;
#endif

	/*power on pin enable */
	gpiod_set_value(pdata->gpio_p_on, 1);
	usleep_range(10000, 11000);
	/*power reset pin enable */
	gpiod_set_value(pdata->gpio_reset, 1);
	usleep_range(10000, 11000);

	TRACE("%s %s: Anx7625 power on !\n", LOG_TAG, __func__);
}

void anx7625_hardware_reset(int enable)
{
#ifdef CONFIG_OF
	struct anx7625_platform_data *pdata = g_pdata;
#else
	struct anx7625_platform_data *pdata = anx7625_client->dev.platform_data;
#endif
	gpiod_set_value(pdata->gpio_reset, enable);
}

void anx7625_power_standby(void)
{
#ifdef CONFIG_OF
	struct anx7625_platform_data *pdata = g_pdata;
#else
	struct anx7625_platform_data *pdata = anx7625_client->dev.platform_data;
#endif

	gpiod_set_value(pdata->gpio_reset, 0);
	usleep_range(1000, 1100);
	gpiod_set_value(pdata->gpio_p_on, 0);
	usleep_range(1000, 1100);

	TRACE("%s %s: anx7625 power down\n", LOG_TAG, __func__);
}

/*configure DPR toggle*/
void ANX7625_DRP_Enable(void)
{
	/*reset main OCM*/
	WriteReg(RX_P0, OCM_DEBUG_REG_8, 1<<STOP_MAIN_OCM);
	/*config toggle.*/
	WriteReg(TCPC_INTERFACE, TCPC_ROLE_CONTROL, 0x45);
	WriteReg(TCPC_INTERFACE, TCPC_COMMAND, 0x99);
	WriteReg(TCPC_INTERFACE, ANALOG_CTRL_1, 0xA0);
	WriteReg(TCPC_INTERFACE, ANALOG_CTRL_1, 0xE0);

	TRACE("Enable DRP!");
}

/* basic configurations of ANX7625 */
void ANX7625_config(void)
{
	WriteReg(RX_P0, XTAL_FRQ_SEL, XTAL_FRQ_27M);
}

BYTE ANX7625_Chip_Located(void)
{
	BYTE c1, c2;

	MI2_power_on();
	Read_Reg(TCPC_INTERFACE, PRODUCT_ID_L, &c1);
	Read_Reg(TCPC_INTERFACE, PRODUCT_ID_H, &c2);
	anx7625_power_standby();
	if ((c1 == 0x25) && (c2 == 0x76)) {
		TRACE("ANX7625 is detected!\n");
		return 1;
	}
	TRACE("No ANX7625 found!\n");
	return 0;
}

#define FLASH_LOAD_STA      0x05
#define FLASH_LOAD_STA_CHK  (1<<7)

void anx7625_hardware_poweron(void)
{
	int retry_count, i;

	for (retry_count = 0; retry_count < 3; retry_count++) {
		MI2_power_on();
		ANX7625_config();
		for (i = 0; i < OCM_LOADING_TIME; i++) {
			/*Interface work? */
			if ((ReadReg(OCM_SLAVE_I2C_ADDR, FLASH_LOAD_STA)&
				FLASH_LOAD_STA_CHK) == FLASH_LOAD_STA_CHK) {
				TRACE("%s %s: interface initialization\n",
					LOG_TAG, __func__);

#ifdef DISABLE_PD
				/*reset main ocm*/
				WriteReg(RX_P0, 0x88,  0x40);
				/*Disable PD*/
				WriteReg(OCM_SLAVE_I2C_ADDR, AP_AV_STATUS, AP_DISABLE_PD);
				/*release main ocm*/
				WriteReg(RX_P0, 0x88,  0x00);
				TRACE("%s: Disable PD\n", LOG_TAG);
#else
				chip_register_init();
				send_initialized_setting();
#endif
				mute_video_flag = 0;

				TRACE("Firmware version %02x%02x,Driver version %s\n",
					ReadReg(OCM_SLAVE_I2C_ADDR, OCM_FW_VERSION),
					ReadReg(OCM_SLAVE_I2C_ADDR, OCM_FW_REVERSION),
					ANX7625_DRV_VERSION);
				return;
			}
			usleep_range(1000, 1100);
		}
		anx7625_power_standby();
	}
}

#define RG_EN_OTG  (0x1<<0x3)

void anx7625_vbus_control(bool on)
{
#ifdef SUP_VBUS_CTL
#ifdef CONFIG_OF
	struct anx7625_platform_data *pdata = g_pdata;
#else
	struct anx7625_platform_data *pdata = anx7625_client->dev.platform_data;
#endif
	if (on)
		gpiod_set_value(pdata->gpio_vbus_ctrl, ENABLE_VBUS_OUTPUT);
	else
		gpiod_set_value(pdata->gpio_vbus_ctrl, DISABLE_VBUS_OUTPUT);
#endif
}

void anx7625_main_process(void)
{
#ifdef DYNAMIC_CONFIG_MIPI
	struct anx7625_data *td;

	td = the_chip_anx7625;
#endif

	TRACE("%s %s:cable_connected=%d power_status=%d\n",
		LOG_TAG, __func__, cable_connected,
		(unsigned int)atomic_read(&anx7625_power_status));

	/* do main loop, do what you want to do */
	if (auto_start) {
		auto_start = 0;
		mute_video_flag = 0;
		if (ANX7625_Chip_Located() == 0) {
			debug_on = 1;
			return;
		}

#if AUTO_UPDATE_OCM_FW
		burnhexauto();
#endif

#ifndef DISABLE_PD
		MI2_power_on();
		ANX7625_DRP_Enable();
		usleep_range(1000, 1100);
		anx7625_power_standby();
#endif

#ifdef DYNAMIC_CONFIG_MIPI
		displaytimer_init();
#else
	#ifdef ENABLE_DRM
		//drm_init();//TODO MX1
		the_chip_anx7625->init_done = 1;
	#else
		/* dongle already inserted, trigger cable-det isr to check. */
		anx7625_cbl_det_isr(1, the_chip_anx7625);
	#endif
#endif

		return;
	}

#ifdef DYNAMIC_CONFIG_MIPI
	if (DBA_init_done == 1) {
		/*check dongle status. */
		anx7625_cbl_det_isr(1, the_chip_anx7625);
		DBA_init_done++;
		return;
	}
#endif

#ifdef ENABLE_DRM
	if (the_chip_anx7625->init_done == 1) {
		/*check dongle status. */
		anx7625_cbl_det_isr(1, the_chip_anx7625);
		the_chip_anx7625->init_done++;
		return;
	}
#endif

	if (atomic_read(&anx7625_power_status) == 0) {
		if (cable_connected == 1) {
			atomic_set(&anx7625_power_status, 1);
			anx7625_hardware_poweron();
			return;
		}
	} else {
		if (cable_connected == 0) {
			atomic_set(&anx7625_power_status, 0);
#ifdef SUP_VBUS_CTL
			/*Disable VBUS supply.*/
			anx7625_vbus_control(0);
			/*gpiod_set_value(platform->pdata->gpio_vbus_ctrl,
				DISABLE_VBUS_OUTPUT);*/
#endif

			if (hpd_status >= 1)
				anx7625_stop_dp_work();

			ANX7625_DRP_Enable();
			usleep_range(1000, 1100);
			clear_sys_sta_bak();
			mute_video_flag = 0;
			anx7625_power_standby();
			return;
		}

		if (alert_arrived)
			anx7625_handle_intr_comm();
	}
}

static void anx7625_work_func(struct work_struct *work)
{
	struct anx7625_data *td = container_of(work, struct anx7625_data,
						work.work);

	mutex_lock(&td->lock);
	anx7625_main_process();
	mutex_unlock(&td->lock);
}

void anx7625_restart_work(int workqueu_timer)
{
	struct anx7625_data *td;

	td = the_chip_anx7625;
	if (td != NULL) {
		queue_delayed_work(td->workqueue, &td->work,
				msecs_to_jiffies(workqueu_timer));
	}
}

void anx7625_stop_dp_work(void)
{
#ifdef DYNAMIC_CONFIG_MIPI
	struct anx7625_data *td;

	td = the_chip_anx7625;
	/* Notify DBA framework disconnect event */
	if (td != NULL)
		anx7625_notify_clients(&td->dev_info,
						MSM_DBA_CB_HPD_DISCONNECT);
#endif

	/* hpd changed */
	TRACE("anx7625_stop_dp_work: mute_flag: %d\n",
		(unsigned int)mute_video_flag);

	DP_Process_Stop();
	hpd_status = 0;
	mute_video_flag = 0;
	delay_tab_id = 0;
}

#ifndef DISABLE_PD
#ifdef CABLE_DET_PIN_HAS_GLITCH
static unsigned char confirmed_cable_det(void *data)
{
	struct anx7625_data *platform = data;

	unsigned int count = 10;
	unsigned int cable_det_count = 0;
	u8 val = 0;

	do {
		val = gpiod_get_value(platform->pdata->gpio_cbl_det);
		if (val == DONGLE_CABLE_INSERT)
			cable_det_count++;
		usleep_range(1000, 1100);
	} while (count--);

	if (cable_det_count > 7)
		return 1;
	else if (cable_det_count < 2)
		return 0;
	else
		return atomic_read(&anx7625_power_status);
}
#endif
#endif

int anx7625_mipi_timing_setting(void);

void anx7625_start_dp(void)
{
	/* hpd changed */
	TRACE("anx7625_start_dp: mute_flag: %d\n",
		(unsigned int)mute_video_flag);

	if (hpd_status >= 2) {
		TRACE("anx7625 filter useless HPD,  %d\n", hpd_status);
		return;
	}

	hpd_status++;

	DP_Process_Start();

#ifdef DYNAMIC_CONFIG_MIPI
	/* Notify DBA framework connect event */
	if (delay_tab_id == 0) {
		anx7625_notify_clients(&(the_chip_anx7625->dev_info),
						MSM_DBA_CB_HPD_CONNECT);
	}
#else
	if (delay_tab_id == 0) {
		if (default_dpi_config < 0x20) {
			DBG_PRINT("command_DPI_Configuration\n");
			command_DPI_Configuration(default_dpi_config);
		} else if (default_dsi_config < 0x20) {
			DBG_PRINT("command_DSI_Configuration\n");
			command_DSI_Configuration(default_dsi_config);
		}
		if (default_audio_config < 0x20) {
			API_Configure_Audio_Input(default_audio_config);
		}
	}
#endif
}

irqreturn_t anx7625_cbl_det_isr(int irq, void *data)
{
#ifndef DISABLE_PD
	struct anx7625_data *platform = data;
#endif

	if (debug_on)
		return IRQ_NONE;

#ifdef DYNAMIC_CONFIG_MIPI
	if (DBA_init_done == 0)
		return IRQ_NONE;
#endif

#ifdef ENABLE_DRM
  if (platform->init_done == 0) {
    return IRQ_NONE;
  }
#endif

#ifdef DISABLE_PD
	cable_connected = DONGLE_CABLE_INSERT;
#else
#ifdef CABLE_DET_PIN_HAS_GLITCH
	cable_connected = confirmed_cable_det((void *)platform);
#else
	cable_connected = gpiod_get_value(platform->pdata->gpio_cbl_det);
#endif

#endif

	TRACE("%s %s : cable plug pin status %d\n", LOG_TAG,
		__func__, cable_connected);

	if (cable_connected == DONGLE_CABLE_INSERT) {
		if (atomic_read(&anx7625_power_status) == 1)
			return IRQ_HANDLED;
		cable_connected = 1;
		anx7625_restart_work(1);
	} else {
		if (atomic_read(&anx7625_power_status) == 0)
			return IRQ_HANDLED;
		cable_connected = 0;
		anx7625_restart_work(1);
	}

	return IRQ_HANDLED;
}

#ifdef SUP_INT_VECTOR
irqreturn_t anx7625_intr_comm_isr(int irq, void *data)
{
	if (atomic_read(&anx7625_power_status) != 1)
		return IRQ_NONE;

	alert_arrived = 1;
	anx7625_restart_work(1);
	return IRQ_HANDLED;
}

void anx7625_handle_intr_comm(void)
{
	unsigned char c;

	c = ReadReg(TCPC_INTERFACE, INTR_ALERT_1);

	TRACE("%s %s : ======I=====alert=%02x\n",
		LOG_TAG, __func__, (uint)c);

	if (c & INTR_SOFTWARE_INT)
		handle_intr_vector();

	if (c & INTR_RECEIVED_MSG)
		/*Received interface message*/
		handle_msg_rcv_intr();

	while (ReadReg(OCM_SLAVE_I2C_ADDR, INTERFACE_CHANGE_INT) != 0)
		handle_intr_vector();

	WriteReg(TCPC_INTERFACE, INTR_ALERT_1, 0xFF);

	if ((gpiod_get_value(the_chip_anx7625->pdata->gpio_intr_comm)) == 0) {
		alert_arrived = 1;
		anx7625_restart_work(1);
		TRACE("%s %s : comm isr pin still low, re-enter\n",
			LOG_TAG, __func__);
	} else {
		alert_arrived = 0;
		TRACE("%s %s : comm isr pin cleared\n",
			LOG_TAG, __func__);
	}
}
#endif	// SUP_INT_VECTOR

#ifdef CONFIG_OF
static int anx7625_parse_dt(struct device *dev,
	struct anx7625_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_p_on =
		devm_gpiod_get_optional(dev, "p_on", GPIOD_OUT_LOW);
	if (pdata->gpio_p_on) {
		DBG_PRINT("p_on %d.\n", desc_to_gpio(pdata->gpio_p_on));
	} else {
		DBG_ERROR("gpio p_on is NULL.\n");
		return -ENODEV;
	}

	pdata->gpio_reset =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (pdata->gpio_reset) {
		DBG_PRINT("reset %d.\n", desc_to_gpio(pdata->gpio_reset));
	} else {
		DBG_ERROR("gpio reset is NULL.\n");
		return -ENODEV;
	}

#ifndef DISABLE_PD
	pdata->gpio_cbl_det = devm_gpiod_get_optional(dev, "cbl_det", GPIOD_IN);
	if (pdata->gpio_cbl_det) {
		DBG_PRINT("cbl_det %d.\n",desc_to_gpio(pdata->gpio_cbl_det));
	} else {
		DBG_ERROR("gpio cbl_det is NULL.\n");
		return -ENODEV;
	}
#endif

#ifdef SUP_VBUS_CTL
	/*reuse previous unless gpio(v33_ctrl) for vbus control*/
	pdata->gpio_vbus_ctrl = devm_gpiod_get_optional(dev, "v33_ctrl", GPIOD_OUT_LOW);
	if (pdata->gpio_vbus_ctrl) {
		DBG_PRINT("v33_ctrl %d.\n",desc_to_gpio(pdata->gpio_vbus_ctrl));
	} else {
		DBG_ERROR("gpio v33_ctrl is NULL.\n");
		return -ENODEV;
	}
#endif
#ifdef SUP_INT_VECTOR
	pdata->gpio_intr_comm = devm_gpiod_get_optional(dev, "intr_comm", GPIOD_IN);
	if (pdata->gpio_intr_comm) {
		DBG_PRINT("intr_comm %d.\n",desc_to_gpio(pdata->gpio_intr_comm));
	} else {
		DBG_ERROR("gpio intr_comm is NULL.\n");
		return -ENODEV;
	}
#endif

#ifdef ENABLE_DRM
	pdata->mipi_dsi_host_node = of_graph_get_remote_node(np, 0, 0);
	if (pdata->mipi_dsi_host_node) {
		of_node_put(pdata->mipi_dsi_host_node);
		DBG_PRINT("found dsi host node.\n");
	} else {
		DBG_PRINT("dsi host node NOT FOUND.\n");
	}
#endif    // ENABLE_DRM

	DBG_PRINT("Done\n");
	return 0;
}
#else
static int anx7625_parse_dt(struct device *dev,
	struct anx7625_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifdef DYNAMIC_CONFIG_MIPI

static int anx7625_register_dba(struct anx7625_data *pdata)
{
	struct msm_dba_ops *client_ops;
	struct msm_dba_device_ops *dev_ops;

	if (!pdata)
		return -EINVAL;

	client_ops = &pdata->dev_info.client_ops;
	dev_ops = &pdata->dev_info.dev_ops;

	client_ops->power_on        = NULL;
	client_ops->video_on        = anx7625_mipi_timing_setting;
	client_ops->configure_audio = anx7625_audio_setting;
	client_ops->hdcp_enable     = NULL;
	client_ops->hdmi_cec_on     = NULL;
	client_ops->hdmi_cec_write  = NULL;
	client_ops->hdmi_cec_read   = NULL;
	client_ops->get_edid_size   = NULL;
	client_ops->get_raw_edid    = anx7625_get_raw_edid;
	client_ops->check_hpd       = NULL;

	strlcpy(pdata->dev_info.chip_name, "anx7625",
			sizeof(pdata->dev_info.chip_name));

	pdata->dev_info.instance_id = 0;

	mutex_init(&pdata->dev_info.dev_mutex);

	INIT_LIST_HEAD(&pdata->dev_info.client_list);

	return msm_dba_add_probed_device(&pdata->dev_info);
}

void anx7625_notify_clients(struct msm_dba_device_info *dev,
		enum msm_dba_callback_event event)
{
	struct msm_dba_client_info *c;
	struct list_head *pos = NULL;

	TRACE("%s++\n", __func__);

	if (!dev) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}

	list_for_each(pos, &dev->client_list) {
		c = list_entry(pos, struct msm_dba_client_info, list);

		TRACE("%s:notifying event %d to client %s\n", __func__,
			event, c->client_name);

		if (c && c->cb)
			c->cb(c->cb_data, event);
	}

	TRACE("%s--\n", __func__);
}

int anx7625_mipi_timing_setting(void *client, bool on,
			struct msm_dba_video_cfg *cfg, u32 flags)
{

	if (cable_connected == 0)
		return 0;

	if (!client || !cfg) {
		pr_err("%s: invalid platform data\n", __func__);
		command_Mute_Video(1);
		return 0;
	}

	TRACE("%s: anx7625_mipi_timing_setting(), v_active=%d, h_active=%d, pclk=%d\n",
		__func__, cfg->v_active, cfg->h_active, cfg->pclk_khz);
	TRACE(" vic=%d, hdmi_mode=%d, num_of_input_lanes=%d, scaninfo=%d\n",
		cfg->vic, cfg->hdmi_mode, cfg->num_of_input_lanes, cfg->scaninfo);

	TRACE("cfg->pclk_khz h= %04x\n", ((cfg->pclk_khz)>>16)&0xffff);
	TRACE("cfg->pclk_khz l= %04x\n", ((cfg->pclk_khz))&0xffff);

	TRACE(" h_active = %d, hfp = %d, hpw = %d, hbp = %d\n",
		 cfg->h_active, cfg->h_front_porch,
		cfg->h_pulse_width, cfg->h_back_porch);

	TRACE(" v_active = %d, vfp = %d, vpw = %d, vbp = %d\n",
		 cfg->v_active, cfg->v_front_porch,
		cfg->v_pulse_width, cfg->v_back_porch);

	if (cfg->v_active <= 480) {
		TRACE("480P\n");
		default_dsi_config = RESOLUTION_480P_DSI;
	} else if ((cfg->v_active > 480) && ((cfg->v_active <= 720))) {
		TRACE("720P\n");
		default_dsi_config = RESOLUTION_720P_DSI;
	} else if ((cfg->v_active > 720) && ((cfg->v_active <= 1080))) {
		default_dsi_config = RESOLUTION_QCOM820_1080P60_DSI;
		TRACE("1080P\n");
	} else if (cfg->v_active > 1080) {
		default_dsi_config = RESOLUTION_QCOM820_1080P60_DSI;
		TRACE("Out max support, set to 1080P\n");
	} else {
		TRACE("unknown resolution\n");
		default_dsi_config = RESOLUTION_480P_DSI;
	}
	/*command_DSI_Configuration(default_dsi_config);*/
	DSI_Configuration(default_dsi_config);
	hpd_status = 1;
	return 0;
}

int anx7625_audio_setting(void *client,
	struct msm_dba_audio_cfg *cfg, u32 flags)
{

	if (cable_connected == 0)
		return 0;

	if (!client || !cfg) {
		pr_err("%s: invalid platform data\n", __func__);
		return 0;
	}

	if (cfg->interface != MSM_DBA_AUDIO_I2S_INTERFACE) {
		pr_err("%s: invalid i2s config.\n", __func__);
		return 0;
	}

	if (cfg->sampling_rate == MSM_DBA_AUDIO_32KHZ)
		default_audio_config = AUDIO_I2S_2CH_32K;
	else if (cfg->sampling_rate == MSM_DBA_AUDIO_44P1KHZ)
		default_audio_config = AUDIO_I2S_2CH_441K;
	else if (cfg->sampling_rate == MSM_DBA_AUDIO_48KHZ)
		default_audio_config = AUDIO_I2S_2CH_48K;
	else if (cfg->sampling_rate == MSM_DBA_AUDIO_88P2KHZ)
		default_audio_config = AUDIO_I2S_2CH_882K;
	else if (cfg->sampling_rate == MSM_DBA_AUDIO_96KHZ)
		default_audio_config = AUDIO_I2S_2CH_96K;
	else if (cfg->sampling_rate == MSM_DBA_AUDIO_176P4KHZ)
		default_audio_config = AUDIO_I2S_2CH_1764K;
	else if (cfg->sampling_rate == MSM_DBA_AUDIO_192KHZ)
		default_audio_config = AUDIO_I2S_2CH_192K;
	else
		default_audio_config = AUDIO_I2S_2CH_32K;

	command_Configure_Audio_Input(default_audio_config);

	return 0;
}

int anx7625_get_raw_edid(void *client,
			u32 size, char *buf, u32 flags)
{
	int block_num;
	struct s_edid_data *p_edid =
		(struct s_edid_data *)slimport_edid_p;

	if (!buf) {
		pr_err("%s: invalid data\n", __func__);
		goto end;
	}

	if (!p_edid) {
		pr_err("%s: invalid edid data\n", __func__);
		size = 0;
		goto end;
	}

	memcpy((uint8_t *)buf,
		(uint8_t *)(p_edid->EDID_block_data), (p_edid->edid_block_num + 1) * 128);
	block_num = p_edid->edid_block_num;

	if (block_num >= 0)
		size = min_t(u32, (block_num + 1) * 128, 4 * 128);
	else
		size = 0;

	TRACE("%s: memcpy EDID block, size=%d\n", __func__, size);

end:
	return 0;
}

#endif

#ifdef ENABLE_DRM
static inline struct anx7625_data *connector_to_anx7625(struct drm_connector *c)
{
	return container_of(c, struct anx7625_data, connector);
}

static inline struct anx7625_data *bridge_to_anx7625(struct drm_bridge *bridge)
{
	return container_of(bridge, struct anx7625_data, bridge);
}

int anx7625_mipi_timing_setting(void)
{
  struct anx7625_data *ctx = the_chip_anx7625;

  if (cable_connected == 0) {
    DBG_PRINT("Cable is not connected\n");
    return 0;
  }

  //if (!client || !cfg) {
  //  pr_err("%s: invalid platform data\n", __func__);
  //  command_Mute_Video(1);
  //  return 0;
  //}

  DBG_PRINT("v_active=%d, h_active=%d, pclk=%d\n",
            ctx->dt.vactive.min, ctx->dt.hactive.min, ctx->dt.pixelclock.min);
  //DBG_PRINT("vic=%d, hdmi_mode=%d, num_of_input_lanes=%d, scaninfo=%d\n",
  //          cfg->vic, cfg->hdmi_mode, cfg->num_of_input_lanes, cfg->scaninfo);

  DBG_PRINT("ctx->dt.pixelclock.min h=%04x, l=%04x\n",
            ((ctx->dt.pixelclock.min)>>16)&0xffff,
            ((ctx->dt.pixelclock.min))&0xffff);

  DBG_PRINT("h_active = %d, hfp = %d, hpw = %d, hbp = %d\n",
            ctx->dt.hactive.min, ctx->dt.hfront_porch.min,
            ctx->dt.hsync_len.min, ctx->dt.hback_porch.min);

  DBG_PRINT("v_active = %d, vfp = %d, vpw = %d, vbp = %d\n",
            ctx->dt.vactive.min, ctx->dt.vfront_porch.min,
            ctx->dt.vsync_len.min, ctx->dt.vback_porch.min);

  if (ctx->dt.vactive.min <= 480) {
    DBG_PRINT("480P\n");
    default_dsi_config = RESOLUTION_480P_DSI;
  } else if ((ctx->dt.vactive.min > 480) && ((ctx->dt.vactive.min <= 720))) {
    DBG_PRINT("720P\n");
    default_dsi_config = RESOLUTION_720P_DSI;
  } else if ((ctx->dt.vactive.min > 720) && ((ctx->dt.vactive.min <= 1080))) {
    default_dsi_config = RESOLUTION_QCOM820_1080P60_DSI;
    DBG_PRINT("1080P\n");
  } else if (ctx->dt.vactive.min > 1080) {
    default_dsi_config = RESOLUTION_QCOM820_1080P60_DSI;
    DBG_PRINT("Out max support, set to 1080P\n");
  } else {
    DBG_PRINT("unknown resolution\n");
    default_dsi_config = RESOLUTION_480P_DSI;
  }
  /*command_DSI_Configuration(default_dsi_config);*/
  DSI_Configuration(default_dsi_config);
  hpd_status = 1;
  return 0;
}


void log_dump(void *buf, size_t len)
{
  u8   *p;
  int   i;
  char  dump[80];
  int   l;

  p =(u8*)buf;
  for (i=0; i<len; i++) {
    if ((i & 0xF) == 0) {
      l = sprintf(dump, "%08X:", i);
    }
    l += sprintf(dump + l, " %02X", p[i]);
    if ((i & 0xF) == 0xF) {
      l += sprintf(dump + l, "\n");
      printk(KERN_ERR"%s", dump);
    }
  }
  if ((i & 0xF) != 0x0) {
    l += sprintf(dump + l, "\n");
    printk(KERN_ERR"%s", dump);
  }
}
static int adx7625_get_edid_block(void *data, u8 *buf, unsigned int block,
                                  size_t len)
{
#if 0
  unsigned char edid_blocks[FOUR_BLOCK_SIZE];
  unsigned char blocks_num;
  
  //DBG_PRINT("block %d, len %d\n", block, len);
  blocks_num = sp_tx_edid_read(edid_blocks);
  //DBG_PRINT("blocks_num %d\n", blocks_num);
  //log_dump(edid_blocks, (blocks_num + 1)*128);
  if ((block & 0x1) == 0) {
    memcpy(buf, edid_blocks, len);
  } else {
    memcpy(buf, edid_blocks + 128, len);
  }
#else
  struct s_edid_data *p_edid = (struct s_edid_data *)slimport_edid_p;
  
  if (!slimport_edid_p) {
    DBG_ERROR("p_edid is NULL\n");
		return -EINVAL;
  }
//	log_dump(p_edid->EDID_block_data, (blocks_num + 1)*128);
	if ((block & 0x1) == 0) {
		memcpy(buf, p_edid->EDID_block_data, len);
	} else {
		memcpy(buf, p_edid->EDID_block_data + 128, len);
	}

#endif
  return 0;
}

static int anx7625_get_modes(struct drm_connector *connector)
{
#if 0
	struct anx7625_data *ctx = connector_to_anx7625(connector);
	int num_modes = 0;
	struct device *dev = &anx7625_client->dev;
	struct s_edid_data *p_edid = (struct s_edid_data *)slimport_edid_p;
	int ret;

	DBG_PRINT("drm get modes\n");

	ret = drm_connector_update_edid_property(connector, (struct edid *)&p_edid->EDID_block_data);
	if (ret) {
		DBG_ERROR("drm_connector_update_edid_property return %d\n", ret);
	}

	num_modes = drm_add_edid_modes(connector,
				       (struct edid *)&p_edid->EDID_block_data);
	DBG_PRINT("num_modes(%d)\n", num_modes);

	return num_modes;
#else
  struct edid    *edid;
  char            name[16];
  unsigned int    count;
  u32             bus_format = MEDIA_BUS_FMT_RGB888_1X24;
  int             ret;

  DBG_PRINT("connector %p\n", (void*)connector);

  if (atomic_read(&anx7625_power_status) == 0) {
    if (cable_connected == 1) {
      anx7625_hardware_poweron();
    } else {
      DBG_ERROR("The cable is disconnected\n");
      return -EINVAL;
    }
  }

  edid = drm_do_get_edid(connector, adx7625_get_edid_block, the_chip_anx7625);
  DBG_PRINT("edid %p\n", (void*)edid);

  drm_edid_get_monitor_name(edid, name, 16);
  DBG_PRINT("Monitor name: %s\n", name);

  if (atomic_read(&anx7625_power_status) == 0) {
    anx7625_power_standby();
  }

  ret = drm_connector_update_edid_property(connector, edid);
  if (ret) {
    DBG_PRINT("drm_connector_update_edid_property return %d\n", ret);
  }

  count = drm_add_edid_modes(connector, edid);
  DBG_PRINT("drm_add_edid_modes return %d\n", count);

	kfree(edid);

  connector->display_info.bus_flags = DRM_BUS_FLAG_DE_LOW |
                                      DRM_BUS_FLAG_PIXDATA_NEGEDGE;

  ret = drm_display_info_set_bus_formats(&connector->display_info,
                                         &bus_format, 1);
  if (ret) {
    DBG_ERROR("Fail on drm_display_info_set_bus_formats\n");
    return ret;
  }

  return count;
#endif
}

static enum drm_mode_status
anx7625_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
//  struct device        *dev = &anx7625_client->dev;

	//DRM_DEV_DEBUG_DRIVER(dev, "drm mode valid verify. clock %d\n", mode->clock);

	if (mode->clock > SUPPORT_PIXEL_CLOCK) {
		DBG_ERROR("mode->clock > SUPPORT_PIXEL_CLOCK %d, %d\n", mode->clock, SUPPORT_PIXEL_CLOCK);
		return MODE_CLOCK_HIGH;
	}
	return MODE_OK;
}

static struct drm_connector_helper_funcs anx7625_connector_helper_funcs = {
	.get_modes  = anx7625_get_modes,
	.mode_valid = anx7625_mode_valid,
};

static enum drm_connector_status anx7625_detect(struct drm_connector *connector,
						bool force)
{
	struct device        *dev = &anx7625_client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "anx7625_detect: drm detect\n");

	if (!cable_connected) {
DRM_DEV_DEBUG_DRIVER(dev, "anx7625_detect:cable_connected=0 return connector_status_disconnected\n");
		return connector_status_disconnected;
	}
DRM_DEV_DEBUG_DRIVER(dev, "anx7625_detect:return connector_status_connected\n");
	return connector_status_connected;
}

static const struct drm_connector_funcs anx7625_connector_funcs = {
	.fill_modes             = drm_helper_probe_single_connector_modes,
	.detect                 = anx7625_detect,
	.destroy                = drm_connector_cleanup,
	.reset                  = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_connector_destroy_state,
};

static int anx7625_attach_dsi(struct anx7625_data *ctx)
{
	struct mipi_dsi_host   *host;
	struct mipi_dsi_device *dsi;
	struct device_node     *mipi_host_node;
	struct device          *dev = &anx7625_client->dev;
	const struct mipi_dsi_device_info info = {
		.type    = "anx7625",
		.channel = 0,
		.node    = NULL,
	};

	DRM_DEV_DEBUG_DRIVER(dev, "attach dsi\n");

	mipi_host_node = ctx->pdata->mipi_dsi_host_node;
	if (!mipi_host_node) {
		DRM_ERROR("dsi host is not configured.\n");
		return -EINVAL;
	}

	host = of_find_mipi_dsi_host_by_node(mipi_host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host.\n");
		return -EINVAL;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device.\n");
		return -EINVAL;
	}

	dsi->lanes      = 4;
	dsi->format     = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO  |
		MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		MIPI_DSI_MODE_EOT_PACKET       |
		MIPI_DSI_MODE_VIDEO_HSE        ;

	if (mipi_dsi_attach(dsi) < 0) {
		DRM_ERROR("failed to attach dsi to host.\n");
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

static int anx7625_bridge_attach(struct drm_bridge *bridge)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	int err;
	struct device *dev = &anx7625_client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "drm attach\n");
	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
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
		DRM_ERROR("Failed to attach to dsi : %d\n", err);
		drm_connector_unregister(&ctx->connector);
		return err;
	}

	ctx->bridge_attached = 1;
	DRM_DEV_DEBUG_DRIVER(dev, "drm attached\n");

	return 0;
}

static enum drm_mode_status
anx7625_bridge_mode_valid(struct drm_bridge *bridge,
			const struct drm_display_mode *mode)
{
	struct device *dev = &anx7625_client->dev;

	//DRM_DEV_DEBUG_DRIVER(dev, "drm mode checking. Clock %d\n", mode->clock);

	/* Max 1200p at 5.4 Ghz, one lane, pixel clock 300M */
	if (mode->clock > SUPPORT_PIXEL_CLOCK) {
		DRM_DEV_DEBUG_DRIVER(dev,
				     "drm mode invalid, pixelclock too high (%d > %d).\n",
				     mode->clock, SUPPORT_PIXEL_CLOCK);
		return MODE_CLOCK_HIGH;
	}

	//DRM_DEV_DEBUG_DRIVER(dev, "drm mode valid.\n");

	return MODE_OK;
}

static void anx7625_bridge_mode_set(struct drm_bridge *bridge,
				const struct drm_display_mode *old_mode,
				const struct drm_display_mode *mode)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device *dev = &anx7625_client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "drm mode set\n");

	mutex_lock(&ctx->lock);

	ctx->dt.pixelclock.min   = mode->clock;
	ctx->dt.hactive.min      = mode->hdisplay;
	ctx->dt.hsync_len.min    = mode->hsync_end - mode->hsync_start;
	ctx->dt.hfront_porch.min = mode->hsync_start - mode->hdisplay;
	ctx->dt.hback_porch.min  = mode->htotal - mode->hsync_end;
	ctx->dt.vactive.min      = mode->vdisplay;
	ctx->dt.vsync_len.min    = mode->vsync_end - mode->vsync_start;
	ctx->dt.vfront_porch.min = mode->vsync_start - mode->vdisplay;
	ctx->dt.vback_porch.min  = mode->vtotal - mode->vsync_end;

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

	mutex_unlock(&ctx->lock);
}

static void anx7625_bridge_enable(struct drm_bridge *bridge)
{
	//struct anx7625_data  *ctx = bridge_to_anx7625(bridge);
	struct device        *dev = &anx7625_client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "drm bridge enable\n");

	//if (WARN_ON(!atomic_read(&anx7625_power_status)))
	//  return;

	anx7625_start_dp();
}

static void anx7625_bridge_disable(struct drm_bridge *bridge)
{
	struct anx7625_data *ctx = bridge_to_anx7625(bridge);
	struct device       *dev = &anx7625_client->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "drm bridge disable\n");

	//if (WARN_ON(!atomic_read(&anx7625_power_status)))
	//  return;


	mutex_lock(&ctx->lock);

	anx7625_stop_dp_work();


	mutex_unlock(&ctx->lock);
}

static const struct drm_bridge_funcs anx7625_bridge_funcs = {
	.attach       = anx7625_bridge_attach,
	.detach       = anx7625_bridge_detach,
	.disable      = anx7625_bridge_disable,
	.mode_valid   = anx7625_bridge_mode_valid,
	.mode_set     = anx7625_bridge_mode_set,
	.enable       = anx7625_bridge_enable,
};
#endif    // ENABLE_DRM

static int anx7625_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct anx7625_data          *platform;
	struct anx7625_platform_data *pdata;
	int                           ret = 0;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s:anx7625's i2c bus doesn't support\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	platform = kzalloc(sizeof(struct anx7625_data), GFP_KERNEL);
	if (!platform) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
					sizeof(struct anx7625_platform_data),
					GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;

		/* device tree parsing function call */
		ret = anx7625_parse_dt(&client->dev, pdata);
		if (ret != 0)	/* if occurs error */
			goto err0;

		platform->pdata = pdata;
	} else {
		platform->pdata = client->dev.platform_data;
	}

	/* to access global platform data */
	g_pdata = platform->pdata;
	the_chip_anx7625 = platform;
	anx7625_client = client;
	anx7625_client->addr = (OCM_SLAVE_I2C_ADDR1 >> 1);

	atomic_set(&anx7625_power_status, 0);

	mutex_init(&platform->lock);

	if (!platform->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	INIT_DELAYED_WORK(&platform->work, anx7625_work_func);

	platform->workqueue =
		create_singlethread_workqueue("anx7625_work");
	if (platform->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

#ifndef DISABLE_PD

	platform->pdata->cbl_det_irq = gpiod_to_irq(platform->pdata->gpio_cbl_det);
	if (platform->pdata->cbl_det_irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err1;
	}

#ifdef USE_WAKE_LOCK
	wake_lock_init(&platform->anx7625_lock, WAKE_LOCK_SUSPEND,
		"anx7625_wake_lock");
#endif

	ret = request_threaded_irq(platform->pdata->cbl_det_irq, NULL, anx7625_cbl_det_isr,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
				| IRQF_ONESHOT, "anx7625-cbl-det", platform);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err3;
	}

	ret = irq_set_irq_wake(platform->pdata->cbl_det_irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err4;
	}

	ret = enable_irq_wake(platform->pdata->cbl_det_irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err4;
	}
#endif

#ifdef SUP_INT_VECTOR
	client->irq = gpiod_to_irq(platform->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get anx7625 gpio comm irq\n", __func__);
		goto err3;
	}

	ret = request_threaded_irq(client->irq, NULL, anx7625_intr_comm_isr,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"anx7625-intr-comm", platform);

	if (ret < 0) {
		pr_err("%s : failed to request interface irq\n", __func__);
		goto err4;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for interface communaction", __func__);
		goto err4;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for interface communaction", __func__);
		goto err4;
	}
#endif

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err4;
	}

	/*add work function*/
	queue_delayed_work(platform->workqueue, &platform->work,
		msecs_to_jiffies(1000));

#ifdef DYNAMIC_CONFIG_MIPI
	/* Register msm dba device */
	ret = anx7625_register_dba(platform);
	if (ret) {
		pr_err("%s: Error registering with DBA %d\n",
			__func__, ret);
	}
#endif

#ifdef ENABLE_DRM
	platform->bridge.funcs = &anx7625_bridge_funcs;
	if (IS_ENABLED(CONFIG_OF))
		platform->bridge.of_node = client->dev.of_node;
	drm_bridge_add(&platform->bridge);
#endif    // ENABLE_DRM

	TRACE("anx7625_i2c_probe successfully %s %s end\n",
		LOG_TAG, __func__);
	goto exit;

err4:
	free_irq(client->irq, platform);

err3:
#ifndef DISABLE_PD
	free_irq(platform->pdata->cbl_det_irq, platform);
#endif

err1:
	destroy_workqueue(platform->workqueue);
err0:
	anx7625_client = NULL;
	kfree(platform);
exit:
	return ret;
}

static int anx7625_i2c_remove(struct i2c_client *client)
{
	struct anx7625_data *platform = the_chip_anx7625;

#ifdef ENABLE_DRM
	drm_bridge_remove(&platform->bridge);
#endif
	destory_sysfs_interfaces(&client->dev);
#ifndef DISABLE_PD
	free_irq(platform->pdata->cbl_det_irq, platform);
#endif
	free_irq(client->irq, platform);
	destroy_workqueue(platform->workqueue);
#ifdef USE_WAKE_LOCK
	wake_lock_destroy(&platform->anx7625_lock);
#endif
	kfree(platform);
	return 0;
}
#ifdef USE_I2C_POWER_MGMT
static int anx7625_i2c_suspend(
	struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int anx7625_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static const struct i2c_device_id anx7625_id[] = {
	{"anx7625", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, anx7625_id);

#ifdef CONFIG_OF
static const struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,anx7625", },
	{},
};
#endif

static struct i2c_driver anx7625_driver = {
	.driver = {
		.name = "anx7625",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = anx_match_table,
#endif
	},
	.probe = anx7625_i2c_probe,
	.remove = anx7625_i2c_remove,
#ifdef USE_I2C_POWER_MGMT
	.suspend = anx7625_i2c_suspend,
	.resume = anx7625_i2c_resume,
#endif

	.id_table = anx7625_id,
};

/*
int anx7625_mipi_dsi_probe(struct mipi_dsi_device *dsi)
{
  DBG_PRINT("\n");
  return 0;
}
int anx7625_mipi_dsi_remove(struct mipi_dsi_device *dsi)
{
  DBG_PRINT("\n");
  return 0;
}
void anx7625_mipi_dsi_shutdown(struct mipi_dsi_device *dsi)
{
  DBG_PRINT("\n");
  return 0;
}
*/
static struct mipi_dsi_driver anx7625_dsi_driver = {
  .driver.name = "mi2",
  //.probe    = anx7625_mipi_dsi_probe,
  //.remove   = anx7625_mipi_dsi_remove,
  //.shutdown = anx7625_mipi_dsi_shutdown,
};

static void __init anx7625_init_async(
	void *data, async_cookie_t cookie)
{
	int ret = 0;

#ifdef DEBUG_LOG_OUTPUT
	slimport_log_on = true;
#else
	slimport_log_on = false;
#endif

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI)) {
		mipi_dsi_driver_register(&anx7625_dsi_driver);
	}

	ret = i2c_add_driver(&anx7625_driver);
	if (ret < 0)
		pr_err("%s: failed to register anx7625 i2c driver\n", __func__);
}

static int __init anx7625_init(void)
{
	TRACE("%s:\n", __func__);
	async_schedule(anx7625_init_async, NULL);
	return 0;
}

static void __exit anx7625_exit(void)
{
	TRACE("%s:\n", __func__);
	i2c_del_driver(&anx7625_driver);

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&anx7625_dsi_driver);
}

int slimport_anx7625_init(void)
{
	TRACE("%s:\n", __func__);
	return 0;
}

ssize_t anx7625_send_pd_cmd(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int cmd;
	int result;

	result = kstrtoint(buf, 10, &cmd);
	switch (cmd) {
	case TYPE_PWR_SRC_CAP:
		send_pd_msg(TYPE_PWR_SRC_CAP, 0, 0);
		break;

	case TYPE_DP_SNK_IDENTITY:
		send_pd_msg(TYPE_DP_SNK_IDENTITY, 0, 0);
		break;

	case TYPE_PSWAP_REQ:
		send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
		break;
	case TYPE_DSWAP_REQ:
		send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
		break;

	case TYPE_GOTO_MIN_REQ:
		send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
		break;

	case TYPE_PWR_OBJ_REQ:
		interface_send_request();
		break;
	case TYPE_ACCEPT:
		interface_send_accept();
		break;
	case TYPE_REJECT:
		interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		send_pd_msg(TYPE_SOFT_RST, 0, 0);
		break;
	case TYPE_HARD_RST:
		send_pd_msg(TYPE_HARD_RST, 0, 0);
		break;

	}
	return count;
}

ssize_t anx7625_send_pswap(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 3, "%d\n", send_power_swap());
}

ssize_t anx7625_send_dswap(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 3, "%d\n", send_data_swap());
}

ssize_t anx7625_get_data_role(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 3, "%d\n", get_data_role());
}

ssize_t anx7625_get_power_role(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 3, "%d\n", get_power_role());
}

ssize_t anx7625_rd_reg(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int v_addr, cmd;
	int result;

	result = sscanf(buf, "%x  %x", &v_addr, &cmd);
	pr_info("reg[%x] = %x\n", cmd, ReadReg(v_addr, cmd));

	return count;
}

ssize_t anx7625_wr_reg(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int v_addr, cmd, val;
	int result;

	result = sscanf(buf, "%x  %x  %x", &v_addr, &cmd, &val);
	pr_info("c %x val %x\n", cmd, val);
	WriteReg(v_addr, cmd, val);
	pr_info("reg[%x] = %x\n", cmd, ReadReg(v_addr, cmd));
	return count;
}

ssize_t anx7625_dump_register(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int i = 0;
	int val;
	int result;
	unsigned char  pLine[100];

	memset(pLine, 0, 100);
	/*result = sscanf(buf, "%x", &val);*/
	result = kstrtoint(buf, 16, &val);

	pr_info(" dump register (0x%x)......\n", val);
	pr_info("	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (i = 0; i < 256; i++) {
		snprintf(&(pLine[(i%0x10)*3]), 4, "%02X ", ReadReg(val, i));
		if ((i & 0x0f) == 0x0f)
			pr_info("[%02x] %s\n", i - 0x0f, pLine);
	}
	pr_info("\ndown!\n");
	return count;
}

/* dump all registers */
/* Usage: dumpall */
static void dumpall(void)
{
	unsigned char DevAddr;
	char DevAddrString[6+2+1];  /* (6+2) characters + NULL terminator*/
	char addr_string[] = {
		0x54, 0x58, 0x70, 0x72, 0x7a, 0x7e, 0x84 };

	for (DevAddr = 0; DevAddr < sizeof(addr_string); DevAddr++) {
		snprintf(DevAddrString, 3, "%02x", addr_string[DevAddr]);
		anx7625_dump_register(NULL, NULL,
			DevAddrString, sizeof(DevAddrString));
	}
}

ssize_t anx7625_erase_hex(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	MI2_power_on();
	command_erase_mainfw();
	return 1;
}

ssize_t anx7625_burn_hex(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	burnhex(0);
	return 1;
}

ssize_t anx7625_read_hex(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int cmd, val;
	int result;

	result = sscanf(buf, "%x  %x", &cmd, &val);
	pr_info("readhex()\n");
	MI2_power_on();
	command_flash_read((unsigned int)cmd, (unsigned long)val);
	pr_info("\n");

	return count;
}

ssize_t anx7625_dpcd_read(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int addrh, addrm, addrl;
	int result;

	result = sscanf(buf, "%x %x %x", &addrh, &addrm, &addrl);
	if (result == 3) {
		unsigned char buff[2];

		sp_tx_aux_dpcdread_bytes(addrh, addrm, addrl, 1, buff);
		pr_info("aux_value = 0x%02x\n", (uint)buff[0]);
	} else {
		pr_info("input parameter error");
	}
	return count;
}

ssize_t anx7625_dpcd_write(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{
	int addrh, addrm, addrl, val;
	unsigned char buff[16];
	int result;

	result = sscanf(buf, "%x  %x  %x  %x",
		&addrh, &addrm, &addrl, &val);
	if (result == 4) {
		buff[0] = (unsigned char)val;
		sp_tx_aux_dpcdwrite_bytes(
			addrh, addrm, addrl, 1, buff);
	} else {
		pr_info("error input parameter.");
	}
	return count;
}

ssize_t anx7625_dump_edid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint k, j;
	unsigned char   blocks_num;
	unsigned char edid_blocks[256*2];
	unsigned char  pLine[50];

	blocks_num = sp_tx_edid_read(edid_blocks);

	for (k = 0, j = 0; k < (128 * ((uint)blocks_num + 1)); k++) {
		if ((k&0x0f) == 0) {
			snprintf(&pLine[j], 14, "edid:[%02hhx] %02hhx ",
				(uint)(k / 0x10), (uint)edid_blocks[k]);
			j = j + 13;
		} else {
			snprintf(&pLine[j], 4, "%02hhx ", (uint)edid_blocks[k]);
			j = j + 3;
		}
		if ((k&0x0f) == 0x0f) {
			pr_info("%s\n", pLine);
			j = 0;
		}
	}
	return snprintf(buf, 5, "OK!\n");
}

void anx7625_dpi_config(int table_id)
{
	command_DPI_Configuration(table_id);
}

void anx7625_dsi_config(int table_id)
{
	command_DSI_Configuration(table_id);
}

void anx7625_audio_config(int table_id)
{
	command_Configure_Audio_Input(table_id);
}

void anx7625_dsc_config(int table_id, int ratio)
{
	pr_info("dsc configure table id %d, dsi config:%d\n",
		(uint)table_id, (uint)ratio);
	if (ratio == 0)
		command_DPI_DSC_Configuration(table_id);
	else
		command_DSI_DSC_Configuration(table_id);
}

ssize_t anx7625_debug(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int param[4];
	int result, i;
	char CommandName[20];

	memset(param, 0, sizeof(param));
	result = sscanf(buf, "%s %d %d %d %d", CommandName,
		param, param+1, param+2, param+3);
	pr_info("anx7625 cmd[%s", CommandName);
	for (i = 0; i < result - 1; i++)
		pr_info(" 0x%x", param[i]);
	pr_info("]\n");

	if (strcmp(CommandName, "poweron") == 0) {
		pr_info("MI2_power_on\n");
		MI2_power_on();
	} else if (strcmp(CommandName, "powerdown") == 0) {
		anx7625_power_standby();
	} else if (strcmp(CommandName, "debugon") == 0) {
		debug_on = 1;
		pr_info("debug_on = %d\n", debug_on);
	} else if (strcmp(CommandName, "debugoff") == 0) {
		debug_on = 0;
		pr_info("debug_on = %d\n", debug_on);
	} else if (strcmp(CommandName, "erasehex") == 0) {
		if ((param[0] == 0) && (param[1] == 0))
			command_erase_mainfw();/*erase main fw*/
		else
			/*erase number of sector from index*/
			command_erase_sector(param[0], param[1]);

	} else if (strcmp(CommandName, "burnhex") == 0) {
		debug_on = 1;
		MI2_power_on();
		if (param[0] == 0) { /*update main OCM*/
			command_erase_mainfw();
			burnhex(0);
		} else if (param[0] == 1) { /*update secure OCM*/
			command_erase_securefw();
			burnhex(1);
		} else
			pr_info("Unknown parameter for burnhex.");
		debug_on = 0;
	} else if (strcmp(CommandName, "readhex") == 0) {
		if ((param[0] == 0) && (param[1] == 0))
			command_flash_read(0x1000, 0x100);
		else
			command_flash_read(param[0], param[1]);
	} else if (strcmp(CommandName, "dpidsiaudio") == 0) {
		default_dpi_config = param[0];
		default_dsi_config = param[1];
		default_audio_config = param[2];
		pr_info("default dpi:%d, default dsi:%d, default audio:%d\n",
			default_dpi_config, default_dsi_config,
			default_audio_config);
	} else if (strcmp(CommandName, "dpi") == 0) {
		default_dpi_config = param[0];
		command_Mute_Video(1);
		anx7625_dpi_config(param[0]);
	} else if (strcmp(CommandName, "dsi") == 0) {
		default_dsi_config = param[0];
		command_Mute_Video(1);
		anx7625_dsi_config(param[0]);
	} else if (strcmp(CommandName, "dsi+") == 0) {
		ulong new_val;

		new_val = param[0]*1000 + param[1];
		reconfig_current_pclk(default_dsi_config, 1, new_val);
		command_Mute_Video(1);
		anx7625_dsi_config(default_dsi_config);
	} else if (strcmp(CommandName, "dsi-") == 0) {
		ulong new_val;

		new_val = param[0]*1000 + param[1];
		reconfig_current_pclk(default_dsi_config, 0, new_val);
		command_Mute_Video(1);
		anx7625_dsi_config(default_dsi_config);
	} else if (strcmp(CommandName, "audio") == 0) {
		default_audio_config = param[0];
		anx7625_audio_config(param[0]);
	} else if (strcmp(CommandName, "dsc") == 0) {
		/*default_dpi_config = param[0];*/
		command_Mute_Video(1);
		anx7625_dsc_config(param[0], param[1]);
	} else if (strcmp(CommandName, "show") == 0) {
		sp_tx_show_information();
	} else if (strcmp(CommandName, "dumpall") == 0) {
		dumpall();
	} else if (strcmp(CommandName, "mute") == 0) {
		command_Mute_Video(param[0]);
	} else {
		pr_info("Usage:\n");
		pr_info("  echo poweron > cmd             :");
		pr_info("			power on\n");
		pr_info("  echo powerdown > cmd           :");
		pr_info("			power off\n");
		pr_info("  echo debugon > cmd             :");
		pr_info("		debug on\n");
		pr_info("  echo debugoff > cmd            :");
		pr_info("			debug off\n");
		pr_info("  echo erasehex > cmd            :");
		pr_info("			erase main fw\n");
		pr_info("  echo burnhex [index] > cmd     :");
		pr_info("	burn FW into flash[0:Main OCM][1:Secure]\n");
		pr_info("  echo readhex [addr] [cnt]> cmd :");
		pr_info("			read bytes from flash\n");
		pr_info("  echo dpi [index] > cmd         :");
		pr_info("			configure dpi with table[index]\n");
		pr_info("  echo dsi [index] > cmd         :");
		pr_info("			configure dsi with table[index]\n");
		pr_info("  echo audio [index] > cmd       :");
		pr_info("			configure audio with table[index]\n");
		pr_info("  echo dpidsiaudio [dpi] [dsi] [audio]> cmd  :\n");
		pr_info("		configure default dpi dsi audio");
		pr_info("			dpi/dsi/audio function.\n");
		pr_info("  echo dsc [index][flag] > cmd         :");
		pr_info("			configure dsc with [index]&[flag]\n");
		pr_info("  echo dpstart > cmd         :");
		pr_info("			Start DP process\n");
		pr_info("  echo show > cmd            :");
		pr_info("			Show DP result information\n");
		pr_info("  echo dumpall > cmd            :");
		pr_info("			Dump anx7625 all register\n");
	}

	return count;
}

/* for debugging */
static struct device_attribute anx7625_device_attrs[] = {
	__ATTR(pdcmd,    S_IWUSR, NULL, anx7625_send_pd_cmd),
	__ATTR(rdreg,    S_IWUSR, NULL, anx7625_rd_reg),
	__ATTR(wrreg,    S_IWUSR, NULL, anx7625_wr_reg),
	__ATTR(dumpreg,  S_IWUSR, NULL, anx7625_dump_register),
	__ATTR(prole,    S_IRUGO, anx7625_get_power_role, NULL),
	__ATTR(drole,    S_IRUGO, anx7625_get_data_role, NULL),
	__ATTR(pswap,    S_IRUGO, anx7625_send_pswap, NULL),
	__ATTR(dswap,    S_IRUGO, anx7625_send_dswap, NULL),
	__ATTR(dpcdr,    S_IWUSR, NULL, anx7625_dpcd_read),
	__ATTR(dpcdw,    S_IWUSR, NULL, anx7625_dpcd_write),
	__ATTR(dumpedid, S_IRUGO, anx7625_dump_edid, NULL),
	__ATTR(cmd,      S_IWUSR, NULL, anx7625_debug)
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	TRACE("anx7625 create system fs interface ...\n");
	for (i = 0; i < ARRAY_SIZE(anx7625_device_attrs); i++)
		if (device_create_file(dev, &anx7625_device_attrs[i]))
			goto error;
	TRACE("success\n");
	return 0;
error:

	for (; i >= 0; i--)
		device_remove_file(dev, &anx7625_device_attrs[i]);
	pr_err("%s %s: anx7625 Unable to create interface",
		LOG_TAG, __func__);
	return -EINVAL;
}

static int destory_sysfs_interfaces(struct device *dev)
{
	int i;

	TRACE("anx7625 destory system fs interface ...\n");

	for (i = 0; i < ARRAY_SIZE(anx7625_device_attrs); i++)
		device_remove_file(dev, &anx7625_device_attrs[i]);

	return 0;
}

module_init(anx7625_init);
module_exit(anx7625_exit);

MODULE_DESCRIPTION("USB PD anx7625 driver");
MODULE_AUTHOR("Li Zhen <zhenli@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(ANX7625_DRV_VERSION);
