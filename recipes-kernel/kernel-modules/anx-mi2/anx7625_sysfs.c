/**
 * 
 * cd /sys/devices/platform/soc@0/soc@0:bus@30800000/30a20000.i2c/i2c-0/0-0058
 * echo help > cmd
 */

#include <linux/device.h>

#include "anx7625.h"

extern struct anx7625_data *anx7625_ctx;
void anx7625_power_standby(struct anx7625_data *ctx);
void anx7625_power_on(struct anx7625_data *ctx);


int anx7625_reg_read(struct anx7625_data *ctx,
			    struct i2c_client *client, u8 reg_addr);
int anx7625_reg_write(struct anx7625_data *ctx,
			     struct i2c_client *client, u8 reg_addr, u8 reg_val);


#define DEBUG 1

#ifdef DEBUG
#define DBG_PRINT(fmt, ...) 	printk(KERN_ERR "SYSFS:%s:"fmt, __func__, ##__VA_ARGS__)
#endif



#define OCM_SLAVE_I2C_ADDR1     0x7E
#define DATA_ROLE 0x20


extern u8 pd_rdo[];

struct i2c_client *anx_i2c_get(u8 DevAddr)
{
	switch (DevAddr) {
	case TX_P0_ADDR: return anx7625_ctx->i2c.tx_p0_client;
	case TX_P1_ADDR: return anx7625_ctx->i2c.tx_p1_client;
	case TX_P2_ADDR: return anx7625_ctx->i2c.tx_p2_client;
	case RX_P0_ADDR: return anx7625_ctx->i2c.rx_p0_client;
	case RX_P1_ADDR: return anx7625_ctx->i2c.rx_p1_client;
	case RX_P2_ADDR: return anx7625_ctx->i2c.rx_p2_client;
	case TCPC_INTERFACE_ADDR: return anx7625_ctx->i2c.tcpc_client;
	}
	return NULL;
}
/**
 */
unsigned char ReadReg(unsigned char DevAddr, unsigned char RegAddr)
{
	return anx7625_reg_read(anx7625_ctx, anx_i2c_get(DevAddr), RegAddr);
}

/**
 */
void WriteReg(unsigned char DevAddr, unsigned char RegAddr, unsigned char RegVal)
{
	anx7625_reg_write(anx7625_ctx, anx_i2c_get(DevAddr), RegAddr, RegVal);
}

s8 get_data_role(void)
{
	u8 status;

	/*fetch the data role */
	status = ReadReg(OCM_SLAVE_I2C_ADDR1, SYSTEM_STSTUS);
	return (status & DATA_ROLE) != 0;
}

s8 get_power_role(void)
{
	u8 status;

	/*fetch the power role */
	status = ReadReg(OCM_SLAVE_I2C_ADDR1, SYSTEM_STSTUS);
	return (status & VBUS_STATUS) == 0;
}

/**
 */
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
		anx7625_dump_register(NULL, NULL, DevAddrString, sizeof(DevAddrString));
	}
}

ssize_t anx7625_erase_hex(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	// TODO MI2_power_on();
	// TODO command_erase_mainfw();
	return 1;
}

ssize_t anx7625_burn_hex(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	// TODO burnhex(0);
	return 1;
}

ssize_t anx7625_read_hex(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	/* TODO 
	int cmd, val;
	int result;

	result = sscanf(buf, "%x  %x", &cmd, &val);
	pr_info("readhex()\n");
	MI2_power_on();
	command_flash_read((unsigned int)cmd, (unsigned long)val);
	pr_info("\n");

	return count;
	*/
	return 0;
}

ssize_t anx7625_dpcd_read(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{/*
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
	*/return 0;
}

ssize_t anx7625_dpcd_write(struct device *dev,
                           struct device_attribute *attr,
                           const char *buf, size_t count)
{/*
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
	*/return 0;
}

ssize_t anx7625_dump_edid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/*TODO
	uint k, j;
	unsigned char blocks_num;
	unsigned char edid_blocks[256*2];
	unsigned char pLine[50];

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
	*/
	return snprintf(buf, 5, "OK!\n");
}

void anx7625_dpi_config(int table_id)
{
	// TODO command_DPI_Configuration(table_id);
}

void anx7625_dsi_config(int table_id)
{
	// TODO command_DSI_Configuration(table_id);
}

void anx7625_audio_config(int table_id)
{
	// TODO command_Configure_Audio_Input(table_id);
}

void anx7625_dsc_config(int table_id, int ratio)
{
	/* TODO 
	pr_info("dsc configure table id %d, dsi config:%d\n",
		(uint)table_id, (uint)ratio);
	if (ratio == 0)
		command_DPI_DSC_Configuration(table_id);
	else
		command_DSI_DSC_Configuration(table_id);
	*/
}

ssize_t anx7625_debug(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int param[4];
	int result, i;
	char CommandName[20];

	memset(param, 0, sizeof(param));
	result = sscanf(buf, "%s %d %d %d %d",
	                CommandName, param, param+1, param+2, param+3);
	pr_info("anx7625 cmd[%s", CommandName);
	for (i = 0; i < result - 1; i++)
		pr_info(" 0x%x", param[i]);
	pr_info("]\n");

	if (strcmp(CommandName, "poweron") == 0) {
		pr_info("MI2_power_on\n");
		//MI2_power_on();
		anx7625_power_on(anx7625_ctx);
	} else if (strcmp(CommandName, "powerdown") == 0) {
		anx7625_power_standby(anx7625_ctx);
#if 0
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
#endif
	} else if (strcmp(CommandName, "dumpall") == 0) {
		dumpall();
#if 0
	} else if (strcmp(CommandName, "mute") == 0) {
		command_Mute_Video(param[0]);
#endif
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

static struct device_attribute anx7625_device_attrs[] = {
	__ATTR(pdcmd   , S_IWUSR, NULL                  , anx7625_send_pd_cmd),
	__ATTR(rdreg   , S_IWUSR, NULL                  , anx7625_rd_reg),
	__ATTR(wrreg   , S_IWUSR, NULL                  , anx7625_wr_reg),
	__ATTR(dumpreg , S_IWUSR, NULL                  , anx7625_dump_register),
	__ATTR(prole   , S_IRUGO, anx7625_get_power_role, NULL),
	__ATTR(drole   , S_IRUGO, anx7625_get_data_role , NULL),
	__ATTR(pswap   , S_IRUGO, anx7625_send_pswap    , NULL),
	__ATTR(dswap   , S_IRUGO, anx7625_send_dswap    , NULL),
	__ATTR(dpcdr   , S_IWUSR, NULL                  , anx7625_dpcd_read),
	__ATTR(dpcdw   , S_IWUSR, NULL                  , anx7625_dpcd_write),
	__ATTR(dumpedid, S_IRUGO, anx7625_dump_edid     , NULL),
	__ATTR(cmd     , S_IWUSR, NULL                  , anx7625_debug)
};

/**
 */
int anx7625_sysfs_create(struct device *dev)
{
	int i;

	DBG_PRINT("anx7625 create system fs interface ...\n");
	for (i = 0; i < ARRAY_SIZE(anx7625_device_attrs); i++) {
		if (device_create_file(dev, &anx7625_device_attrs[i])) {
			goto error;
		}
	}
	DBG_PRINT("success\n");
	return 0;

error:
	for (; i >= 0; i--) {
		device_remove_file(dev, &anx7625_device_attrs[i]);
	}
	DBG_PRINT("ERROR: anx7625 Unable to create interface");
	return -EINVAL;
}

/**
 */
int anx7625_sysfs_destory(struct device *dev)
{
	int i;

	DBG_PRINT("anx7625 destory system fs interface ...\n");

	for (i = 0; i < ARRAY_SIZE(anx7625_device_attrs); i++) {
		device_remove_file(dev, &anx7625_device_attrs[i]);
	}
	return 0;
}
