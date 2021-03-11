/**
 * TCPM: USB Type-C Port Controller Manager
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/usb/pd.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>

#include "anx7625.h"


extern struct anx7625_data *anx7625_ctx;

int anx7625_reg_read(struct anx7625_data *ctx,
			    struct i2c_client *client, u8 reg_addr);
int anx7625_reg_block_read(struct anx7625_data *ctx,
				  struct i2c_client *client,
				  u8 reg_addr, u8 len, u8 *buf);
int anx7625_reg_write(struct anx7625_data *ctx,
			     struct i2c_client *client, u8 reg_addr, u8 reg_val);
int anx7625_write_or(struct anx7625_data *ctx, struct i2c_client *client,
			    u8 offset, u8 mask);
int anx7625_write_and(struct anx7625_data *ctx,
			     struct i2c_client *client, u8 offset, u8 mask);
int anx7625_write_and_or(struct anx7625_data *ctx,
				struct i2c_client *client, u8 offset,
				u8 and_mask, u8 or_mask);


#define OCM_DEBUG_REG_8             0x88
#define STOP_MAIN_OCM           6

#define DEBUG 1

#ifdef DEBUG
#define DBG_PRINT(fmt, ...) 	printk(KERN_ERR "TCPM:%s:"fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG

char *str_port_type[] = {
	"PORT_SRC",
	"PORT_SNK",
	"PORT_DRP",
};

char *str_data_role[] = {
	"DEVICE",
	"HOST",
};

char *str_role[] = {
	"SINK",
	"SOURCE",
};

char *str_cc_status[] = {
	"CC_OPEN",
	"CC_RA",
	"CC_RD",
	"CC_RP_DEF",
	"CC_RP_1_5",
	"CC_RP_3_0",
};

char *str_cc_polarity[] = {
	"POLARITY_CC1",
	"POLARITY_CC2",
};
#endif

/**
 */
struct anx7625_tcpm {
	struct device         *dev;
	struct tcpm_port      *port;
	struct tcpc_dev        tcpc;
	int                    vbus;
	enum typec_cc_polarity polarity;
	enum typec_cc_status   cc1;
	enum typec_cc_status   cc2;
};

struct anx7625_tcpm *anx7625_tcpm;

/**
 */
static int anx7625_tcpm_init(struct tcpc_dev *tcpc)
{
	DBG_PRINT("\n");
	return 0;
}

/**
 */
static int anx7625_tcpm_get_vbus(struct tcpc_dev *tcpc)
{
	/* TODO
	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &reg);
	if (ret < 0)
		return ret;

	return !!(reg & TCPC_POWER_STATUS_VBUS_PRES);
	*/
	DBG_PRINT("vbus %d\n", anx7625_tcpm->vbus);
	return anx7625_tcpm->vbus;
}


/**
 * TODO da rivedere con polarity
 */
static int anx7625_tcpm_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	//DBG_PRINT("cc %d %s\n", cc, str_cc_status[cc]);

	/*
enum typec_cc_status {
	TYPEC_CC_OPEN,
	TYPEC_CC_RA,
	TYPEC_CC_RD,
	TYPEC_CC_RP_DEF,
	TYPEC_CC_RP_1_5,
	TYPEC_CC_RP_3_0,
};

5:4 RP_VALUE 00: Rp default, 01: Rp 1.5A, 10: Rp 3.0A, 11:reserved
3:2 CC2_CONTROL 00: Ra, 01: Rp, 10: Rd, 11: Open.
1:0 CC1_CONTROL 00: Ra, 01: Rp, 10: Rd, 11: Open.

*/
#if 0
	u8 reg;

	reg = anx7625_reg_read(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
	                       TCPC_ROLE_CONTROL);
	reg &= 0xC0;
	switch (cc) {
	case TYPEC_CC_OPEN  : reg |= 0x0F; break;
	case TYPEC_CC_RA    : reg |= 0x00; break;
	case TYPEC_CC_RD    : reg |= 0x0A; break;
	case TYPEC_CC_RP_DEF: reg |= 0x05; break;
	case TYPEC_CC_RP_1_5: reg |= 0x15; break;
	case TYPEC_CC_RP_3_0: reg |= 0x25; break;
	}
	DBG_PRINT("cc %d %s, poalrity %d, write %02X in TCPC_ROLE_CONTROL\n",
	          cc, str_cc_status[cc], anx7625_tcpm->polarity, reg);
	anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
	                  TCPC_ROLE_CONTROL, reg);
#endif
#if 0
	u8 ccc; // CC control
	u8 rpv; // RP Value
	u8 reg;

	switch (cc) {
	case TYPEC_CC_OPEN  : ccc = 3; rpv = 0x00; break;
	case TYPEC_CC_RA    : ccc = 0; rpv = 0x00; break;
	case TYPEC_CC_RD    : ccc = 2; rpv = 0x00; break;
	case TYPEC_CC_RP_DEF: ccc = 1; rpv = 0x00; break;
	case TYPEC_CC_RP_1_5: ccc = 1; rpv = 0x10; break;
	case TYPEC_CC_RP_3_0: ccc = 1; rpv = 0x20; break;
	}

	reg = anx7625_reg_read(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
	                       TCPC_ROLE_CONTROL);
	reg &= 0xC0;
	reg |= rpv;
	if (anx7625_tcpm->polarity) {
		reg |= ccc | (3 << 2);
	} else {
		reg |= 3 | (ccc << 2);
	}
	DBG_PRINT("cc %d %s, poalrity %d, write %02X in TCPC_ROLE_CONTROL\n",
	          cc, str_cc_status[cc], anx7625_tcpm->polarity, reg);
	anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
	                  TCPC_ROLE_CONTROL, reg);
#endif
	return 0;
}

/**
 */
static int anx7625_tcpm_get_cc(struct tcpc_dev *tcpc,
			enum typec_cc_status *cc1, enum typec_cc_status *cc2)
{
	*cc1 = anx7625_tcpm->cc1;
	*cc2 = anx7625_tcpm->cc2;
	DBG_PRINT("cc1 %d %s, cc2 %d %s\n",
	          anx7625_tcpm->cc1, str_cc_status[anx7625_tcpm->cc1],
	          anx7625_tcpm->cc2, str_cc_status[anx7625_tcpm->cc2]);
	return 0;
}

/**
 */
static int anx7625_tcpm_set_vbus(struct tcpc_dev *tcpc, bool source, bool sink)
{
	DBG_PRINT("source %d, sink %d\n", source, sink);
	return 0;
}

/**
 */
static int anx7625_tcpm_set_polarity(struct tcpc_dev *tcpc,
		enum typec_cc_polarity polarity)
{
/*
enum typec_cc_polarity {
	TYPEC_POLARITY_CC1,
	TYPEC_POLARITY_CC2,
};
*/

	anx7625_tcpm->polarity = polarity;
	DBG_PRINT("polarity %d %s\n", polarity, str_cc_polarity[polarity]);
	return 0;
}

/**
 */
static int anx7625_tcpm_set_vconn(struct tcpc_dev *tcpc, bool enable)
{
	DBG_PRINT("enable %d\n", enable);
	return 0;
}

/**
 */
static int anx7625_tcpm_start_toggling(struct tcpc_dev *tcpc,
				enum typec_port_type port_type,
				enum typec_cc_status cc)
{
		DBG_PRINT("port_type %d %s, cc %d %s\n",
	          port_type, str_port_type[port_type],
	          cc, str_cc_status[cc]);

#if 0
	int ret;
	struct tcpci *tcpci = tcpc_to_tcpci(tcpc);
	unsigned int reg = TCPC_ROLE_CTRL_DRP;

	/* Handle vendor drp toggling */
	if (tcpci->data->start_drp_toggling) {
		ret = tcpci->data->start_drp_toggling(tcpci, tcpci->data, cc);
		if (ret < 0)
			return ret;
	}

	switch (cc) {
	default:
	case TYPEC_CC_RP_DEF:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_1_5:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_3_0:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 << TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	}
	if (cc == TYPEC_CC_RD)
		reg |= (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			   (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
	else
		reg |= (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			   (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT);
	ret = regmap_write(tcpci->regmap, TCPC_ROLE_CTRL, reg);
	if (ret < 0)
		return ret;

	return regmap_write(tcpci->regmap, TCPC_COMMAND,
			    TCPC_CMD_LOOK4CONNECTION);
#endif
#if 0
	u8 reg;
	int ret;

	DBG_PRINT("port_type %d %s, cc %d %s\n",
	          port_type, str_port_type[port_type],
	          cc, str_cc_status[cc]);

	switch (cc) {
	case TYPEC_CC_OPEN  : reg |= 0x0F; break;
	case TYPEC_CC_RA    : reg |= 0x00; break;
	case TYPEC_CC_RD    : reg |= 0x0A; break;
	case TYPEC_CC_RP_DEF: reg |= 0x05; break;
	case TYPEC_CC_RP_1_5: reg |= 0x15; break;
	case TYPEC_CC_RP_3_0: reg |= 0x25; break;
	}
	reg |= 0x40; // enable DRP
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
	                        TCPC_ROLE_CONTROL, reg);
	if (ret < 0) {
		return ret;
	}

	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
					OCM_DEBUG_REG_8, 1<<STOP_MAIN_OCM);

	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
					TCPC_COMMAND, 0x99);
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
					TCPC_ANALOG_CTRL_1, 0xA0);
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
					TCPC_ANALOG_CTRL_1, 0xE0);
#endif
	return 0;
}

/**
 */
static int anx7625_tcpm_set_pd_rx(struct tcpc_dev *tcpc, bool enable)
{
	DBG_PRINT("enable %d\n", enable);
	return 0;
}

/**
 */
static int anx7625_tcpm_set_roles(struct tcpc_dev *tcpc, bool attached,
			   enum typec_role role, enum typec_data_role data)
{
	DBG_PRINT("attached %d, role %s, data %s\n",
	          attached, str_role[role], str_data_role[data]);
	return 0;
}

/**
 */
static int anx7625_tcpm_pd_transmit(struct tcpc_dev *tcpc,
			     enum tcpm_transmit_type type,
			     const struct pd_message *msg)
{
	DBG_PRINT("type %d\n", type);
	return 0;
}

/**
 *configure DPR toggle
 */
void anx7625_DRP_Enable(void)
{
	int ret;
	/*reset main OCM*/
	//WriteReg(RX_P0, OCM_DEBUG_REG_8, 1<<STOP_MAIN_OCM);
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
					OCM_DEBUG_REG_8, 1<<STOP_MAIN_OCM);

	/*config toggle.*/
	//WriteReg(TCPC_INTERFACE, TCPC_ROLE_CONTROL, 0x45);
	//WriteReg(TCPC_INTERFACE, TCPC_COMMAND, 0x99);
	//WriteReg(TCPC_INTERFACE, TCPC_ANALOG_CTRL_1, 0xA0);
	//WriteReg(TCPC_INTERFACE, TCPC_ANALOG_CTRL_1, 0xE0);
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
					TCPC_ROLE_CONTROL, 0x45);
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
					TCPC_COMMAND, 0x99);
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
					TCPC_ANALOG_CTRL_1, 0xA0);
	ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.tcpc_client,
					TCPC_ANALOG_CTRL_1, 0xE0);
	
	DBG_PRINT("Enable DRP!");
}

/**
 */
int anx7625_tcpm_change(int sys_status, int ivector, int cc_status)
{
	DBG_PRINT("sys_status %X, ivector %X, cc_status %X\n",
						sys_status, ivector, cc_status);

	switch (cc_status & 0x0F) {
	case 0x00: anx7625_tcpm->cc1 = TYPEC_CC_OPEN  ; break;
	case 0x01: anx7625_tcpm->cc1 = TYPEC_CC_RD    ; break;
	case 0x02: anx7625_tcpm->cc1 = TYPEC_CC_RA    ; break;
	case 0x04: anx7625_tcpm->cc1 = TYPEC_CC_RP_DEF; break;
	case 0x08: anx7625_tcpm->cc1 = TYPEC_CC_RP_1_5; break;
	case 0x0C: anx7625_tcpm->cc1 = TYPEC_CC_RP_3_0; break;
	default:
		DBG_PRINT("CC1: '%X' Reserved\n", cc_status & 0x0F);
	}

	switch (cc_status & 0xF0) {
	case  0x00: anx7625_tcpm->cc2 = TYPEC_CC_OPEN  ; break;
	case  0x10: anx7625_tcpm->cc2 = TYPEC_CC_RD    ; break;
	case  0x20: anx7625_tcpm->cc2 = TYPEC_CC_RA    ; break;
	case  0x40: anx7625_tcpm->cc2 = TYPEC_CC_RP_DEF; break;
	case  0x80: anx7625_tcpm->cc2 = TYPEC_CC_RP_1_5; break;
	case  0xC0: anx7625_tcpm->cc2 = TYPEC_CC_RP_3_0; break;
	default:
		DBG_PRINT("CC2: '%X' Reserved\n", (cc_status & 0xF0)>>4);
	}

	anx7625_tcpm->vbus = (sys_status & BIT(3))? 1: 0;  // TODO to verify

	if (ivector & BIT(4)) {
		DBG_PRINT("CC status change\n");
		tcpm_cc_change(anx7625_tcpm->port);
	}
	
	if (ivector & BIT(3)) {
		DBG_PRINT("VBUS change\n");
		tcpm_vbus_change(anx7625_tcpm->port);
		/*
		// TODO to verify
		if (anx7625_tcpm->vbus) {
			tcpm_vbus_change(anx7625_tcpm->port);
		} else {
			tcpm_tcpc_reset(anx7625_tcpm->port);
		}*/
	}
	return 0;
}

/**
 */
int anx7625_tcpm_probe(void)
{
	struct device *dev = &anx7625_ctx->client->dev;

	DBG_PRINT("start\n");

	anx7625_tcpm = devm_kzalloc(dev, sizeof(*anx7625_tcpm), GFP_KERNEL);
	if (!anx7625_tcpm) {
		return -ENOMEM;
	}

	anx7625_tcpm->dev = dev;

	anx7625_tcpm->tcpc.init           = anx7625_tcpm_init;
	anx7625_tcpm->tcpc.get_vbus       = anx7625_tcpm_get_vbus;
	anx7625_tcpm->tcpc.set_vbus       = anx7625_tcpm_set_vbus;
	anx7625_tcpm->tcpc.set_cc         = anx7625_tcpm_set_cc;
	anx7625_tcpm->tcpc.get_cc         = anx7625_tcpm_get_cc;
	anx7625_tcpm->tcpc.set_polarity   = anx7625_tcpm_set_polarity;
	anx7625_tcpm->tcpc.set_vconn      = anx7625_tcpm_set_vconn;
	anx7625_tcpm->tcpc.start_toggling = anx7625_tcpm_start_toggling;
	anx7625_tcpm->tcpc.set_pd_rx      = anx7625_tcpm_set_pd_rx;
	anx7625_tcpm->tcpc.set_roles      = anx7625_tcpm_set_roles;
	anx7625_tcpm->tcpc.pd_transmit    = anx7625_tcpm_pd_transmit;
	
	//anx7625_tcpm->controls_vbus = true; /* XXX */

	anx7625_tcpm->tcpc.fwnode = device_get_named_child_node(dev, "connector");
	if (!anx7625_tcpm->tcpc.fwnode) {
		DBG_PRINT("ERROR Can't find connector node\n");
		dev_err(dev, "Can't find connector node.\n");
		return -EINVAL;
	}
	DBG_PRINT("Connector node found\n");

	anx7625_tcpm->port = tcpm_register_port(dev, &anx7625_tcpm->tcpc);
	if (IS_ERR(anx7625_tcpm->port)) {
		DBG_PRINT("ERROR on tcpm_register_port\n");
		return -1;// ERR_CAST(anx7625_tcpm->port);
	}

	DBG_PRINT("DONE\n");
	return 0;
}

/**
 */
int anx7625_tcpm_release(void)
{
	DBG_PRINT("\n");
	tcpm_unregister_port(anx7625_tcpm->port);
	return 0;
}
