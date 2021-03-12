/**
 */

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




#define DEBUG 1

#ifdef DEBUG
#define DBG_PRINT(fmt, ...) 	printk(KERN_ERR "PD:%s:"fmt, __func__, ##__VA_ARGS__)
#endif


#define INTR_MASK_SETTING 0x0
#define INTERACE_TIMEOUT_MS 26


#define InterfaceSendBuf_Addr 0xc0
#define InterfaceRecvBuf_Addr 0xe0


#define INTERFACE_CHANGE_INT    0x44
#define RECEIVED_MSG              0x01
#define RECEIVED_ACK              0x02
//#define VCONN_CHANGE              0x04
//#define VBUS_CHANGE               0x08
//#define CC_STATUS_CHANGE          0x10
//#define DATA_ROLE_CHANGE          0x20
//#define PR_CONSUMER_GOT_POWER     0x40
//#define HPD_STATUS_CHANGE         0x80


/***************
 */

#define PD_ONE_DATA_OBJECT_SIZE  4
#define PD_MAX_DATA_OBJECT_NUM   7
#define VDO_SIZE                (PD_ONE_DATA_OBJECT_SIZE * PD_MAX_DATA_OBJECT_NUM)


u8 send_src_cap(const u8 *src_caps, u8 src_caps_size);
u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size);
u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size);
u8 send_src_dp_cap(const u8 *dp_caps, u8 dp_caps_size);
u8 send_dp_snk_identity(const u8 *snk_ident, u8 snk_ident_size);
u8 send_vdm(const u8 *vdm, u8 size);
u8 send_svid(const u8 *svid, u8 size);
u8 send_rdo(const u8 *rdo, u8 size);
u8 send_power_swap(void);
u8 send_data_swap(void);
u8 send_accept(void);
u8 send_reject(void);
u8 send_soft_reset(void);
u8 send_hard_reset(void);



void send_initialized_setting(void);

	
#define PDO_FIXED_FLAGS (PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP)
	
/*5000mv voltage*/
#define PD_VOLTAGE_5V 5000

#define PD_MAX_VOLTAGE_20V 20000
#define PD_MAX_VOLTAGE_21V 21000

/*0.9A current */
#define PD_CURRENT_900MA   900
#define PD_CURRENT_1500MA 1500

#define PD_CURRENT_3A   3000

#define PD_POWER_15W  15000

#define PD_POWER_60W  60000

/* PDO : Power Data Object
* 1. The vSafe5V Fixed Supply Object shall always be the first object.
* 2. The remaining Fixed Supply Objects,
* if present, shall be sent in voltage order; lowest to highest.
* 3. The Battery Supply Objects,
* if present shall be sent in Minimum Voltage order; lowest to highest.
* 4. The Variable Supply (non battery) Objects,
* if present, shall be sent in Minimum Voltage order; lowest to highest.
*/
#define PDO_TYPE_FIXED ((u32)0 << 30)
#define PDO_TYPE_BATTERY ((u32)1 << 30)
#define PDO_TYPE_VARIABLE ((u32)2 << 30)
#define PDO_TYPE_MASK ((u32)3 << 30)
#define PDO_FIXED_DUAL_ROLE ((u32)1 << 29)	/* Dual role device */
#define PDO_FIXED_SUSPEND ((u32)1 << 28)	/* USB Suspend supported */
#define PDO_FIXED_EXTERNAL ((u32)1 << 27)	/* Externally powered */
#define PDO_FIXED_COMM_CAP ((u32)1 << 26)	/* USB Communications Capable */
#define PDO_FIXED_DATA_SWAP ((u32)1 << 25)	/* Data role swap command */
#define PDO_FIXED_PEAK_CURR ((u32)1 << 20)	/* [21..20] Peak current */
/* Voltage in 50mV units */
#define PDO_FIXED_VOLT(mv) (u32)((((u32)mv)/50) << 10)
/* Max current in 10mA units */
#define PDO_FIXED_CURR(ma) (u32)((((u32)ma)/10))

/*build a fixed PDO packet*/
#define PDO_FIXED(mv, ma, flags) \
	(PDO_FIXED_VOLT(mv)\
	| PDO_FIXED_CURR(ma)\
	| (flags))


/*Pos in Data Object, the first index number begin from 0 */
#define PDO_INDEX(n, dat) (dat << (n * PD_ONE_DATA_OBJECT_SIZE*sizeof(u8)))
#define PDO_VAR_MAX_VOLT(mv) ((((mv) / 50) & 0x3FF) << 20)
#define PDO_VAR_MIN_VOLT(mv) ((((mv) / 50) & 0x3FF) << 10)
#define PDO_VAR_OP_CURR(ma) ((((ma) / 10) & 0x3FF) << 0)

#define PDO_VAR(min_mv, max_mv, op_ma) \
	(PDO_VAR_MIN_VOLT(min_mv) | PDO_VAR_MAX_VOLT(max_mv) | \
	PDO_VAR_OP_CURR(op_ma) | PDO_TYPE_VARIABLE)
#define PDO_BATT_MAX_VOLT(mv) ((((mv) / 50) & 0x3FF) << 20)
#define PDO_BATT_MIN_VOLT(mv) ((((mv) / 50) & 0x3FF) << 10)
#define PDO_BATT_OP_POWER(mw) ((((mw) / 250) & 0x3FF) << 0)
#define PDO_BATT(min_mv, max_mv, op_mw) \
	(PDO_BATT_MIN_VOLT(min_mv)\
	| PDO_BATT_MAX_VOLT(max_mv)\
	| PDO_BATT_OP_POWER(op_mw)\
	| PDO_TYPE_BATTERY)
	
	
/* init setting for TYPE_PWR_SRC_CAP */
static u32 init_src_caps[1] = {
	/*5V, 1.5A, Fixed */
	PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_1500MA, PDO_FIXED_FLAGS)
};

/* init setting for TYPE_PWR_SNK_CAP */
static u32 init_snk_cap[3] = {
	/*5V, 0.9A, Fixed */
	PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS),
	/*min 5V, max 20V, power 60W, battery */
	PDO_BATT(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_21V, PD_POWER_15W),
	/*min5V, max 5V, current 3A, variable */
	PDO_VAR(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_21V, PD_CURRENT_3A)
};
/* init setting for TYPE_SVID */
static u8 init_svid[4] = { 0x00, 0x00, 0x01, 0xff };
/* init setting for TYPE_DP_SNK_IDENTITY */
static u8 init_snk_ident[16] = {
	0x00, 0x00, 0x00, 0xec,	/*snk_id_hdr */
	0x00, 0x00, 0x00, 0x00,	/*snk_cert */
	0x00, 0x00, 0x00, 0x00,	/*snk_prd*/
	0x39, 0x00, 0x00, 0x51		/*5snk_ama*/
};

static u8 pd_src_pdo_cnt = 2;
static u8 pd_src_pdo[VDO_SIZE] = {
	/*5V 0.9A , 5V 1.5 */
	0x5A, 0x90, 0x01, 0x2A, 0x96, 0x90, 0x01, 0x2A
};
	
u8 pd_rdo[PD_ONE_DATA_OBJECT_SIZE];


static u8 pd_snk_pdo_cnt = 3;
static u8 pd_snk_pdo[VDO_SIZE];


u8 configure_DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 src_dp_caps[PD_ONE_DATA_OBJECT_SIZE];


unsigned char InterfaceSendRetryCount = 3;


/**
 * @desc:  The Interface helps customer to register one's
 *	interesting callback function of the specific
 *	USB PD message type, when the REGISTERED message
 *	arrive, the customer's callback function will be executed.
 *
 *	!!!! Becarefully !!!!
 *  Because the USB PD TIMING limatation, the callback function
 *  should be designed to follow USB PD timing requiment.
 *
 * @param:
 *	type: PD message type, define enum PD_MSG_TYPE
 *	func: callback function pointer
 *		it's sepecific definaction is:u8 (*)(void *, u8)
 *
 * @return:  1: success 0: fail
 *
 */
typedef u8(*pd_callback_t) (void *, u8);
u8 register_pd_msg_callback_func(enum PD_MSG_TYPE type, pd_callback_t fnc);
pd_callback_t get_pd_callback_fnc(enum PD_MSG_TYPE type);

u8 polling_interface_msg(int timeout_ms);

u8 dispatch_rcvd_pd_msg(enum PD_MSG_TYPE type, void *para, u8 para_len);
char *interface_to_str(unsigned char header_type);


/**
 */
void handle_msg_rcv_intr(void)
{
	if ((~INTR_MASK_SETTING) & RECEIVED_MSG) {
		polling_interface_msg(INTERACE_TIMEOUT_MS);
	}
	// TODO per ora non serve (fa solo pduf???) pd_ext_message_handling();
}

/* Desc: polling private interface interrupt request message
 * Args: timeout,  block timeout time
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 * Interface's Format:
 *	1Byte Len + 1Byte Type + Len Bytes Data + 1Byte checksum
 */
u8 polling_interface_msg(int timeout_ms)
{
	unsigned char ReadDataBuf[32];
	unsigned char global_i, checksum;
	int ret;

	DBG_PRINT("read block from InterfaceRecvBuf_Addr!\n");
//	ReadBlockReg(OCM_SLAVE_I2C_ADDR, InterfaceRecvBuf_Addr,
//		32, (unsigned char *)ReadDataBuf);
	ret = anx7625_reg_block_read(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
				  InterfaceRecvBuf_Addr, 32, (unsigned char *)ReadDataBuf);
	if (ret<0) {
		DBG_PRINT("read block from InterfaceRecvBuf_Addr CMD_FAIL!\n");
		return CMD_FAIL;
	}
	DBG_PRINT("ReadDataBuf[0] = %d\n", ReadDataBuf[0]);

	if (ReadDataBuf[0] != 0) {
//		WriteReg(OCM_SLAVE_I2C_ADDR, InterfaceRecvBuf_Addr, 0);
		ret = anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
					InterfaceRecvBuf_Addr, 0);
		if (ret<0) {
			return CMD_FAIL;
		}

		checksum = 0;
		for (global_i = 0; global_i < ReadDataBuf[0] + 2; global_i++) {
			checksum += ReadDataBuf[global_i];
		}
		if (checksum == 0) {
			DBG_PRINT(">>(%02X) %s\n", ReadDataBuf[1], interface_to_str(ReadDataBuf[1]));
			dispatch_rcvd_pd_msg((enum PD_MSG_TYPE)ReadDataBuf[1],
				&(ReadDataBuf[2]), ReadDataBuf[0] - 1);
			return CMD_SUCCESS;
		} else {
			DBG_PRINT("checksum error!\n");
			return CMD_FAIL;
		}
	}
	return CMD_FAIL;
}


/**
 * @desc: The Interface that AP sends the specific USB PD command to anx7625
 *
 * @param:
 *	type: PD message type, define enum PD_MSG_TYPE.
 *	buf: the sepecific paramter pointer according to the message type
 *		eg: when AP update its source capability type=TYPE_PWR_SRC_CAP
 *		"buf" contains the content of PDO object,its format USB PD spec
 *		customer can easily packeted it through PDO_FIXED_XXX macro:
 *		default5Vsafe 5V,0.9A -> PDO_FIXED(5000,900, PDO_FIXED_FLAGS)
 *	size: the paramter content length. if buf is null, it should be 0
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 */
u8 send_pd_msg(enum PD_MSG_TYPE type, const char *buf, u8 size)
{
	u8 rst = 0;

	switch (type) {
	case TYPE_PWR_SRC_CAP:
		rst = send_src_cap(buf, size);
		break;
	case TYPE_PWR_SNK_CAP:
		rst = send_snk_cap(buf, size);
		break;
	case TYPE_DP_SNK_IDENTITY:
		rst = interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
				(u8 *)buf, size, INTERFACE_TIMEOUT);
		break;
	case TYPE_SVID:
		rst = send_svid(buf, size);
		break;
	case TYPE_GET_DP_SNK_CAP:
		rst = interface_send_msg_timeout(TYPE_GET_DP_SNK_CAP, NULL, 0,
				INTERFACE_TIMEOUT);
		break;
	case TYPE_PSWAP_REQ:
		rst = send_power_swap();
		break;
	case TYPE_DSWAP_REQ:
		rst = send_data_swap();
		break;
	case TYPE_GOTO_MIN_REQ:
		rst = interface_send_gotomin();
		break;
	case TYPE_VDM:
		rst = send_vdm(buf, size);
		break;
	case TYPE_DP_SNK_CFG:
		rst = send_dp_snk_cfg(buf, size);
		break;
	case TYPE_PWR_OBJ_REQ:
		rst = send_rdo(buf, size);
		break;
	case TYPE_ACCEPT:
		rst = interface_send_accept();
		break;
	case TYPE_REJECT:
		rst = interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		rst = interface_send_soft_rst();
		break;
	case TYPE_HARD_RST:
		rst = interface_send_hard_rst();
		break;
	default:
		pr_info("unknown type %x\n", type);
		rst = 0;
		break;
	}
	if (rst == CMD_FAIL) {
		pr_err("Cmd %x Fail.\n", type);
		return CMD_FAIL;
	}

	return rst;
}

/**
 * @desc:The Interface that AP handle the specific USB PD command from anx7625
 *
 * @param:
 *	type: PD message type, define enum PD_MSG_TYPE.
 *	para: the sepecific paramter pointer
 *	para_len: the paramter ponter's content length
 *		if buf is null, it should be 0
 *
 * @return:  0: success 1: fail
 *
 */
u8 dispatch_rcvd_pd_msg(enum PD_MSG_TYPE type, void *para, u8 para_len)
{
	u8 rst = 0;
	pd_callback_t fnc = get_pd_callback_fnc(type);

DBG_PRINT("type %d para %p para_len %d\n", type, para, para_len);

	if (fnc != 0) {
		rst = (*fnc)(para, para_len);
		return rst;
	}

	switch (type) {
	case TYPE_PWR_SRC_CAP:
		/* execute the receved source capability's  handle function */
//#ifndef AUTO_RDO_ENABLE
//		rst = recv_pd_source_caps_default_callback(para, para_len);
DBG_PRINT("recv_pd_source_caps_default_callback\n");
//#endif
		break;
	case TYPE_PWR_SNK_CAP:
		/* received peer's sink caps */
//		rst = recv_pd_sink_caps_default_callback(para, para_len);
DBG_PRINT("recv_pd_sink_caps_default_callback\n");
		break;
	case TYPE_PWR_OBJ_REQ:
		/* evaluate RDO and give accpet or reject */
//#ifndef AUTO_RDO_ENABLE
//		rst = recv_pd_pwr_object_req_default_callback(para, para_len);
DBG_PRINT("recv_pd_pwr_object_req_default_callback\n");
//#endif
		break;
	case TYPE_DSWAP_REQ:
		/* execute the receved handle function */
//		rst = recv_pd_dswap_default_callback(para, para_len);
DBG_PRINT("recv_pd_dswap_default_callback\n");
		break;
	case TYPE_PSWAP_REQ:
		/* execute the receved handle function */
//		rst = recv_pd_pswap_default_callback(para, para_len);
DBG_PRINT("recv_pd_pswap_default_callback\n");
		break;
	case TYPE_VDM:
		break;
	case TYPE_ACCEPT:
//		rst = recv_pd_accept_default_callback(para, para_len);
DBG_PRINT("recv_pd_accept_default_callback\n");
		break;
	case TYPE_EXT_SRC_CAP:
	case TYPE_EXT_SRC_STS:
	case TYPE_EXT_GET_BATT_CAP:
	case TYPE_EXT_GET_BATT_STS:
	case TYPE_EXT_BATT_CAP:
	case TYPE_EXT_GET_MFR_INFO:
	case TYPE_EXT_MFR_INFO:
	case TYPE_EXT_PDFU_REQUEST:
	case TYPE_EXT_PDFU_RESPONSE:
	case TYPE_EXT_BATT_STS:
	case TYPE_EXT_ALERT:
	case TYPE_EXT_NOT_SUPPORTED:
	case TYPE_EXT_GET_SRC_CAP:
	case TYPE_EXT_GET_SRC_STS:
	case TYPE_EXT_FR_SWAP:
DBG_PRINT(" Recv: PD3_MSG\n");
		//return recv_ext_msg_callback(type, para, para_len);
DBG_PRINT("recv_ext_msg_callback\n");
return 0;
	case TYPE_RESPONSE_TO_REQ:
		/* execute the receved handle function */
//		rst = recv_pd_cmd_rsp_default_callback(para, para_len);
DBG_PRINT("recv_pd_cmd_rsp_default_callback\n");
		break;
	case TYPE_DP_ALT_ENTER:
		break;
	case TYPE_DP_ALT_EXIT:
		break;
	case TYPE_HARD_RST:
//		rst = recv_pd_hard_rst_default_callback(para, para_len);
DBG_PRINT("recv_pd_hard_rst_default_callback\n");
		break;
	default:
		rst = 0;
		break;
	}
	return rst;
}

/**
 */
static pd_callback_t pd_callback_array[256] = { 0 };

/**
 */
pd_callback_t get_pd_callback_fnc(enum PD_MSG_TYPE type)
{
	pd_callback_t fnc = 0;

	if (type < 256) {
		fnc = pd_callback_array[type];
	}
	return fnc;
}



char *interface_to_str(unsigned char header_type)
{
	return (header_type == TYPE_PWR_SRC_CAP) ? "src cap" :
	       (header_type == TYPE_PWR_SNK_CAP) ? "snk cap" :
	       (header_type == TYPE_PWR_OBJ_REQ) ? "RDO" :
	       (header_type == TYPE_DP_SNK_IDENTITY) ? "snk identity" :
	       (header_type == TYPE_SVID) ? "svid" :
	       (header_type == TYPE_PSWAP_REQ) ? "PR_SWAP" :
	       (header_type == TYPE_DSWAP_REQ) ? "DR_SWAP" :
	       (header_type == TYPE_GOTO_MIN_REQ) ? "GOTO_MIN" :
	       (header_type == TYPE_DP_ALT_ENTER) ? "DPALT_ENTER" :
	       (header_type == TYPE_DP_ALT_EXIT) ? "DPALT_EXIT" :
	       (header_type == TYPE_VCONN_SWAP_REQ) ? "VCONN_SWAP" :
	       (header_type == TYPE_GET_DP_SNK_CAP) ? "GET_SINK_DP_CAP" :
	       (header_type == TYPE_DP_SNK_CFG) ? "dp cap" :
	       (header_type == TYPE_SOFT_RST) ? "Soft Reset" :
	       (header_type == TYPE_HARD_RST) ? "Hard Reset" :
	       (header_type == TYPE_RESTART) ? "Restart" :
	       (header_type == TYPE_PD_STATUS_REQ) ? "PD Status" :
	       (header_type == TYPE_ACCEPT) ? "ACCEPT" :
	       (header_type == TYPE_REJECT) ? "REJECT" :
	       (header_type == TYPE_VDM) ? "VDM" :
	       (header_type == TYPE_RESPONSE_TO_REQ) ? "Response to Request" :
	       (header_type == TYPE_SOP_PRIME) ? "SOP'" :
	       (header_type == TYPE_SOP_DOUBLE_PRIME) ? "SOP\"" :
//#ifdef USE_PD30
	       (header_type == TYPE_EXT_SRC_CAP) ? "PD3_SRC_CAP" :
	       (header_type == TYPE_EXT_SRC_STS) ? "PD3_SRC_STS" :
	       (header_type == TYPE_EXT_GET_BATT_CAP) ? "PD3_GET_BATT_CAP" :
	       (header_type == TYPE_EXT_GET_BATT_STS) ? "PD3_GET_BATT_STS" :
	       (header_type == TYPE_EXT_BATT_CAP) ? "PD3_BATT_CAP" :
	       (header_type == TYPE_EXT_GET_MFR_INFO) ? "PD3_GET_MFR_INFO" :
	       (header_type == TYPE_EXT_MFR_INFO) ? "PD3_MFR_INFO" :
	       (header_type == TYPE_EXT_PDFU_REQUEST) ? "PD3_PDFU_REQUEST" :
	       (header_type == TYPE_EXT_PDFU_RESPONSE) ? "PD3_PDFU_RESPONSE" :
	       (header_type == TYPE_EXT_BATT_STS) ? "PD3_BATT_STS" :
	       (header_type == TYPE_EXT_ALERT) ? "PD3_ALERT" :
	       (header_type == TYPE_EXT_NOT_SUPPORTED) ? "PD3_NOT_SUPPORTED" :
	       (header_type == TYPE_EXT_GET_SRC_CAP) ? "PD3_GET_SRC_CAP" :
	       (header_type == TYPE_EXT_GET_SRC_STS) ? "PD3_GET_SRC_STS" :
	       (header_type == TYPE_EXT_FR_SWAP) ? "PD3_FR_SWAP" :
	       (header_type == TYPE_FR_SWAP_SIGNAL) ? "PD3_FR_SWAP_SIGNAL" :
//#endif
	       "Unknown";
}



/**
 * @desc:   The Interface AP set the source capability to anx7625
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability,
 *                         which can be packed by PDO_FIXED_XXX macro
 *                eg: default5Vsafe src_cap(5V, 0.9A fixed) -->
 *			PDO_FIXED(5000,900, PDO_FIXED_FLAGS)
 *
 *                src_caps_size: source capability's size
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_src_cap(const u8 *src_caps, u8 src_caps_size)
{
	if (src_caps == NULL)
		return CMD_FAIL;
	if ((src_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
		(src_caps_size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	memcpy(pd_src_pdo, src_caps, src_caps_size);
	pd_src_pdo_cnt = src_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*send source capabilities message to anx7625 really */
	return interface_send_msg_timeout(TYPE_PWR_SRC_CAP, pd_src_pdo,
		pd_src_pdo_cnt * PD_ONE_DATA_OBJECT_SIZE, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure)
		the sink capability to anx7625's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size)
{
	memcpy(pd_snk_pdo, snk_caps, snk_caps_size);
	pd_snk_pdo_cnt = snk_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*configure sink cap */
	return interface_send_msg_timeout(TYPE_PWR_SNK_CAP, pd_snk_pdo,
			pd_snk_pdo_cnt * 4, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure)
		the DP's sink capability to anx7625's downstream device
 *
 * @param:  dp_snk_caps: PDO buffer pointer of DP sink capability
 *
 *                dp_snk_caps_size: DP sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size)
{
	memcpy(configure_DP_caps, dp_snk_caps, dp_snk_caps_size);
	/*configure sink cap */
	return interface_send_msg_timeout(
		TYPE_DP_SNK_CFG, configure_DP_caps, 4, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP initialze
		the DP's capability of anx7625, as source device
 *
 * @param:  dp_caps: DP's capability  pointer of source
 *
 *                dp_caps_size: source DP capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_src_dp_cap(const u8 *dp_caps, u8 dp_caps_size)
{
	if (dp_caps == NULL)
		return CMD_FAIL;
	if ((dp_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	      (dp_caps_size / PD_ONE_DATA_OBJECT_SIZE) >
	      PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}

	memcpy(src_dp_caps, dp_caps, dp_caps_size);

	/*configure source DP cap */
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
			src_dp_caps, dp_caps_size, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP initialze
		the DP's sink identity of anx7625, as sink device
 *
 * @param:  snk_ident: DP's sink identity
 *
 *                snk_ident_size: DP's sink identity length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_dp_snk_identity(const u8 *snk_ident, u8 snk_ident_size)
{
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
			(u8 *) snk_ident, snk_ident_size, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The Interface AP set the VDM packet to anx7625
 *
 * @param:  vdm:  object buffer pointer of VDM
 *
 *                size: vdm packet size
 *
 *@return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_vdm(const u8 *vdm, u8 size)
{
	u8 tmp[32] = { 0 };

	if (vdm == NULL)
		return CMD_FAIL;
	if (size > 3 && size < 32) {
		memcpy(tmp, vdm, size);
		if (tmp[2] == 0x01 && tmp[3] == 0x00) {
			tmp[3] = 0x40;
			return interface_send_msg_timeout(TYPE_VDM, tmp, size,
				 INTERFACE_TIMEOUT);
		}
	}
	return 1;
}

/**
 * @desc:   The Interface AP set the SVID packet to anx7625
 *
 * @param:  svid:  object buffer pointer of svid
 *
 *                size: svid packet size
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_svid(const u8 *svid, u8 size)
{
	u8 tmp[4] = {
		0
	};
	if (svid == NULL || size != 4)
		return CMD_FAIL;
	memcpy(tmp, svid, size);
	return interface_send_msg_timeout(TYPE_SVID, tmp, size,
			INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure)
		the sink capability to anx7625's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 */
u8 send_rdo(const u8 *rdo, u8 size)
{
	u8 i;

	if (rdo == NULL)
		return CMD_FAIL;
	if ((size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
		(size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	for (i = 0; i < size; i++)
		pd_rdo[i] = *rdo++;

	return interface_send_msg_timeout(TYPE_PWR_OBJ_REQ, pd_rdo, size,
			INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send  PR_Swap command to anx7625
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_power_swap(void)
{
	return interface_pr_swap();
}

/**
 * @desc:   The interface AP will send DR_Swap command to anx7625
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_data_swap(void)
{
	return interface_dr_swap();
}

/**
 * @desc:   The interface AP will send accpet command to anx7625
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_accept(void)
{
	return interface_send_msg_timeout(TYPE_ACCEPT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send reject command to anx7625
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_reject(void)
{
	return interface_send_msg_timeout(TYPE_REJECT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send soft reset command to anx7625
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_soft_reset(void)
{
	return interface_send_soft_rst();
}

/**
 * @desc:   The interface AP will send hard reset command to anx7625
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_hard_reset(void)
{
	return interface_send_hard_rst();
}

/**
 */
inline unsigned char cac_checksum(unsigned char *pSendBuf, unsigned char len)
{
	unsigned char i;
	unsigned char checksum;

	checksum = 0;
	for (i = 0; i < len; i++) {
		checksum += *(pSendBuf + i);
	}
	return (u8) (0 - checksum);
}


/* Desc: send interface message to ocm
 * Args: timeout,  block timeout time
@return:  0: success, 1: reject, 2: fail, 3: busy
 */
u8 interface_send_msg_timeout(u8 type, u8 *pbuf, u8 len, int timeout_ms)
{
	unsigned char c, sending_len;
	unsigned char WriteDataBuf[32];
DBG_PRINT("type %X, len %d, timeout_ms %d\n",
	type, len, timeout_ms);
	/* full, return 0 */
	WriteDataBuf[0] = len + 1;	/* cmd */
	WriteDataBuf[1] = type;
	memcpy(WriteDataBuf + 2, pbuf, len);
	/* cmd + checksum */
	WriteDataBuf[len + 2] = cac_checksum(WriteDataBuf, len + 1 + 1);

	sending_len = WriteDataBuf[0] + 2;

//	c = ReadReg(OCM_SLAVE_I2C_ADDR, InterfaceSendBuf_Addr);
	c = anx7625_reg_read(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
											 InterfaceSendBuf_Addr);

	/* retry*/
	if (InterfaceSendRetryCount && c) {
		unsigned char count = InterfaceSendRetryCount;

		while (count) {
			usleep_range(1000, 1100);
			//c = ReadReg(OCM_SLAVE_I2C_ADDR, InterfaceSendBuf_Addr);
			c = anx7625_reg_read(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
											 InterfaceSendBuf_Addr);
			if (c == 0)
				break;
			count--;
		}
	}
	if (c == 0) {
//		WriteBlockReg(OCM_SLAVE_I2C_ADDR,
//			InterfaceSendBuf_Addr + 1 , sending_len - 1, &WriteDataBuf[1]);
		int i;
		for (i = 1; i < sending_len - 1; i++) {
			anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
					InterfaceSendBuf_Addr + i, WriteDataBuf[i]);
		}
//		WriteReg(OCM_SLAVE_I2C_ADDR, InterfaceSendBuf_Addr, WriteDataBuf[0]);
		anx7625_reg_write(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
					InterfaceSendBuf_Addr, WriteDataBuf[0]);
	} else {
		DBG_PRINT("Tx Buf Full\n");
	}
	return 0;
}

/**
 */
void send_initialized_setting(void)
{
	unsigned char send_init_setting_state, c;

	send_init_setting_state = 1;

	do {
		DBG_PRINT("send_init_setting_state %d\n", send_init_setting_state);
		switch (send_init_setting_state) {
		default:
		case 0:
			break;
		case 1:
			/* send TYPE_PWR_SRC_CAP init setting */
			send_pd_msg(TYPE_PWR_SRC_CAP,
				(const char *)init_src_caps,
				sizeof(init_src_caps));
			send_init_setting_state++;
			break;
		case 2:
//			c = ReadReg(OCM_SLAVE_I2C_ADDR2, InterfaceSendBuf_Addr);
			c = anx7625_reg_read(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
											 InterfaceSendBuf_Addr);

			if (c == 0)
				send_init_setting_state++;
			else
				break;
		case 3:
			/* send TYPE_PWR_SNK_CAP init setting */
			send_pd_msg(TYPE_PWR_SNK_CAP,
				(const char *)init_snk_cap,
				sizeof(init_snk_cap));
			send_init_setting_state++;
			break;
		case 4:
//			c = ReadReg(OCM_SLAVE_I2C_ADDR2, InterfaceSendBuf_Addr);
			c = anx7625_reg_read(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
											 InterfaceSendBuf_Addr);
			if (c == 0)
				send_init_setting_state++;
			else
				break;
		case 5:
			/* send TYPE_DP_SNK_IDENTITY init setting */
			send_pd_msg(TYPE_DP_SNK_IDENTITY,
				init_snk_ident,
				sizeof(init_snk_ident));
			send_init_setting_state++;
			break;
		case 6:
//			c = ReadReg(OCM_SLAVE_I2C_ADDR2, InterfaceSendBuf_Addr);
			c = anx7625_reg_read(anx7625_ctx, anx7625_ctx->i2c.rx_p0_client,
											 InterfaceSendBuf_Addr);
			if (c == 0)
				send_init_setting_state++;
			else
				break;
		case 7:
			/* send TYPE_SVID init setting */
			send_pd_msg(TYPE_SVID, init_svid, sizeof(init_svid));
			{/*Send SetVAR message to delay first CC message*/
			/*
			static u8 init_firstMsg_delay[5] = { 0x10, 0x00, 0x02, 0x01, 0x2f };
			interface_send_msg_timeout(0xfd, init_firstMsg_delay, sizeof(init_firstMsg_delay),
				INTERFACE_TIMEOUT);
			*/}
			send_init_setting_state++;
			break;
		case 8:
		case 9:
			send_init_setting_state = 0;
			break;
		}
	} while (send_init_setting_state != 0);
	DBG_PRINT("DONE\n");
}
