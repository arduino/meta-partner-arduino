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
#define DBG_PRINT(fmt, ...) 	printk(KERN_ERR "PD:"fmt, ##__VA_ARGS__)
#endif


#define INTR_MASK_SETTING 0x0
#define INTERACE_TIMEOUT_MS 26

#define INTERFACE_TIMEOUT 30

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


enum PD_MSG_TYPE {
	TYPE_PWR_SRC_CAP = 0x00,
	TYPE_PWR_SNK_CAP = 0x01,
	TYPE_DP_SNK_IDENTITY = 0x02,
	TYPE_SVID = 0x03,
	TYPE_GET_DP_SNK_CAP = 0x04,
	TYPE_ACCEPT = 0x05,
	TYPE_REJECT = 0x06,
	TYPE_PSWAP_REQ = 0x10,
	TYPE_DSWAP_REQ = 0x11,
	TYPE_GOTO_MIN_REQ = 0x12,
	TYPE_VCONN_SWAP_REQ = 0x13,
	TYPE_VDM = 0x14,
	TYPE_DP_SNK_CFG = 0x15,
	TYPE_PWR_OBJ_REQ = 0x16,
	TYPE_PD_STATUS_REQ = 0x17,
	TYPE_DP_ALT_ENTER = 0x19,
	TYPE_DP_ALT_EXIT = 0x1A,
	TYPE_GET_SNK_CAP = 0x1B,
	TYPE_SOP_PRIME = 0x1C,
	TYPE_SOP_DOUBLE_PRIME = 0x1D,
	TYPE_RESPONSE_TO_REQ = 0xF0,
	TYPE_SOFT_RST = 0xF1,
	TYPE_HARD_RST = 0xF2,
	TYPE_RESTART = 0xF3,
	TYPE_EXT_SRC_CAP = 0xA1, /* Source_Capabilities_Extended*/
	TYPE_EXT_SRC_STS = 0xA2, /* Source_Status*/
	TYPE_EXT_GET_BATT_CAP  = 0xA3, /* Get_Battery_Cap*/
	TYPE_EXT_GET_BATT_STS = 0xA4, /* Get_Battery_ Status*/
	TYPE_EXT_BATT_CAP = 0xA5, /* Battery_Capabilities*/
	TYPE_EXT_GET_MFR_INFO = 0xA6, /* Get_Manufacturer_Info*/
	TYPE_EXT_MFR_INFO = 0xA7, /* Manufacturer_Info*/
	TYPE_EXT_PDFU_REQUEST = 0xA8, /* FW update Request*/
	TYPE_EXT_PDFU_RESPONSE = 0xA9, /* FW update Response*/
	TYPE_EXT_BATT_STS = 0xAA, /* PD_DATA_BATTERY_STATUS*/
	TYPE_EXT_ALERT = 0xAB, /* PD_DATA_ALERT*/
	TYPE_EXT_NOT_SUPPORTED = 0xAC, /* PD_CTRL_NOT_SUPPORTED*/
	TYPE_EXT_GET_SRC_CAP = 0xAD, /* PD_CTRL_GET_SOURCE_CAP_EXTENDED*/
	TYPE_EXT_GET_SRC_STS = 0xAE, /* PD_CTRL_GET_STATUS*/
	TYPE_EXT_FR_SWAP = 0xAF,  /* PD_CTRL_FR_SWAP*/
	TYPE_FR_SWAP_SIGNAL = 0xB0, /* Fast Role Swap signal*/
};

/*Comands status*/
enum interface_status {
	CMD_SUCCESS,
	CMD_REJECT,
	CMD_FAIL,
	CMD_BUSY,
	CMD_STATUS
};


/* Control Message type */
enum pd_ctrl_msg_type {
	/* 0 Reserved */
	PD_CTRL_GOOD_CRC = 1,
	PD_CTRL_GOTO_MIN = 2,
	PD_CTRL_ACCEPT = 3,
	PD_CTRL_REJECT = 4,
	PD_CTRL_PING = 5,
	PD_CTRL_PS_RDY = 6,
	PD_CTRL_GET_SOURCE_CAP = 7,
	PD_CTRL_GET_SINK_CAP = 8,
	PD_CTRL_DR_SWAP = 9,
	PD_CTRL_PR_SWAP = 10,
	PD_CTRL_VCONN_SWAP = 11,
	PD_CTRL_WAIT = 12,
	PD_CTRL_SOFT_RESET = 13,
	/* 14-15 Reserved */
	PD_CTRL_NOT_SUPPORTED = 16,
	PD_CTRL_GET_SOURCE_CAP_EXTENDED = 17,
	PD_CTRL_GET_STATUS = 18,
	PD_CTRL_FR_SWAP = 19,
	/* 20-31 Reserved */
};

/* Data message type */
enum pd_data_msg_type {
	/* 0 Reserved */
	PD_DATA_SOURCE_CAP = 1,
	PD_DATA_REQUEST = 2,
	PD_DATA_BIST = 3,
	PD_DATA_SINK_CAP = 4,
	PD_DATA_BATTERY_STATUS = 5,
	PD_DATA_ALERT = 6,
	/* 7-14 Reserved */
	PD_DATA_VENDOR_DEF = 15,
};

/* Extended message type */
enum pd_ext_msg_type {
	/* 0 Reserved */
	PD_EXT_SOURCE_CAP = 1,
	PD_EXT_STATUS = 2,
	PD_EXT_GET_BATTERY_CAP = 3,
	PD_EXT_GET_BATTERY_STATUS = 4,
	PD_EXT_BATTERY_CAP = 5,
	PD_EXT_GET_MANUFACTURER_INFO = 6,
	PD_EXT_MANUFACTURER_INFO = 7,
	PD_EXT_SECURITY_REQUEST = 8,
	PD_EXT_SECURITY_RESPONSE = 9,
	PD_EXT_FW_UPDATE_REQUEST = 10,
	PD_EXT_FW_UPDATE_RESPONSE = 11,
	/* 12-31 Reserved */
};


struct ext_message_header {
	/* Data Size*/
	unsigned int data_size:9;
		/* Reserved*/
	unsigned int reserved:1;
		/* Request Chunk*/
	unsigned int request_chunk:1;
		/* Chunk Number*/
	unsigned int chunk_number:4;
		/* Chunked*/
	unsigned int chunked:1;
};


#define BYTE_SWAP(word) { (word) = ((word)<<8) | ((word)>>8); }
#define BYTE_SWAP4(dword) \
	{ (dword) = (((dword) & 0x000000FF) << 24) | \
		(((dword) & 0x0000FF00) <<  8) | \
		(((dword) & 0x00FF0000) >>  8) | \
		(((dword) & 0xFF000000) >> 24); }

#define USB_PD_EXT_HEADER(sendbuf, size, request, number, chunk) \
do { \
	((struct ext_message_header *)sendbuf)->data_size = size; \
	((struct ext_message_header *)sendbuf)->reserved = 0; \
	((struct ext_message_header *)sendbuf)->request_chunk = request; \
	((struct ext_message_header *)sendbuf)->chunk_number = number; \
	((struct ext_message_header *)sendbuf)->chunked = chunk; \
	BYTE_SWAP(*(unsigned int *)sendbuf); \
} while (0)


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
// TODO spostare in un .h tipo pd_internal o pd_private





#define USB_PD_EXT_HEADER_SIZE 2


/* SendBlock() state*/
enum send_block_state_type {
	SEND_BLOCK_IDLE = 0,
	SEND_BLOCK_CHUNKING = 1,
	SEND_BLOCK_FINISH = 2,
	SEND_BLOCK_TIMEOUT = 3,
};

/* RecvBlock() status*/
enum recv_block_status_type {
	RECV_BLOCK_IDLE = 0,
	RECV_BLOCK_CHUNKING = 1,
	RECV_BLOCK_FINISH = 2,
	RECV_BLOCK_CHUNK_REQUEST = 3,
};

#define CHUNK_SIZE 26
#define CHUNK_WAIT_REQUEST_TIMEOUT (20<<1) /* 80ms*/

static unsigned char SendBlockType;
static unsigned char *SendBlockDataBuf;
static unsigned int SendBlockDataSize;
static unsigned char SendBlockState = SEND_BLOCK_IDLE;

static unsigned char ChunkSize;
static unsigned char ChunkNumber;
static unsigned char ChunkTotal;
static unsigned char ChunkRequestWaiting;

unsigned char pd_block_recv_type;
unsigned int pd_block_recv_len;
unsigned char pd_block_recv_state = RECV_BLOCK_IDLE;
unsigned char pd_block_recv_buf[260];
static unsigned char chunk_buf[30];


unsigned int PDExtSend_timer;


void pd_ext_message_handling(void);
unsigned char SendBlockIdle(void);

unsigned char SendBlock(unsigned char type, unsigned char *pData, unsigned int BlockSize);




unsigned char send_ext_msg(unsigned char is_ext, unsigned char type,
	unsigned char *pbuf, unsigned char buf_len, unsigned char type_sop);





/**
 */
void handle_msg_rcv_intr(void)
{
	if ((~INTR_MASK_SETTING) & RECEIVED_MSG)
		polling_interface_msg(INTERACE_TIMEOUT_MS);

	pd_ext_message_handling();
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
 */
void pd_ext_message_handling(void)
{
	if (!SendBlockIdle()) {
		SendBlock(0, 0, 0);
	}
// TODO cosa fa????? PF firmware update
//#if USE_PDFU
//	/* PD Firmware Update*/
//	pdfu_initiator_handling();
//	pdfu_responder_handling();
//#endif
}

/**
 */
unsigned char SendBlockIdle(void)
{
	return SendBlockState == SEND_BLOCK_IDLE;
}

/**
 */
unsigned char SendBlock(unsigned char type, unsigned char *pData, unsigned int BlockSize)
{
	unsigned char buf_len;
	unsigned char ret;

DBG_PRINT("type %X, pData %p, BlockSize %d, SendBlockState %d \n",
					type, (void*)pData, BlockSize, SendBlockState );

	if (SendBlockState == SEND_BLOCK_IDLE) {
		if (pData && BlockSize) {
			SendBlockType = type;
			SendBlockDataBuf = (unsigned char *)pData;
			SendBlockDataSize = BlockSize;
			ChunkTotal = SendBlockDataSize / CHUNK_SIZE +
			             ((SendBlockDataSize % CHUNK_SIZE) ? 1 : 0);
			ChunkNumber = 0;
			ChunkRequestWaiting = 0;
			SendBlockState = SEND_BLOCK_CHUNKING;
		}
	}

	if (SendBlockState == SEND_BLOCK_CHUNKING) {
		if (ChunkNumber < ChunkTotal) {
			if (ChunkRequestWaiting == 1) {
				if (!PDExtSend_timer)
					SendBlockState = SEND_BLOCK_TIMEOUT;
			} else {
				if ((ChunkNumber == ChunkTotal - 1) &&
					(SendBlockDataSize % CHUNK_SIZE != 0))
					ChunkSize = (SendBlockDataSize % CHUNK_SIZE);
				else
					ChunkSize = CHUNK_SIZE;

				USB_PD_EXT_HEADER((chunk_buf),
					SendBlockDataSize, 0, ChunkNumber, 1);
				if (ChunkSize)
					memcpy(
				    chunk_buf + USB_PD_EXT_HEADER_SIZE,
				    SendBlockDataBuf + ChunkNumber * CHUNK_SIZE,
				    ChunkSize);
				buf_len = USB_PD_EXT_HEADER_SIZE + ChunkSize;

				if (buf_len & 0x03) {
					unsigned char delta = 4 - (buf_len & 0x03);
					memset(chunk_buf + buf_len, 0, delta);
					buf_len += delta;
				}
//#if DEBUG_MSG
//TODO metter un dump array				TRACE_ARRAY(chunk_buf, buf_len);
{
	char log[256];
	int len;
	int i;
	len = sprintf(log, "chunk buffer len %d\n", buf_len);
	for (i=0; i<buf_len; i++) {
		len += sprintf(log + len, " %02X", chunk_buf[i]); 
	}
	len += sprintf(log + len, "\n");
	DBG_PRINT("%s", log);
}
//#endif
				if (buf_len) {
					ret = send_ext_msg(1, SendBlockType, chunk_buf, buf_len, 0);

					if (ChunkNumber == ChunkTotal - 1)
						SendBlockState = SEND_BLOCK_FINISH;
					else {
						/* wait chunk request*/
						ChunkRequestWaiting = 1;
						PDExtSend_timer = CHUNK_WAIT_REQUEST_TIMEOUT;
					}
				}
			}
		}
	} else if (SendBlockState == SEND_BLOCK_FINISH) {
		/* do somethig, if need*/
		SendBlockState = SEND_BLOCK_IDLE;
	} else if (SendBlockState == SEND_BLOCK_TIMEOUT) {
		/* do somethig, if need*/
		SendBlockState = SEND_BLOCK_IDLE;
	}
	return SendBlockState;
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

DBG_PRINT("dispatch_rcvd_pd_msg type %d para %p para_len %d\n", type, para, para_len);

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






char ConvertPd3TypeToInterfaceType(
	unsigned char ext, unsigned char pd3_type, unsigned char *type)
{
	if (ext) {
		switch (pd3_type) {
		case PD_EXT_SOURCE_CAP:
			*type = TYPE_EXT_SRC_CAP;
			break;
		case PD_EXT_STATUS:
			*type = TYPE_EXT_SRC_STS;
			break;
		case PD_EXT_GET_BATTERY_CAP:
			*type = TYPE_EXT_GET_BATT_CAP;
			break;
		case PD_EXT_GET_BATTERY_STATUS:
			*type = TYPE_EXT_GET_BATT_STS;
			break;
		case PD_EXT_BATTERY_CAP:
			*type = TYPE_EXT_BATT_CAP;
			break;
		case PD_EXT_GET_MANUFACTURER_INFO:
			*type = TYPE_EXT_GET_MFR_INFO;
			break;
		case PD_EXT_MANUFACTURER_INFO:
			*type = TYPE_EXT_MFR_INFO;
			break;
		case PD_EXT_SECURITY_REQUEST:
		case PD_EXT_SECURITY_RESPONSE:
			/* not support*/
			pd3_type = 0;
			break;
		case PD_EXT_FW_UPDATE_REQUEST:
			*type = TYPE_EXT_PDFU_REQUEST;
			break;
		case PD_EXT_FW_UPDATE_RESPONSE:
			*type = TYPE_EXT_PDFU_RESPONSE;
			break;
		default:
			pd3_type = 0;
			break;
		}
	} else {
		switch (pd3_type) {
		case PD_DATA_BATTERY_STATUS:
			*type = TYPE_EXT_BATT_STS;
			break;
		case PD_DATA_ALERT:
			*type = TYPE_EXT_ALERT;
			break;

		case PD_CTRL_NOT_SUPPORTED:
			*type = TYPE_EXT_NOT_SUPPORTED;
			break;
		case PD_CTRL_GET_SOURCE_CAP_EXTENDED:
			*type = TYPE_EXT_GET_SRC_CAP;
			break;
		case PD_CTRL_GET_STATUS:
			*type = TYPE_EXT_GET_SRC_STS;
			break;
		case PD_CTRL_FR_SWAP:
			*type = TYPE_EXT_FR_SWAP;
			break;

		default:
			pd3_type = 0;
			break;
		}
	}
	if (pd3_type)
		return 0;
	else
		return -EPERM;
}


inline unsigned char cac_checksum(unsigned char *pSendBuf, unsigned char len)
{
	unsigned char i;
	unsigned char checksum;

	checksum = 0;
	for (i = 0; i < len; i++)
		checksum += *(pSendBuf + i);

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


unsigned char send_ext_msg(unsigned char is_ext, unsigned char type,
	unsigned char *pbuf, unsigned char buf_len, unsigned char type_sop)
{

	u8 temp_buf[40];

	if (ConvertPd3TypeToInterfaceType(is_ext, type, &type) < 0)
		return CMD_FAIL;

	temp_buf[0] = type_sop;
	memcpy(&temp_buf[1], pbuf, buf_len);

	return interface_send_msg_timeout(type, temp_buf, buf_len + 1,
			INTERFACE_TIMEOUT);

}

