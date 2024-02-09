/**
 * X8H7 CAN driver
 */

#ifndef __X8H7_CAN_H
#define __X8H7_CAN_H

/**
 * INCLUDES
 */

#include <linux/timer.h>
#include <linux/can/dev.h>
#include <linux/workqueue.h>

/**
 * DEFINES
 */
#define X8H7_CAN1_PERIPH    0x03
#define X8H7_CAN2_PERIPH    0x04

#define X8H7_CAN_OC_INIT    0x10
#define X8H7_CAN_OC_DEINIT  0x11
#define X8H7_CAN_OC_BITTIM  0x12
#define X8H7_CAN_OC_SEND    0x01
#define X8H7_CAN_OC_RECV    0x01
#define X8H7_CAN_OC_STS     0x40
#define X8H7_CAN_OC_FLT     0x50

#define X8H7_CAN_STS_INT_TX      0x01
#define X8H7_CAN_STS_INT_RX      0x02
#define X8H7_CAN_STS_INT_ERR     0x04

#define X8H7_CAN_STS_FLG_RX_OVR  0x01  // Receive Buffer Overflow
#define X8H7_CAN_STS_FLG_TX_BO   0x02  // Bus-Off
#define X8H7_CAN_STS_FLG_TX_EP   0x04  // Transmit Error-Passive
#define X8H7_CAN_STS_FLG_RX_EP   0x08  // Receive Error-Passive
#define X8H7_CAN_STS_FLG_TX_WAR  0x10  // Transmit Error Warning
#define X8H7_CAN_STS_FLG_RX_WAR  0x20  // Receive Error Warning
#define X8H7_CAN_STS_FLG_EWARN   0x40  // Error Warning
#define X8H7_CAN_STS_FLG_TX_OVR  0x80  // Transmit Buffer Overflow

#define X8H7_CAN_HEADER_SIZE        5
#define X8H7_CAN_FRAME_MAX_DATA_LEN 8

#define X8H7_STD_FLT_MAX  128
#define X8H7_EXT_FLT_MAX   64

#define X8H7_TX_FIFO_SIZE  32

/**
 * TYPEDEFS
 */
union x8h7_can_init_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
  } field;
  uint8_t buf[sizeof(uint32_t) /* can_bitrate_Hz */ + sizeof(uint32_t) /* time_segment_1 */ + sizeof(uint32_t) /* time_segment_2 */ + sizeof(uint32_t) /* sync_jump_width */];
};

union x8h7_can_bittiming_message
{
  struct __attribute__((packed))
  {
    uint32_t baud_rate_prescaler;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
    uint32_t sync_jump_width;
  } field;
  uint8_t buf[sizeof(uint32_t) /* can_bitrate_Hz */ + sizeof(uint32_t) /* time_segment_1 */ + sizeof(uint32_t) /* time_segment_2 */ + sizeof(uint32_t) /* sync_jump_width */];
};

union x8h7_can_filter_message
{
  struct __attribute__((packed))
  {
    uint32_t idx;
    uint32_t id;
    uint32_t mask;
  } field;
  uint8_t buf[sizeof(uint32_t) /* idx */ + sizeof(uint32_t) /* id */ + sizeof(uint32_t) /* mask */];
};

union x8h7_can_frame_message
{
  struct __attribute__((packed))
  {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[X8H7_CAN_FRAME_MAX_DATA_LEN];
  } field;
  uint8_t buf[X8H7_CAN_HEADER_SIZE + X8H7_CAN_FRAME_MAX_DATA_LEN];
};

struct x8h7_can_frame_message_tx_obj_buf
{
  uint8_t                      head;
  uint8_t                      tail;
  uint8_t                      num_elems;
  union x8h7_can_frame_message data[X8H7_TX_FIFO_SIZE];
};

struct x8h7_can_priv {
  struct can_priv           can;
  struct net_device        *net;
  struct device            *dev;
  int                       periph;

  struct x8h7_can_frame_message_tx_obj_buf tx_obj_buf;

  int                       tx_len;

  struct workqueue_struct  *wq;
  struct work_struct        work;

  struct can_filter         std_flt[X8H7_STD_FLT_MAX];
  struct can_filter         ext_flt[X8H7_EXT_FLT_MAX];

  struct mutex              lock;
};

#endif
