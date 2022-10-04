/**
 * X8H7 CAN driver
 */

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>

#include "x8h7.h"

#define DRIVER_NAME "x8h7_can"

/* DEBUG HANDLING */
// #define DEBUG
#include "debug.h"
#ifdef DEBUG
  #define DBG_CAN_STATE(d, s) { \
    DBG_PRINT("device %s:CAN State: %d CAN controller %s\n", d, s, can_sts(s)); \
  }
#else
  #define DBG_CAN_STATE(i, s)
#endif

// Peripheral code
#define X8H7_CAN1_PERIPH  0x03
#define X8H7_CAN2_PERIPH  0x04
// Op code
#define X8H7_CAN_OC_CFG   0x10
#define X8H7_CAN_OC_SEND  0x01
#define X8H7_CAN_OC_RECV  0x01
#define X8H7_CAN_OC_STS   0x40
#define X8H7_CAN_OC_FLT   0x50

#define X8H7_CAN_STS_INT_TX      0x01
#define X8H7_CAN_STS_INT_RX      0x02
#define X8H7_CAN_STS_INT_ERR     0x04
/*
MERRE: Message Error
WAKIE: Wake-up Interrupt Enable
ERRIE: Error Interrupt Enable bit (multiple sources in EFLG register
TX2IE: Transmit Buffer 2 Empty
RX1IE: Receive Buffer 1 Full I      questo non serve
*/

#define X8H7_CAN_STS_FLG_RX_OVR  0x01  // Receive Buffer Overflow
#define X8H7_CAN_STS_FLG_TX_BO   0x02  // Bus-Off
#define X8H7_CAN_STS_FLG_TX_EP   0x04  // Transmit Error-Passive
#define X8H7_CAN_STS_FLG_RX_EP   0x08  // Receive Error-Passive
#define X8H7_CAN_STS_FLG_TX_WAR  0x10  // Transmit Error Warning
#define X8H7_CAN_STS_FLG_RX_WAR  0x20  // Receive Error Warning
#define X8H7_CAN_STS_FLG_EWARN   0x40  // Error Warning

#define CAN_FRAME_MAX_DATA_LEN	8
#define X8H7_CAN_SIZE   (5+CAN_FRAME_MAX_DATA_LEN)

#define AFTER_SUSPEND_UP      1
#define AFTER_SUSPEND_DOWN    2
#define AFTER_SUSPEND_POWER   4
#define AFTER_SUSPEND_RESTART 8

#define X8H7_FLT_EXT      0x80000000
#define X8H7_STD_FLT_MAX  128
#define X8H7_EXT_FLT_MAX   64

/**
 */
struct x8h7_can_filter {
  u32   id;
  u32   mask;
};

/**
 */
struct x8h7_can_priv {
  struct can_priv           can;
  struct net_device        *net;
  struct device            *dev;
  int                       periph;

  //wait_queue_head_t         wait;
  //int                       rx_cnt;
  //x8h7_pkt_t                rx_pkt;

  struct sk_buff           *tx_skb;
  int                       tx_len;

  struct workqueue_struct  *wq;
  struct work_struct        tx_work;
  struct work_struct        restart_work;

  int                       force_quit;
  int                       after_suspend;
  int                       restart_tx;

  struct x8h7_can_filter    std_flt[X8H7_STD_FLT_MAX];
  struct x8h7_can_filter    ext_flt[X8H7_EXT_FLT_MAX];
  //struct mutex        lock;
};

/**
 */
static const struct can_bittiming_const x8h7_can_bittiming_const = {
  .name      = DRIVER_NAME,   /* Name of the CAN controller hardware */
  .tseg1_min =  3,            /* Time segement 1 = prop_seg + phase_seg1 */
  .tseg1_max = 16,
  .tseg2_min =  2,            /* Time segement 2 = phase_seg2 */
  .tseg2_max =  8,
  .sjw_max   =  4,            /* Synchronisation jump width */
  .brp_min   =  1,            /* Bit-rate prescaler */
  .brp_max   = 64,
  .brp_inc   =  1,
};

//static void x8h7_can_hw_rx(struct x8h7_can_priv *priv);
static void x8h7_can_error_skb(struct net_device *net, int can_id, int data1);

/**
 */
static char* can_sts(enum can_state sts)
{
  switch(sts){
  case CAN_STATE_ERROR_ACTIVE : return "is error active";
  case CAN_STATE_ERROR_WARNING: return "is error active, warning level is reached";
  case CAN_STATE_ERROR_PASSIVE: return "is error passive";
  case CAN_STATE_BUS_OFF      : return "went into Bus Off";
  case CAN_STATE_STOPPED      : return "is in stopped mode";
  case CAN_STATE_SLEEPING     : return "is in Sleep mode";
  default                     : return "is unknown state";
  }
}

/**
 */
static void x8h7_can_status(struct x8h7_can_priv *priv, u8 intf, u8 eflag)
{
  struct net_device  *net = priv->net;
  enum can_state      new_state;
  int                 can_id = 0;
  int                 data1 = 0;

  DBG_PRINT("\n");
  /* Update can state */
  if (eflag & X8H7_CAN_STS_FLG_TX_BO) {
    new_state = CAN_STATE_BUS_OFF;
    can_id |= CAN_ERR_BUSOFF;
  } else if (eflag & X8H7_CAN_STS_FLG_TX_EP) {
    new_state = CAN_STATE_ERROR_PASSIVE;
    can_id |= CAN_ERR_CRTL;
    data1 |= CAN_ERR_CRTL_TX_PASSIVE;
  } else if (eflag & X8H7_CAN_STS_FLG_RX_EP) {
    new_state = CAN_STATE_ERROR_PASSIVE;
    can_id |= CAN_ERR_CRTL;
    data1 |= CAN_ERR_CRTL_RX_PASSIVE;
  } else if (eflag & X8H7_CAN_STS_FLG_TX_WAR) {
    new_state = CAN_STATE_ERROR_WARNING;
    can_id |= CAN_ERR_CRTL;
    data1 |= CAN_ERR_CRTL_TX_WARNING;
  } else if (eflag & X8H7_CAN_STS_FLG_RX_WAR) {
    new_state = CAN_STATE_ERROR_WARNING;
    can_id |= CAN_ERR_CRTL;
    data1 |= CAN_ERR_CRTL_RX_WARNING;
  } else {
    new_state = CAN_STATE_ERROR_ACTIVE;
  }

  /* Update can state statistics */
  switch (priv->can.state) {
  case CAN_STATE_ERROR_ACTIVE:
    if (new_state >= CAN_STATE_ERROR_WARNING &&
        new_state <= CAN_STATE_BUS_OFF) {
      priv->can.can_stats.error_warning++;
    }
    /* fall through */
  case CAN_STATE_ERROR_WARNING:
    if (new_state >= CAN_STATE_ERROR_PASSIVE &&
        new_state <= CAN_STATE_BUS_OFF) {
      priv->can.can_stats.error_passive++;
    }
    break;
  default:
    break;
  }
  priv->can.state = new_state;
  DBG_CAN_STATE(priv->net->name, priv->can.state);

  //if (intf & CANINTF_ERRIF) {
    /* Handle overflow counters */
    if (eflag & X8H7_CAN_STS_FLG_RX_OVR) {
      net->stats.rx_over_errors++;
      net->stats.rx_errors++;
      can_id |= CAN_ERR_CRTL;
      data1 |= CAN_ERR_CRTL_RX_OVERFLOW;
      x8h7_can_error_skb(net, can_id, data1);
    }
  //}

  if (priv->can.state == CAN_STATE_BUS_OFF) {
    if (priv->can.restart_ms == 0) {
      // @TODO: priv->force_quit = 1;
      priv->can.can_stats.bus_off++;
      can_bus_off(net);
      // @TODO: mcp251x_hw_sleep(spi);
      //break;
      return;
    }
  }

  if (intf & X8H7_CAN_STS_INT_TX) {
    net->stats.tx_packets++;
    net->stats.tx_bytes += priv->tx_len - 1;
    can_led_event(net, CAN_LED_EVENT_TX);
    if (priv->tx_len) {
      can_get_echo_skb(net, 0);
      priv->tx_len = 0;
    }
    netif_wake_queue(net);
  }
}

/**
 */
static void x8h7_can_hook(void *arg, x8h7_pkt_t *pkt)
{
  struct x8h7_can_priv  *priv = (struct x8h7_can_priv*)arg;

  switch(pkt->opcode) {
  case X8H7_CAN_OC_RECV:
    if (pkt->size < 5) {
      DBG_ERROR("received packed is too short (%d)\n", pkt->size);
      return;
    } else {
      struct sk_buff   *skb;
      struct can_frame *frame;

      skb = alloc_can_skb(priv->net, &frame);
      if (!skb) {
        dev_err(priv->dev, "cannot allocate RX skb\n");
        priv->net->stats.rx_dropped++;
        return;
      }
      frame->can_id = (pkt->data[3] << 24) | (pkt->data[2] << 16) |
                      (pkt->data[1] <<  8) | pkt->data[0];
      frame->can_dlc = get_can_dlc(pkt->data[4] & 0x0F);
      memcpy(frame->data, pkt->data + 5, frame->can_dlc);
      priv->net->stats.rx_packets++;
      priv->net->stats.rx_bytes += frame->can_dlc;
      can_led_event(priv->net, CAN_LED_EVENT_RX);
      netif_rx_ni(skb);
    }
    break;
  case X8H7_CAN_OC_STS:
    DBG_PRINT("received status %02X %02X\n", pkt->data[0], pkt->data[1]);
    x8h7_can_status(priv, pkt->data[0], pkt->data[1]);
    break;
  }
}

/**
 */
/*
static int x8h7_can_pkt_get(struct x8h7_can_priv *priv)
{
  long ret;

  ret = wait_event_interruptible_timeout(priv->wait,
                                         priv->rx_cnt != 0,
                                         X8H7_RX_TIMEOUT);
  if (!ret) {
    DBG_ERROR("timeout expired");
    return -1;
  }
  priv->rx_cnt--;
  return 0;
}
*/

/**
 */
static int x8h7_can_hw_reset(struct x8h7_can_priv *priv)
{
  DBG_PRINT("\n");
/*
  unsigned long timeout;
  int ret;

  // Wait for oscillator startup timer after power up
  mdelay(MCP251X_OST_DELAY_MS);

  priv->spi_tx_buf[0] = INSTRUCTION_RESET;
  ret = mcp251x_spi_trans(spi, 1);
  if (ret)
    return ret;

  // Wait for oscillator startup timer after reset
  mdelay(MCP251X_OST_DELAY_MS);

  // Wait for reset to finish
  timeout = jiffies + HZ;
  while ((mcp251x_read_reg(spi, CANSTAT) & CANCTRL_REQOP_MASK) !=
        CANCTRL_REQOP_CONF) {
    usleep_range(MCP251X_OST_DELAY_MS * 1000,
          MCP251X_OST_DELAY_MS * 1000 * 2);

    if (time_after(jiffies, timeout)) {
      dev_err(&spi->dev,
        "MCP251x didn't enter in conf mode after reset\n");
      return -EBUSY;
    }
  }
*/
  return 0;
}

/**
 */
static void x8h7_can_clean(struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);

  DBG_PRINT("\n");
  if (priv->tx_skb || priv->tx_len) {
    net->stats.tx_errors++;
  }
  dev_kfree_skb(priv->tx_skb);
  if (priv->tx_len) {
    can_free_echo_skb(priv->net, 0);
  }
  priv->tx_skb = NULL;
  priv->tx_len = 0;
}

/**
 */
static int x8h7_can_setup(struct x8h7_can_priv *priv)
{
  struct can_bittiming *bt = &priv->can.bittiming;
  uint32_t              frequency_requested;

  DBG_PRINT("sjw: %d, brp: %d, phase_seg1: %d, prop_seg: %d, phase_seg2: %d, freq: %d ctrlmode: %08X\n",
            bt->sjw,
            bt->brp,
            bt->phase_seg1,
            bt->prop_seg,
            bt->phase_seg2,
            priv->can.clock.freq,
            priv->can.ctrlmode);

  // Reconstruct frequency since the lower level API accepts the "raw" bitrate
  frequency_requested = (priv->can.clock.freq / bt->brp) /
                    (bt->sjw + bt->phase_seg1 + bt->prop_seg + bt->phase_seg2);

  x8h7_pkt_enq(priv->periph, X8H7_CAN_OC_CFG, sizeof(frequency_requested), &(frequency_requested));
  x8h7_pkt_send();

  return 0;
}

/**
 */
static int x8h7_can_set_normal_mode(struct x8h7_can_priv *priv)
{
//  unsigned long  timeout;

  DBG_PRINT("\n");
  /* Enable interrupts */
  x8h7_hook_set(priv->periph, x8h7_can_hook, priv);

  if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
    /* Put device into loopback mode */
    DBG_PRINT("Put device into loopback mode\n");
  } else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
    /* Put device into listen-only mode */
    DBG_PRINT("Put device into listen-only mode\n");
  } else {
    /* Put device into normal mode */
    DBG_PRINT("Put device into normal mode. Can wait for the device to enter normal mode\n");

    //mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_NORMAL);

    /* Wait for the device to enter normal mode */
    /*timeout = jiffies + HZ;
    while (mcp251x_read_reg(spi, CANSTAT) & CANCTRL_REQOP_MASK) {
      schedule();
      if (time_after(jiffies, timeout)) {
        dev_err(&spi->dev, "MCP251x didn't enter in normal mode\n");
        return -EBUSY;
      }
    }*/
  }
  priv->can.state = CAN_STATE_ERROR_ACTIVE;
  return 0;
}

/**
 */
static void x8h7_can_error_skb(struct net_device *net, int can_id, int data1)
{
  struct sk_buff *skb;
  struct can_frame *frame;

  DBG_PRINT("\n");
  skb = alloc_can_err_skb(net, &frame);
  if (skb) {
    frame->can_id |= can_id;
    frame->data[1] = data1;
    netif_rx_ni(skb);
  } else {
    netdev_err(net, "cannot allocate error skb\n");
  }
}

/**
 */
#if 0
static void x8h7_can_hw_rx(struct x8h7_can_priv *priv)
{
  struct sk_buff   *skb;
  struct can_frame *frame;
  uint8_t          *data = priv->rx_pkt.data;

  skb = alloc_can_skb(priv->net, &frame);
  if (!skb) {
    dev_err(priv->dev, "cannot allocate RX skb\n");
    priv->net->stats.rx_dropped++;
    return;
  }

  /*
   * Controller Area Network Identifier structure
   *
   * bit 0-28 : CAN identifier (11/29 bit)
   * bit 29   : error message frame flag (0 = data frame, 1 = error message)
   * bit 30   : remote transmission request flag (1 = rtr frame)
   * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
   *
   * CAN_EFF_FLAG 0x80000000U   EFF/SFF is set in the MSB
   * CAN_RTR_FLAG 0x40000000U   remote transmission request
   * CAN_ERR_FLAG 0x20000000U   error message frame
   */
  if (data[0] & 0x80) {
    /* Extended ID format */
    frame->can_id = CAN_EFF_FLAG;
    frame->can_id |= ((data[0] & 0x1F) << 24) |
                     (data[1] << 16) | (data[2] << 8) | data[3];
  } else {
    /* Standard ID format */
    frame->can_id = ((data[2] & 0x01) << 8) | data[3];
  }
  /* Remote transmission request */
  if (data[0] & 0x40) {
    frame->can_id |= CAN_RTR_FLAG;
  }

  /* Data length */
  frame->can_dlc = get_can_dlc(data[4] & 0x0F);
  memcpy(frame->data, data + 5, frame->can_dlc);

  priv->net->stats.rx_packets++;
  priv->net->stats.rx_bytes += frame->can_dlc;

  can_led_event(priv->net, CAN_LED_EVENT_RX);

  netif_rx_ni(skb);
}
#endif

/**
 */
static void x8h7_can_hw_tx(struct x8h7_can_priv *priv, struct can_frame *frame)
{
/*
  u32 exide;      // Extended ID Enable
  u32 sid;        // Standard ID
  u32 eid;        // Extended ID
  u32 rtr;        // Remote transmission
*/
  u8  data[X8H7_CAN_SIZE];

  DBG_PRINT("\n");
/*
  exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0;
  if (exide) {
    sid = (frame->can_id & CAN_EFF_MASK) >> 18;
  } else {
    sid = frame->can_id & CAN_SFF_MASK;
  }
  eid = frame->can_id & CAN_EFF_MASK;
  rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;
*/

  data[0] = frame->can_id;
  data[1] = frame->can_id >> 8;
  data[2] = frame->can_id >> 16;
  data[3] = frame->can_id >> 24;
  data[4] = frame->can_dlc;
  memcpy(data + 5, frame->data, frame->can_dlc);

if (1) {
  char  str[1024];
  int   len;
  int   i;

  len = 0;
  for (i=0; i<5+frame->can_dlc; i++) {
    len += sprintf(str, " %02X", data[i]);
  }
  DBG_PRINT("Send %s to H7\n", str);
}

  x8h7_pkt_enq(priv->periph, X8H7_CAN_OC_SEND, 5+frame->can_dlc, data);
  x8h7_pkt_send();
}

/**
 */
static void x8h7_can_tx_work_handler(struct work_struct *ws)
{
  struct x8h7_can_priv  *priv = container_of(ws, struct x8h7_can_priv, tx_work);
  struct net_device     *net = priv->net;
  struct can_frame      *frame;

  DBG_PRINT("\n");
  //mutex_lock(&priv->mcp_lock);
  if (priv->tx_skb) {
    if (priv->can.state == CAN_STATE_BUS_OFF) {
      DBG_PRINT("CAN_STATE_BUS_OFF\n");
      x8h7_can_clean(net);
    } else {
      DBG_PRINT("Send frame\n");
      frame = (struct can_frame *)priv->tx_skb->data;

      if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN) {
        frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;
      }

      x8h7_can_hw_tx(priv, frame);

      priv->tx_len = 1 + frame->can_dlc;
      can_put_echo_skb(priv->tx_skb, net, 0);
      priv->tx_skb = NULL;
    }
  }
  //mutex_unlock(&priv->mcp_lock);
}

/**
 */
static void x8h7_can_restart_work_handler(struct work_struct *ws)
{
  struct x8h7_can_priv *priv = container_of(ws, struct x8h7_can_priv, restart_work);
  struct net_device    *net = priv->net;

  DBG_PRINT("\n");
  //mutex_lock(&priv->mcp_lock);
  if (priv->after_suspend) {
    x8h7_can_hw_reset(priv);
    x8h7_can_setup(priv);
    priv->force_quit = 0;
    if (priv->after_suspend & AFTER_SUSPEND_RESTART) {
      DBG_PRINT("AFTER_SUSPEND_RESTART\n");
      x8h7_can_set_normal_mode(priv);
    } else if (priv->after_suspend & AFTER_SUSPEND_UP) {
      DBG_PRINT("AFTER_SUSPEND_UP\n");
      netif_device_attach(net);
      x8h7_can_clean(net);
      x8h7_can_set_normal_mode(priv);
      netif_wake_queue(net);
    } else {
      //mcp251x_hw_sleep(spi); @TODO:
    }
    priv->after_suspend = 0;
  }

  if (priv->restart_tx) {
    DBG_PRINT("restart_tx\n");
    priv->restart_tx = 0;
    //mcp251x_write_reg(spi, TXBCTRL(0), 0); @TODO: cosa fa?
    x8h7_can_clean(net);
    netif_wake_queue(net);
    x8h7_can_error_skb(net, CAN_ERR_RESTARTED, 0);
  }
  //mutex_unlock(&priv->mcp_lock);
}

/**
 */
static int x8h7_can_open(struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);
  int                   ret;

  DBG_PRINT("\n");

  ret = open_candev(net);
  if (ret) {
    DBG_ERROR("unable to set initial baudrate!\n");
    return ret;
  }

  priv->force_quit = 0;
  priv->tx_skb     = NULL;
  priv->tx_len     = 0;

  priv->wq = alloc_workqueue("x8h7_can_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
  if (!priv->wq) {
    ret = -ENOMEM;
    goto out_clean;
  }
  INIT_WORK(&priv->tx_work, x8h7_can_tx_work_handler);
  INIT_WORK(&priv->restart_work, x8h7_can_restart_work_handler);

  ret = x8h7_can_hw_reset(priv);
  if (ret) {
    goto out_free_wq;
  }
  ret = x8h7_can_setup(priv);
  if (ret) {
    goto out_free_wq;
  }
  ret = x8h7_can_set_normal_mode(priv);
  if (ret) {
    goto out_free_wq;
  }

  can_led_event(net, CAN_LED_EVENT_OPEN);

  netif_wake_queue(net);

  return 0;

out_free_wq:
  destroy_workqueue(priv->wq);
out_clean:
  x8h7_hook_set(priv->periph, NULL, NULL);
//out_close:
  close_candev(net);
  return ret;
}

/**
 */
static int x8h7_can_stop(struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);

  DBG_PRINT("\n");

  close_candev(net);
  priv->force_quit = 1;
  x8h7_hook_set(priv->periph, NULL, NULL);
  destroy_workqueue(priv->wq);
  priv->wq = NULL;

  priv->can.state = CAN_STATE_STOPPED;
  can_led_event(net, CAN_LED_EVENT_STOP);

  return 0;
}

/**
 */
static netdev_tx_t x8h7_can_start_xmit(struct sk_buff *skb,
                                       struct net_device *net)
{
  struct x8h7_can_priv *priv = netdev_priv(net);

  DBG_PRINT("\n");

  if (priv->tx_skb || priv->tx_len) {
    DBG_ERROR("hard_xmit called while tx busy\n");
    return NETDEV_TX_BUSY;
  }

  if (can_dropped_invalid_skb(net, skb)) {
    return NETDEV_TX_OK;
  }

  netif_stop_queue(net);
  priv->tx_skb = skb;
  queue_work(priv->wq, &priv->tx_work);

  return NETDEV_TX_OK;
}

/**
 */
static int x8h7_can_do_set_mode(struct net_device *net, enum can_mode mode)
{
  struct x8h7_can_priv *priv = netdev_priv(net);
  DBG_PRINT("\n");

  switch (mode) {
  case CAN_MODE_START:
    x8h7_can_clean(net);
    /* We have to delay work since SPI I/O may sleep */
    priv->can.state = CAN_STATE_ERROR_ACTIVE;
    priv->restart_tx = 1;
    if (priv->can.restart_ms == 0) {
      priv->after_suspend = AFTER_SUSPEND_RESTART;
    }
    queue_work(priv->wq, &priv->restart_work);
    break;
  default:
    return -EOPNOTSUPP;
  }

  return 0;
}

/**
 */
static int x8h7_can_do_get_berr_counter(const struct net_device *net,
                                        struct can_berr_counter *bec)
{
  //struct x8h7_can_priv *priv = netdev_priv(net);
//@TODO: to be read from device
  bec->txerr = 0;
  bec->rxerr = 0;

  return 0;
}

/**
 */
static const struct net_device_ops x8h7_can_netdev_ops = {
  .ndo_open       = x8h7_can_open,
  .ndo_stop       = x8h7_can_stop,
  .ndo_start_xmit = x8h7_can_start_xmit,
  //.ndo_change_mtu = can_change_mtu,
};

/**
 */
static int x8h7_can_filter(struct x8h7_can_priv *priv,
                           const char *buf, int type)
{
  u32   idx;
  u32   id;
  u32   mask;
  int   ret;
  u32   data[3];

  ret = sscanf(buf, "%x %x %x", &idx, &id, &mask);
  if (ret != 3) {
    DBG_ERROR("invalid num of params\n");
    return -1;
  }

  if (type == 0) {
    if ((idx >= X8H7_STD_FLT_MAX) ||
        (id & ~0x7FF) || (mask & ~0x7FF)) {
      DBG_ERROR("invalid params\n");
      return -1;
    }
    priv->std_flt[idx].id   = id;
    priv->std_flt[idx].mask = mask;
  } else {
    if ((idx >= X8H7_EXT_FLT_MAX) ||
        (id & ~0x1FFFFFFF) || (mask & ~0x1FFFFFFF)) {
      DBG_ERROR("invalid params\n");
      return -1;
    }
    priv->ext_flt[idx].id   = id;
    priv->ext_flt[idx].mask = mask;
    idx |= X8H7_FLT_EXT;
  }

  DBG_PRINT("SEND idx %X, id %X, mask %X\n", idx, id, mask);

  data[0] = idx;
  data[1] = id;
  data[2] = mask;
  x8h7_pkt_enq(priv->periph, X8H7_CAN_OC_FLT, sizeof(data), data);
  x8h7_pkt_send();

  return 0;
}

/**
 * Standard id filter show
 */
static ssize_t x8h7_can_sf_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   len;
  int                   i;

  len = 0;
  for (i=0; i<X8H7_STD_FLT_MAX; i++) {
    if (priv->std_flt[i].mask) {
      len += snprintf(buf + len, PAGE_SIZE - len,
                      "%02X %08X %08X\n",
                      i, priv->std_flt[i].id, priv->std_flt[i].mask);
    }
  }
  return len;
}

/**
 * Standard id filter set
 */
static ssize_t x8h7_can_sf_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   ret;

  ret = x8h7_can_filter(priv, buf, 0);
  if (ret) {
    DBG_ERROR("set filter\n");
    return -1;
  }
  return count;
}

/**
 * Extended id filter show
 */
static ssize_t x8h7_can_ef_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   len;
  int                   i;

  len = 0;
  for (i=0; i<X8H7_STD_FLT_MAX; i++) {
    if (priv->ext_flt[i].mask) {
      len += snprintf(buf + len, PAGE_SIZE - len,
                      "%02X %08X %08X\n",
                      i, priv->ext_flt[i].id, priv->ext_flt[i].mask);
    }
  }
  return len;
}

/**
 * Extended id filter set
 */
static ssize_t x8h7_can_ef_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   ret;

  ret = x8h7_can_filter(priv, buf, 1);
  if (ret) {
    DBG_ERROR("set filter\n");
    return -1;
  }
  return count;
}

/**
 * Status show
 */
static ssize_t x8h7_can_sts_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
  struct x8h7_can_priv *priv = netdev_priv(to_net_dev(dev));
  int                   len;

  len = snprintf(buf, PAGE_SIZE,
                  "status         %d %s\n"
                  "error warning  %d\n"
                  "error passive  %d\n"
                  "bus off        %d\n"
                  "tx packets     %ld\n"
                  "tx bytes       %ld\n"
                  "rx packets     %ld\n"
                  "rx bytes       %ld\n"
                  "rx over_errors %ld\n"
                  "rx errors      %ld\n"
                  "rx dropped     %ld\n",
                  priv->can.state, can_sts(priv->can.state),
                  priv->can.can_stats.error_warning,
                  priv->can.can_stats.error_passive,
                  priv->can.can_stats.bus_off,
                  priv->net->stats.tx_packets,
                  priv->net->stats.tx_bytes,
                  priv->net->stats.rx_packets,
                  priv->net->stats.rx_bytes,
                  priv->net->stats.rx_over_errors,
                  priv->net->stats.rx_errors,
                  priv->net->stats.rx_dropped);
  return len;
}

static DEVICE_ATTR(std_flt, 0644, x8h7_can_sf_show, x8h7_can_sf_store);
static DEVICE_ATTR(ext_flt, 0644, x8h7_can_ef_show, x8h7_can_ef_store);
static DEVICE_ATTR(status , 0644, x8h7_can_sts_show, NULL);

static struct attribute *x8h7_can_sysfs_attrs[] = {
  &dev_attr_std_flt.attr,
  &dev_attr_ext_flt.attr,
  &dev_attr_status.attr,
  //....
  NULL,
};

static const struct attribute_group x8h7_can_sysfs_attr_group = {
  .name = "x8h7can",
  .attrs = (struct attribute **)x8h7_can_sysfs_attrs,
};

/**
 */
static int x8h7_can_probe(struct platform_device *pdev)
{
  struct net_device    *net;
  struct x8h7_can_priv *priv;
  int                   err;

  u32 clock_freq = 100000000;

  DBG_PRINT("\n");

  if (pdev->dev.of_node) {
    of_property_read_u32(pdev->dev.of_node, "clock-frequency", &clock_freq);
  }

//  init_waitqueue_head(&priv->wait);

  net = alloc_candev(sizeof(struct x8h7_can_priv), 1);
  if (!net) {
    return -ENOMEM;
  }

  net->netdev_ops = &x8h7_can_netdev_ops;
  net->flags |= IFF_ECHO;
  net->sysfs_groups[0] = &x8h7_can_sysfs_attr_group;

  priv = netdev_priv(net);

  priv->can.clock.freq          = clock_freq;
  priv->can.bittiming_const     = &x8h7_can_bittiming_const;
  priv->can.do_set_mode         = x8h7_can_do_set_mode;
  priv->can.do_get_berr_counter = x8h7_can_do_get_berr_counter;
  priv->can.ctrlmode_supported  = CAN_CTRLMODE_LOOPBACK      |
                                  CAN_CTRLMODE_LISTENONLY    |
                                  CAN_CTRLMODE_3_SAMPLES     ;/*|
                                  CAN_CTRLMODE_BERR_REPORTING;*/
  priv->net = net;
  //priv->clk = clk;

  platform_set_drvdata(pdev, priv);

  SET_NETDEV_DEV(net, &pdev->dev);

  err = register_candev(net);
  if (err) {
    dev_err(&pdev->dev, "registering netdev failed\n");
    goto failed_register;
  }
  DBG_PRINT("net device registered %s, "
            "ifindex: %d, if_port %d, dev_id: %d, dev_port %d\n",
            net->name, net->ifindex, net->if_port, net->dev_id, net ->dev_port);

  devm_can_led_init(net);

  if (net->name[3] == '0') {
    priv->periph = X8H7_CAN1_PERIPH;
  } else {
    priv->periph = X8H7_CAN2_PERIPH;
  }
  priv->dev = &pdev->dev;
  DBG_PRINT("periph: %d DONE\n", priv->periph);

  netdev_info(net, "X8H7 CAN successfully initialized.\n");

  return 0;

failed_register:
  DBG_ERROR("\n");
  free_candev(net);
  return err;
}

/**
 */
static int x8h7_can_remove(struct platform_device *pdev)
{
  struct x8h7_can_priv *priv = platform_get_drvdata(pdev);
  struct net_device    *net = priv->net;

  unregister_candev(net);
  free_candev(net);

  return 0;
}

/**
 */
static const struct of_device_id x8h7_can_of_match[] = {
  { .compatible = "portenta,x8h7_can", },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, x8h7_can_of_match);

/**
 */
static const struct platform_device_id x8h7_can_id_table[] = {
  { .name = "x8h7_can", },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, x8h7_can_id_table);

/**
 */
static struct platform_driver x8h7_can_driver = {
  .driver = {
    .name           = DRIVER_NAME,
    //.pm             = &x8h7_can_pm_ops,
    .of_match_table = x8h7_can_of_match,
  },
  .probe    = x8h7_can_probe,
  .remove   = x8h7_can_remove,
  .id_table = x8h7_can_id_table,
};

module_platform_driver(x8h7_can_driver);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_DESCRIPTION("Arduino Portenta X8 CAN driver");
MODULE_LICENSE("GPL v2");
