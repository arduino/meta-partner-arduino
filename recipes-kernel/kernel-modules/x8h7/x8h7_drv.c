// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * Portenta X8
 * X8 H7 communication protocol
 * How it works:
 * This driver has two entry points:
 * 1) From userspace spidev char driver
 * 2) From kernel space peripheral subdrivers (/dev/adc, /dev/pwm, etc) calls that can invoke exported functions x8h7_pkt_enq x8h7_pkt_send x8h7_hook_set
 * Data transmission on SPI can be triggered by following sources:
 * a) From this module (accessing previous entry points) to h7
 * b) After reception of interrupt from h7
 *
 * Based on:
 *  Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *    Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007
 *    David Brownell (simplification, cleanup)
 * Copyright (C) 2021 X8H7
 *    "Massimiliano Agneni <massimiliano@iptronix.com>"
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#include "x8h7.h"

#define DRIVER_NAME     "x8h7"
#define X8H7_BUF_SIZE   (64*1024)

//#define DEBUG
#include "debug.h"


struct spidev_data {
  struct spi_device  *spi;
  struct mutex        buf_lock;
  u32                 speed_hz;
  u8                 *x8h7_txb;
  u16                 x8h7_txl;
  u8                 *x8h7_rxb;
};


static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*-------------------------------------------------------------------------*/

struct spidev_data  *x8h7_spidev;

/**
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint16_t      size;
  uint16_t      checksum;
} x8h7_pkthdr_t;

/**
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint8_t   peripheral;
  uint8_t   opcode;
  uint16_t  size;
} x8h7_subpkt_t;

#define X8H7_PERIPH_NUM   16
x8h7_hook_t x8h7_hook[X8H7_PERIPH_NUM] = {};
void *x8h7_hook_priv[X8H7_PERIPH_NUM];

void (*x8h7_dbg)(void*, uint8_t*, uint16_t);
void *x8h7_dbg_priv;

/**
 */
#if defined(DEBUG)

#define X8H7_PERIPH_ADC     0x01
#define X8H7_PERIPH_PWM     0x02
#define X8H7_PERIPH_FDCAN1  0x03
#define X8H7_PERIPH_FDCAN2  0x04
#define X8H7_PERIPH_UART    0x05
#define X8H7_PERIPH_RTC     0x06
#define X8H7_PERIPH_GPIO    0x07
#define X8H7_PERIPH_H7      0x09
#define X8H7_PERIPH_UI      0x0A

/**
 */
char* to_peripheral_string(uint8_t peripheral) {
  switch (peripheral) {
  case X8H7_PERIPH_H7    : return "H7";
  case X8H7_PERIPH_ADC   : return "ADC";
  case X8H7_PERIPH_PWM   : return "PWM";
  case X8H7_PERIPH_FDCAN1: return "FDCAN1";
  case X8H7_PERIPH_FDCAN2: return "FDCAN2";
  case X8H7_PERIPH_UART  : return "UART";
  case X8H7_PERIPH_RTC   : return "RTC";
  case X8H7_PERIPH_GPIO  : return "GPIO";
  case X8H7_PERIPH_UI    : return "UI";
  default                : return "UNKNOWN";
  }
}

/**
 */
void pkt_dump(char *title, void *data)
{
  x8h7_pkthdr_t  *hdr;
  x8h7_subpkt_t  *pkt;
  uint8_t        *ptr;
  uint16_t        len;
  char            data_str[8192];
  int             data_len;
  int             i;
  int             err;

  ptr = data;
  hdr = (x8h7_pkthdr_t*)ptr;
  err = (hdr->size != 0) && ((hdr->size ^ 0x5555) != hdr->checksum);
  printk("%s: Header size %d %04X, checksum %04X %s\n",
         title,
         hdr->size, hdr->size, hdr->checksum,
         err ? "ERROR" : "OK");
  if (err) {
    return;
  }
  len = hdr->size;
  ptr += sizeof(x8h7_pkthdr_t);
  while (len > 0) {
    pkt = (x8h7_subpkt_t*)ptr;
    ptr += sizeof(x8h7_subpkt_t);

    data_len = 0;
    data_str[0] = 0;
    for (i=0; i<pkt->size; i++) {
      data_len += sprintf(data_str + data_len, " %02X", ptr[i]);
    }
    printk("- PKT peripheral: %d %s, opcode: %d, size: %d data: %s\n",
           pkt->peripheral, to_peripheral_string(pkt->peripheral),
           pkt->opcode, pkt->size, data_str);

    ptr += pkt->size;
    len -= (sizeof(x8h7_subpkt_t) + pkt->size);
  };
}
#else
void pkt_dump(char *title, void *data) {}
#endif

/**
 */
int x8h7_pkt_enq(uint8_t peripheral, uint8_t opcode, uint16_t size, void *data)
{
  struct spidev_data *spidev = x8h7_spidev;
  x8h7_pkthdr_t      *hdr;
  x8h7_subpkt_t      *pkt;
  uint8_t            *ptr;

  mutex_lock(&spidev->buf_lock);

  ptr = spidev->x8h7_txb;
  hdr = (x8h7_pkthdr_t*)ptr;

  if ((hdr->size + sizeof(x8h7_subpkt_t) + size) < X8H7_BUF_SIZE) {
    ptr += sizeof(x8h7_pkthdr_t) + hdr->size;
    pkt = (x8h7_subpkt_t*)ptr;
    pkt->peripheral = peripheral;
    pkt->opcode     = opcode;
    pkt->size       = size;
    ptr += sizeof(x8h7_subpkt_t);
    if (size) {
      if (!data) {
        memset(ptr, 0, size);
      } else {
        memcpy(ptr, data, size);
      }
    }
    hdr->size += sizeof(x8h7_subpkt_t) + size;
    hdr->checksum = hdr->size ^ 0x5555;
    spidev->x8h7_txl = hdr->size;
    mutex_unlock(&spidev->buf_lock);
    return 0;
  }

  mutex_unlock(&spidev->buf_lock);

  return -1;
}
EXPORT_SYMBOL_GPL(x8h7_pkt_enq);

/**
 * Function to parse data coming from h7
 * and dispatch to peripheral
 */
static int pkt_parse(struct spidev_data *spidev)
{
  x8h7_pkthdr_t  *hdr;
  x8h7_subpkt_t  *pkt;
  uint8_t        *ptr;
  uint16_t        size;
  int             i;

  pkt_dump("Parse", spidev->x8h7_rxb);

  ptr = spidev->x8h7_rxb;
  hdr = (x8h7_pkthdr_t*)ptr;
  size = hdr->size;
  ptr += sizeof(x8h7_pkthdr_t);

  /* Loop to parse data from h7 and dispatch to correct peripheral */
  do {
    pkt = (x8h7_subpkt_t*)ptr;
    ptr += sizeof(x8h7_subpkt_t);

    i = pkt->peripheral;
    if (i < X8H7_PERIPH_NUM) {
      if (x8h7_hook[i]) {
        x8h7_pkt_t p;
        p.peripheral = pkt->peripheral;
        p.opcode     = pkt->opcode;
        p.size       = pkt->size;
        if (p.size > X8H7_PKT_SIZE) {
          DBG_ERROR("packet size is %d\n", pkt->size);
          p.size = X8H7_PKT_SIZE;
        }
        memcpy(p.data, ptr, p.size);
        x8h7_hook[i](x8h7_hook_priv[i], &p);
      }
    }

    ptr += pkt->size;
    size -= (sizeof(x8h7_subpkt_t) + pkt->size);
  } while (size > 0);

  return 0;
}

/**
 */
int x8h7_spi_trx(struct spi_device *spi,
                 void *tx_buf, void* rx_buf, unsigned len)
{
  struct spi_transfer   t = {};
  struct spi_message    m;
  int                   ret;

  t.tx_buf = tx_buf;
  t.rx_buf = rx_buf;
  t.len    = len;
  t.speed_hz = x8h7_spidev->speed_hz;

  spi_message_init(&m);
  spi_message_add_tail(&t, &m);

  ret = spi_sync(spi, &m);
  if (ret) {
    DBG_ERROR("spi transfer failed: ret = %d\n", ret);
  }
  return ret;
}

/**
 * Function to send/receive physically data over SPI,
 * moreover in this function we process received data
 * and dispatch to corresponding peripheral
 * @TODO: remove arg?
 */
static inline int x8h7_pkt_send_priv(int arg)
{
  struct spidev_data   *spidev = x8h7_spidev;
  x8h7_pkthdr_t        *hdr;
  int                   len;

  DBG_PRINT("Send %d bytes\n", spidev->x8h7_txl);
  mutex_lock(&spidev->buf_lock);

  x8h7_spi_trx(spidev->spi,
               spidev->x8h7_txb, spidev->x8h7_rxb, sizeof(x8h7_pkthdr_t));

  hdr = (x8h7_pkthdr_t*)spidev->x8h7_rxb;
  if ((hdr->size != 0) && ((hdr->size ^ 0x5555) != hdr->checksum)) {
    DBG_ERROR("Out of sync %x %x\n", hdr->size, hdr->checksum);
    mutex_unlock(&spidev->buf_lock);
    return -1;
  }

  len = max(hdr->size, spidev->x8h7_txl);
  if (len == 0) {
    DBG_ERROR("Transaction length is zero\n");
    x8h7_spi_trx(spidev->spi,
                 spidev->x8h7_txb + sizeof(x8h7_pkthdr_t), spidev->x8h7_rxb,
                 sizeof(x8h7_pkthdr_t));
    mutex_unlock(&spidev->buf_lock);
    return 0;
  }

  pkt_dump("Send", spidev->x8h7_txb);

  x8h7_spi_trx(spidev->spi,
               spidev->x8h7_txb + sizeof(x8h7_pkthdr_t),
               spidev->x8h7_rxb + sizeof(x8h7_pkthdr_t), len);

  hdr = (x8h7_pkthdr_t*)spidev->x8h7_rxb;
  // @TODO: Add control
  if (hdr->size) {
    if (x8h7_dbg) {
      x8h7_dbg(x8h7_dbg_priv, spidev->x8h7_rxb, hdr->size);
    } else {
      pkt_parse(spidev);
    }
  }

  memset(spidev->x8h7_txb, 0, X8H7_BUF_SIZE);
  spidev->x8h7_txl = 0;

  mutex_unlock(&spidev->buf_lock);
  return 0;
}

/**
 */
int x8h7_pkt_send(void)
{
  return x8h7_pkt_send_priv(0);
}
EXPORT_SYMBOL_GPL(x8h7_pkt_send);

/**
 */
int x8h7_hook_set(uint8_t idx, x8h7_hook_t hook, void *priv)
{
  if (idx >= X8H7_PERIPH_NUM) {
    return -1;
  }
  x8h7_hook[idx] = hook;
  x8h7_hook_priv[idx] = priv;
  return 0;
}
EXPORT_SYMBOL_GPL(x8h7_hook_set);

/**
 */
int x8h7_dbg_set(void (*hook)(void*, uint8_t*, uint16_t), void *priv)
{
  x8h7_dbg = hook;
  x8h7_dbg_priv = priv;
  return 0;
}
EXPORT_SYMBOL_GPL(x8h7_dbg_set);

/**
 * Interrupt handler
 */
static irqreturn_t x8h7_threaded_isr(int irq, void *data)
{
  //struct spidev_data  *spidev = (struct spidev_data*)data;

  DBG_PRINT("Got IRQ from H7\n");
  x8h7_pkt_send_priv(1);

  return IRQ_HANDLED;
}

static int x8h7_probe(struct spi_device *spi)
{
  struct spidev_data  *spidev;
  int                  status;

  /*
   * spidev should never be referenced in DT without a specific
   * compatible string, it is a Linux implementation thing
   * rather than a description of the hardware.
   */
  WARN(spi->dev.of_node &&
      of_device_is_compatible(spi->dev.of_node, "spidev"),
      "%pOF: buggy DT: spidev listed directly in DT\n", spi->dev.of_node);

  /* Allocate driver data */
  spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
  if (!spidev)
    return -ENOMEM;

  /* Initialize the driver data */
  spidev->spi = spi;
  mutex_init(&spidev->buf_lock);

  spidev->speed_hz = spi->max_speed_hz;

  status = 0;

  /* interrupt request */
  if (spi->irq > 0) {
    int ret;
    ret = devm_request_threaded_irq(&spi->dev, spi->irq,
                                    NULL, x8h7_threaded_isr,
                                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                    "x8h7", spidev);
    if (ret) {
      DBG_ERROR("Failed request IRQ #%d\n", spi->irq);
      status = -ENODEV;
    }
    DBG_PRINT("IRQ request irq %d OK\n", spi->irq);
  }

  if (status == 0) {
    spidev->x8h7_txb = devm_kzalloc(&spi->dev, X8H7_BUF_SIZE, GFP_KERNEL);
    if (!spidev->x8h7_txb) {
      DBG_ERROR("X8H7 Tx buffer memory fail\n");
      status = -ENOMEM;
    }
  }

  if (status == 0) {
    spidev->x8h7_rxb = devm_kzalloc(&spi->dev, X8H7_BUF_SIZE, GFP_KERNEL);
    if (!spidev->x8h7_rxb) {
      DBG_ERROR("X8H7 Rx buffer memory fail\n");
      status = -ENOMEM;
    }
  }

  x8h7_spidev = spidev;

  if (status == 0)
    spi_set_drvdata(spi, spidev);
  else
    kfree(spidev);

  return status;
}

static int x8h7_remove(struct spi_device *spi)
{
  struct spidev_data	*spidev = spi_get_drvdata(spi);

  /* make sure ops on existing fds can abort cleanly */
  kfree(spidev);

  return 0;
}

static struct spi_driver x8h7_driver = {
  .driver = {
    .name             = "x8h7",
  },
  .probe  = x8h7_probe,
  .remove = x8h7_remove,
};

module_spi_driver(x8h7_driver);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com>");
MODULE_DESCRIPTION("Arduino x8h7 SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:x8h7");
