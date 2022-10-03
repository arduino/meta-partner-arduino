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


/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR      153   /* assigned */
#define N_SPI_MINORS       32   /* ... up to 256 */


static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK   (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
                        | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                        | SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
                        | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct spidev_data {
  dev_t               devt;
  spinlock_t          spi_lock;
  struct spi_device  *spi;
  struct list_head    device_entry;

  /* TX/RX buffers are NULL unless this device is open (users > 0) */
  struct mutex        buf_lock;
  unsigned            users;
  u8                 *tx_buffer;
  u8                 *rx_buffer;
  u32                 speed_hz;

  u8                 *x8h7_txb;
  u16                 x8h7_txl;
  u8                 *x8h7_rxb;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

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

#if 0
/**
 */
typedef struct {
  x8h7_hook_t         hook;
  void               *priv;
  wait_queue_head_t   x8h7_wait;
  int                 rx_cnt;
  x8h7_pkt_t          rx_pkt;
} x8h7_periph_t;

x8h7_periph_t x8h7_periph[X8H7_PERIPH_NUM];
#endif

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

/*-------------------------------------------------------------------------*/

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
  int                 status;
  struct spi_device  *spi;

  spin_lock_irq(&spidev->spi_lock);
  spi = spidev->spi;
  spin_unlock_irq(&spidev->spi_lock);

  if (spi == NULL)
    status = -ESHUTDOWN;
  else
    status = spi_sync(spi, message);

  if (status == 0)
    status = message->actual_length;

  return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
  struct spi_transfer t = {
      .tx_buf   = spidev->tx_buffer,
      .len      = len,
      .speed_hz = spidev->speed_hz,
    };
  struct spi_message  m;

  spi_message_init(&m);
  spi_message_add_tail(&t, &m);
  return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
  struct spi_transfer t = {
      .rx_buf   = spidev->rx_buffer,
      .len      = len,
      .speed_hz = spidev->speed_hz,
    };
  struct spi_message  m;

  spi_message_init(&m);
  spi_message_add_tail(&t, &m);
  return spidev_sync(spidev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
  struct spidev_data  *spidev;
  ssize_t              status = 0;

  /* chipselect only toggles at start or end of operation */
  if (count > bufsiz)
    return -EMSGSIZE;

  spidev = filp->private_data;

  mutex_lock(&spidev->buf_lock);

  status = spidev_sync_read(spidev, count);
  if (status > 0) {
    unsigned long	missing;

    missing = copy_to_user(buf, spidev->rx_buffer, status);
    if (missing == status)
      status = -EFAULT;
    else
      status = status - missing;
  }
  mutex_unlock(&spidev->buf_lock);
  return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
             size_t count, loff_t *f_pos)
{
  struct spidev_data  *spidev;
  ssize_t              status = 0;
  unsigned long        missing;

  /* chipselect only toggles at start or end of operation */
  if (count > bufsiz)
    return -EMSGSIZE;

  spidev = filp->private_data;

  mutex_lock(&spidev->buf_lock);
  missing = copy_from_user(spidev->tx_buffer, buf, count);
  if (missing == 0)
    status = spidev_sync_write(spidev, count);
  else
    status = -EFAULT;
  mutex_unlock(&spidev->buf_lock);
  return status;
}

static int spidev_message(struct spidev_data *spidev,
                          struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
  struct spi_message       msg;
  struct spi_transfer     *k_xfers;
  struct spi_transfer     *k_tmp;
  struct spi_ioc_transfer *u_tmp;
  unsigned                 n, total, tx_total, rx_total;
  u8                      *tx_buf, *rx_buf;
  int                      status = -EFAULT;

  spi_message_init(&msg);
  k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
  if (k_xfers == NULL)
    return -ENOMEM;

  /* Construct spi_message, copying any tx data to bounce buffer.
   * We walk the array of user-provided transfers, using each one
   * to initialize a kernel version of the same transfer.
   */
  tx_buf = spidev->tx_buffer;
  rx_buf = spidev->rx_buffer;
  total = 0;
  tx_total = 0;
  rx_total = 0;
  for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
      n;
      n--, k_tmp++, u_tmp++) {
    /* Ensure that also following allocations from rx_buf/tx_buf will meet
     * DMA alignment requirements.
     */
    unsigned int len_aligned = ALIGN(u_tmp->len, ARCH_KMALLOC_MINALIGN);

    k_tmp->len = u_tmp->len;

    total += k_tmp->len;
    /* Since the function returns the total length of transfers
     * on success, restrict the total to positive int values to
     * avoid the return value looking like an error.  Also check
     * each transfer length to avoid arithmetic overflow.
     */
    if (total > INT_MAX || k_tmp->len > INT_MAX) {
      status = -EMSGSIZE;
      goto done;
    }

    if (u_tmp->rx_buf) {
      /* this transfer needs space in RX bounce buffer */
      rx_total += len_aligned;
      if (rx_total > bufsiz) {
        status = -EMSGSIZE;
        goto done;
      }
      k_tmp->rx_buf = rx_buf;
      rx_buf += len_aligned;
    }
    if (u_tmp->tx_buf) {
      /* this transfer needs space in TX bounce buffer */
      tx_total += len_aligned;
      if (tx_total > bufsiz) {
        status = -EMSGSIZE;
        goto done;
      }
      k_tmp->tx_buf = tx_buf;
      if (copy_from_user(tx_buf, (const u8 __user *)
            (uintptr_t) u_tmp->tx_buf,
          u_tmp->len))
        goto done;
      tx_buf += len_aligned;
    }

    k_tmp->cs_change = !!u_tmp->cs_change;
    k_tmp->tx_nbits = u_tmp->tx_nbits;
    k_tmp->rx_nbits = u_tmp->rx_nbits;
    k_tmp->bits_per_word = u_tmp->bits_per_word;
    k_tmp->delay_usecs = u_tmp->delay_usecs;
    k_tmp->speed_hz = u_tmp->speed_hz;
    k_tmp->word_delay.value = u_tmp->word_delay_usecs;
    k_tmp->word_delay.unit = SPI_DELAY_UNIT_USECS;
    if (!k_tmp->speed_hz)
      k_tmp->speed_hz = spidev->speed_hz;
#ifdef VERBOSE
    dev_dbg(&spidev->spi->dev,
      "  xfer len %u %s%s%s%dbits %u usec %u usec %uHz\n",
      u_tmp->len,
      u_tmp->rx_buf ? "rx " : "",
      u_tmp->tx_buf ? "tx " : "",
      u_tmp->cs_change ? "cs " : "",
      u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
      u_tmp->delay_usecs,
      u_tmp->word_delay_usecs,
      u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
    spi_message_add_tail(k_tmp, &msg);
  }

  status = spidev_sync(spidev, &msg);
  if (status < 0)
    goto done;

  /* copy any rx data out of bounce buffer */
  for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
      n;
      n--, k_tmp++, u_tmp++) {
    if (u_tmp->rx_buf) {
      if (copy_to_user((u8 __user *)
          (uintptr_t) u_tmp->rx_buf, k_tmp->rx_buf,
          u_tmp->len)) {
        status = -EFAULT;
        goto done;
      }
    }
  }
  status = total;

done:
  kfree(k_xfers);
  return status;
}

static struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
                       unsigned *n_ioc)
{
  u32 tmp;

  /* Check type, command number and direction */
  if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC
      || _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
      || _IOC_DIR(cmd) != _IOC_WRITE)
    return ERR_PTR(-ENOTTY);

  tmp = _IOC_SIZE(cmd);
  if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
    return ERR_PTR(-EINVAL);
  *n_ioc = tmp / sizeof(struct spi_ioc_transfer);
  if (*n_ioc == 0)
    return NULL;

  /* copy into scratch area */
  return memdup_user(u_ioc, tmp);
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int                      retval = 0;
  struct spidev_data      *spidev;
  struct spi_device       *spi;
  u32                      tmp;
  unsigned                 n_ioc;
  struct spi_ioc_transfer *ioc;

  /* Check type and command number */
  if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
    return -ENOTTY;

  /* guard against device removal before, or while,
   * we issue this ioctl.
   */
  spidev = filp->private_data;
  spin_lock_irq(&spidev->spi_lock);
  spi = spi_dev_get(spidev->spi);
  spin_unlock_irq(&spidev->spi_lock);

  if (spi == NULL)
    return -ESHUTDOWN;

  /* use the buffer lock here for triple duty:
   *  - prevent I/O (from us) so calling spi_setup() is safe;
   *  - prevent concurrent SPI_IOC_WR_* from morphing
   *    data fields while SPI_IOC_RD_* reads them;
   *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
   */
  mutex_lock(&spidev->buf_lock);

  switch (cmd) {
  /* read requests */
  case SPI_IOC_RD_MODE:
    retval = put_user(spi->mode & SPI_MODE_MASK,
          (__u8 __user *)arg);
    break;
  case SPI_IOC_RD_MODE32:
    retval = put_user(spi->mode & SPI_MODE_MASK,
          (__u32 __user *)arg);
    break;
  case SPI_IOC_RD_LSB_FIRST:
    retval = put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
          (__u8 __user *)arg);
    break;
  case SPI_IOC_RD_BITS_PER_WORD:
    retval = put_user(spi->bits_per_word, (__u8 __user *)arg);
    break;
  case SPI_IOC_RD_MAX_SPEED_HZ:
    retval = put_user(spidev->speed_hz, (__u32 __user *)arg);
    break;

  /* write requests */
  case SPI_IOC_WR_MODE:
  case SPI_IOC_WR_MODE32:
    if (cmd == SPI_IOC_WR_MODE)
      retval = get_user(tmp, (u8 __user *)arg);
    else
      retval = get_user(tmp, (u32 __user *)arg);
    if (retval == 0) {
      struct spi_controller *ctlr = spi->controller;
      u32	save = spi->mode;

      if (tmp & ~SPI_MODE_MASK) {
        retval = -EINVAL;
        break;
      }

      if (ctlr->use_gpio_descriptors && ctlr->cs_gpiods &&
          ctlr->cs_gpiods[spi->chip_select])
        tmp |= SPI_CS_HIGH;

      tmp |= spi->mode & ~SPI_MODE_MASK;
      spi->mode = (u16)tmp;
      retval = spi_setup(spi);
      if (retval < 0)
        spi->mode = save;
      else
        dev_dbg(&spi->dev, "spi mode %x\n", tmp);
    }
    break;
  case SPI_IOC_WR_LSB_FIRST:
    retval = get_user(tmp, (__u8 __user *)arg);
    if (retval == 0) {
      u32	save = spi->mode;

      if (tmp)
        spi->mode |= SPI_LSB_FIRST;
      else
        spi->mode &= ~SPI_LSB_FIRST;
      retval = spi_setup(spi);
      if (retval < 0)
        spi->mode = save;
      else
        dev_dbg(&spi->dev, "%csb first\n",
            tmp ? 'l' : 'm');
    }
    break;
  case SPI_IOC_WR_BITS_PER_WORD:
    retval = get_user(tmp, (__u8 __user *)arg);
    if (retval == 0) {
      u8	save = spi->bits_per_word;

      spi->bits_per_word = tmp;
      retval = spi_setup(spi);
      if (retval < 0)
        spi->bits_per_word = save;
      else
        dev_dbg(&spi->dev, "%d bits per word\n", tmp);
    }
    break;
  case SPI_IOC_WR_MAX_SPEED_HZ:
    retval = get_user(tmp, (__u32 __user *)arg);
    if (retval == 0) {
      u32	save = spi->max_speed_hz;

      spi->max_speed_hz = tmp;
      retval = spi_setup(spi);
      if (retval >= 0)
        spidev->speed_hz = tmp;
      else
        dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
      spi->max_speed_hz = save;
    }
    break;

  default:
    /* segmented and/or full-duplex I/O request */
    /* Check message and copy into scratch area */
    ioc = spidev_get_ioc_message(cmd,
        (struct spi_ioc_transfer __user *)arg, &n_ioc);
    if (IS_ERR(ioc)) {
      retval = PTR_ERR(ioc);
      break;
    }
    if (!ioc)
      break;	/* n_ioc is also 0 */

    /* translate to spi_message, execute */
    retval = spidev_message(spidev, ioc, n_ioc);
    kfree(ioc);
    break;
  }

  mutex_unlock(&spidev->buf_lock);
  spi_dev_put(spi);
  return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioc_message(struct file *filp, unsigned int cmd,
                          unsigned long arg)
{
  struct spi_ioc_transfer __user  *u_ioc;
  int                              retval = 0;
  struct spidev_data              *spidev;
  struct spi_device               *spi;
  unsigned                         n_ioc, n;
  struct spi_ioc_transfer         *ioc;

  u_ioc = (struct spi_ioc_transfer __user *) compat_ptr(arg);

  /* guard against device removal before, or while,
   * we issue this ioctl.
   */
  spidev = filp->private_data;
  spin_lock_irq(&spidev->spi_lock);
  spi = spi_dev_get(spidev->spi);
  spin_unlock_irq(&spidev->spi_lock);

  if (spi == NULL)
    return -ESHUTDOWN;

  /* SPI_IOC_MESSAGE needs the buffer locked "normally" */
  mutex_lock(&spidev->buf_lock);

  /* Check message and copy into scratch area */
  ioc = spidev_get_ioc_message(cmd, u_ioc, &n_ioc);
  if (IS_ERR(ioc)) {
    retval = PTR_ERR(ioc);
    goto done;
  }
  if (!ioc)
    goto done;	/* n_ioc is also 0 */

  /* Convert buffer pointers */
  for (n = 0; n < n_ioc; n++) {
    ioc[n].rx_buf = (uintptr_t) compat_ptr(ioc[n].rx_buf);
    ioc[n].tx_buf = (uintptr_t) compat_ptr(ioc[n].tx_buf);
  }

  /* translate to spi_message, execute */
  retval = spidev_message(spidev, ioc, n_ioc);
  kfree(ioc);

done:
  mutex_unlock(&spidev->buf_lock);
  spi_dev_put(spi);
  return retval;
}

static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  if (_IOC_TYPE(cmd) == SPI_IOC_MAGIC
      && _IOC_NR(cmd) == _IOC_NR(SPI_IOC_MESSAGE(0))
      && _IOC_DIR(cmd) == _IOC_WRITE)
    return spidev_compat_ioc_message(filp, cmd, arg);

  return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
  struct spidev_data  *spidev;
  int                  status = -ENXIO;

  mutex_lock(&device_list_lock);

  list_for_each_entry(spidev, &device_list, device_entry) {
    if (spidev->devt == inode->i_rdev) {
      status = 0;
      break;
    }
  }

  if (status) {
    pr_debug("spidev: nothing for minor %d\n", iminor(inode));
    goto err_find_dev;
  }

  if (!spidev->tx_buffer) {
    spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
    if (!spidev->tx_buffer) {
      dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
      status = -ENOMEM;
      goto err_find_dev;
    }
  }

  if (!spidev->rx_buffer) {
    spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
    if (!spidev->rx_buffer) {
      dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
      status = -ENOMEM;
      goto err_alloc_rx_buf;
    }
  }

  spidev->users++;
  filp->private_data = spidev;
  stream_open(inode, filp);

  mutex_unlock(&device_list_lock);
  return 0;

err_alloc_rx_buf:
  kfree(spidev->tx_buffer);
  spidev->tx_buffer = NULL;
err_find_dev:
  mutex_unlock(&device_list_lock);
  return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
  struct spidev_data  *spidev;
  int                  dofree;

  mutex_lock(&device_list_lock);
  spidev = filp->private_data;
  filp->private_data = NULL;

  spin_lock_irq(&spidev->spi_lock);
  /* ... after we unbound from the underlying device? */
  dofree = (spidev->spi == NULL);
  spin_unlock_irq(&spidev->spi_lock);

  /* last close? */
  spidev->users--;
  if (!spidev->users) {

    kfree(spidev->tx_buffer);
    spidev->tx_buffer = NULL;

    kfree(spidev->rx_buffer);
    spidev->rx_buffer = NULL;

    if (dofree)
      kfree(spidev);
    else
      spidev->speed_hz = spidev->spi->max_speed_hz;
  }
#ifdef CONFIG_SPI_SLAVE
  if (!dofree)
    spi_slave_abort(spidev->spi);
#endif
  mutex_unlock(&device_list_lock);

  return 0;
}

static const struct file_operations spidev_fops = {
  .owner          = THIS_MODULE,
  /* REVISIT switch to aio primitives, so that userspace
   * gets more complete API coverage.  It'll simplify things
   * too, except for the locking.
   */
  .write          = spidev_write,
  .read           = spidev_read,
  .unlocked_ioctl = spidev_ioctl,
  .compat_ioctl   = spidev_compat_ioctl,
  .open           = spidev_open,
  .release        = spidev_release,
  .llseek         = no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

#ifdef CONFIG_OF
static const struct of_device_id spidev_dt_ids[] = {
  { .compatible = "portenta,x8h7"},
  {},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);
#endif

/*-------------------------------------------------------------------------*/


/*************** Sysfs functions **********************/
static ssize_t sysfs_show_speed(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%d", x8h7_spidev->speed_hz);
}

static ssize_t sysfs_store_speed(struct kobject *kobj,
                struct kobj_attribute *attr,const char *buf, size_t count)
{
        sscanf(buf, "%d", &x8h7_spidev->speed_hz);
        return count;
}


struct kobject *kobj_ref;
struct kobj_attribute etx_attr = __ATTR(speed, 0660, sysfs_show_speed, sysfs_store_speed);

static int spidev_probe(struct spi_device *spi)
{
  struct spidev_data  *spidev;
  int                  status;
  unsigned long        minor;

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
  spin_lock_init(&spidev->spi_lock);
  mutex_init(&spidev->buf_lock);

  INIT_LIST_HEAD(&spidev->device_entry);

  /* If we can allocate a minor number, hook up this device.
   * Reusing minors is fine so long as udev or mdev is working.
   */
  mutex_lock(&device_list_lock);
  minor = find_first_zero_bit(minors, N_SPI_MINORS);
  if (minor < N_SPI_MINORS) {
    struct device *dev;

    spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
    dev = device_create(spidev_class, &spi->dev, spidev->devt,
                        spidev, "spidev%d.%d",
                        spi->master->bus_num, spi->chip_select);
    status = PTR_ERR_OR_ZERO(dev);
  } else {
    dev_dbg(&spi->dev, "no minor number available!\n");
    status = -ENODEV;
  }
  if (status == 0) {
    set_bit(minor, minors);
    list_add(&spidev->device_entry, &device_list);
  }
  mutex_unlock(&device_list_lock);

  spidev->speed_hz = spi->max_speed_hz;

  if (status == 0) {
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

  kobj_ref = kobject_create_and_add("x8h7_spidev", kernel_kobj);

  /*Creating sysfs file for etx_value*/
  if(sysfs_create_file(kobj_ref, &etx_attr.attr)){
    DBG_ERROR("Cannot create sysfs file\n");
  }

  return status;
}

static int spidev:remove(struct spi_device *spi)
{
  struct spidev_data	*spidev = spi_get_drvdata(spi);

  /* prevent new opens */
  mutex_lock(&device_list_lock);

  if (spi->irq > 0) {
    free_irq(spi->irq, spidev);
    spi->irq = 0;
  }

  /* make sure ops on existing fds can abort cleanly */
  spin_lock_irq(&spidev->spi_lock);
  spidev->spi = NULL;
  spin_unlock_irq(&spidev->spi_lock);

  list_del(&spidev->device_entry);
  device_destroy(spidev_class, spidev->devt);
  clear_bit(MINOR(spidev->devt), minors);
  if (spidev->users == 0)
    kfree(spidev);
  mutex_unlock(&device_list_lock);

  return 0;
}

static struct spi_driver spidev_spi_driver = {
  .driver = {
    .name             = "spidev",
    .of_match_table   = of_match_ptr(spidev_dt_ids),
  },
  .probe  = spidev_probe,
  .remove = spidev:remove,

  /* NOTE:  suspend/resume methods are not necessary here.
  * We don't do anything except pass the requests to/from
  * the underlying controller.  The refrigerator handles
  * most issues; the controller driver handles the rest.
  */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
  int status;

  /* Claim our 256 reserved device numbers.  Then register a class
   * that will key udev/mdev to add/remove /dev nodes.  Last, register
   * the driver which manages those device numbers.
   */
  BUILD_BUG_ON(N_SPI_MINORS > 256);
  status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
  if (status < 0)
    return status;

  spidev_class = class_create(THIS_MODULE, "spidev");
  if (IS_ERR(spidev_class)) {
    unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
    return PTR_ERR(spidev_class);
  }

  status = spi_register_driver(&spidev_spi_driver);
  if (status < 0) {
    class_destroy(spidev_class);
    unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
  }
  return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
  spi_unregister_driver(&spidev_spi_driver);
  class_destroy(spidev_class);
  unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>,"
              "Massimiliano Agneni <massimiliano@iptronix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
