/**
 * X8H7 UART driver
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <asm/irq.h>

#include "x8h7.h"

#define DRIVER_NAME "x8h7_uart"

//#define DEBUG
#include "debug.h"

#define PORT_X8H7_UART    1000 // @TODO: add this define in serial.h


#define X8H7_UART_BAUD_MIN          0
#define X8H7_UART_BAUD_MAX    2000000

// Peripheral code
#define X8H7_UART_PERIPH 0x05

// Op code
#define X8H7_UART_OC_CONFIGURE      0x10 // BAUD | DATA_MODE 2 byte
#define X8H7_UART_OC_GET_LINESTATE  0x20 // 2 byte
#define X8H7_UART_OC_DATA           0x01 // variable
#define X8H7_UART_OC_STATUS         0x02 // received uart status e.g. busy state

// byte size
#define X8H7_UART_CFG_BS_5  0x0001
#define X8H7_UART_CFG_BS_6  0x0001
#define X8H7_UART_CFG_BS_7  0x0001
#define X8H7_UART_CFG_BS_8  0x0001

// stop bits
#define X8H7_UART_CFG_SB_1  0x0000
#define X8H7_UART_CFG_SB_2  0x0001

// parity
#define X8H7_UART_CFG_PAR_N  0x0000
#define X8H7_UART_CFG_PAR_O  0x0000
#define X8H7_UART_CFG_PAR_E  0x0000
#define X8H7_UART_CFG_PAR_M  0x0000
#define X8H7_UART_CFG_PAR_S  0x0000

// mode
#define X8H7_UART_CFG_MODE_NORMAL   0x0000
#define X8H7_UART_CFG_MODE_HWHS     0x0000
#define X8H7_UART_CFG_MODE_RS485    0x0000

// uart control
#define X8H7_UART_CTRL_RS485  0x0001
#define X8H7_UART_CTRL_RTS    0x0002
#define X8H7_UART_CTRL_DTR    0x0004
#define X8H7_UART_CTRL_DSR    0x0010
#define X8H7_UART_CTRL_CTS    0x0020
#define X8H7_UART_CTRL_DCD    0x0040

// uart status
#define X8H7_UART_STATUS_TX_EMPTY  0x01

enum UARTParity {
  PARITY_EVEN = 0,
  PARITY_ODD,
  PARITY_NONE,
};

struct __attribute__((packed, aligned(4))) uartPacket {
  uint8_t  bits        :  4; // LSB
  uint8_t  stop_bits   :  2;
  uint8_t  parity      :  2;
  uint8_t  flow_control:  1;
  uint32_t baud        : 23; // MSB
};
// 32-9 MSB 8 7-6 5-4 3-0 LSB

#define to_x8h7_uart_port(_port) \
                container_of(_port, struct x8h7_uart_port, port)

/**
 */
#define X8H7_UART_MAJOR   204
#define MINOR_START         5

#define X8H7_UART_NR_PORTS  1


#define X8H7_UART_CFG_SEND  0x00000001
#define X8H7_UART_TRANSMIT  0x00000002

/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo in case CTS has been dropped.
 */
#define MCTRL_TIMEOUT	(250*HZ/1000)

struct x8h7_uart_port {
  struct uart_port          port;
  struct timer_list         timer;
  unsigned int              old_status;

  /* Low level I/O work */
  struct work_struct        work;
  struct workqueue_struct  *workqueue;
  uint32_t                  flags;

  struct uartPacket         cfg;

  /* X8H7 */
  wait_queue_head_t         wait;
  int                       rx_cnt;
  x8h7_pkt_t                rx_pkt;
  uint8_t                   status; // used to handle busy tx and other stuff
};

struct x8h7_uart_port x8h7_uart_ports[X8H7_UART_NR_PORTS];

static void x8h7_uart_stop_tx(struct uart_port *port);
static void x8h7_uart_mctrl_check(struct x8h7_uart_port *sport);
static void x8h7_uart_rx_chars(struct x8h7_uart_port *sport);
static void x8h7_uart_tx_chars(struct x8h7_uart_port *sport);


/**
 */
static void x8h7_uart_hook(void *priv, x8h7_pkt_t *pkt)
{
  struct x8h7_uart_port  *sport = (struct x8h7_uart_port*)priv;

  memcpy(&sport->rx_pkt, pkt, sizeof(x8h7_pkt_t));

  switch(sport->rx_pkt.opcode) {
  case X8H7_UART_OC_DATA:
    /* Byte or break signal received */
    x8h7_uart_rx_chars(sport);
    break;
  case X8H7_UART_OC_STATUS:
    sport->status = sport->rx_pkt.data[0]; // @TODO: implement this on H7 side
    break;
  }

  sport->rx_cnt++;
}

/**
 */
static void x8h7_uart_rx_chars(struct x8h7_uart_port *sport)
{
  unsigned int  ch;
  int           i;
  int           ret;

  DBG_PRINT("size: %d\n", sport->rx_pkt.size);
  for (i=0 ; i<sport->rx_pkt.size; i++) {
    ch = sport->rx_pkt.data[0 + i];
    ret = uart_handle_sysrq_char(&sport->port, ch);
    if (!ret) {
      // @TODO: fix parameters
      unsigned int status = 0;
      unsigned int overrun = 0;
      unsigned int flg = TTY_NORMAL;
      DBG_PRINT("add char '%c'\n", ch);
      uart_insert_char(&sport->port, status, overrun, ch, flg);
      sport->port.icount.rx++;
    }
  }

  //spin_unlock(&sport->port.lock);
  tty_flip_buffer_push(&sport->port.state->port);
  //spin_lock(&sport->port.lock);
}

/**
 */
static void x8h7_uart_tx_chars(struct x8h7_uart_port *sport)
{
  struct circ_buf  *xmit = &sport->port.state->xmit;

#if 0
  //@TODO Xon Xoff protocol
  size = 0;

  if (sport->port.x_char) {
    data[size] = sport->port.x_char;
    size++;
    sport->port.icount.tx++;
    sport->port.x_char = 0;
    return;
  }
#endif
  /*
   * Check the modem control lines before
   * transmitting anything.
   */
  x8h7_uart_mctrl_check(sport);

  if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
    x8h7_uart_stop_tx(&sport->port);
    return;
  }

  /*
   * TX while bytes available
   */
  sport->flags |= X8H7_UART_TRANSMIT;
  DBG_PRINT("work queue triggered\n");
  queue_work(sport->workqueue, &sport->work);
}

/**
 * Handle any change of modem status signal since we were last called.
 */
static void x8h7_uart_mctrl_check(struct x8h7_uart_port *sport)
{
  unsigned int status, changed;

  status = sport->port.ops->get_mctrl(&sport->port);
  changed = status ^ sport->old_status;

  if (changed == 0) {
    return;
  }
  sport->old_status = status;

  if (changed & TIOCM_RI) {
    sport->port.icount.rng++;
  }
  if (changed & TIOCM_DSR) {
    sport->port.icount.dsr++;
  }
  if (changed & TIOCM_CAR) {
    uart_handle_dcd_change(&sport->port, status & TIOCM_CAR);
  }
  if (changed & TIOCM_CTS) {
    uart_handle_cts_change(&sport->port, status & TIOCM_CTS);
  }
}

/**
 * This is our per-port timeout handler, for checking the
 * modem status signals: RI DSR CAR CTS.
 */
/*static void x8h7_uart_timeout(struct timer_list *t)
{
}*/

/**
 * Stop receiving - port is in process of being closed.
 */
static void x8h7_uart_stop_rx(struct uart_port *port)
{
  DBG_PRINT("No operation\n");
}

/**
 * Return TIOCSER_TEMT when transmitter is not busy.
 * @TODO: need to understand when H7 is still busy
 * in trasmitting data on uart due to slower throughput in
 * comparison with spi. Introduce status byte?
 */
static unsigned int x8h7_uart_tx_empty(struct uart_port *port)
{
  struct x8h7_uart_port  *sport = to_x8h7_uart_port(port);
  return ((sport->flags & X8H7_UART_TRANSMIT) ? 0 : TIOCSER_TEMT);
}

/**
 */
static void x8h7_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
//struct x8h7_uart_port  *sport = (struct x8h7_uart_port *)port;
  uint16_t                control;

  DBG_PRINT("\n");
  control = 0;

  if (port->rs485.flags & SER_RS485_ENABLED) {
    control |= X8H7_UART_CTRL_RS485;
  }
  if (mctrl & TIOCM_RTS) {
    control |= X8H7_UART_CTRL_RTS;
  }
  if (mctrl & TIOCM_DTR) {
    control |= X8H7_UART_CTRL_DTR;
  }
//@TODO: tolto per BUG: scheduling while atomic: insmod/573/0x00000002 trovare un sistema
  //x8h7_pkt_enq(X8H7_UART_PERIPH, X8H7_UART_OC_GET_LINESTATE, 2, &control);
  //x8h7_pkt_send();
}

/**
 */
static unsigned int x8h7_uart_get_mctrl(struct uart_port *port)
{
  /* DCD and DSR are not wired and CTS/RTS is handled automatically
  * so just indicate DSR and CAR asserted
  * @TODO: add also TIOCM_CTS?
  */
  return TIOCM_DSR | TIOCM_CAR;
}

/**
 *
 */
static void x8h7_uart_stop_tx(struct uart_port *port)
{
  DBG_PRINT("No operation\n");
}

/**
 *
 */
static void x8h7_uart_start_tx(struct uart_port *port)
{
  struct x8h7_uart_port *sport = to_x8h7_uart_port(port);

  DBG_PRINT("\n");
  x8h7_uart_tx_chars(sport);
}

/**
 * Control the transmission of a break signal
 */
static void x8h7_uart_break_ctl(struct uart_port *port, int break_state)
{
  struct x8h7_uart_port  *sport = to_x8h7_uart_port(port);
  unsigned long           flags;

  DBG_PRINT("\n");
  spin_lock_irqsave(&sport->port.lock, flags);
  spin_unlock_irqrestore(&sport->port.lock, flags);
}

/**
 * Perform initialization and enable port for reception
 */
static int x8h7_uart_startup(struct uart_port *port)
{
  //struct x8h7_uart_port *sport = to_x8h7_uart_port(port);

  return 0;
}

/**
 * Disable the port
 */
static void x8h7_uart_shutdown(struct uart_port *port)
{
  //struct x8h7_uart_port *sport = to_x8h7_uart_port(port);
  DBG_PRINT("\n");
  /*
   * Disable all interrupts
   * Clear all interrupts
   * Free the interrupt
   */
  //destroy_workqueue(sport->workqueue);
}

/**
 * Change the port parameters
 */
static void x8h7_uart_set_termios(struct uart_port *port,
                                  struct ktermios *termios,
                                  struct ktermios *old)
{
  struct x8h7_uart_port  *sport = to_x8h7_uart_port(port);
  unsigned int            baud;

  memset(&sport->cfg, 0, sizeof(sport->cfg));

  /* byte size */
  /*
  switch (termios->c_cflag & CSIZE) {
  case CS5:
    sport->cfg |= X8H7_UART_CFG_BS_5;
    break;
  case CS6:
    sport->cfg |= X8H7_UART_CFG_BS_6;
    break;
  case CS7:
    sport->cfg |= X8H7_UART_CFG_BS_7;
    break;
  default:
    sport->cfg |= X8H7_UART_CFG_BS_8;
    break;
  }
*/
  switch (termios->c_cflag & CSIZE) {
  case CS7:
    sport->cfg.bits = 7;
    break;
  default:
    sport->cfg.bits = 8;
    break;
  }

  /* stop bits */
  /*if (termios->c_cflag & CSTOPB) {
    sport->cfg |= X8H7_UART_CFG_SB_2;
  }*/
  if (termios->c_cflag & CSTOPB) {
    sport->cfg.stop_bits = 2;
  } else {
    sport->cfg.stop_bits = 1;
  }

  /* parity */
  /*
  if (termios->c_cflag & PARENB) {
    // Mark or Space parity
    if (termios->c_cflag & CMSPAR) {
      if (termios->c_cflag & PARODD) {
        sport->cfg |= X8H7_UART_CFG_PAR_M;
      } else {
        sport->cfg |= X8H7_UART_CFG_PAR_S;
      }
    } else if (termios->c_cflag & PARODD) {
      sport->cfg |= X8H7_UART_CFG_PAR_O;
    } else {
      sport->cfg |= X8H7_UART_CFG_PAR_E;
    }
  } else {
    sport->cfg |= X8H7_UART_CFG_PAR_N;
  }
*/
  if (termios->c_cflag & PARENB) {
    if (termios->c_cflag & PARODD) {
      sport->cfg.parity |= PARITY_ODD;
    } else {
      sport->cfg.parity |= PARITY_EVEN;
    }
  } else {
    sport->cfg.parity = PARITY_NONE;
  }

  /* baud */
  baud = uart_get_baud_rate(port, termios, old,
                            X8H7_UART_BAUD_MIN, X8H7_UART_BAUD_MAX);
  DBG_PRINT("baud: %d\n", baud);

  sport->cfg.baud = baud;

  /*
   * disable interrupts and drain transmitter
   * then, disable everything
   * Reset the Rx and Tx FIFOs too
   */
  sport->flags |= X8H7_UART_CFG_SEND;
  DBG_PRINT("work queue triggered\n");
  queue_work(sport->workqueue, &sport->work);

  //spin_unlock_irqrestore(&sport->port.lock, flags);
}

static const char *x8h7_uart_type(struct uart_port *port)
{
  struct x8h7_uart_port *sport = to_x8h7_uart_port(port);

  DBG_PRINT("\n");
  return sport->port.type == PORT_X8H7_UART ? "X8H7_UART" : NULL;
}

/**
 *
 */
static int x8h7_uart_request_port(struct uart_port *port)
{
  struct x8h7_uart_port *sport = to_x8h7_uart_port(port);

  DBG_PRINT("\n");
  x8h7_hook_set(X8H7_UART_PERIPH, x8h7_uart_hook, sport);
  return 0;
}

/**
 *
 */
static void x8h7_uart_release_port(struct uart_port *port)
{
  DBG_PRINT("\n");
  x8h7_hook_set(X8H7_UART_PERIPH, NULL, NULL);
}

/*
 * Configure/autoconfigure the port.
 */
static void x8h7_uart_config_port(struct uart_port *port, int flags)
{
  struct x8h7_uart_port *sport = to_x8h7_uart_port(port);

  DBG_PRINT("\n");
  if (flags & UART_CONFIG_TYPE &&
      x8h7_uart_request_port(&sport->port) == 0) {
    sport->port.type = PORT_X8H7_UART;
  }
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 */
static int x8h7_uart_verify_port(struct uart_port *port,
                                 struct serial_struct *ser)
{
  //struct x8h7_uart_port  *sport = to_x8h7_uart_port(port);
  //int                     ret = 0;

  DBG_PRINT("\n");
  /*
  if (ser->type != PORT_UNKNOWN && ser->type != PORT_X8H7_UART)
    ret = -EINVAL;
  if (sport->port.irq != ser->irq)
    ret = -EINVAL;
  if (ser->io_type != SERIAL_IO_MEM)
    ret = -EINVAL;
  if (sport->port.uartclk / 16 != ser->baud_base)
    ret = -EINVAL;
  if ((void *)sport->port.mapbase != ser->iomem_base)
    ret = -EINVAL;
  if (sport->port.iobase != ser->port)
    ret = -EINVAL;
  if (ser->hub6 != 0)
    ret = -EINVAL;
  */
  return 0;  //ret;
}

static const struct uart_ops x8h7_uart_pops = {
  .tx_empty     = x8h7_uart_tx_empty,
  .set_mctrl    = x8h7_uart_set_mctrl,
  .get_mctrl    = x8h7_uart_get_mctrl,
  .stop_tx      = x8h7_uart_stop_tx,
  .start_tx     = x8h7_uart_start_tx,
  .stop_rx      = x8h7_uart_stop_rx,
  .break_ctl    = x8h7_uart_break_ctl,
  .startup      = x8h7_uart_startup,
  .shutdown     = x8h7_uart_shutdown,
  .set_termios  = x8h7_uart_set_termios,
  .type         = x8h7_uart_type,
  .request_port = x8h7_uart_request_port,
  .release_port = x8h7_uart_release_port,
  .config_port  = x8h7_uart_config_port,
  .verify_port  = x8h7_uart_verify_port,
};

/**
 */
static struct uart_driver x8h7_uart = {
  .owner       = THIS_MODULE,
  .driver_name = "ttyX",
  .dev_name    = "ttyX",
  .major       = X8H7_UART_MAJOR,
  .minor       = MINOR_START,
  .nr          = X8H7_UART_NR_PORTS,
  .cons        = NULL,
};

/**
 * main workqueue for uart module
 */
static void x8h7_uart_work_func(struct work_struct *work)
{
  struct x8h7_uart_port *sport = container_of(work, struct x8h7_uart_port, work);

  DBG_PRINT("work queue start\n");
  DBG_PRINT("FLAGS %08X\n", sport->flags);

  if (sport->flags & X8H7_UART_CFG_SEND) {
    sport->flags &= ~X8H7_UART_CFG_SEND;
    x8h7_pkt_enq(X8H7_UART_PERIPH, X8H7_UART_OC_CONFIGURE,
                 sizeof(sport->cfg), &sport->cfg);
    x8h7_pkt_send();
  }
  if (sport->flags & X8H7_UART_TRANSMIT) {
    struct circ_buf  *xmit = &sport->port.state->xmit;
    uint8_t           txb[X8H7_PKT_SIZE];
    uint16_t          size;

    sport->flags &= ~X8H7_UART_TRANSMIT;

    while (!uart_circ_empty(xmit)) {

      for (size = 0; size < X8H7_PKT_SIZE && !uart_circ_empty(xmit); size++) {
        txb[size] = xmit->buf[xmit->tail];

        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
      }

      //send pkt
      x8h7_pkt_enq(X8H7_UART_PERIPH, X8H7_UART_OC_DATA, size, txb);
      x8h7_pkt_send();

      sport->port.icount.tx += size;
    }
    //uart_circ_clear(xmit);
  }
  DBG_PRINT("work queue end\n");
}

/**
 */
static int x8h7_uart_probe(struct platform_device *pdev)
{
  int i = 0;

  init_waitqueue_head(&x8h7_uart_ports[i].wait);

  x8h7_uart_ports[i].port.type     = 150;
  x8h7_uart_ports[i].port.fifosize = 32;
  x8h7_uart_ports[i].port.flags    = 0;
  x8h7_uart_ports[i].port.iotype   = SERIAL_IO_PORT;
  x8h7_uart_ports[i].port.iobase   = 0;
  x8h7_uart_ports[i].port.membase  = (void __iomem *)~0;
  x8h7_uart_ports[i].port.uartclk  = 24*1000*1000;
  x8h7_uart_ports[i].port.ops      = &x8h7_uart_pops;

  x8h7_uart_ports[i].port.line = 0;
  x8h7_uart_ports[i].port.dev  = &pdev->dev;
  x8h7_uart_ports[i].port.irq  = 0;

  uart_add_one_port(&x8h7_uart, &x8h7_uart_ports[i].port);
  platform_set_drvdata(pdev, &x8h7_uart_ports[i]);
  x8h7_hook_set(X8H7_UART_PERIPH, x8h7_uart_hook, &x8h7_uart_ports[i]);

  INIT_WORK(&x8h7_uart_ports[i].work, x8h7_uart_work_func);
  x8h7_uart_ports[i].workqueue = create_workqueue("x8h7_uart_work");
  if (!x8h7_uart_ports[i].workqueue) {
    DBG_ERROR("fail to create work queue\n");
    return -ENOMEM;
  }

  DBG_PRINT("probed\n");
  return 0;
}

static int x8h7_uart_remove(struct platform_device *pdev)
{
  struct x8h7_uart_port *sport = platform_get_drvdata(pdev);

  DBG_PRINT("destroying work queue\n");
  destroy_workqueue(sport->workqueue);
  if (sport) {
    uart_remove_one_port(&x8h7_uart, &sport->port);
  }
  return 0;
}


static const struct of_device_id x8h7_uart_dt_ids[] = {
  { .compatible = "portenta,x8h7_uart" },
  { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, x8h7_uart_dt_ids);

static struct platform_driver x8h7_uart_driver = {
  .driver           = {
    .name           = DRIVER_NAME,
    .of_match_table = x8h7_uart_dt_ids,
  },
  .probe    = x8h7_uart_probe,
  .remove   = x8h7_uart_remove,
};

static int __init x8h7_uart_init(void)
{
  int ret;

  printk(KERN_INFO "Serial: X8H7 UART driver\n");

  ret = uart_register_driver(&x8h7_uart);
  if (ret == 0) {
    ret = platform_driver_register(&x8h7_uart_driver);
    if (ret) {
      uart_unregister_driver(&x8h7_uart);
    }
  }
  return ret;
}

static void __exit x8h7_uart_exit(void)
{
  platform_driver_unregister(&x8h7_uart_driver);
  uart_unregister_driver(&x8h7_uart);
}

module_init(x8h7_uart_init);
module_exit(x8h7_uart_exit);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_DESCRIPTION("Arduino Portenta X8 UART driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_CHARDEV_MAJOR(X8H7_UART_MAJOR);
//MODULE_ALIAS("platform:x8h7_uart");
