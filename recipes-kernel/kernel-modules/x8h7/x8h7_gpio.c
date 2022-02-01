/**
 * X8H7 GPIO driver
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include "x8h7.h"

#define DRIVER_NAME "x8h7_gpio"

#define DEBUG
#include "debug.h"

// Peripheral code
#define X8H7_GPIO_PERIPH 0x07
// Op code
#define X8H7_GPIO_OC_DIR    0x10
#define X8H7_GPIO_OC_IRQ_TYPE    0x11
#define X8H7_GPIO_OC_WR     0x20
#define X8H7_GPIO_OC_RD     0x30
#define X8H7_GPIO_OC_IEN    0x40
#define X8H7_GPIO_OC_INT    0x50
#define X8H7_GPIO_OC_IACK   0x60

//#define GPIO_MODE_INPUT         0x00   /*!< Input Floating Mode */
//#define GPIO_MODE_OUTPUT_PP     0x01   /*!< Output Push Pull Mode */
//#define GPIO_MODE_OUTPUT_OD     0x11   /*!< Output Open Drain Mode */

#define GPIO_MODE_INPUT         0x00   /*!< Input Floating Mode */
#define GPIO_MODE_IN_RE         0x01   /*!< Input interrupt rising edge */
#define GPIO_MODE_IN_FE         0x02   /*!< Input interrupt falling edge */
#define GPIO_MODE_IN_AH         0x04   /*!< Input interrupt active high */
#define GPIO_MODE_IN_AL         0x08   /*!< Input interrupt active low */

#define GPIO_MODE_OUTPUT_PP     0x01   /*!< Output Push Pull Mode */
#define GPIO_MODE_OUTPUT_OD     0x11   /*!< Output Open Drain Mode */

//#define GPIO_INT_EDGE_RISING    0x1
//#define GPIO_INT_EDGE_FALLING   0x2
//#define GPIO_INT_MASK           (GPIO_INT_EDGE_RISING | GPIO_INT_EDGE_FALLING)
//#define GPIO_INT_MASK_SIZE      2

#define X8H7_GPIO_NUM   34

struct x8h7_gpio_info {
  struct device      *dev;
  wait_queue_head_t   wait;
  int                 rx_cnt;
  x8h7_pkt_t          rx_pkt;
  struct gpio_chip    gc;
  uint32_t            gpio_dir;
  uint32_t            gpio_val;
  uint32_t            gpio_ien;
  struct irq_domain  *irq;
};


static void x8h7_gpio_hook(void *priv, x8h7_pkt_t *pkt)
{
  struct x8h7_gpio_info  *inf = (struct x8h7_gpio_info*)priv;

  if ((pkt->peripheral == X8H7_GPIO_PERIPH) &&
      (pkt->opcode == X8H7_GPIO_OC_INT) &&
      (pkt->size == 1)) {
    if (pkt->data[0] < X8H7_GPIO_NUM) {
      int ret;
      ret = generic_handle_irq(irq_linear_revmap(inf->irq, pkt->data[0]));
      DBG_PRINT("call generic_handle_irq(%d) return %d\n",
                irq_linear_revmap(inf->irq, pkt->data[0]), ret);
    }
  } else {
    memcpy(&inf->rx_pkt, pkt, sizeof(x8h7_pkt_t));
    inf->rx_cnt++;
    wake_up_interruptible(&inf->wait);
  }
}

static int x8h7_gpio_pkt_get(struct x8h7_gpio_info *inf)
{
  long ret;

  ret = wait_event_interruptible_timeout(inf->wait,
                                         inf->rx_cnt != 0,
                                         X8H7_RX_TIMEOUT);
  if (!ret) {
    DBG_ERROR("timeout expired");
    return -1;
  }
  inf->rx_cnt--;
  return 0;
}

static int x8h7_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
  struct x8h7_gpio_info  *inf = gpiochip_get_data(chip);
  uint8_t                 data[2];

  DBG_PRINT("offset: %d\n", offset);
  if (offset >= inf->gc.ngpio) {
    DBG_ERROR("offset out of reange\n");
    return -EINVAL;
  }
  inf->gpio_dir &= ~(1 << offset);

  data[0] = offset;
  data[1] = GPIO_MODE_INPUT;

  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_DIR, 2, data);
  x8h7_pkt_send();

  DBG_PRINT(" dir %08X\n", inf->gpio_dir);
  return 0;
}

static int x8h7_gpio_get(struct gpio_chip *chip, unsigned offset)
{
  struct x8h7_gpio_info  *inf = gpiochip_get_data(chip);
  uint8_t                 data[1];

  DBG_PRINT("offset: %d\n", offset);
  if (offset >= inf->gc.ngpio) {
    DBG_ERROR("offset out of reange\n");
    return -EINVAL;
  }

  data[0] = offset;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_RD, 1, data);
  x8h7_pkt_send();
  x8h7_gpio_pkt_get(inf);
  if ((inf->rx_pkt.peripheral == X8H7_GPIO_PERIPH) &&
      (inf->rx_pkt.opcode == X8H7_GPIO_OC_RD) &&
      (inf->rx_pkt.size == 2)) {
    if (inf->rx_pkt.data[1]) {
      inf->gpio_val |= 1 << offset;
    } else {
      inf->gpio_val &= ~(1 << offset);
    }
  }
  DBG_PRINT("read %08X\n", inf->gpio_val);
  return !!(inf->gpio_val & (1 << offset));
}

static int x8h7_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
          int value)
{
  struct x8h7_gpio_info  *inf = gpiochip_get_data(chip);
  uint8_t                 data[2];

  DBG_PRINT("offset: %d, value: %d\n", offset, value);
  if (offset >= inf->gc.ngpio) {
    DBG_ERROR("offset out of reange\n");
    return -EINVAL;
  }
  inf->gpio_dir |= (1 << offset);
  if (value) {
    inf->gpio_val |= (1 << offset);
  } else {
    inf->gpio_val &= ~(1 << offset);
  }
  data[0] = offset;
  data[1] = !!value;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_WR, 2, data);
  data[1] = GPIO_MODE_OUTPUT_PP;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_DIR, 2, data);
  x8h7_pkt_send();

  DBG_PRINT("dir %08X write %08X\n", inf->gpio_dir, inf->gpio_val);
  return 0;
}

static void x8h7_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
  struct x8h7_gpio_info  *inf = gpiochip_get_data(chip);
  uint8_t                 data[2];

  DBG_PRINT("offset: %d, value: %d\n", offset, value);
  if (offset >= inf->gc.ngpio) {
    DBG_ERROR("offset out of reange\n");
    return;
  }

  if (value) {
    inf->gpio_val |= (1 << offset);
  } else {
    inf->gpio_val &= ~(1 << offset);
  }

  data[0] = offset;
  data[1] = !!value;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_WR, 2, data);
  x8h7_pkt_send();

  DBG_PRINT("write %08X\n", inf->gpio_val);
}

static int x8h7_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
  struct x8h7_gpio_info  *inf = gpiochip_get_data(chip);

  DBG_PRINT("offset: %d\n", offset);
  if (inf->gpio_dir & (1 << offset)) {
    return GPIOF_DIR_OUT;
  }
  return GPIOF_DIR_IN;
}


static int x8h7_gpio_set_config(struct gpio_chip *chip, unsigned int offset,
                                unsigned long config)
{
//  struct x8h7_gpio_info  *inf = gpiochip_get_data(chip);
  uint8_t                 data[2];

  DBG_PRINT("offset: %d, config: %ld\n", offset, config);
  data[0] = offset;
  switch (pinconf_to_config_param(config)) {
  case PIN_CONFIG_DRIVE_OPEN_DRAIN:
    data[1] = GPIO_MODE_OUTPUT_OD;
    break;
  case PIN_CONFIG_DRIVE_PUSH_PULL:
    data[1] = GPIO_MODE_OUTPUT_PP;
    break;
  default:
    return -ENOTSUPP;
  }
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_DIR, 2, data);
  x8h7_pkt_send();

  return 0;
}

static int x8h7_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
  struct x8h7_gpio_info  *inf = gpiochip_get_data(gc);

  if (inf->irq && offset < X8H7_GPIO_NUM) {
    int irq;
    irq = irq_create_mapping(inf->irq, offset);
    DBG_PRINT("offset %d, irq %d\n", offset, irq);
    return irq;
  } else {
    DBG_ERROR("\n");
    return -ENXIO;
  }
}

static void x8h7_gpio_irq_unmask(struct irq_data *d)
{
  struct x8h7_gpio_info  *inf = irq_data_get_irq_chip_data(d);
  uint8_t                 data[2];
  unsigned long           irq;

  irq = irqd_to_hwirq(d);
  inf->gpio_ien &= ~(1 << irq);
  DBG_PRINT("irq %ld, ien %08Xld\n", irq, inf->gpio_ien);

  data[0] = irq;
  data[1] = 1;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IEN, 2, data);
  x8h7_pkt_send();
}

static void x8h7_gpio_irq_mask(struct irq_data *d)
{
  struct x8h7_gpio_info  *inf = irq_data_get_irq_chip_data(d);
  uint8_t                 data[2];
  unsigned long           irq;

  irq = irqd_to_hwirq(d);
  inf->gpio_ien |= (1 << irq);
  DBG_PRINT("irq %ld, ien %08Xld\n", irq, inf->gpio_ien);

  data[0] = irq;
  data[1] = 0;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IEN, 2, data);
  x8h7_pkt_send();
}

static void x8h7_gpio_irq_ack(struct irq_data *d)
{
//  struct x8h7_gpio_info  *inf = irq_data_get_irq_chip_data(d);
  uint8_t                 data[1];
  unsigned long           irq;

  irq = irqd_to_hwirq(d);
  DBG_PRINT("irq %ld\n", irqd_to_hwirq(d));
  data[0] = irq;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IACK, 1, data);
  x8h7_pkt_send();
}

static int x8h7_gpio_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
  //struct x8h7_gpio_info *inf = irq_data_get_irq_chip_data(d);
  uint8_t                 data[2];
  //struct gpio_chip *gc = &inf->gc;
  //unsigned long flags;

  DBG_PRINT("irq %ld flow_type %d\n", irqd_to_hwirq(d), flow_type);

  switch (flow_type) {
  case IRQ_TYPE_EDGE_RISING:
    data[1] = GPIO_MODE_IN_RE;
    break;
  case IRQ_TYPE_EDGE_FALLING:
    data[1] = GPIO_MODE_IN_FE;
    break;
    break;
  case IRQ_TYPE_EDGE_BOTH:
    data[1] = GPIO_MODE_IN_RE | GPIO_MODE_IN_FE;
    break;
  case IRQ_TYPE_LEVEL_HIGH:
    data[1] = GPIO_MODE_IN_AH;
    break;
  case IRQ_TYPE_LEVEL_LOW:
    data[1] = GPIO_MODE_IN_AL;
    break;
  default:
    return -EINVAL;
  }

  data[0] = irqd_to_hwirq(d);
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IRQ_TYPE, 2, data);
  x8h7_pkt_send();
  return 0;
}

static struct irq_chip x8h7_gpio_irq_chip = {
  .name         = "x8h7-gpio",
  .irq_unmask   = x8h7_gpio_irq_unmask,
  .irq_mask     = x8h7_gpio_irq_mask,
  .irq_ack      = x8h7_gpio_irq_ack,
  .irq_set_type = x8h7_gpio_irq_set_type,
};

static int x8h7_gpio_irq_map(struct irq_domain *h, unsigned int irq,
                             irq_hw_number_t hwirq)
{
  irq_set_chip_data(irq, h->host_data);
  irq_set_chip_and_handler(irq, &x8h7_gpio_irq_chip, handle_edge_irq);

  return 0;
}

static const struct irq_domain_ops x8h7_gpio_irq_ops = {
  .map   = x8h7_gpio_irq_map,
  .xlate = irq_domain_xlate_twocell,
};

static int x8h7_gpio_probe(struct platform_device *pdev)
{
  struct x8h7_gpio_info  *inf;
  struct device_node     *node = pdev->dev.of_node;
  int                     base;
  int                     ret;

  inf = devm_kzalloc(&pdev->dev, sizeof(*inf), GFP_KERNEL);
  if (!inf) {
    return -ENODEV;
  }

  ret = of_property_read_u32(node, "base", &base);
  if (ret) {
    base = 160;
    DBG_ERROR("reading GPIO base from DTB failed, use default\n");
  }

  inf->dev = &pdev->dev;
  init_waitqueue_head(&inf->wait);
  inf->rx_cnt = 0;

  inf->gc.label            = "x8h7_gpio";
  inf->gc.direction_input  = x8h7_gpio_direction_input;
  inf->gc.get              = x8h7_gpio_get;
  inf->gc.direction_output = x8h7_gpio_direction_output;
  inf->gc.set              = x8h7_gpio_set;
  inf->gc.get_direction    = x8h7_gpio_get_direction;
  inf->gc.set_config       = x8h7_gpio_set_config;
  inf->gc.to_irq           = x8h7_gpio_to_irq;
  inf->gc.base             = base;
  inf->gc.ngpio            = X8H7_GPIO_NUM;
  inf->gc.parent           = &pdev->dev;
  inf->gc.of_node          = pdev->dev.of_node;

  inf->irq = irq_domain_add_linear(node, X8H7_GPIO_NUM,
                                   &x8h7_gpio_irq_ops, inf);
  if (!inf->irq) {
    DBG_ERROR("Failed to add irq domain\n");
    //return 0;
  }

  platform_set_drvdata(pdev, inf);

  ret = devm_gpiochip_add_data(inf->dev, &inf->gc, inf);
  if (ret < 0) {
    DBG_ERROR("Failed to add gpio\n");
    return ret;
  }

  x8h7_hook_set(X8H7_GPIO_PERIPH, x8h7_gpio_hook, inf);

  return 0;
}

static int x8h7_gpio_remove(struct platform_device *pdev)
{
//  struct x8h7_gpio_info *inf = platform_get_drvdata(pdev);

  return 0;
}

static const struct of_device_id x8h7_gpio_of_match[] = {
  { .compatible = "portenta,x8h7_gpio"},
  { },
};

static struct platform_driver x8h7_gpio_driver = {
  .driver = {
    .name           = "x8h7_gpio",
    .of_match_table = x8h7_gpio_of_match,
  },
  .probe  = x8h7_gpio_probe,
  .remove = x8h7_gpio_remove,
};

module_platform_driver(x8h7_gpio_driver);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_DESCRIPTION("Arduino Portenta X8 GPIO driver");
MODULE_LICENSE("GPL v2");
