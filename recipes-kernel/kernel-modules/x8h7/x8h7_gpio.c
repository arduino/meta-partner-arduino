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
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/workqueue.h>

#include "x8h7.h"

#define DRIVER_NAME "x8h7_gpio"

//#define DEBUG
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
  int                 tx_cnt;
  x8h7_pkt_t          rx_pkt;
  struct pinctrl_dev *pctldev;
  struct pinctrl_desc pinctrl_desc;
  struct gpio_chip    gc;
  uint32_t            gpio_dir;
  uint32_t            gpio_val;
  uint8_t             gpio_ien;
  uint8_t             irq_conf;
  struct irq_domain  *irq;
  struct mutex lock;
  struct work_struct work;
  struct workqueue_struct *workqueue;
  uint64_t ack_irq;
};

// @TODO: add remaining gpios
static const struct pinctrl_pin_desc x8h7_gpio_34_pins[] = {
  PINCTRL_PIN(0, "gpio0"),
  PINCTRL_PIN(1, "gpio1"),
  PINCTRL_PIN(2, "gpio2"),
  PINCTRL_PIN(3, "gpio3"),
  PINCTRL_PIN(4, "gpio4"),
  PINCTRL_PIN(5, "gpio5"),
  PINCTRL_PIN(6, "gpio6"),
  PINCTRL_PIN(7, "gpio7"),
  PINCTRL_PIN(8, "gpio8"),
  PINCTRL_PIN(9, "gpio9"),
  PINCTRL_PIN(10, "gpio10"),
  PINCTRL_PIN(11, "gpio11"),
  PINCTRL_PIN(12, "gpio12"),
  PINCTRL_PIN(13, "gpio13"),
  PINCTRL_PIN(14, "gpio14"),
  PINCTRL_PIN(15, "gpio15"),
  PINCTRL_PIN(16, "gpio16"),
  PINCTRL_PIN(17, "gpio17"),
  PINCTRL_PIN(18, "gpio18"),
  PINCTRL_PIN(19, "gpio19"),
  PINCTRL_PIN(20, "gpio20"),
  PINCTRL_PIN(21, "gpio21"),
  PINCTRL_PIN(22, "gpio22"),
  PINCTRL_PIN(23, "gpio23"),
  PINCTRL_PIN(24, "gpio24"),
  PINCTRL_PIN(25, "gpio25"),
  PINCTRL_PIN(26, "gpio26"),
  PINCTRL_PIN(27, "gpio27"),
  PINCTRL_PIN(28, "gpio28"),
  PINCTRL_PIN(29, "gpio29"),
  PINCTRL_PIN(30, "gpio30"),
  PINCTRL_PIN(31, "gpio31"),
  PINCTRL_PIN(32, "gpio32"),
  PINCTRL_PIN(33, "gpio33"),
};

/* We can't use x8h7_pkt_send directly in x8h7_gpio_hook since it's a deadlock */
static void x8h7_gpio_irq_ack(struct x8h7_gpio_info *inf)
{
  if(inf->workqueue)
    queue_work(inf->workqueue, &inf->work);
}

/* Worqueue for gpio_irq_ack handling */
static void gpio_irq_ack_work_func(struct work_struct *work)
{
  struct x8h7_gpio_info *inf = container_of(work, struct x8h7_gpio_info, work);
  uint8_t                 data[2];

  DBG_PRINT("\n");
  data[0] = inf->ack_irq;
  data[1] = 0x55;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IACK, 2, data);
  x8h7_pkt_send();
  return;
}

static void x8h7_gpio_hook(void *priv, x8h7_pkt_t *pkt)
{
  struct x8h7_gpio_info  *inf = (struct x8h7_gpio_info*)priv;
  unsigned long irq = 0;
  uint8_t hwirq = 0;

  if ((pkt->peripheral == X8H7_GPIO_PERIPH) &&
      (pkt->opcode == X8H7_GPIO_OC_INT) &&
      (pkt->size == 1)) {
    if (pkt->data[0] < X8H7_GPIO_NUM) {
      hwirq = pkt->data[0];
      irq = irq_linear_revmap(inf->irq, hwirq);
      handle_nested_irq(irq);
      DBG_PRINT("call handle_nested_irq(%d)\n", hwirq);
      inf->ack_irq = hwirq;
      x8h7_gpio_irq_ack(inf);
      DBG_PRINT("call x8h7_gpio_irq_ack(%d)\n", inf->ack_irq);
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
  if (x8h7_gpio_pkt_get(inf) < 0)
    return -ETIMEDOUT;

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
  inf->gpio_ien = 1;

  // Send mask
  data[0] = irq;
  data[1] = inf->gpio_ien;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IEN, 2, data);
  inf->tx_cnt++;

  DBG_PRINT("irq %ld, ien %02X\n", irq, inf->gpio_ien);
}

static void x8h7_gpio_irq_mask(struct irq_data *d)
{
  struct x8h7_gpio_info  *inf = irq_data_get_irq_chip_data(d);
  uint8_t                 data[2];
  unsigned long           irq;

  irq = irqd_to_hwirq(d);
  inf->gpio_ien = 0;

  // Send mask
  data[0] = irq;
  data[1] = inf->gpio_ien;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IEN, 2, data);
  inf->tx_cnt++;

  DBG_PRINT("irq %ld, ien %02X\n", irq, inf->gpio_ien);
}

static int x8h7_gpio_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
  struct x8h7_gpio_info *inf = irq_data_get_irq_chip_data(d);
  uint8_t                 data[2];
  unsigned long           irq;

  irq = irqd_to_hwirq(d);
  DBG_PRINT("irq %ld flow_type %d\n", irq, flow_type);

  switch (flow_type) {
  case IRQ_TYPE_EDGE_RISING:
    inf->irq_conf = GPIO_MODE_IN_RE;
    break;
  case IRQ_TYPE_EDGE_FALLING:
    inf->irq_conf = GPIO_MODE_IN_FE;
    break;
  case IRQ_TYPE_EDGE_BOTH:
    inf->irq_conf = GPIO_MODE_IN_RE | GPIO_MODE_IN_FE;
    break;
  case IRQ_TYPE_LEVEL_HIGH:
    DBG_ERROR("trigger on active high not supported.\n");
    return -EINVAL;
    break;
  case IRQ_TYPE_LEVEL_LOW:
    DBG_ERROR("trigger on active low not supported.\n");
    return -EINVAL;
    break;
  default:
    return -EINVAL;
  }

  // Send interrupt type
  data[0] = irq;
  data[1] = inf->irq_conf;
  x8h7_pkt_enq(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_IRQ_TYPE, 2, data);
  inf->tx_cnt++;

  return 0;
}

static void x8h7_gpio_irq_bus_lock(struct irq_data *d)
{
  struct x8h7_gpio_info *inf = irq_data_get_irq_chip_data(d);

  DBG_PRINT("\n");
  mutex_lock(&inf->lock);
}

/* This is where we actually send spi packets to sync irq data */
static void x8h7_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
  struct x8h7_gpio_info *inf = irq_data_get_irq_chip_data(d);

  DBG_PRINT("\n");

  /* Invoke send only if there are pending events */
  if(inf->tx_cnt != 0) {
    x8h7_pkt_send();
    inf->tx_cnt = 0;
  }
  mutex_unlock(&inf->lock);
}

static struct irq_chip x8h7_gpio_irq_chip = {
  .name         = "x8h7_gpio-irq",
  .irq_unmask   = x8h7_gpio_irq_unmask,
  .irq_mask     = x8h7_gpio_irq_mask,
  .irq_set_type = x8h7_gpio_irq_set_type,
  .irq_bus_lock = x8h7_gpio_irq_bus_lock,
  .irq_bus_sync_unlock = x8h7_gpio_irq_bus_sync_unlock,
};

static int x8h7_gpio_irq_map(struct irq_domain *h, unsigned int irq,
                             irq_hw_number_t hwirq)
{
  irq_set_chip_data(irq, h->host_data);
  irq_set_chip_and_handler(irq, &x8h7_gpio_irq_chip, handle_edge_irq);
  irq_set_nested_thread(irq, 1);
  irq_set_noprobe(irq);

  return 0;
}

static const struct irq_domain_ops x8h7_gpio_irq_ops = {
  .map   = x8h7_gpio_irq_map,
  .xlate = irq_domain_xlate_twocell,
};

static int x8h7_gpio_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
  return 0;
}

static const char *x8h7_gpio_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
                                                 unsigned int group)
{
  return NULL;
}

static int x8h7_gpio_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
                                         unsigned int group,
                                         const unsigned int **pins,
                                         unsigned int *num_pins)
{
  return -ENOTSUPP;
}

static const struct pinctrl_ops x8h7_gpio_pinctrl_ops = {
  .get_groups_count = x8h7_gpio_pinctrl_get_groups_count,
  .get_group_name = x8h7_gpio_pinctrl_get_group_name,
  .get_group_pins = x8h7_gpio_pinctrl_get_group_pins,
// @TODO: fixme map ops needed?
#if 0
#ifdef CONFIG_OF
  dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
  dt_free_map = pinctrl_utils_free_map,
#endif
#endif
};

static int x8h7_gpio_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
                              unsigned long *config)
{
  struct x8h7_gpio_info *inf = pinctrl_dev_get_drvdata(pctldev);
  unsigned int param = pinconf_to_config_param(*config);
  int ret;
  u32 arg;

  switch (param) {
  case PIN_CONFIG_OUTPUT:
    ret = x8h7_gpio_get_direction(&inf->gc, pin);
    if (ret < 0)
      return ret;

    if (ret)
      return -EINVAL;

    ret = x8h7_gpio_get(&inf->gc, pin);
    if (ret < 0)
      return ret;

    arg = ret;
    break;

  default:
    DBG_ERROR("Feature not supported, param = %d", param);
    return -ENOTSUPP;
  }

  *config = pinconf_to_config_packed(param, arg);

  return 0;
}

static int x8h7_gpio_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
                              unsigned long *configs, unsigned int num_configs)
{
  struct x8h7_gpio_info *inf = pinctrl_dev_get_drvdata(pctldev);
  enum pin_config_param param;
  u32 arg;
  int i;
  int ret;

  for (i = 0; i < num_configs; i++) {
    param = pinconf_to_config_param(configs[i]);
    arg = pinconf_to_config_argument(configs[i]);

    switch (param) {
    case PIN_CONFIG_OUTPUT:
      ret = x8h7_gpio_direction_output(&inf->gc,
                                         pin, arg);
      if (ret < 0)
        return ret;

      break;

    default:
      DBG_ERROR("Feature not supported, param = %d", param);
      return -ENOTSUPP;
    }
  } /* for each config */

  return 0;
}

static const struct pinconf_ops x8h7_gpio_pinconf_ops = {
  .pin_config_get = x8h7_gpio_pinconf_get,
  .pin_config_set = x8h7_gpio_pinconf_set,
  .is_generic = true,
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
  inf->tx_cnt = 0;

  mutex_init(&inf->lock);

  INIT_WORK(&inf->work, gpio_irq_ack_work_func);
  inf->workqueue = create_workqueue("x8h7_gpio_irq_ack_work");
  if (!inf->workqueue) {
    DBG_ERROR("Failed to create work queue\n");
    ret = -ENOMEM;
  }

  /* Pinctrl_desc */
  inf->pinctrl_desc.name = "x8h7_gpio-pinctrl";
  inf->pinctrl_desc.pctlops = &x8h7_gpio_pinctrl_ops;
  inf->pinctrl_desc.confops = &x8h7_gpio_pinconf_ops;
  inf->pinctrl_desc.pins = x8h7_gpio_34_pins;
  inf->pinctrl_desc.npins = X8H7_GPIO_NUM;
  inf->pinctrl_desc.owner = THIS_MODULE;

  ret = devm_pinctrl_register_and_init(inf->dev, &inf->pinctrl_desc,
               inf, &inf->pctldev);
  if (ret) {
    DBG_ERROR("Failed to register pinctrl device\n");
    return ret;
  }

  ret = pinctrl_enable(inf->pctldev);
  if (ret) {
    DBG_ERROR("Failed to enable pinctrl device\n");
    return ret;
  }

  /* Register GPIO controller */
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
#ifdef CONFIG_OF_GPIO
  inf->gc.of_node          = pdev->dev.of_node;
#endif

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
