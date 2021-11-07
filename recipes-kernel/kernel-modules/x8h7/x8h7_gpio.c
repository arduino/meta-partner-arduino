/**
 * X8H7 GPIO driver
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include "x8h7_ioctl.h"
#include "x8h7.h"

#define DRIVER_NAME "x8h7_gpio"

#define DEBUG
#include "debug.h"

// Peripheral code
#define X8H7_GPIO_PERIPH 0x07
// Op code
#define X8H7_GPIO_OC_DIR 0x10
#define X8H7_GPIO_OC_WR  0x20
#define X8H7_GPIO_OC_RD  0x30

struct x8h7_gpio_info {
  struct device      *dev;
  wait_queue_head_t   wait;
  int                 rx_cnt;
  x8h7_pkt_t          rx_pkt;
  struct gpio_chip    gc;
  uint32_t            gpio_dir;
  uint32_t            gpio_val;
};

#define  GPIO_MODE_INPUT       0x00   /*!< Input Floating Mode */
#define  GPIO_MODE_OUTPUT_PP   0x01   /*!< Output Push Pull Mode */
#define  GPIO_MODE_OUTPUT_OD   0x11   /*!< Output Open Drain Mode */

static void x8h7_gpio_hook(void *priv, x8h7_pkt_t *pkt)
{
  struct x8h7_gpio_info  *inf = (struct x8h7_gpio_info*)priv;

  memcpy(&inf->rx_pkt, pkt, sizeof(x8h7_pkt_t));
  inf->rx_cnt++;
  wake_up_interruptible(&inf->wait);
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
  struct x8h7_gpio_info  *inf = gpiochip_get_data(chip);
  uint8_t                 data[2];

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
  inf->gc.to_irq           = NULL;
  inf->gc.base             = base;
  inf->gc.ngpio            = 7;
  inf->gc.parent           = &pdev->dev;
  inf->gc.of_node          = pdev->dev.of_node;

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
