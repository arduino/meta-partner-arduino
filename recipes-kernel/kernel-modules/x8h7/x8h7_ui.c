/**
 * User Code Interface
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "x8h7.h"

#define DRIVER_NAME "x8h7_ui"

//#define DEBUG
#include "debug.h"

// Peripheral code
#define X8H7_UI_PERIPH      0x0A
// Op code
#define X8H7_UI_OC_DATA     0x01

#define X8H7_UI_DATA_MAX    (1 * 1024)

struct x8h7_ui_priv {
  struct device      *dev;
  dev_t               dev_num;
  struct cdev         cdev;
  struct class       *cl;

  uint8_t             rx_data[X8H7_UI_DATA_MAX];
  uint16_t            rx_len;
};

static DECLARE_WAIT_QUEUE_HEAD(wq);

struct x8h7_ui_priv *x8h7_ui;

static void x8h7_ui_hook(void *prv, x8h7_pkt_t *pkt)
{
  struct x8h7_ui_priv  *priv = (struct x8h7_ui_priv*)prv;

  //DBG_PRINT("received %d bytes\n", pkt->size);
  if (priv->rx_len + pkt->size > X8H7_UI_DATA_MAX) {
    goto wake_read;
  }

  memcpy(&priv->rx_data[priv->rx_len], pkt->data, pkt->size);
  priv->rx_len += pkt->size;

wake_read:
  wake_up_interruptible(&wq);
}

static int x8h7_ui_open(struct inode *inode, struct file *file)
{
//  DBG_PRINT("\n");
  return 0;
}

static int x8h7_ui_release(struct inode *inode, struct file *file)
{
//  DBG_PRINT("\n");
  return 0;
}

/*
static long x8h7_ui_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  DBG_PRINT("\n");
  return 0;
}
*/

static ssize_t x8h7_ui_read(struct file *file,
                            char __user *buf, size_t count, loff_t *offset)
{
  struct x8h7_ui_priv *priv = x8h7_ui;
  ssize_t ret;

  // call this only in case of O_BLOCK
  wait_event_interruptible(wq, priv->rx_len != 0);

  *offset = 0;
  //DBG_PRINT("cpoy to user %d bytes\n", count);
  ret = simple_read_from_buffer(buf, count, offset, priv->rx_data, priv->rx_len);
  priv->rx_len = 0;

  return ret;
}

static ssize_t x8h7_ui_write(struct file *file,
                             const char __user *buf, size_t count, loff_t *offset)
{
  size_t          len;
  unsigned long   ret;
  uint8_t         data[X8H7_UI_DATA_MAX];

  if (count >= X8H7_UI_DATA_MAX) {
    len = X8H7_UI_DATA_MAX;
  } else {
    len = count;
  }

  ret = copy_from_user(data, buf, len);
  if (ret) {
    DBG_ERROR("Could't copy %zd bytes from the user\n", ret);
    return -EFAULT;
  }

  x8h7_pkt_enq(X8H7_UI_PERIPH, X8H7_UI_OC_DATA, len, data);
  x8h7_pkt_send();

  return len;
}

 struct file_operations fops = {
  .open    = x8h7_ui_open,
  .release = x8h7_ui_release,
  .read    = x8h7_ui_read,
  .write   = x8h7_ui_write,
};

static int x8h7_ui_probe(struct platform_device *pdev)
{
  struct x8h7_ui_priv    *priv;
//  struct device_node     *node = pdev->dev.of_node;
  int                     ret;

  priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
  if (!priv) {
    return -ENODEV;
  }
/*
  ret = of_property_read_u32(node, "base", &base);
  if (ret) {
    base = 0;
    DBG_ERROR("reading param from DTB failed, use default\n");
  }
*/
  x8h7_ui = priv;
  platform_set_drvdata(pdev, priv);

  /* we will get the major number dynamically this is recommended please read ldd3*/
  ret = alloc_chrdev_region(&priv->dev_num, 0, 1, DRIVER_NAME);
  if (ret < 0) {
    DBG_ERROR("failed to allocate major number\n");
    return ret;
  }
  DBG_PRINT("major number of our device is %d\n", MAJOR(priv->dev_num));

  DBG_PRINT("Class creation\n");
  priv->cl = class_create(THIS_MODULE, DRIVER_NAME);
  if (priv->cl == NULL) {
    DBG_ERROR("Class creation failed\n");
    unregister_chrdev_region(priv->dev_num, 1);
    return -1;
  }

  DBG_PRINT("Device creation\n");
  priv->dev = device_create(priv->cl, NULL, priv->dev_num, NULL, DRIVER_NAME);
  if (IS_ERR(priv->dev)) {
    DBG_ERROR("Device creation failed\n");
    class_destroy(priv->cl);
    unregister_chrdev_region(priv->dev_num, 1);
    return -1;
  }

  DBG_PRINT("Device addition\n");
  cdev_init(&priv->cdev, &fops);
  if (cdev_add(&priv->cdev, priv->dev_num, 1) == -1) {
    DBG_ERROR("Device addition failed\n");
    device_destroy(priv->cl, priv->dev_num);
    class_destroy(priv->cl);
    unregister_chrdev_region(priv->dev_num, 1);
    return -1;
  }

  x8h7_ui->rx_len = 0;

  x8h7_hook_set(X8H7_UI_PERIPH, x8h7_ui_hook, priv);

  return 0;
}

static int x8h7_ui_remove(struct platform_device *pdev)
{
  struct x8h7_ui_priv *priv = platform_get_drvdata(pdev);

  x8h7_hook_set(X8H7_UI_PERIPH, NULL, NULL);
  cdev_del(&priv->cdev);
  device_destroy(priv->cl, priv->dev_num);
  class_destroy(priv->cl);
  unregister_chrdev_region(priv->dev_num, 1);
  return 0;
}

static const struct of_device_id x8h7_ui_of_match[] = {
  { .compatible = "portenta,x8h7_ui"},
  { },
};

static struct platform_driver x8h7_ui_driver = {
  .driver = {
    .name           = "x8h7_ui",
    .of_match_table = x8h7_ui_of_match,
  },
  .probe  = x8h7_ui_probe,
  .remove = x8h7_ui_remove,
};

module_platform_driver(x8h7_ui_driver);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_DESCRIPTION("Arduino Portenta X8 GPIO driver");
MODULE_LICENSE("GPL v2");
