/**
 * H7 coprocessor Interface
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "x8h7.h"
#include "x8h7_ioctl.h"

#define DRIVER_NAME "x8h7_h7"

//#define DEBUG
#include "debug.h"

// Peripheral code
#define X8H7_H7_PERIPH      0x09
// Op code
#define X8H7_H7_OC_FW_GET   0x10

#define X8H7_H7_DATA_MAX    (4096)
#define X8H7_H7_LIST_SIZE   16
#define X8H7_H7_RX_TIMEOUT  (2 * HZ)


struct x8h7_h7_buf {
  uint8_t             data[X8H7_H7_DATA_MAX];
  uint16_t            len;
  struct list_head    list;
};

struct x8h7_h7_priv {
  struct device      *dev;
  dev_t               dev_num;
  struct cdev         cdev;
  struct class       *cl;

  uint32_t            mode;

  wait_queue_head_t   wait;
  int                 cnt;
/*
  uint8_t             rx_data[X8H7_H7_DATA_MAX];
  uint16_t            rx_len;
*/
  x8h7_pkt_t          rx_pkt;

  int                 rxindex;
  struct list_head    rxqueue;
  struct x8h7_h7_buf  buf[X8H7_H7_LIST_SIZE];
};

struct x8h7_h7_priv *x8h7_h7;
/*
static void x8h7_h7_init_rx_queue(struct x8h7_h7_priv *priv)
{
  int i;

  INIT_LIST_HEAD(&priv->rxqueue);
  for (i=0; i<X8H7_H7_LIST_SIZE; i++) {
    priv->buf[i].buf.flags = V4L2_BUF_FLAG_MAPPED;
    priv->buf[i].buf.bytesused = 0;
  }
}
*/

static void x8h7_h7_hook(void *prv, x8h7_pkt_t *pkt)
{
  struct x8h7_h7_priv *priv = (struct x8h7_h7_priv *)prv;
  /*
  if ((pkt->peripheral == X8H7_H7_PERIPH) &&
      (pkt->opcode == X8H7_H7_OC_FW_GET) &&
      (pkt->size >= 1)) {
    // copy fw version
    pkt->data[pkt->size] = 0;
    DBG_PRINT("FW ver size %d %s\n", pkt->size, pkt->data);
  }
*/
  memcpy(&priv->rx_pkt, pkt, sizeof(x8h7_pkt_t));
  priv->cnt++;
  wake_up_interruptible(&priv->wait);
}

static int x8h7_h7_pkt_get(struct x8h7_h7_priv *priv)
{
  long ret;

  ret = wait_event_interruptible_timeout(priv->wait,
                                         priv->cnt != 0,
                                         X8H7_RX_TIMEOUT);
  if (!ret) {
    DBG_ERROR("timeout expired");
    return -1;
  }
  priv->cnt--;
  return 0;
}

static void x8h7_h7_dbg(void *prv, uint8_t *data, uint16_t len)
{
  struct x8h7_h7_priv  *priv = (struct x8h7_h7_priv*)prv;

  //if (priv->cnt) {
    //DBG_ERROR("Receive override cnt %d\n", priv->cnt);
  //}
  //DBG_PRINT("Received %d bytes\n", len);
/**/
  memcpy(priv->buf[priv->rxindex].data, data, len);
  priv->buf[priv->rxindex].len = len;
  list_add_tail(&priv->buf[priv->rxindex].list, &priv->rxqueue);
  priv->rxindex++;
  if (priv->rxindex >= X8H7_H7_LIST_SIZE) {
    priv->rxindex = 0;
  }
/**/
/*
  memcpy(priv->rx_data, data, len);
  priv->rx_len = len;
*/
  priv->cnt++;
  wake_up_interruptible(&priv->wait);
}

static int x8h7_h7_open(struct inode *inode, struct file *file)
{
//  DBG_PRINT("\n");
  return 0;
}

static int x8h7_h7_release(struct inode *inode, struct file *file)
{
//  DBG_PRINT("\n");
  return 0;
}

static ssize_t x8h7_h7_read(struct file *file,
                            char __user *buf, size_t count, loff_t *offset)
{
  struct x8h7_h7_priv  *priv = x8h7_h7;
  long                  ret;
  struct x8h7_h7_buf *rxbuf;

  if ((priv->mode & X8H7_MODE_DEBUG) == 0) {
    return -1;
  }
  ret = wait_event_interruptible_timeout(priv->wait,
                                         priv->cnt != 0,
                                         X8H7_H7_RX_TIMEOUT);
  if (!ret) {
    return -1;
  }
/**/
  rxbuf = list_entry(priv->rxqueue.next, struct x8h7_h7_buf, list);
  list_del(priv->rxqueue.next);
  if (count > rxbuf->len) {
    count = rxbuf->len;
  }
  if (rxbuf->len) {
    rxbuf->len = 0;
    //DBG_PRINT("copy to user %d bytes\n", count);
    if (copy_to_user(buf, rxbuf->data, count)) {
      return -EFAULT;
    }
  }
/**/
/*
  if (count > priv->rx_len) {
    count = priv->rx_len;
  }
  if (priv->rx_len) {
    priv->rx_len = 0;
    DBG_PRINT("copy to user %d bytes\n", count);
    if (copy_to_user(buf, priv->rx_data, count)) {
      return -EFAULT;
    }
  }
*/
  priv->cnt--;
  return count;
}

static ssize_t x8h7_h7_write(struct file *file,
                             const char __user *buf, size_t count, loff_t *offset)
{
  /*
  size_t          len;
  unsigned long   ret;
  uint8_t         data[X8H7_H7_DATA_MAX];

  if (count >= X8H7_H7_DATA_MAX) {
    len = X8H7_H7_DATA_MAX;
  } else {
    len = count;
  }

  ret = copy_from_user(data, buf, len);
  if (ret) {
    DBG_ERROR("Could't copy %zd bytes from the user\n", ret);
    return -EFAULT;
  }

  x8h7_pkt_enq(X8H7_H7_PERIPH, X8H7_H7_OC_DATA, len, data);
  x8h7_pkt_send();

  return len;
  */
  return 0;
}

static long x8h7_h7_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  struct x8h7_h7_priv  *priv = x8h7_h7;
  u32                   tmp;
  x8h7_pkt_t            pkt;
  long                  retval = 0;

  DBG_PRINT("cmd %X\n", cmd);

  if (unlikely(_IOC_TYPE(cmd) != X8H7_IOCTL_MAGIC)) {
    DBG_ERROR("ioctl called with wrong magic number: %d\n", _IOC_TYPE(cmd));
    return -ENOTTY;
  }

  if (unlikely(_IOC_NR(cmd) > X8H7_IOCTL_MAXNR)) {
    DBG_ERROR("ioctl called with wrong command number: %d\n", _IOC_NR(cmd));
    return -ENOTTY;
  }
  switch (cmd) {
  case X8H7_IOCTL_FW_VER:
    x8h7_pkt_enq(X8H7_H7_PERIPH, X8H7_H7_OC_FW_GET, 0, NULL);
    x8h7_pkt_send();
    retval = x8h7_h7_pkt_get(priv);
    if (retval < 0) {
      return -ETIMEDOUT;
    }
    if ((priv->rx_pkt.peripheral == X8H7_H7_PERIPH) &&
        (priv->rx_pkt.opcode == X8H7_H7_OC_FW_GET) &&
        (priv->rx_pkt.size >= 1)) {

      if (copy_to_user((void __user *)arg, &priv->rx_pkt, sizeof(x8h7_pkt_t))) {
        DBG_ERROR("couldn't version information to user.");
        return -EFAULT;
      }
      //return priv->rx_pkt.size;
      //pkt->data[pkt->size] = 0;
      //DBG_PRINT("FW ver size %d %s\n", pkt->size, pkt->data);
      retval = 0;
    } else {
      return -EFAULT;
    }

    //pkt_dump("FW Version", (void*)&x8h7_spidev->rx_pkt);
    break;
  case X8H7_IOCTL_MODE_SET:
    retval = get_user(tmp, (u32 __user *)arg);
    if (retval == 0) {
      priv->mode = tmp;
      DBG_PRINT("Set mode to %08X\n", priv->mode);
      if (priv->mode & X8H7_MODE_DEBUG) {
        x8h7_dbg_set(x8h7_h7_dbg, x8h7_h7);
      } else {
        x8h7_dbg_set(NULL, NULL);
      }
    }
    break;
  case X8H7_IOCTL_MODE_GET:
    retval = put_user(priv->mode, (__u32 __user *)arg);
    break;
  case X8H7_IOCTL_PKT_ENQ:
    retval = copy_from_user(&pkt, (void __user *)arg, sizeof(pkt));
    if (retval == 0) {
      retval = x8h7_pkt_enq(pkt.peripheral, pkt.opcode, pkt.size, pkt.data);
      DBG_PRINT("x8h7_pkt_enq(%02X, %02X, %04X, ...) return %d\n",
                pkt.peripheral, pkt.opcode, pkt.size, retval);
    }
    break;
  case X8H7_IOCTL_PKT_SEND:
    retval = x8h7_pkt_send();
    DBG_PRINT("x8h7_pkt_send() return: %ld\n", retval);
    break;
  default:
    retval = -ENOTTY;
    break;
  }
/*
@TODO: if needed
  case X8H7_IOCTL_INT_WAIT:
  case X8H7_IOCTL_PKT_INIT:
*/
  if (retval) {
    DBG_ERROR("ioctl return error: %ld\n", retval);
  }
  return retval;
}


ssize_t x8h7_read_firmware_version(char * buf, size_t const buf_size)
{
  struct x8h7_h7_priv * priv = x8h7_h7;

  x8h7_pkt_enq(X8H7_H7_PERIPH, X8H7_H7_OC_FW_GET, 0, NULL);
  x8h7_pkt_send();
  if (x8h7_h7_pkt_get(priv) < 0)
    return -ETIMEDOUT;

  if ((priv->rx_pkt.peripheral == X8H7_H7_PERIPH) &&
      (priv->rx_pkt.opcode == X8H7_H7_OC_FW_GET) &&
      (priv->rx_pkt.size >= 1)) {

      memcpy(buf,
             &priv->rx_pkt.data,
             (priv->rx_pkt.size < buf_size) ? priv->rx_pkt.size : buf_size);
  } else {
    return -EFAULT;
  }

  return strlen(buf);
}

/* This function allows to read the current firmware of
 * from the H7 and display it by reading from a sysfs node,
 * i.e.
 *   $ cat /sys/kernel/x8h7_firmware/version
 *   0.0.2-next-15c3fee-20220303-dirty
 */
static ssize_t sysfs_show_version(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
        return x8h7_read_firmware_version(buf, PAGE_SIZE);
}

struct kobject *kobj_ref_x8h7_firmware_version;
struct kobj_attribute x8h7_firmware_version_attr = __ATTR(version, 0444, sysfs_show_version, NULL);


 struct file_operations fops = {
  .open           = x8h7_h7_open,
  .release        = x8h7_h7_release,
  .read           = x8h7_h7_read,
  .write          = x8h7_h7_write,
  .unlocked_ioctl = x8h7_h7_ioctl,
};

static int x8h7_h7_probe(struct platform_device *pdev)
{
  struct x8h7_h7_priv    *priv;
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
  x8h7_h7 = priv;
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

  init_waitqueue_head(&priv->wait);
/**/
  INIT_LIST_HEAD(&priv->rxqueue);
  priv->rxindex = 0;
/**/
  x8h7_hook_set(X8H7_H7_PERIPH, x8h7_h7_hook, priv);

  /* Creating a sysfs entry for reading the
   * firmware version of the X8H7 firmware.
   */
  kobj_ref_x8h7_firmware_version = kobject_create_and_add("x8h7_firmware", kernel_kobj);
  if(sysfs_create_file(kobj_ref_x8h7_firmware_version, &x8h7_firmware_version_attr.attr)){
    DBG_ERROR("Cannot create 'x8h7_firmware' sysfs file\n");
  }

  return 0;
}

static int x8h7_h7_remove(struct platform_device *pdev)
{
  struct x8h7_h7_priv *priv = platform_get_drvdata(pdev);

  x8h7_hook_set(X8H7_H7_PERIPH, NULL, NULL);
  cdev_del(&priv->cdev);
  device_destroy(priv->cl, priv->dev_num);
  class_destroy(priv->cl);
  unregister_chrdev_region(priv->dev_num, 1);
  return 0;
}

static const struct of_device_id x8h7_h7_of_match[] = {
  { .compatible = "portenta,x8h7_h7"},
  { },
};

static struct platform_driver x8h7_h7_driver = {
  .driver = {
    .name           = "x8h7_h7",
    .of_match_table = x8h7_h7_of_match,
  },
  .probe  = x8h7_h7_probe,
  .remove = x8h7_h7_remove,
};

module_platform_driver(x8h7_h7_driver);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_DESCRIPTION("Arduino Portenta X8 GPIO driver");
MODULE_LICENSE("GPL v2");
