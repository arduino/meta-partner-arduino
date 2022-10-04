/**
 * X8H7 ADC IIO driver
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>

#include "x8h7.h"

#define DRIVER_NAME     "x8h7_adc"

// #define DEBUG
#include "debug.h"

// Peripheral code
#define X8H7_ADC_PERIPH   0x01
// Op code
#define X8H7_ADC_OC_CONFIGURE      0x10
#define X8H7_ADC_OC_DATA           0x01

#define X8H7_ADC_NUM  8

struct x8h7_adc_val {
  uint16_t            val;
  wait_queue_head_t   wait;
  int                 cnt;
};

struct x8h7_adc {
  struct device      *dev;
  struct mutex        lock;
  struct x8h7_adc_val val[X8H7_ADC_NUM];
};

#define X8H7_ADC_CHAN(_idx) {                          \
  .type                     = IIO_VOLTAGE,             \
  .indexed                  = 1,                       \
  .channel                  = _idx,                    \
  .info_mask_separate       = BIT(IIO_CHAN_INFO_RAW),  \
  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
}

static const struct iio_chan_spec x8h7_adc_iio_channels[] = {
  X8H7_ADC_CHAN(0),
  X8H7_ADC_CHAN(1),
  X8H7_ADC_CHAN(2),
  X8H7_ADC_CHAN(3),
  X8H7_ADC_CHAN(4),
  X8H7_ADC_CHAN(5),
  X8H7_ADC_CHAN(6),
  X8H7_ADC_CHAN(7),
};

static void x8h7_adc_hook(void *priv, x8h7_pkt_t *pkt)
{
  struct x8h7_adc  *adc = (struct x8h7_adc*)priv;
  uint8_t           ch;

  ch = pkt->opcode - 1;
  if (ch < X8H7_ADC_NUM) {
    adc->val[ch].val = *((uint16_t*)pkt->data);
    adc->val[ch].cnt++;
    wake_up_interruptible(&adc->val[ch].wait);
  }
}

static int x8h7_adc_pkt_get(struct x8h7_adc *adc, unsigned int ch)
{
  long ret;

  ret = wait_event_interruptible_timeout(adc->val[ch].wait,
                                         adc->val[ch].cnt != 0,
                                         X8H7_RX_TIMEOUT);
  if (!ret) {
    DBG_ERROR("timeout expired");
    return -1;
  }
  adc->val[ch].cnt--;
  return 0;
}

static int x8h7_adc_read_chan(struct x8h7_adc *adc, unsigned int ch)
{
  x8h7_pkt_enq(X8H7_ADC_PERIPH, ch + 1, 0, NULL);
  x8h7_pkt_send();
  if (x8h7_adc_pkt_get(adc, ch) < 0)
    return -ETIMEDOUT;

  return adc->val[ch].val;
}

static int x8h7_adc_read_raw(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan,
                             int *val, int *val2, long mask)
{
  struct x8h7_adc *adc = iio_priv(indio_dev);

  switch (mask) {
  case IIO_CHAN_INFO_RAW:
    mutex_lock(&adc->lock);
    *val = x8h7_adc_read_chan(adc, chan->channel);
    mutex_unlock(&adc->lock);
    if (*val == -1) {
      return -EIO;
    }
    return IIO_VAL_INT;

  case IIO_CHAN_INFO_SCALE:
    *val = 1; // regulator_get_voltage(adc->vref) / 1000;
    *val2 = 10;
    return IIO_VAL_FRACTIONAL_LOG2;

  case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
    return 0;
  }

  return -EINVAL;
}

static const struct iio_info x8h7_adc_info = {
  .read_raw = x8h7_adc_read_raw,
};

static int x8h7_adc_probe(struct platform_device *pdev)
{
  struct iio_dev   *indio_dev;
  struct x8h7_adc  *adc;
  int               ret;
  int               i;

  indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
  if (!indio_dev) {
    return -ENOMEM;
  }

  platform_set_drvdata(pdev, indio_dev);
  adc = iio_priv(indio_dev);
  adc->dev = &pdev->dev;
  mutex_init(&adc->lock);
#if 0
  init_waitqueue_head(&adc->wait);
#else
  for (i=0; i<X8H7_ADC_NUM; i++) {
    init_waitqueue_head(&adc->val[i].wait);
  }
#endif
  indio_dev->name         = dev_name(&pdev->dev);
  indio_dev->dev.parent   = &pdev->dev;
  indio_dev->info         = &x8h7_adc_info;
  indio_dev->modes        = INDIO_DIRECT_MODE;
  indio_dev->channels     = x8h7_adc_iio_channels;
  indio_dev->num_channels = ARRAY_SIZE(x8h7_adc_iio_channels);

  ret = iio_device_register(indio_dev);
  if (ret) {
    dev_err(&pdev->dev, "unable to register device\n");
    return ret;
  }

  x8h7_hook_set(X8H7_ADC_PERIPH, x8h7_adc_hook, adc);

  return 0;
}

static int x8h7_adc_remove(struct platform_device *pdev)
{
  return 0;
}

static const struct of_device_id x8h7_adc_match[] = {
  { .compatible = "portenta,x8h7_adc" },
  { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, x8h7_adc_match);

static struct platform_driver x8h7_adc_driver = {
  .probe  = x8h7_adc_probe,
  .remove = x8h7_adc_remove,
  .driver = {
    .name           = "x8h7_adc",
    .of_match_table = x8h7_adc_match,
  },
};
module_platform_driver(x8h7_adc_driver);

MODULE_DESCRIPTION("Arduino Portenta X8 ADC driver");
MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_LICENSE("GPL v2");
