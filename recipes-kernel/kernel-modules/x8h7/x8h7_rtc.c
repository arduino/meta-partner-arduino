/**
 * X8H7 RTC driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include "x8h7.h"

#define DRIVER_NAME "x8h7_rtc"
#define DEVICE_NAME "x8h7_rtc"

//#define DEBUG
#include "debug.h"

// Peripheral code
#define X8H7_RTC_PERIPH 0x06
// Op code
#define X8H7_RTC_SET_DATE   0x01
#define X8H7_RTC_GET_DATE   0x02
#define X8H7_RTC_SET_ALARM  0x11
#define X8H7_RTC_GET_ALARM  0x12
#define X8H7_RTC_ALARM_IEN  0x13
#define X8H7_RTC_ALARM_INT  0x14

struct x8h7_rtc {
  struct rtc_device  *rtc;
  int                 alarm_enabled;
  int                 alarm_pending;
  wait_queue_head_t   wait;
  int                 rx_cnt;
  x8h7_pkt_t          rx_pkt;
};

static void x8h7_rtc_hook(void *priv, x8h7_pkt_t *pkt)
{
  struct x8h7_rtc  *rtc = (struct x8h7_rtc*)priv;

  if ((pkt->peripheral == X8H7_RTC_PERIPH) &&
      (pkt->opcode == X8H7_RTC_ALARM_INT) &&
      (pkt->size == 1)) {
    rtc->alarm_pending = 1;
    rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);
  } else {
    memcpy(&rtc->rx_pkt, pkt, sizeof(x8h7_pkt_t));
    rtc->rx_cnt++;
    wake_up_interruptible(&rtc->wait);
  }
}

static int x8h7_rtc_pkt_get(struct x8h7_rtc *rtc)
{
  long ret;

  ret = wait_event_interruptible_timeout(rtc->wait,
                                         rtc->rx_cnt != 0,
                                         X8H7_RX_TIMEOUT);
  if (!ret) {
    DBG_ERROR("timeout expired");
    return -1;
  }
  rtc->rx_cnt--;
  return 0;
}

static int x8h7_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
  struct x8h7_rtc *rtc = dev_get_drvdata(dev);

  DBG_PRINT("\n");
  x8h7_pkt_enq(X8H7_RTC_PERIPH, X8H7_RTC_GET_DATE, 0, NULL);
  x8h7_pkt_send();
  if (x8h7_rtc_pkt_get(rtc) < 0)
    return -ETIMEDOUT;

  if ((rtc->rx_pkt.peripheral == X8H7_RTC_PERIPH) &&
      (rtc->rx_pkt.opcode == X8H7_RTC_GET_DATE) &&
      (rtc->rx_pkt.size == 7)) {
    tm->tm_sec  = rtc->rx_pkt.data[0x00];
    tm->tm_min  = rtc->rx_pkt.data[0x01];
    tm->tm_hour = rtc->rx_pkt.data[0x02];
    tm->tm_mday = rtc->rx_pkt.data[0x03];
    tm->tm_mon  = rtc->rx_pkt.data[0x04];
    tm->tm_year = rtc->rx_pkt.data[0x05] + 100;
    tm->tm_wday = rtc->rx_pkt.data[0x06];
  } else {
    DBG_ERROR("Invalid response\n");
  }
  return 0;
}

static int x8h7_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
  uint8_t   data[7];

  DBG_PRINT("%02d:%02d:%02d %d/%d/%d\n",
            tm->tm_hour, tm->tm_min, tm->tm_sec,
            tm->tm_year, tm->tm_mon, tm->tm_mday);

  data[0x00] = tm->tm_sec ;
  data[0x01] = tm->tm_min ;
  data[0x02] = tm->tm_hour;
  data[0x03] = tm->tm_mday;
  data[0x04] = tm->tm_mon ;
  data[0x05] = tm->tm_year - 100;
  data[0x06] = tm->tm_wday;

  x8h7_pkt_enq(X8H7_RTC_PERIPH, X8H7_RTC_SET_DATE, 7, data);
  x8h7_pkt_send();

  return 0;
}

/*
struct rtc_wkalrm {
  unsigned char enabled;  // 0 = alarm disabled, 1 = alarm enabled
  unsigned char pending;  // 0 = alarm not pending, 1 = alarm pending
  struct rtc_time time;   // time the alarm is set to
};
*/
int x8h7_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *wa)
{
  struct x8h7_rtc *rtc = dev_get_drvdata(dev);

  DBG_PRINT("\n");

  wa->enabled = rtc->alarm_enabled;
  wa->pending = rtc->alarm_pending;

  x8h7_pkt_enq(X8H7_RTC_PERIPH, X8H7_RTC_GET_ALARM, 0, NULL);
  x8h7_pkt_send();
  if (x8h7_rtc_pkt_get(rtc) < 0)
    return -ETIMEDOUT;

  if ((rtc->rx_pkt.peripheral == X8H7_RTC_PERIPH) &&
      (rtc->rx_pkt.opcode == X8H7_RTC_GET_ALARM) &&
      (rtc->rx_pkt.size == 7)) {
    wa->time.tm_sec  = rtc->rx_pkt.data[0x00];
    wa->time.tm_min  = rtc->rx_pkt.data[0x01];
    wa->time.tm_hour = rtc->rx_pkt.data[0x02];
    wa->time.tm_mday = rtc->rx_pkt.data[0x03];
    wa->time.tm_mon  = rtc->rx_pkt.data[0x04];
    wa->time.tm_year = rtc->rx_pkt.data[0x05] + 100;
    wa->time.tm_wday = rtc->rx_pkt.data[0x06];
  } else {
    DBG_ERROR("Invalid response\n");
  }
  return rtc_valid_tm(&wa->time);;
}

int x8h7_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *wa)
{
  struct x8h7_rtc  *rtc = dev_get_drvdata(dev);
  uint8_t           data[7];

  DBG_PRINT("%02d:%02d:%02d %d/%d/%d ena %d pending %d\n",
            wa->time.tm_hour, wa->time.tm_min, wa->time.tm_sec,
            wa->time.tm_year, wa->time.tm_mon, wa->time.tm_mday,
            wa->enabled, wa->pending);
  data[0x00] = wa->time.tm_sec ;
  data[0x01] = wa->time.tm_min ;
  data[0x02] = wa->time.tm_hour;
  data[0x03] = wa->time.tm_mday;
  data[0x04] = wa->time.tm_mon ;
  data[0x05] = wa->time.tm_year - 100;
  data[0x06] = wa->time.tm_wday;

  x8h7_pkt_enq(X8H7_RTC_PERIPH, X8H7_RTC_SET_ALARM, 7, data);
  x8h7_pkt_send();

  if (!wa->enabled) {
    rtc->alarm_pending = 0;
  }
  rtc->alarm_enabled = wa->enabled;
  return 0;
}

int x8h7_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
  struct x8h7_rtc  *rtc = dev_get_drvdata(dev);
  uint8_t           data[1];

  rtc->alarm_enabled = enabled;

  if (enabled) {
    data[0] = RTC_AF;
  } else {
    data[0] = ~RTC_AF;
  }
  x8h7_pkt_enq(X8H7_RTC_PERIPH, X8H7_RTC_ALARM_IEN, 1, data);
  x8h7_pkt_send();
  return 0;
}

static const struct rtc_class_ops x8h7_rtc_ops = {
  .read_time        = x8h7_rtc_read_time,
  .set_time         = x8h7_rtc_set_time,
#if 0
  .read_alarm       = x8h7_rtc_read_alarm,
  .set_alarm        = x8h7_rtc_set_alarm,
  .alarm_irq_enable	= x8h7_rtc_alarm_irq_enable,
#endif
};

static int x8h7_rtc_probe(struct platform_device *pdev)
{
  struct x8h7_rtc  *p;
  int               err = -ENOMEM;

  p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
  if (!p) {
    goto out;
  }

  init_waitqueue_head(&p->wait);

  platform_set_drvdata(pdev, p);

  p->rtc = devm_rtc_device_register(&pdev->dev, DEVICE_NAME,
                                    &x8h7_rtc_ops, THIS_MODULE);
  if (IS_ERR(p->rtc)) {
    err = PTR_ERR(p->rtc);
    goto out;
  }

  x8h7_hook_set(X8H7_RTC_PERIPH, x8h7_rtc_hook, p);

  err = 0;
out:
  return err;
}

static const struct of_device_id x8h7_rtc_dt_ids[] = {
  { .compatible = "portenta,x8h7_rtc", },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, x8h7_rtc_dt_ids);

static struct platform_driver x8h7_rtc_driver = {
  .driver = {
    .name           = DRIVER_NAME,
    .of_match_table = of_match_ptr(x8h7_rtc_dt_ids),
  },
  .probe = x8h7_rtc_probe,
};

module_platform_driver(x8h7_rtc_driver);

MODULE_AUTHOR("Massimiliano Agneni <massimiliano@iptronix.com");
MODULE_DESCRIPTION("Arduino Portenta X8 RTC driver");
MODULE_LICENSE("GPL v2");
