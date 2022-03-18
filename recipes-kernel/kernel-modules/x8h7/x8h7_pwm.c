/**
 * X8H7 PWM
 * driver
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#include "x8h7.h"

#define DRIVER_NAME   "x8h7_pwm"

//#define DEBUG
#include "debug.h"

// Peripheral code
#define X8H7_PWM_PERIPH 0x02

struct __attribute__((packed, aligned(4))) pwmPacket {
  uint8_t  enable  :  1;
  uint8_t  polarity:  1;
  uint32_t duty    : 30;
  uint32_t period  : 32;
};

struct x8h7_pwm_chip {
  struct pwm_chip   chip;
  struct pwmPacket  pkt;
  wait_queue_head_t wait;
  int               cnt;
};

#define to_x8h7_pwm_chip(_chip) container_of(_chip, struct x8h7_pwm_chip, chip)

static int x8h7_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                           int duty_ns, int period_ns)
{
  struct x8h7_pwm_chip *x8h7 = to_x8h7_pwm_chip(chip);

  DBG_PRINT("duty_ns: %d, period_ns: %d\n", duty_ns, period_ns);
  //@TODO: period_ns must be greater than 953
  x8h7->pkt.duty = duty_ns;
  x8h7->pkt.period = period_ns;
  x8h7_pkt_enq(X8H7_PWM_PERIPH, pwm->hwpwm, sizeof(x8h7->pkt), &x8h7->pkt);
  x8h7_pkt_send();

  return 0;
}


static void x8h7_pwm_hook(void *priv, x8h7_pkt_t *pkt)
{
  struct x8h7_pwm_chip  *pwm = (struct x8h7_pwm_chip*)priv;
  uint8_t           ch;

  ch = pkt->opcode & 0xF;
  if (ch < 10) {
    struct pwmPacket* packet = (struct pwmPacket*)(pkt->data);
    pwm->pkt.duty = packet->duty;
    pwm->pkt.period = packet->period;
    pwm->cnt++;
    wake_up_interruptible(&pwm->wait);
  }
}

static int x8h7_pwm_pkt_get(struct x8h7_pwm_chip *pwm, unsigned int timeout)
{
  long ret;

  ret = wait_event_interruptible_timeout(pwm->wait,
                                         pwm->cnt != 0,
                                         timeout);
  if (!ret) {
    DBG_ERROR("timeout expired");
    return -1;
  }
  pwm->cnt--;
  return 0;
}

static int x8h7_pwm_capture(struct pwm_chip *chip, struct pwm_device *pwm,
		                        struct pwm_capture *result, unsigned long timeout)
{
  struct x8h7_pwm_chip *x8h7 = to_x8h7_pwm_chip(chip);

  //@TODO: period_ns must be greater than 953
  x8h7_pkt_enq(X8H7_PWM_PERIPH, pwm->hwpwm | 0x60, sizeof(x8h7->pkt), &x8h7->pkt);
  x8h7_pkt_send();

  x8h7->pkt.duty = 0;
  x8h7->pkt.period = 0;

  if (x8h7_pwm_pkt_get(x8h7, timeout) < 0)
    return -ETIMEDOUT;

  result->duty_cycle = x8h7->pkt.duty;
  result->period = x8h7->pkt.period;

  DBG_PRINT("duty_ns: %d, period_ns: %d\n", result->duty_cycle, result->period);

  return 0;
}

static int x8h7_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
  struct x8h7_pwm_chip *x8h7 = to_x8h7_pwm_chip(chip);

  DBG_PRINT("\n");
  x8h7->pkt.enable = 1;
  x8h7_pkt_enq(X8H7_PWM_PERIPH, pwm->hwpwm, sizeof(x8h7->pkt), &x8h7->pkt);
  x8h7_pkt_send();

  return 0;
}

static void x8h7_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
  struct x8h7_pwm_chip *x8h7 = to_x8h7_pwm_chip(chip);

  DBG_PRINT("\n");
  x8h7->pkt.enable = 0;
  x8h7_pkt_enq(X8H7_PWM_PERIPH, pwm->hwpwm, sizeof(x8h7->pkt), &x8h7->pkt);
  x8h7_pkt_send();
}

static const struct pwm_ops x8h7_pwm_ops = {
  .config  = x8h7_pwm_config,
  .enable  = x8h7_pwm_enable,
  .disable = x8h7_pwm_disable,
  .capture = x8h7_pwm_capture,
  .owner   = THIS_MODULE,
};

static const struct of_device_id x8h7_pwm_dt_ids[] = {
  { .compatible = "portenta,x8h7_pwm", },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, x8h7_pwm_dt_ids);

static int x8h7_pwm_probe(struct platform_device *pdev)
{
  struct x8h7_pwm_chip  *x8h7_pwm;
  int                    ret;

  x8h7_pwm = devm_kzalloc(&pdev->dev, sizeof(*x8h7_pwm), GFP_KERNEL);
  if (!x8h7_pwm) {
    return -ENOMEM;
  }

  x8h7_pwm->chip.dev  = &pdev->dev;
  x8h7_pwm->chip.ops  = &x8h7_pwm_ops;
  x8h7_pwm->chip.base = -1;
  x8h7_pwm->chip.npwm = 10;

  init_waitqueue_head(&x8h7_pwm->wait);

  ret = pwmchip_add(&x8h7_pwm->chip);
  if (ret < 0) {
    dev_err(&pdev->dev, "failed to add PWM chip %d\n", ret);
    return ret;
  }

  platform_set_drvdata(pdev, x8h7_pwm);

  x8h7_hook_set(X8H7_PWM_PERIPH, x8h7_pwm_hook, x8h7_pwm);

  return ret;
}

static int x8h7_pwm_remove(struct platform_device *pdev)
{
  struct x8h7_pwm_chip *x8h7_pwm = platform_get_drvdata(pdev);

  return pwmchip_remove(&x8h7_pwm->chip);
}

static struct platform_driver x8h7_pwm_driver = {
  .driver = {
    .name           = DRIVER_NAME,
    .of_match_table = of_match_ptr(x8h7_pwm_dt_ids),
  },
  .probe  = x8h7_pwm_probe,
  .remove = x8h7_pwm_remove,
};
module_platform_driver(x8h7_pwm_driver);

MODULE_ALIAS("platform:x8h7-pwm");
MODULE_AUTHOR("Massimiliano Agneni <massimilianmo@iptronix.com>");
MODULE_DESCRIPTION("Aeduino portenta X8 PWM driver");
MODULE_LICENSE("GPL v2");
