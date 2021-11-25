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

#define DEBUG
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

  ret = pwmchip_add(&x8h7_pwm->chip);
  if (ret < 0) {
    dev_err(&pdev->dev, "failed to add PWM chip %d\n", ret);
    return ret;
  }

  platform_set_drvdata(pdev, x8h7_pwm);

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
