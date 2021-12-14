fatload mmc ${mmcdev}:1 ${loadaddr} /uEnv.txt
env import -t ${loadaddr} ${filesize}

# Som ov list
setenv som_ovl ' \
  ov_som_lbee5kl1dx \
  ov_som_x8h7'

# Breakout carrier ov list
setenv breakout_ovl ' \
  ov_carrier_breakout_gpio \
  ov_carrier_breakout_i2s \
  ov_carrier_breakout_sai \
  ov_carrier_breakout_pdm \
  ov_carrier_breakout_pwm \
  ov_carrier_breakout_sdc \
  ov_carrier_breakout_spdif \
  ov_carrier_breakout_spi0 \
  ov_carrier_breakout_uart1 \
  ov_carrier_breakout_uart3 \
  ov_carrier_breakout_usbfs'

# Max carrier ov list
# power supply usbc only
setenv max_ovl ' \
  ov_carrier_max_usbfs \
  ov_carrier_max_sdc \
  ov_carrier_max_cs42l52 \
  ov_carrier_enuc_lora'

# Max carrier ov list
# power supply +9Vdc (VIN)
setenv max_ovl_full ' \
  ov_carrier_enuc_bq24195 \
  ov_carrier_max_usbfs \
  ov_carrier_max_cs42l52 \
  ov_carrier_max_sara-r4 \
  ov_carrier_enuc_lora'

# Following variables can be used to disable
# auto carrier detection mechanism
# setenv carrier_custom 1
# setenv overlays 'ov_name1 ov_name2...'

run bootcmd
