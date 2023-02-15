fatload mmc ${mmcdev}:1 ${loadaddr} /uEnv.txt
env import -t ${loadaddr} ${filesize}

# Som ov list
setenv som_ovl ' \
  ov_som_lbee5kl1dx'

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
# Warning!!! You need to provide +9Vdc (VIN)
# otherwise some peripherals won't work
setenv max_ovl ' \
  ov_carrier_enuc_bq24195 \
  ov_carrier_max_usbfs \
  ov_carrier_max_sdc \
  ov_carrier_max_cs42l52 \
  ov_carrier_max_sara-r4 \
  ov_carrier_enuc_lora'

# Rasp-Tenta carrier ov list
setenv rasp_base_ovl ' \
  ov_carrier_rasptenta_base \
  ov_carrier_rasptenta_ov5647_camera_mipi'

# Following variables can be used to disable
# auto carrier detection mechanism
# setenv carrier_custom 1
# setenv overlays 'ov_name1 ov_name2...'

run bootcmd
