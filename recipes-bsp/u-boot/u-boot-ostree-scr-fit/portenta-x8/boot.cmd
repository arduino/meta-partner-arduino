# Default boot type and device
setenv bootlimit 5
setenv devtype mmc
setenv devnum 2
setenv bootpart 1
setenv rootpart 2
setenv ovl_addr 0x43600000

# Boot image files
setenv fdt_file_final arduino_${fdt_file}
setenv fit_addr 0x43800000

# Boot firmware updates
setenv bootloader 42
setenv bootloader2 300
setenv bootloader_s 1042
setenv bootloader2_s 1300
setenv bootloader_image "imx-boot"
setenv bootloader_s_image ${bootloader_image}
setenv bootloader2_image "u-boot.itb"
setenv bootloader2_s_image ${bootloader2_image}
setenv uboot_hwpart 1

# Som ov list
setenv som_ovl ' \
  ov_som_lbee5kl1dx \
  ov_som_x8h7'

# Breakout carrier ov list
setenv breakout_ovl ' \
  ov_carrier_breakout_gpio \
  ov_carrier_breakout_i2c0 \
  ov_carrier_breakout_i2c1 \
  ov_carrier_breakout_i2c2 \
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

# eNUC carrier ov list
setenv enuc_ovl ' \
  ov_carrier_enuc_bq24195 \
  ov_carrier_enuc_usbfs \
  ov_carrier_enuc_cs42l52 \
  ov_carrier_enuc_sara-r4 \
  ov_carrier_enuc_lora'

# Max carrier ov list
setenv max_ovl ' \
  ov_carrier_enuc_bq24195 \
  ov_carrier_enuc_usbfs \
  ov_carrier_max_cs42l52 \
  ov_carrier_max_sara-r4 \
  ov_carrier_enuc_lora'

# Following variables can be used to disable
# auto carrier detection mechanism
# setenv carrier_custom 1
# setenv overlays '${som_ovl}'

@@INCLUDE_COMMON@@
