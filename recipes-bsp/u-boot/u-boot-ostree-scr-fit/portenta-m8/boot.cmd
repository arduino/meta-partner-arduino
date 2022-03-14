echo "Using arduino_${fdt_file}"

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

# Following variables can be used to disable
# auto carrier detection mechanism
# setenv carrier_custom 1
# setenv overlays 'ov_name1 ov_name2...'

setenv bootcmd_dtb 'imxtract ${fit_addr}#conf@@FIT_NODE_SEPARATOR@@${fdt_file_final} fdt@@FIT_NODE_SEPARATOR@@${fdt_file_final} ${fdt_addr}; fdt addr ${fdt_addr}'
setenv ovl_set_envsave ' \
  if env exist old_carrier_name; then \
    if test "${carrier_name}" = "${old_carrier_name}"; then \
      setenv envsave 0 \
    else \
      setenv envsave 1 \
      setenv old_carrier_name $carrier_name \
    fi \
  else \
    setenv envsave 1 \
  fi'
setenv bootcmd_ovl_auto_detect ' \
  if env exist carrier_custom; then true; \
  else \
    if test "${is_on_carrier}" = "yes"; then \
      if test "${carrier_name}" = "breakout"; then \
        setenv overlays $som_ovl $breakout_ovl \
      fi \
      if test "${carrier_name}" = "max"; then \
        setenv overlays $som_ovl $max_ovl \
      fi \
      run ovl_set_envsave \
    fi \
  fi'
setenv bootcmd_saveenv 'if test "${envsave}" = "1"; then run saveenv_mmc; fi'
setenv bootcmd_overlay ' \
  for ov in ${overlays}; do; \
    if imxtract ${fit_addr}#conf@@FIT_NODE_SEPARATOR@@${fdt_file_final} fdt@@FIT_NODE_SEPARATOR@@${ov}.dtbo ${ovl_addr}; then \
      fdt resize 0x${filesize}; \
      fdt apply ${ovl_addr}; \
      echo "Applied ${ov}.dtbo to DTB"; \
    else \
      echo "WARN: ${ov}.dtbo not found!"; \
    fi; \
  done'

setenv bootcmd_load_fw ' \
  run bootcmd_dtb; \
  run bootcmd_ovl_auto_detect; \
  run bootcmd_overlay; \
  run bootcmd_saveenv; \
  setenv bootcmd_run \
    bootm ${fit_addr}#conf@@FIT_NODE_SEPARATOR@@${fdt_file_final} ${fit_addr}#conf@@FIT_NODE_SEPARATOR@@${fdt_file_final} ${fdt_addr}'

@@INCLUDE_COMMON_IMX@@
@@INCLUDE_COMMON@@
