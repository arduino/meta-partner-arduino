echo "Using arduino_${fdt_file}"

# Default boot type and device
setenv bootlimit 3
setenv devtype mmc
setenv devnum 2
setenv ovl_addr 0x43600000

setenv bootcmd_resetvars 'setenv kernel_image; setenv bootargs; setenv kernel_image2; setenv bootargs2'
setenv bootcmd_otenv 'run bootcmd_resetvars; ext4load ${devtype} ${devnum}:2 ${loadaddr} /boot/loader/uEnv.txt; env import -t ${loadaddr} ${filesize}'
setenv bootcmd_load_f 'ext4load ${devtype} ${devnum}:2 ${loadaddr} "/boot"${kernel_image}'
setenv bootcmd_dtb 'imxtract ${loadaddr}#conf@arduino_${fdt_file} fdt@arduino_${fdt_file} ${fdt_addr}; fdt addr ${fdt_addr}'
setenv bootcmd_overlay ' \
  for ov in ${overlays}; do; \
    if imxtract ${loadaddr}#conf@arduino_${fdt_file} fdt@${ov}.dtbo ${ovl_addr}; then \
      fdt resize 0x${filesize}; \
      fdt apply ${ovl_addr}; \
      echo "Applied ${ov}.dtbo to DTB"; \
    else \
      echo "WARN: ${ov}.dtbo not found!"; \
    fi; \
  done'
setenv bootcmd_run 'bootm ${loadaddr}#conf@arduino_${fdt_file} ${loadaddr}#conf@arduino_${fdt_file} ${fdt_addr}'
setenv bootcmd_rollbackenv 'setenv kernel_image ${kernel_image2}; setenv bootargs ${bootargs2}'
setenv bootcmd_set_rollback 'if test ! "${rollback}" = "1"; then setenv rollback 1; setenv upgrade_available 0; saveenv; fi'
setenv bootostree 'run bootcmd_load_f; run bootcmd_dtb; run bootcmd_overlay; run bootcmd_run'
setenv altbootcmd 'run bootcmd_otenv; run bootcmd_set_rollback; if test -n "${kernel_image2}"; then run bootcmd_rollbackenv; fi; run bootostree; reset'

# default overlays
if env exist overlays; then true; else setenv overlays 'ov_som_lbee5kl1dx'; saveenv; fi

# make sure env exists
if test ! -e ${devtype} ${devnum}:1 uboot.env; then saveenv; fi

if test "${rollback}" = "1"; then run altbootcmd; else run bootcmd_otenv; run bootostree; reset; fi

reset
