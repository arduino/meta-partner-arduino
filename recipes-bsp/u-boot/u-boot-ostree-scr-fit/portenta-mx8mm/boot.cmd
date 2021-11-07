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

@@INCLUDE_COMMON@@
