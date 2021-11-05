MACHINEOVERRIDES =. "mx8:mx8m:mx8mm:imx8mm-lpddr4-evk:portenta-mx8mm:"

require conf/machine/include/imx-base.inc
require conf/machine/include/tune-cortexa53.inc

IMX_DEFAULT_BSP = "nxp"

MACHINE_FEATURES += " pci wifi bluetooth optee alsa"

KERNEL_DEVICETREE = ""
KERNEL_DEVICETREE_remove_portenta-mx8mm = " freescale/imx8mm-evkb.dtb"
PREFERRED_PROVIDER_virtual/dtb ?= "lmp-device-tree"

UBOOT_SUFFIX = "bin"

SPL_BINARY = "spl/u-boot-spl.bin"

ATF_PLATFORM = "imx8mm"
ATF_LOAD_ADDR = "0x920000"

# Set DDR FIRMWARE
DDR_FIRMWARE_NAME = "lpddr4_pmu_train_1d_imem.bin lpddr4_pmu_train_1d_dmem.bin lpddr4_pmu_train_2d_imem.bin lpddr4_pmu_train_2d_dmem.bin"

# Set imx-mkimage boot target
IMXBOOT_TARGETS = "${@bb.utils.contains('UBOOT_CONFIG', 'fspi', 'flash_evk_flexspi', 'flash_evk', d)}"
IMX_BOOT_SOC_TARGET = "iMX8MM"

# Set Serial console
SERIAL_CONSOLE = "115200;ttymxc1"

BOOT_SPACE = "65536"

LOADADDR = ""
UBOOT_MAKE_TARGET = ""

IMX_BOOT_SEEK = "33"
# make sure we build the devicetree and add it to lmp-base builds
EXTRA_IMAGEDEPENDS_append = " virtual/dtb "

OSTREE_KERNEL_ARGS_portenta-mx8mm ?= "console=tty1 console=ttymxc1,115200 earlycon=ec_imx6q,0x30890000,115200 root=/dev/mmcblk2p2 rootfstype=ext4"
OPTEE_BIN_EXT = "8mm"

# Add additional firmware
MACHINE_FIRMWARE_append = "linux-firmware linux-firmware-cyw-fmac-fw linux-firmware-cyw-fmac-nvram linux-firmware-cyw-bt-patch"

# Add kernel-module recipe recommendations
MACHINE_EXTRA_RRECOMMENDS += " bq24195 cs42l52 anx7625"

# Portenta MX8MM has a Murata 1DX wifi/bt module which uses custom recipes
# linux-firmware-cyw-fmac-fw, linux-firmware-cyw-fmac-nvram and linux-firmware-cyw-bt-patch.
# Make sure we avoid default linux bcm43430 firmware
CORE_IMAGE_BASE_INSTALL_remove_portenta-mx8mm = " \
    linux-firmware-bcm43430 \
"