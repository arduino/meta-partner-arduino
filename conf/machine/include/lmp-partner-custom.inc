# LMP partner specific customizations (either replace or extend options as defined by meta-lmp)

PACKAGECONFIG:append:pn-networkmanager:portenta-mx8mm = " modemmanager"

# drop idle timeout in weston
PACKAGECONFIG:append:pn-weston-init:portenta-mx8mm = " no-idle-timeout"

## Portenta i.MX8MMini with EBBR / SystemReady
UBOOT_SIGN_ENABLE:sota:portenta-mx8mm-ebbr = ""
KERNEL_IMAGETYPE:sota:portenta-mx8mm-ebbr ?= "Image"
KERNEL_CLASSES:sota:portenta-mx8mm-ebbr ?= ""
EFI_PROVIDER:portenta-mx8mm-ebbr ?= "grub-efi"
OSTREE_BOOTLOADER:portenta-mx8mm-ebbr ?= "grub"
IMAGE_EFI_BOOT_FILES:sota:portenta-mx8mm-ebbr ?= "${@make_dtb_boot_files(d)}"
WKS_FILE:sota:portenta-mx8mm-ebbr ?= "efidisk-sota.wks.in"
