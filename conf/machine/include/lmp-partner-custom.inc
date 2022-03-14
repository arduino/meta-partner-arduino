# LMP partner specific customizations (either replace or extend options as defined by meta-lmp)

PACKAGECONFIG_append_pn-networkmanager_portenta-mx8mm = " modemmanager"

# drop idle timeout in weston
PACKAGECONFIG_append_pn-weston-init_portenta-mx8mm = " no-idle-timeout"

## Portenta i.MX8MMini with EBBR / SystemReady
UBOOT_SIGN_ENABLE_sota_portenta-mx8mm-ebbr = ""
KERNEL_IMAGETYPE_sota_portenta-mx8mm-ebbr ?= "Image"
KERNEL_CLASSES_sota_portenta-mx8mm-ebbr ?= ""
EFI_PROVIDER_portenta-mx8mm-ebbr ?= "grub-efi"
OSTREE_BOOTLOADER_portenta-mx8mm-ebbr ?= "grub"
IMAGE_EFI_BOOT_FILES_sota_portenta-mx8mm-ebbr ?= "${@make_dtb_boot_files(d)}"
WKS_FILE_sota_portenta-mx8mm-ebbr ?= "efidisk-sota.wks"
