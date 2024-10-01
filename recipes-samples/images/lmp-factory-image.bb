SUMMARY = "Minimal factory image which includes OTA Lite, Docker, and OpenSSH support"

require recipes-samples/images/lmp-image-common.inc

# Factory tooling requires SOTA (OSTree + Aktualizr-lite)
require ${@bb.utils.contains('DISTRO_FEATURES', 'sota', 'recipes-samples/images/lmp-feature-factory.inc', '', d)}

# Enable wayland related recipes if required by DISTRO
require ${@bb.utils.contains('DISTRO_FEATURES', 'wayland', 'recipes-samples/images/lmp-feature-wayland.inc', '', d)}

# Enable IMA support if required by DISTRO
require ${@bb.utils.contains('DISTRO_FEATURES', 'ima', 'recipes-samples/images/lmp-feature-ima.inc', '', d)}

# Enable OP-TEE related recipes if provided by the image
require ${@bb.utils.contains('MACHINE_FEATURES', 'optee', 'recipes-samples/images/lmp-feature-optee.inc', '', d)}

# Enable SE05X related recipes if provided by machine
require ${@bb.utils.contains('MACHINE_FEATURES', 'se05x', 'recipes-samples/images/lmp-feature-se05x.inc', '', d)}

# Enable TPM2 related recipes if provided by machine
require ${@bb.utils.contains('MACHINE_FEATURES', 'tpm2', 'recipes-samples/images/lmp-feature-tpm2.inc', '', d)}

# Enable EFI support if provided by machine
require ${@bb.utils.contains('MACHINE_FEATURES', 'efi', 'recipes-samples/images/lmp-feature-efi.inc', '', d)}

# Enable Xenomai4 related recipes if provided by the image
require ${@bb.utils.contains('MACHINE_FEATURES', 'xeno4', 'recipes-samples/images/lmp-feature-xeno4.inc', '', d)}

# Enable jailhouse related recipes if provided by the machine
require ${@bb.utils.contains('MACHINE_FEATURES', 'jailhouse', 'recipes-samples/images/lmp-feature-jailhouse.inc', '', d)}

require recipes-samples/images/lmp-feature-softhsm.inc
require recipes-samples/images/lmp-feature-wireguard.inc
require recipes-samples/images/lmp-feature-docker.inc
require recipes-samples/images/lmp-feature-wifi.inc
require recipes-samples/images/lmp-feature-ota-utils.inc
require recipes-samples/images/lmp-feature-sbin-path-helper.inc

# Add any partner layer requirements
include recipes-samples/images/lmp-partner-image.inc

IMAGE_FEATURES += "ssh-server-openssh"

CORE_IMAGE_BASE_INSTALL_GPLV3 = "\
    packagegroup-core-full-cmdline-utils \
    packagegroup-core-full-cmdline-multiuser \
"

CORE_IMAGE_BASE_INSTALL += " \
    kernel-modules \
    networkmanager-nmcli \
    git \
    vim \
    packagegroup-core-full-cmdline-extended \
    ${@bb.utils.contains('LMP_DISABLE_GPLV3', '1', '', '${CORE_IMAGE_BASE_INSTALL_GPLV3}', d)} \
"
