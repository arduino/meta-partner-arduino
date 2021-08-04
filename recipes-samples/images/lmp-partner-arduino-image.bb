SUMMARY = "Minimal partner image which includes OTA Lite, Docker, and OpenSSH support"
FILESEXTRAPATHS_prepend := "${THISDIR}/configs:"

SRC_URI += "\
    file://sudoers-arduino \
"

require recipes-samples/images/lmp-image-common.inc

# Factory tooling requires SOTA (OSTree + Aktualizr-lite)
require ${@bb.utils.contains('DISTRO_FEATURES', 'sota', 'recipes-samples/images/lmp-feature-factory.inc', '', d)}

# Enable wayland related recipes if required by DISTRO
require ${@bb.utils.contains('DISTRO_FEATURES', 'wayland', 'recipes-samples/images/lmp-feature-wayland.inc', '', d)}

# Enable OP-TEE related recipes if provided by the image
require ${@bb.utils.contains('MACHINE_FEATURES', 'optee', 'recipes-samples/images/lmp-feature-optee.inc', '', d)}

require recipes-samples/images/lmp-feature-softhsm.inc
require recipes-samples/images/lmp-feature-wireguard.inc
require recipes-samples/images/lmp-feature-docker.inc
require recipes-samples/images/lmp-feature-wifi.inc
require recipes-samples/images/lmp-feature-ota-utils.inc
require recipes-samples/images/lmp-feature-sbin-path-helper.inc

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

# Arduino additions

BOOT = " \
    lmp-device-tree \
"

BRINGUP = " \
    i2c-tools \
    usbutils \
    python3-pyserial \
    opkg \
    minicom \
    stress-ng \
"
# imx-test
# bcm43xx-utils
# brcm-patchram-plus

CORE_IMAGE_BASE_INSTALL += " \
    libdrm \
    modemmanager \
    usb-modeswitch \
    ${BOOT} \
    ${BRINGUP} \
    android-tools \
    arduino-ootb \
"

fakeroot do_populate_rootfs_add_custom_sudoers () {
    # Allow sudo group users to use sudo
    bbwarn Hey you...changing sudoers for arduino ootb!
    install -m 0440 ${WORKDIR}/sudoers-arduino ${IMAGE_ROOTFS}${sysconfdir}/sudoers.d/51-arduino
}

IMAGE_PREPROCESS_COMMAND += "do_populate_rootfs_add_custom_sudoers; "
