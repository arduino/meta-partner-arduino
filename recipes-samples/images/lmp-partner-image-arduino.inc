# Additions for the arduino partner image

FILESEXTRAPATHS:prepend := "${THISDIR}/configs:"

IMAGE_FEATURES:remove "ssh-server-openssh"
IMAGE_FEATURES += "ssh-server-dropbear"

SUDOERS_FILE ?= "sudoers-arduino-lmp"
SRC_URI += "\
    file://${SUDOERS_FILE} \
"

LMP_EXTRA = " \
    lmp-device-tree \
    lmp-auto-hostname \
    compose-apps-early-start \
    tmate \
    libgpiod-tools \
"

ADB = " \
    android-tools \
    android-tools-adbd \
"

ARDUINO = " \
    arduino-ootb \
    m4-proxy \
    udev-rules-portenta \
    libusbgx-config \
"

CORE_IMAGE_BASE_INSTALL += " \
    libdrm \
    usb-modeswitch \
    ${LMP_EXTRA} \
    ${ADB} \
    ${ARDUINO} \
"

# No default password is configured for the device
EXTRA_USERS_PARAMS += "\
    usermod -p '' ${LMP_USER}; \
"

# Packages to be installed in Portenta-X8 machine only
IMAGE_INSTALL:append:portenta-x8 = " openocd"

fakeroot do_populate_rootfs_add_custom_sudoers () {
    # Allow sudo group users to use sudo
    bbwarn Changing sudoers for arduino ootb!
    install -m 0440 ${WORKDIR}/${SUDOERS_FILE} ${IMAGE_ROOTFS}${sysconfdir}/sudoers.d/51-arduino
}

IMAGE_PREPROCESS_COMMAND += "do_populate_rootfs_add_custom_sudoers; "
