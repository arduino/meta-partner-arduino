SUMMARY = "Minimal devel image which includes OTA Lite, Docker, and OpenSSH support"
FILESEXTRAPATHS:prepend := "${THISDIR}/configs:"

require recipes-samples/images/lmp-image-common.inc

# Enable wayland related recipes if required by DISTRO
require ${@bb.utils.contains('DISTRO_FEATURES', 'wayland', 'recipes-samples/images/lmp-feature-wayland.inc', '', d)}

require recipes-samples/images/lmp-feature-docker.inc
require recipes-samples/images/lmp-feature-wifi.inc
require recipes-samples/images/lmp-feature-sbin-path-helper.inc
require recipes-samples/images/lmp-feature-sysctl-hang-crash-helper.inc

EXTRA_IMAGE_FEATURES = "package-management tools-debug"

SRC_URI += "\
    file://sudoers-arduino \
"

IMAGE_FEATURES += "ssh-server-dropbear"

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

LMP_EXTRA = " \
    lmp-device-tree \
    lmp-auto-hostname \
"

ADB = " \
    android-tools \
    android-tools-adbd \
"

ARDUINO = " \
    arduino-ootb \
    m4-proxy \
    udev-rules-portenta \
"

VIDEOTOOLS = " \
    gstreamer1.0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    imx-gst1.0-plugin \
    v4l-utils \
"

OPENCV = " \
    opencv \
"

CORE_IMAGE_BASE_INSTALL += " \
    libdrm \
    usb-modeswitch \
    u-boot-fw-utils \
    ${LMP_EXTRA} \
    ${ADB} \
    ${ARDUINO} \
    ${VIDEOTOOLS} \
    ${OPENCV} \
"

# Packages to be installed in Portenta-X8 machine only
IMAGE_INSTALL:append:portenta-x8 = " openocd"

fakeroot do_populate_rootfs_add_custom_sudoers () {
    # Allow sudo group users to use sudo
    bbwarn Changing sudoers for arduino ootb!
    install -m 0440 ${WORKDIR}/sudoers-arduino ${IMAGE_ROOTFS}${sysconfdir}/sudoers.d/51-arduino
}

IMAGE_PREPROCESS_COMMAND += "do_populate_rootfs_add_custom_sudoers; "