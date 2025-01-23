SUMMARY = "Minimal devel image which includes OTA Lite, Docker, and OpenSSH support"
FILESEXTRAPATHS:prepend := "${THISDIR}/configs:"

require recipes-samples/images/lmp-image-common.inc

# Enable wayland related recipes if required by DISTRO
require ${@bb.utils.contains('DISTRO_FEATURES', 'wayland', 'recipes-samples/images/lmp-feature-wayland.inc', '', d)}

require recipes-samples/images/lmp-feature-docker.inc
require recipes-samples/images/lmp-feature-wifi.inc
require recipes-samples/images/lmp-feature-sbin-path-helper.inc
require recipes-samples/images/lmp-feature-sysctl-hang-crash-helper.inc

# Add any partner layer requirements
include recipes-samples/images/lmp-partner-image.inc

EXTRA_IMAGE_FEATURES = "package-management tools-debug"

SUDOERS_FILE = "sudoers-arduino-lmp-base"

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

# Arduino development changes

# early start not needed for lmp-base image
LMP_EXTRA:remove = "compose-apps-early-start"

VIDEOTOOLS = " \
    gstreamer1.0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    imx-gst1.0-plugin \
    v4l-utils \
    gstreamer1.0-bayer2rgb-neon \
    gst-instruments \
"

OPENCV = " \
    opencv \
"

CORE_IMAGE_BASE_INSTALL += " \
    automount-boot \
    dtc \
    stress-ng \
    u-boot-fio-env \
    u-boot-fw-utils \
    ${OPENCV} \
"

# Custom task to write git SHA to /etc/os-release
write_git_sha() {
    META_LAYER_DIR="${OEROOT}/layers/meta-partner-arduino"
    GIT_SHA=$(cd ${META_LAYER_DIR} && git rev-parse HEAD)
    echo "META_LAYER_GIT_SHA=${GIT_SHA}" >> ${IMAGE_ROOTFS}/etc/os-release
}

ROOTFS_POSTPROCESS_COMMAND += "write_git_sha; "
