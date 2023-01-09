SUMMARY = "V4L2Loopback"
DESCRIPTION = "v4l2loopback module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"


# Use local tarball
SRC_URI = " \
    git://github.com/umlaeute/v4l2loopback.git;protocol=https;branch=main \
    file://build-with-yocto.patch \
"
SRCREV = "1ecf810f0d687b647caa3050ae30cf51b5902afd"

# Make sure our source directory (for the build) matches the directory structure in the tarball
S = "${WORKDIR}/git"

inherit module

FILES:${PN} += "/usr/lib/modules"
