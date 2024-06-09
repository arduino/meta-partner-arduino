SUMMARY = "Hailo PCIe driver"
DESCRIPTION = "The Hailo PCIe driver is required for interacting with a Hailo device over the PCIe interface"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://../../LICENSE;md5=39bba7d2cf0ba1036f2a6e2be52fe3f0"

inherit module

SRC_URI = " \
    git://github.com/hailo-ai/hailort-drivers.git;branch=master;protocol=https \
    file://0001-OE-fix-Makefile.patch \
"
SRCREV = "43e8a9a575654c773edf95789f93cde40c708a6a"

S = "${WORKDIR}/git/linux/pcie"

PV = "v4.17.1"

RPROVIDES_${PN} += "kernel-module-hailo-pcie-driver"
