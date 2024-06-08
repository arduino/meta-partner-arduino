SUMMARY = "Akida PCIe driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://README.md;md5=4cc86db233f193d7ae82206e187d5505"

inherit module

SRC_URI = " \
    git://github.com/Brainchip-Inc/akida_dw_edma.git;branch=master;protocol=https \
    file://0001-OE-fix-Makefile.patch \
"
SRCREV = "656640c901b07ef4540f425a015c72d1653fb9da"

S = "${WORKDIR}/git"

EXTRA_OEMAKE += "KDIR='${KERNEL_SRC}'"

RPROVIDES_${PN} += "kernel-module-aikida-pcie-driver"
