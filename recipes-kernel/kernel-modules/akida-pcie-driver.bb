SUMMARY = "Akida PCIe driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://README.md;md5=b234ee4d69f5fce4486a80fdaf4a4263"

inherit module

SRCREV = "${AUTOREV}"

SRC_URI = " \
    git://github.com/Brainchip-Inc/akida_dw_edma.git;rev=${SRCREV};branch=master;protocol=https \
"

S = "${WORKDIR}/git"

EXTRA_OEMAKE += "KDIR='${KERNEL_SRC}'"

RPROVIDES_${PN} += "kernel-module-aikida-pcie-driver"
