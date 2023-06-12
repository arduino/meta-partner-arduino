SUMMARY = "Google Gasket driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://../LICENSE;md5=b234ee4d69f5fce4486a80fdaf4a4263"

inherit module

SRC_SHA = "f047773516dd65435becf09d8d03e5ef2a9f4165"

SRC_URI = " \
    git://github.com/google/gasket-driver.git;rev=${SRC_SHA};branch=main;protocol=https \
    file://0001-OE-fix-Makefile.patch \
"

S = "${WORKDIR}/git/src"

EXTRA_OEMAKE += "KDIR='${KERNEL_SRC}'"

RPROVIDES_${PN} += "kernel-module-google-gasket-driver"
