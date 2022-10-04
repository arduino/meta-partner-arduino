SUMMARY = "Linux kernel firmware files from Cypress"
DESCRIPTION = "Cypress' Bluetoothpatchfile that is required by hciattach and Bluez"
HOMEPAGE = "https://github.com/murata-wireless/cyw-fmac-fw"
SECTION = "kernel"
LICENSE = "Proprietary"
LIC_FILES_CHKSUM = "file://LICENCE.cypress;md5=cbc5f665d04f741f1e006d2096236ba7"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = " \
    git://github.com/murata-wireless/cyw-bt-patch;protocol=https;branch=master \
    file://BCM43430A1.hcd \
"
SRCREV = "580abcb5b5f06c9ccfb1438b1f52d8bccdff57e6"
PV = "20201214"

S = "${WORKDIR}/git"

do_compile[noexec] = "1"

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/
    install -d ${D}${nonarch_base_libdir}/firmware/brcm

    install -m 0644 ${S}/LICENCE.cypress ${D}${nonarch_base_libdir}/firmware/LICENCE.cyw-bt-patch
    #install -m 0644 ${S}/BCM4343A1_001.002.009.0093.0395.1DX.hcd ${D}${nonarch_base_libdir}/firmware/brcm/BCM43430A1.hcd
    install -m 0644 ${WORKDIR}/BCM43430A1.hcd ${D}${nonarch_base_libdir}/firmware/brcm/BCM43430A1.hcd

    install -d ${D}/${sysconfdir}
    cd ${D}/${sysconfdir}
    ln -s ..${nonarch_base_libdir}/firmware firmware
}

FILES:${PN} = " \
    ${nonarch_base_libdir}/firmware/LICENCE.cyw-bt-patch \
    ${nonarch_base_libdir}/firmware/brcm/BCM43430A1.hcd \
    ${nonarch_base_libdir}/firmware/brcm/BCM43430A1.1DX.hcd \
    ${sysconfdir}/firmware \
"

RCONFLICTS:${PN} = "\
    linux-firmware-bcm43430 \
    linux-firmware-raspbian-bcm43430 \
"
RREPLACES:${PN} = "\
    linux-firmware-bcm43430 \
    linux-firmware-raspbian-bcm43430 \
"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE:portenta-mx8mm = ".*"
