SUMMARY = "Linux kernel firmware files from Cypress"
DESCRIPTION = "Cypress' WLAN firmware with customized CLM Blob"
HOMEPAGE = "https://github.com/murata-wireless/cyw-fmac-fw"
SECTION = "kernel"
LICENSE = "Proprietary"
LIC_FILES_CHKSUM = "file://LICENCE;md5=cbc5f665d04f741f1e006d2096236ba7"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = "git://github.com/murata-wireless/cyw-fmac-fw;protocol=https;branch=master"
SRCREV = "ba140e42c3320262fc52e185c3af93eeb10117df"
PV = "20210112"

S = "${WORKDIR}/git"

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/
    install -d ${D}${nonarch_base_libdir}/firmware/brcm

    install -m 0644 ${S}/LICENCE ${D}${nonarch_base_libdir}/firmware/LICENCE.cyw-fmac-fw
    install -m 0644 ${S}/cyfmac43430-sdio.bin ${D}${nonarch_base_libdir}/firmware/brcm/brcmfmac43430-sdio.bin
    install -m 0644 ${S}/cyfmac43430-sdio.1DX.clm_blob ${D}${nonarch_base_libdir}/firmware/brcm/brcmfmac43430-sdio.clm_blob
}

FILES:${PN} = " \
    ${nonarch_base_libdir}/firmware/LICENCE.cyw-fmac-fw \
    ${nonarch_base_libdir}/firmware/brcm/brcmfmac43430-sdio.bin \
    ${nonarch_base_libdir}/firmware/brcm/brcmfmac43430-sdio.clm_blob \
"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE:portenta-mx8mm = ".*"
