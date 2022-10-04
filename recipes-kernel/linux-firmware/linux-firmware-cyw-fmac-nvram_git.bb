SUMMARY = "Linux kernel firmware files from Cypress"
DESCRIPTION = "Cypress' WLAN NVRAM files"
HOMEPAGE = "https://github.com/murata-wireless/cyw-fmac-nvram"
SECTION = "kernel"
LICENSE = "Proprietary"
LIC_FILES_CHKSUM = "file://LICENCE.cypress;md5=cbc5f665d04f741f1e006d2096236ba7"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = "git://github.com/murata-wireless/cyw-fmac-nvram;protocol=https;branch=master"
SRCREV = "8710e74e79470f666912c3ccadf1e354d6fb209c"
PV = "20210112"

S = "${WORKDIR}/git"

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/
    install -d ${D}${nonarch_base_libdir}/firmware/brcm

    install -m 0644 ${S}/LICENCE.cypress ${D}${nonarch_base_libdir}/firmware/LICENCE.cyw-fmac-nvram
    install -m 0644 ${S}/cyfmac43430-sdio.1DX.txt ${D}${nonarch_base_libdir}/firmware/brcm/brcmfmac43430-sdio.txt
}

FILES:${PN} = " \
    ${nonarch_base_libdir}/firmware/LICENCE.cyw-fmac-nvram \
    ${nonarch_base_libdir}/firmware/brcm/brcmfmac43430-sdio.txt \
"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE:portenta-mx8mm = ".*"
