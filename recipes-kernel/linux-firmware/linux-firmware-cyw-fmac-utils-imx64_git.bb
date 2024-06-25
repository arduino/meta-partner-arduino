SUMMARY = "Test tools from Cypress"
DESCRIPTION = "Cypress' WLAN test tool bin"
HOMEPAGE = "https://github.com/murata-wireless/cyw-fmac-utils-imx64"
SECTION = "devel"
LICENSE = "Proprietary"
LIC_FILES_CHKSUM = "file://LICENCE;md5=cbc5f665d04f741f1e006d2096236ba7"

do_configure[noexec] = "1"
do_compile[noexec] = "1"

INSANE_SKIP:${PN} += "file-rdeps"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = "git://github.com/murata-wireless/cyw-fmac-utils-imx64;protocol=https"
SRCREV = "1bc78d68f9609290b2f6578516011c57691f7815"
PV = "20210713"

S = "${WORKDIR}/git"

do_install() {
    # Copying wl tool binary to /usr/sbin
    install -d ${D}/usr/sbin
    install -m 755 ${S}/wl ${D}/usr/sbin/wl
}

FILES:${PN} += "${sbindir}"

RDEPENDS:${PN} += " libnl-nf"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE:portenta-mx8mm = ".*"
