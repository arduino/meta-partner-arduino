SUMMARY = "Arduino Portenta X8 x8h7 driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r1"
PV = "0.1"

SRC_URI = " \
    git://github.com/arduino/portentax8-x8h7.git;branch=feature/add-fixed-frame;protocol=https \
"
SRCREV = "${AUTOREV}"

S = "${WORKDIR}/git"

do_install:append() {
    bbwarn "Blacklisting x8h7 modules"
    install -d ${D}/${sysconfdir}/modprobe.d
    install -m 644 ${S}/blacklist.conf ${D}/${sysconfdir}/modprobe.d/blacklist.conf
}

FILES:${PN}:append = " ${sysconfdir}"
