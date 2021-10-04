FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:${THISDIR}/u-boot-fio:"

require u-boot-fio-arduino.inc

SRC_URI_prepend_portenta-m8 = " \
    file://lmp-spl-imx.cfg \
"
