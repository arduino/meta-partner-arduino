FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

require u-boot-fio-arduino.inc

SRC_URI_prepend_sota_portenta-m8 = " \
    file://lmp-spl-imx.cfg \
"
