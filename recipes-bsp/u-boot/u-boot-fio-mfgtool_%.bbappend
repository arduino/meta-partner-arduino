FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:${THISDIR}/u-boot-fio:"

require u-boot-fio-arduino.inc

SRC_URI:prepend:portenta-mx8mm = " \
    file://lmp-spl-imx.cfg \
"

SRC_URI:prepend:portenta-x8-preempt-rt = " \
   file://lmp-preempt-rt.cfg \
"
