FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

require u-boot-fio-arduino.inc

SRC_URI:prepend:sota:portenta-mx8mm = " \
    ${@bb.utils.contains('MACHINE_FEATURES', 'ebbr', '', 'file://lmp-spl-imx.cfg', d)} \
"

SRC_URI:prepend:sota:portenta-x8-preempt-rt = " \
   file://lmp-preempt-rt.cfg \
"
