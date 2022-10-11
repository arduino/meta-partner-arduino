SUMMARY = "Arduino Portenta X8 x8h7 driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r1"
PV = "0.1"

SRC_URI = " \
  file://Makefile \
  file://x8h7_drv.c \
  file://x8h7_ioctl.h \
  file://x8h7.h \
  file://x8h7_adc.c \
  file://x8h7_gpio.c \
  file://x8h7_pwm.c \
  file://x8h7_rtc.c \
  file://x8h7_can.c \
  file://x8h7_uart.c \
  file://x8h7_ui.c \
  file://x8h7_h7.c \
  file://debug.h \
  file://COPYING \
  file://blacklist.conf \
"

S = "${WORKDIR}"

do_install:append() {
    bbwarn "Blacklisting x8h7 modules"
    install -d ${D}/${sysconfdir}/modprobe.d
    install -m 644 ${WORKDIR}/blacklist.conf ${D}/${sysconfdir}/modprobe.d/blacklist.conf
}

FILES:${PN}:append = " ${sysconfdir}"
