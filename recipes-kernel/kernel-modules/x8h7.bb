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
"

S = "${WORKDIR}"

#KERNEL_MODULE_AUTOLOAD_append = "x8h7 x8h7_drv x8h7_adc x8h7_gpio x8h7_pwm x8h7_rtc x8h7_can x8h7_uart"

# Following customization is necessary since modules under standard path are executed by default, revert when module devel
# is ended
do_install() {
    bbwarn "Copying x8h7 modules into /home/fio/extra"
	unset CFLAGS CPPFLAGS CXXFLAGS LDFLAGS
	oe_runmake DEPMOD=echo MODLIB="${D}/home/fio" \
	           INSTALL_FW_PATH="${D}${nonarch_base_libdir}/firmware" \
	           CC="${KERNEL_CC}" LD="${KERNEL_LD}" \
	           O=${STAGING_KERNEL_BUILDDIR} \
	           ${MODULES_INSTALL_TARGET}
}

FILES_${PN} = "/"
