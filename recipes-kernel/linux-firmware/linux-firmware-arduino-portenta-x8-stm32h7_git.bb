SUMMARY = "Linux kernel firmware files from Arduino"
DESCRIPTION = "Arduino smt32h7 firmware for Portenta-X8 boards"
HOMEPAGE = "https://github.com/arduino/portentam8-stm32h7-fw"
SECTION = "kernel"
LICENSE = "GPL-3.0-only"
LIC_FILES_CHKSUM = "file://LICENSE;md5=36f5ec3b7b5f969e397ea32973eb0e44"
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = " \
    git://github.com/arduino/portentam8-stm32h7-fw.git;protocol=https;branch=master \
    file://stm32h7-program.service \
    file://m4-led-forwarder.service \
    file://m4_led_forwarder \
    file://monitor-m4-elf-file.path \
    file://monitor-m4-elf-file.service \
"
SRCREV = "f7a7dc907b123f65abe472d18f8d5d57fc76a432"
PV = "0.0.2"

S = "${WORKDIR}/git"

do_compile() {
    bbnote "Building stm32h7 firmware with cortex-m7 arch"
    unset LDFLAGS CFLAGS CPPFLAGS CFLAGS_ASM
    make
}

inherit systemd

SYSTEMD_SERVICE:${PN} = "stm32h7-program.service m4-led-forwarder.service monitor-m4-elf-file.path monitor-m4-elf-file.service"

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/
    install -d ${D}${nonarch_base_libdir}/firmware/arduino/stm32h7-fw

    #install -m 0644 ${S}/LICENSE.arduino ${D}${nonarch_base_libdir}/firmware/LICENSE.arduino-stm32h7-fw
    install -m 0644 ${S}/Makefile ${D}${nonarch_base_libdir}/firmware/LICENSE.arduino-stm32h7-fw
    install -m 0644 ${S}/STM32H747AII6_CM7.bin ${D}${nonarch_base_libdir}/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin

    install -d ${D}/${sysconfdir}
    cd ${D}/${sysconfdir}
    ln -s ..${nonarch_base_libdir}/firmware firmware

    bbwarn "Copying stm32h7 firmware needed by x8h7 package in /usr/arduino/extra"
    install -d ${D}/usr/arduino/extra
    ln -s ../../..${nonarch_base_libdir}/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin ${D}/usr/arduino/extra/STM32H747AII6_CM7.bin

    # Scripts inside package git repo
    install -m 0744 ${S}/scripts/reset.sh ${D}/usr/arduino/extra/reset.sh
    install -m 0744 ${S}/scripts/program.sh ${D}/usr/arduino/extra/program.sh
    install -m 0744 ${S}/scripts/program-h7.sh ${D}/usr/arduino/extra/program-h7.sh
    install -m 0744 ${S}/scripts/load_modules_pre.sh ${D}/usr/arduino/extra/load_modules_pre.sh
    install -m 0744 ${S}/scripts/load_modules_post.sh ${D}/usr/arduino/extra/load_modules_post.sh
    install -m 0744 ${S}/scripts/unload_modules.sh ${D}/usr/arduino/extra/unload_modules.sh
    install -m 0644 ${S}/openocd/openocd_script-imx_gpio.cfg ${D}/usr/arduino/extra/openocd_script-imx_gpio.cfg

    # Systemd service
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/stm32h7-program.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/m4-led-forwarder.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/monitor-m4-elf-file.path ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/monitor-m4-elf-file.service ${D}${systemd_system_unitdir}/

    # Scripts in yocto layer
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/m4_led_forwarder ${D}${bindir}/
}


FILES:${PN} = " \
    ${nonarch_base_libdir}/firmware/LICENSE.arduino-stm32h7-fw \
    ${nonarch_base_libdir}/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin \
    ${sysconfdir}/firmware \
    /usr/arduino/extra/STM32H747AII6_CM7.bin \
    /usr/arduino/extra/reset.sh \
    /usr/arduino/extra/program.sh \
    /usr/arduino/extra/program-h7.sh \
    /usr/arduino/extra/load_modules_pre.sh \
    /usr/arduino/extra/load_modules_post.sh \
    /usr/arduino/extra/unload_modules.sh \
    /usr/arduino/extra/openocd_script-imx_gpio.cfg \
    ${systemd_system_unitdir} \
    ${bindir} \
"

DEPENDS += " \
    gcc-arm-none-eabi-native \
"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE:portenta-mx8mm = ".*"
