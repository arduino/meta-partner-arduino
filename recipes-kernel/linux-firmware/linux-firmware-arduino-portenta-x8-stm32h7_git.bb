SUMMARY = "Linux kernel firmware files from Arduino"
DESCRIPTION = "Arduino smt32h7 firmware for Portenta-X8 boards"
HOMEPAGE = "https://github.com/bcmi-labs/portentam8-stm32h7-fw"
SECTION = "kernel"
LICENSE = "Proprietary"
#LIC_FILES_CHKSUM = "file://LICENSE.arduino;md5=cbc5f665d04f741f1e006d2096236ba7"
LIC_FILES_CHKSUM = "file://Makefile;md5=dd9b8b57a3f9b9e8bd5e1c5c97c04850"

PACKAGE_ARCH = "${MACHINE_ARCH}"

SRC_URI = "git://github.com/bcmi-labs/portentam8-stm32h7-fw.git;protocol=https;branch=refactor"
SRCREV = "f943e7cdf57c7f87995777b25391a7cb1537d1b3"
PV = "0.0.1"

S = "${WORKDIR}/git"

do_compile() {
    bbnote "Building stm32h7 firmware with cortex-m7 arch"
    unset LDFLAGS CFLAGS CPPFLAGS CFLAGS_ASM
    make
}

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/
    install -d ${D}${nonarch_base_libdir}/firmware/arduino/stm32h7-fw

    #install -m 0644 ${S}/LICENSE.arduino ${D}${nonarch_base_libdir}/firmware/LICENSE.arduino-stm32h7-fw
    install -m 0644 ${S}/Makefile ${D}${nonarch_base_libdir}/firmware/LICENSE.arduino-stm32h7-fw
    install -m 0644 ${S}/STM32H747AII6_CM7.bin ${D}${nonarch_base_libdir}/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin

    install -d ${D}/${sysconfdir}
    cd ${D}/${sysconfdir}
    ln -s ..${nonarch_base_libdir}/firmware firmware

    bbwarn "Copying stm32h7 firmware needed by x8h7 package in /home/fio/extra"
    # @TODO: remove me
    install -d ${D}/home/fio/extra
    cd ${D}/home/fio/extra
    ln -s ../../../${nonarch_base_libdir}/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin STM32H747AII6_CM7.bin

    install -m 0744 ${S}/flash.sh ${D}/home/fio/extra/flash.sh
    install -m 0744 ${S}/reset.sh ${D}/home/fio/extra/reset.sh
    install -m 0744 ${S}/program.sh ${D}/home/fio/extra/program.sh
    install -m 0644 ${S}/openocd_script.cfg ${D}/home/fio/extra/openocd_script.cfg
}

FILES_${PN} = " \
    ${nonarch_base_libdir}/firmware/LICENSE.arduino-stm32h7-fw \
    ${nonarch_base_libdir}/firmware/arduino/stm32h7-fw/STM32H747AII6_CM7.bin \
    ${sysconfdir}/firmware \
    /home/fio/extra/STM32H747AII6_CM7.bin \
    /home/fio/extra/flash.sh \
    /home/fio/extra/reset.sh \
    /home/fio/extra/program.sh \
    /home/fio/extra/openocd_script.cfg \
"

DEPENDS += " \
    gcc-arm-none-eabi-native \
"

COMPATIBLE_MACHINE ?= "^$"
COMPATIBLE_MACHINE_portenta-mx8mm = ".*"
