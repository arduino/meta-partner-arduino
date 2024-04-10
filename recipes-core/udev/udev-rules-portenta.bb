DESCRIPTION = "udev rules for Arduino Portenta Boards"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

INHIBIT_DEFAULT_DEPS = "1"

SRC_URI = " \
    file://65-apex.rules \
    file://75-ec200aeu.rules \
    file://90-arduino-devices.rules \
"

do_install () {
    install -d ${D}${sysconfdir}/udev/rules.d
    install -m 0644 ${WORKDIR}/65-apex.rules ${D}${sysconfdir}/udev/rules.d/
    install -m 0644 ${WORKDIR}/75-ec200aeu.rules ${D}${sysconfdir}/udev/rules.d/
    install -m 0644 ${WORKDIR}/90-arduino-devices.rules ${D}${sysconfdir}/udev/rules.d/
}

FILES:${PN} += " \
    ${sysconfdir} \
"
