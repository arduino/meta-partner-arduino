SUMMARY = "Arduino OOTB"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

RDEPENDS:${PN} += "systemd"

SRC_URI = " \
    file://connection-status-led.timer \
    file://connection-status-led.service \
    file://secure-device.service \
    file://create-docker-envfile.service \
    file://offline-update.service \
    file://connection_status_led \
"

inherit systemd

SYSTEMD_SERVICE:${PN} = "connection-status-led.timer connection-status-led.service secure-device.service create-docker-envfile.service offline-update.service"

do_install() {
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/connection-status-led.timer ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/connection-status-led.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/secure-device.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/create-docker-envfile.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/offline-update.service ${D}${systemd_system_unitdir}/

    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/connection_status_led ${D}${bindir}/
}

FILES:${PN} += " \
    ${bindir} \
    ${systemd_system_unitdir} \
"
