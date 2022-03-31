SUMMARY = "Arduino OOTB"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

RDEPENDS_${PN} += "systemd"

SRC_URI = " \
    file://connection-status-led.timer \
    file://connection-status-led.service \
    file://ecm-network.service \
    file://rndis-network.service \
    file://secure-device.service \
    file://connection_status_led \
"

inherit systemd

SYSTEMD_SERVICE_${PN} = "connection-status-led.timer connection-status-led.service rndis-network.service ecm-network.service secure-device.service"

do_install() {
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/connection-status-led.timer ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/connection-status-led.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/rndis-network.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/ecm-network.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/secure-device.service ${D}${systemd_system_unitdir}/

    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/connection_status_led ${D}${bindir}/
}

FILES_${PN} += " \
    ${bindir} \
    ${systemd_system_unitdir} \
"
