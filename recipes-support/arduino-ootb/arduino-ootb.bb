SUMMARY = "Arduino OOTB"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://connection-status-led.timer \
    file://connection-status-led.service \
    file://ncm-network.service \
    file://create-docker-envfile.service \
    file://offline-update.service \
    file://connection_status_led \
    file://ec200a-eu.service \
    file://compose-apps-aklite-offline-run.service \
    file://set_cpu_freq \
    file://disable-adb \
    file://disable-ssh \
"

inherit systemd

SYSTEMD_SERVICE:${PN} = "connection-status-led.timer connection-status-led.service ncm-network.service create-docker-envfile.service offline-update.service ec200a-eu.service compose-apps-aklite-offline-run.service"

do_install() {
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/connection-status-led.timer ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/connection-status-led.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/ncm-network.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/create-docker-envfile.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/offline-update.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/ec200a-eu.service ${D}${systemd_system_unitdir}/
    install -m 0644 ${WORKDIR}/compose-apps-aklite-offline-run.service ${D}${systemd_system_unitdir}/

    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/connection_status_led ${D}${bindir}/
    install -m 0755 ${WORKDIR}/disable-adb ${D}${bindir}/
    install -m 0755 ${WORKDIR}/disable-ssh ${D}${bindir}/
}

do_install:append() {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/set_cpu_freq ${D}${bindir}/
}

RDEPENDS:${PN} += "systemd udev-rules-portenta"

FILES:${PN} += " \
    ${bindir} \
    ${systemd_system_unitdir} \
"
