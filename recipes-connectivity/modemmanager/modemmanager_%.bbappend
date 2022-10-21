FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SYSTEMD_AUTO_ENABLE:portenta-mx8mm = "disable"

SRC_URI += " \
    file://tweak-modemmanager-service.patch \
    file://modem_on.sh \
    file://modem_off.sh \
"

do_install:append () {
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/modem_on.sh ${D}${sbindir}/
    install -m 0755 ${WORKDIR}/modem_off.sh ${D}${sbindir}/
}

FILES:${PN} += " \
    ${sbindir}/modem_on.sh \
    ${sbindir}/modem_off.sh \
"

