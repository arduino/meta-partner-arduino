FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += " \
    file://0001-Add-NETDEVCTL-command-at-the-end.patch \
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

