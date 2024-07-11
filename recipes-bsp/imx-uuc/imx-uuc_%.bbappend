FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += " \
    file://linuxrc \
"

do_configure:prepend () {
    cp ${WORKDIR}/linuxrc ${S}/linuxrc
}
