FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI += "file://arduino-android-gadget-setup"

do_install_append () {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/arduino-android-gadget-setup ${D}${bindir}
}
