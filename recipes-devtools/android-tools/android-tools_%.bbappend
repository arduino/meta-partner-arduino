FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += "\
    file://arduino-android-gadget-setup \
    file://core/0001-Changing-gid-and-uid-to-match-the-existing-user-fio.patch;patchdir=system/core \
    file://core/0002-Adding-some-useful-debug-prints.patch;patchdir=system/core \
"

SRC_URI:remove:pn-${PN} = "file://core/0008-adb-Allow-adbd-to-be-ran-as-root.patch;patchdir=system/core"

do_install:append () {
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/arduino-android-gadget-setup ${D}${bindir}
}
