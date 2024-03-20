SUMMARY = "Automount boot partion for uboot env handling"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

RDEPENDS:${PN} += "systemd"

SRC_URI = " \
    file://automount-boot-partition.service \
"

inherit systemd

SYSTEMD_SERVICE:${PN} = "automount-boot-partition.service"

do_install() {
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/automount-boot-partition.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} += " \
    ${systemd_system_unitdir} \
"