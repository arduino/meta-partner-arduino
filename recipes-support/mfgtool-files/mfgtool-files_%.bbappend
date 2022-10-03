FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:portenta-x8 = " \
    file://probe_emmc.uuu \
    file://probe_sdcard.uuu \
    file://linux_initramfs.uuu \
    file://test_ram.uuu \
"

do_deploy:prepend:portenta-x8() {
    install -d ${DEPLOYDIR}/${PN}
    install -m 0644 ${WORKDIR}/probe_emmc.uuu ${DEPLOYDIR}/${PN}/probe_emmc.uuu
    install -m 0644 ${WORKDIR}/probe_sdcard.uuu ${DEPLOYDIR}/${PN}/probe_sdcard.uuu
    install -m 0644 ${WORKDIR}/linux_initramfs.uuu ${DEPLOYDIR}/${PN}/linux_initramfs.uuu
    install -m 0644 ${WORKDIR}/test_ram.uuu ${DEPLOYDIR}/${PN}/test_ram.uuu
}
