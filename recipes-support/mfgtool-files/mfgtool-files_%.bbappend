FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

# @TODO: fix in meta-lmp to have all binaries at the same version
UUU_RELEASE_MAC = "1.4.193"

SRC_URI_append_portenta-x8 = " \
    https://github.com/NXPmicro/mfgtools/releases/download/uuu_${UUU_RELEASE_MAC}/uuu_mac;downloadfilename=uuu-${UUU_RELEASE_MAC}_mac;name=Mac \
    file://probe_emmc.uuu \
    file://probe_sdcard.uuu \
    file://linux_initramfs.uuu \
    file://test_ram.uuu \
"

SRC_URI[Mac.md5sum] = "d15ea66af17642b77531f35426997c6f"
SRC_URI[Mac.sha256sum] = "179488da2381c71b4b651f1e46ad27e4c46192a96e79a24bd52a5fbb54a0c6b6"

do_deploy_prepend_portenta-x8() {
    install -d ${DEPLOYDIR}/${PN}
    install -m 0644 ${WORKDIR}/probe_emmc.uuu ${DEPLOYDIR}/${PN}/probe_emmc.uuu
    install -m 0644 ${WORKDIR}/probe_sdcard.uuu ${DEPLOYDIR}/${PN}/probe_sdcard.uuu
    install -m 0644 ${WORKDIR}/linux_initramfs.uuu ${DEPLOYDIR}/${PN}/linux_initramfs.uuu
    install -m 0644 ${WORKDIR}/test_ram.uuu ${DEPLOYDIR}/${PN}/test_ram.uuu

    install -m 0755 ${WORKDIR}/uuu-${UUU_RELEASE_MAC}_mac ${DEPLOYDIR}/${PN}/uuu_mac
}
