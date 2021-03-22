FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

do_deploy_append() {
    install -d ${DEPLOYDIR}/devicetree
    install -m 0644 overlays.txt ${DEPLOYDIR}/devicetree
}
