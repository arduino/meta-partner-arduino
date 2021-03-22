FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI = " \
    file://overlays.txt \
"

do_deploy_append() {
    install -d ${DEPLOYDIR}/devicetree
    install -m 0644 overlays.txt ${DEPLOYDIR}/devicetree
}
