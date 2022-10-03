DESCRIPTION = "Portenta X8 Proxy server for communication with M4 arduino sketches via RPC"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

RDEPENDS:${PN} = "bash"
RDEPENDS:${PN}-dev = "bash"

GO_IMPORT = "github.com/arduino/portentax8-m4-proxy"

GO_INSTALL = "${GO_IMPORT}/proxy"

SRC_URI = " \
    git://${GO_IMPORT}.git;protocol=https;destsuffix=${BPN}-${PV}/src/${GO_IMPORT};branch=master \
    file://m4-proxy.service \
"
SRCREV = "b9af564047a2db923443dbc360d1393a24fb5409"

inherit go-mod

do_compile() {
    cd ${B}/src/${GO_INSTALL}
    mkdir -p ${B}/${GO_BUILD_BINDIR}
    ${GO} build ${GOBUILDFLAGS} -o ${B}/${GO_BUILD_BINDIR}/m4_proxy
}

inherit systemd

SYSTEMD_SERVICE:${PN} = "m4-proxy.service"

do_install() {
    # Binary
    install -d ${D}/usr/bin
    install -m 0744 ${B}/bin/linux_arm64/m4_proxy ${D}/usr/bin/m4_proxy

    # Systemd service
    install -d ${D}${systemd_system_unitdir}
    install -m 0644 ${WORKDIR}/m4-proxy.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} += "\
    /usr/bin \
    ${systemd_system_unitdir} \
"
