DESCRIPTION = "Portenta X8 Proxy server for communication with M4 arduino sketches via RPC"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

RDEPENDS_${PN} = "bash"
RDEPENDS_${PN}-dev = "bash"

GO_IMPORT = "github.com/bcmi-labs/portentax8-m4-proxy"

GO_INSTALL = "${GO_IMPORT}/proxy"

SRC_URI = " \
    git://git@${GO_IMPORT}.git;protocol=ssh;destsuffix=${BPN}-${PV}/src/${GO_IMPORT} \
"

SRCREV = "master"

inherit go-mod

do_compile() {
    cd ${B}/src/${GO_INSTALL}
    mkdir -p ${B}/${GO_BUILD_BINDIR}
    ${GO} build ${GOBUILDFLAGS} -o ${B}/${GO_BUILD_BINDIR}/m4-proxy
    cd ${B}
}

USERADD_PACKAGES = "${PN}"