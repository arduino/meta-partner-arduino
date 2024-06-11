FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:lmp-wayland = " \
    file://weston.service.lcdif.patch \
"
