# u-boot

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
    file://0001-Add_Portenta-M8_Board.patch \
    "
