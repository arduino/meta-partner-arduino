FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_prepend_sota_portenta-m8 = " \
    file://lmp-spl-imx.cfg \
"

SRC_URI_append_portenta-m8 = " \
    file://0001-Add_Portenta-M8_Board.patch \
"
