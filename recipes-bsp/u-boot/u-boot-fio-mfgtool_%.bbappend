FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:${THISDIR}/u-boot-fio:"

SRC_URI_prepend_portenta-m8 = " \
    file://lmp-spl-imx.cfg \
"

SRC_URI_append_portenta-m8 = " \
    file://0001-Add_Portenta-M8_Board.patch \
"
