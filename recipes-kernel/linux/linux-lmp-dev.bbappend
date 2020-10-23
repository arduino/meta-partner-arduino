#
#
#

DESCRIPTION = "kernel for Arduino Portenta M8 platform"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
  file://0001-Add_Portenta-M8_Board.patch \
  file://portenta_m8_defconfig \
"
