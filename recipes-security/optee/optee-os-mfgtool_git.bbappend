OPTEEMACHINE_portenta-m8 = "imx-imx8mmevk"

EXTRA_OEMAKE_append_portenta-m8 = " \
    CFG_DT=y CFG_OVERLAY_ADDR=0x43600000 \
"

EXTRA_OEMAKE_append_imx = " \
    CFG_NXP_WORKAROUND_CAAM_LOCKED_BY_HAB=y \
"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
  file://0001-Add_portenta-m8.patch \
"
