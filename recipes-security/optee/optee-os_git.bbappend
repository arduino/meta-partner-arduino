
OPTEEMACHINE_portenta-m8 = "imx-imx8mmportenta"

EXTRA_OEMAKE_append_portenta-m8 = " \
    CFG_CORE_DYN_SHM=n CFG_DT=y CFG_OVERLAY_ADDR=0x43600000 \
"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
  file://0001-Add_portenta-m8.patch \
"
