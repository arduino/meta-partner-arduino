SUMMARY = "Microchip LAN865x T1S driver"
DESCRIPTION = "Driver for Single pair ethernet chips"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

inherit module

PR = "r1"
PV = "0.1"

SRC_URI = " \
  file://Makefile \
  file://src/lan865x.c \
  file://src/microchip_t1s.c \
  file://src/oa_tc6.c \
  file://src/oa_tc6.h \
"

S = "${WORKDIR}"
