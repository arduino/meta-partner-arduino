SUMMARY = "Analogix ANX 7625 driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r1"
PV = "0.1"

SRC_URI = " \
  file://Makefile \
  file://anx7625_drv.c \
  file://anx7625.h \
  file://anx7625_pd.c \
  file://anx7625_tcpm.c \
  file://anx7625_sysfs.c \
  file://COPYING \
"

S = "${WORKDIR}"
