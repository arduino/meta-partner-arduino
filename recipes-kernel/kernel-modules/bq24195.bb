SUMMARY = "Battery charger BQ24195 driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r1"
PV = "0.1"

SRC_URI = " \
  file://Makefile \
  file://bq24190_charger.c \
  file://bq24190_charger.h \
  file://COPYING \
"

S = "${WORKDIR}"
