SUMMARY = "Codec audio cs42l52"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r1"
PV = "0.1"

SRC_URI = " \
  file://Makefile \
  file://cs42l52.c \
  file://cs42l52.h \
  file://COPYING \
"

S = "${WORKDIR}"
