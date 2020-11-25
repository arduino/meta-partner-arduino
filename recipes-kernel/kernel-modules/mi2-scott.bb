SUMMARY = "Analogix ANX 7625 driver"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r1"
PV = "0.1"

SRC_URI = " \
  file://Makefile \
  file://anx7625_display.c \
  file://anx7625_driver.c \
  file://anx7625_pd_fw_update.c \
  file://anx7625_public_interface.c \
  file://anx7625_ocm_code.c \
  file://anx7625_display_table.c \
  file://anx7625_pd30_message.c \
  file://anx7625_private_interface.c \
  file://display.c \
  file://Flash.c \
  file://anx7625_driver.h \
  file://anx7625_display.h \
  file://anx7625_pd_fw_update.h \
  file://anx7625_public_interface.h \
  file://Flash.h \
  file://anx7625_pd30_message.h \
  file://anx7625_private_interface.h \
  file://display.h \
  file://MI2_main_ocm1202.h \
  file://MI2_main_ocm1213.h \
  file://MI2_main_ocm1300.h \
  file://MI2_main_ocm.h \
  file://MI2_REG.h \
  file://MI2_secure_ocm.h \
  file://COPYING \
"

#           file://Kconfig


S = "${WORKDIR}"
