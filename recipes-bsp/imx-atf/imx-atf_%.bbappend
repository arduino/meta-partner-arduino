# Copyright 2017-2018 NXP

DESCRIPTION = "i.MX ARM Trusted Firmware"
SECTION = "BSP"
LICENSE = "BSD-3-Clause"
# LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/BSD-3-Clause;md5=550794465ba0ec5312d6919e203a55f9"


FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-mx8mm = " \
    file://0001-Uart_2_4_Change_permission.patch \
"

SRC_URI_append_portenta-x8 = " \
    file://0001-imx8mm-platform-change-from-uart2-to-uart3.patch \
"
