
inherit devicetree

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

DTB_OVERLAYS = " \
    file://overlays.txt \
    file://ov_som_lbee5kl1dx.dts \
    file://ov_carrier_breakout_uart2.dts \
    file://ov_carrier_breakout_i2c0.dts \
    file://ov_carrier_breakout_usbfs.dts \
    file://ov_carrier_enuc_rs232_sp330.dts \
    file://ov_carrier_enuc_rs485f_sp330.dts \
    file://ov_carrier_enuc_rs485h_sp330.dts \
"

SRC_URI_append_portenta-m8 = " \
    file://anx7625.dtsi \
    file://arduino_portenta-m8.dts \
    file://envie_enuc_carrier.dtsi \
    ${DTB_OVERLAYS} \
"
COMPATIBLE_MACHINE_portenta-m8 = ".*"

do_install_append() {
    install -Dm 0644 ${WORKDIR}/overlays.txt ${D}/boot/devicetree/overlays.txt
}

FILES_${PN} += " \
    /boot/devicetree/overlays.txt \
"
