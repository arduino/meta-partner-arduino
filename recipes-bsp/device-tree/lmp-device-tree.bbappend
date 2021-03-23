
inherit devicetree

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
    file://overlays.txt \
    file://anx7625.dtsi \
    file://envie_enuc_carrier.dtsi \
    file://arduino_portenta-m8.dts \
    file://som-lbee5kl1dx.dts \
    file://rs232.dts \
    file://rs485h.dts \
    file://rs485f.dts \
    file://ov_carrier_breakout_uart2.dts \
"
COMPATIBLE_MACHINE_portenta-m8 = ".*"

do_install_append() {
    install -Dm 0644 ${WORKDIR}/overlays.txt ${D}/boot/devicetree/overlays.txt
}

FILES_${PN} += " \
    /boot/devicetree/overlays.txt \
"
