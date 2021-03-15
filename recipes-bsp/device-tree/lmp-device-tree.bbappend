FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
    file://anx7625.dtsi \
    file://envie_enuc_carrier.dtsi \
    file://arduino_portenta-m8.dts \
    file://som-lbee5kl1dx.dtbo \
    file://rs232.dtbo \
    file://rs485h.dtbo \
    file://rs485f.dtbo \
"
COMPATIBLE_MACHINE_portenta-m8 = ".*"
