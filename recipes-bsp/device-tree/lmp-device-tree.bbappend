FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

DTB_OVERLAYS_portenta-m8 = " \
    file://overlays.txt \
    file://ov_som_lbee5kl1dx.dts \
    file://ov_carrier_breakout_uart0.dts \
    file://ov_carrier_breakout_uart2.dts \
    file://ov_carrier_breakout_uart3.dts \
    file://ov_carrier_breakout_i2c0.dts \
    file://ov_carrier_breakout_i2c1.dts \
    file://ov_carrier_breakout_i2c2.dts \
    file://ov_carrier_breakout_spi0.dts \
    file://ov_carrier_breakout_spdif.dts \
    file://ov_carrier_breakout_i2s.dts \
    file://ov_carrier_breakout_sai.dts \
    file://ov_carrier_breakout_pdm.dts \
    file://ov_carrier_breakout_sdc.dts \
    file://ov_carrier_breakout_usbfs.dts \
    file://ov_carrier_breakout_gpio.dts \
    file://ov_carrier_breakout_pwm.dts \
    file://ov_carrier_enuc_rs232_sp330.dts \
    file://ov_carrier_enuc_rs485f_sp330.dts \
    file://ov_carrier_enuc_rs485h_sp330.dts \
    file://ov_carrier_enuc_cs42l52.dts \
    file://ov_carrier_enuc_bq24195.dts \
    file://ov_carrier_enuc_sara-r4.dts \
    file://ov_carrier_enuc_lora.dts \
    file://ov_carrier_enuc_usbfs.dts \
    file://ov_carrier_max_cs42l52.dts \
"

SRC_URI_append_portenta-m8 = " \
    file://anx7625.dtsi \
    file://arduino_portenta-m8.dts \
    file://envie_enuc_carrier.dtsi \
    ${DTB_OVERLAYS} \
"
COMPATIBLE_MACHINE_portenta-m8 = ".*"

do_install_append_portenta-m8() {
    install -Dm 0644 ${WORKDIR}/overlays.txt ${D}/boot/devicetree/overlays.txt
}

FILES_${PN}_portenta-m8 += " \
    /boot/devicetree/* \
"
