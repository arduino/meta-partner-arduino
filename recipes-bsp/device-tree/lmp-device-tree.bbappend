FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

DTB_OVERLAYS:portenta-mx8mm = " \
    file://ov_som_lbee5kl1dx.dts \
    file://ov_carrier_breakout_uart3.dts \
    file://ov_carrier_breakout_spdif.dts \
    file://ov_carrier_breakout_sdc.dts \
    file://ov_carrier_breakout_usbfs.dts \
    file://ov_carrier_max_usbfs.dts \
    file://ov_carrier_max_sdc.dts \
"

DTB_OVERLAYS:append:portenta-m8 = " \
    file://ov_carrier_breakout_uart0.dts \
    file://ov_carrier_breakout_uart1.dts \
    file://ov_carrier_breakout_spi0.dts \
    file://ov_carrier_breakout_i2s.dts \
    file://ov_carrier_breakout_sai.dts \
    file://ov_carrier_breakout_pdm.dts \
    file://ov_carrier_breakout_gpio.dts \
    file://ov_carrier_breakout_pwm.dts \
    file://ov_carrier_enuc_bq24195.dts \
    file://ov_carrier_max_cs42l52.dts \
    file://ov_carrier_enuc_lora.dts \
    file://ov_carrier_max_sara-r4.dts \
"

DTB_OVERLAYS:append:portenta-x8 = " \
    file://ov_som_x8h7.dts \
    file://ov_carrier_breakout_uart1.dts \
    file://ov_carrier_breakout_spi0.dts \
    file://ov_carrier_breakout_spi1.dts \
    file://ov_carrier_breakout_i2s.dts \
    file://ov_carrier_breakout_sai.dts \
    file://ov_carrier_breakout_pdm.dts \
    file://ov_carrier_breakout_gpio.dts \
    file://ov_carrier_breakout_pwm.dts \
    file://ov_carrier_enuc_bq24195.dts \
    file://ov_carrier_max_cs42l52.dts \
    file://ov_carrier_enuc_lora.dts \
    file://ov_carrier_max_sara-r4.dts \
    file://ov_carrier_max_ov5647_camera_mipi.dts \
    file://ov_carrier_max_imx219_camera_mipi.dts \
    file://ov_carrier_max_pcie_mini.dts \
    file://ov_carrier_rasptenta_base.dts \
    file://ov_carrier_rasptenta_ov5647_camera_mipi.dts \
    file://ov_carrier_rasptenta_imx219_camera_mipi.dts \
    file://ov_rasptenta_iqaudio_codec.dts \
"

SRC_URI:append:portenta-mx8mm = " \
    file://anx7625.dtsi \
    file://arduino_portenta-mx8mm.dtsi \
    ${DTB_OVERLAYS} \
"

SRC_URI:append:portenta-m8 = " \
    file://arduino_portenta-m8.dts \
"

SRC_URI:append:portenta-x8 = " \
    file://arduino_portenta-x8.dts \
"

# This patch is necessary since usbc need to be
# used with specific driver for uuu communication
SRC_URI:append:lmp-mfgtool = " \
    file://force-usbc-device-mfgtool.patch \
"

COMPATIBLE_MACHINE:portenta-mx8mm = ".*"

FILES:${PN}:portenta-mx8mm += " \
    /boot/devicetree/* \
"
