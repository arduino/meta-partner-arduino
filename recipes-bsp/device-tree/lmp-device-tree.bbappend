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
"

DTB_OVERLAYS:append:portenta-x8 = " \
    file://ov_som_x8h7.dts \
    file://ov_som_gpu_vpus.dts \
    file://ov_som_anx7625_video.dts \
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
    file://ov_carrier_max_ov5647_camera_mipi.dts \
    file://ov_carrier_max_imx219_camera_mipi.dts \
    file://ov_carrier_max_pcie_mini.dts \
    file://ov_carrier_rasptenta_base.dts \
    file://ov_carrier_rasptenta_spi.dts \
    file://ov_carrier_rasptenta_ov5647_camera_mipi.dts \
    file://ov_carrier_rasptenta_imx219_camera_mipi.dts \
    file://ov_carrier_rasptenta_imx708_camera_mipi.dts \
    file://ov_carrier_rasptenta_pwm_fan.dts \
    file://ov_rasptenta_iqaudio_codec.dts \
    file://ov_carrier_pika_spark.dts \
    file://ov_carrier_ditto_base.dts \
    file://ov_carrier_mid_ov5647_camera_mipi.dts \
    file://ov_carrier_mid_pcie_mini.dts \
    file://ov_carrier_mid_pro_demo_t1s.dts \
    file://ov_carrier_mid_imx219_camera_mipi.dts \
    file://ov_carrier_mid_dsi_panel.dts \
    file://ov_carrier_mid_dsi_lds101.dts \
    file://ov_som_x8h7_spidev.dts \
"

SRC_URI:append:portenta-mx8mm = " \
    file://arduino_portenta-mx8mm.dtsi \
    ${DTB_OVERLAYS} \
"

SRC_URI:append:portenta-m8 = " \
    file://arduino_portenta-m8.dts \
    file://anx7625.dtsi \
"

SRC_URI:append:portenta-x8 = " \
    file://arduino_portenta-x8.dts \
    file://anx7625_base.dtsi \
"

# This patch is necessary since usbc need to be
# used with specific driver for uuu communication
SRC_URI:append:lmp-mfgtool:portenta-m8 = " \
    file://force-usbc-device-mfgtool-m8.patch \
"
SRC_URI:append:lmp-mfgtool:portenta-x8 = " \
    file://force-usbc-device-mfgtool-x8.patch \
"

COMPATIBLE_MACHINE:portenta-mx8mm = ".*"

FILES:${PN}:portenta-mx8mm += " \
    /boot/devicetree/* \
"
