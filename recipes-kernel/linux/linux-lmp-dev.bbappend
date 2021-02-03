#
#
#

DESCRIPTION = "kernel for Arduino Portenta M8 platform"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
  file://0002-drivers-clk-rtc-enable-RTC-as-clock-input-for-the-pm.patch \
  file://0002-Extcon-GPIO_Add_DT_bindings.patch \
  file://1000-TEMP-FIX_PCIe_reset_after_clock.patch \
  file://0001-drm-bridge-anx7625-early-drop.patch \
  file://0002-kernel-anx7625-add-traces.patch \
  file://rtc-pcf8563-remove-workaround.patch \
  file://drm-anx7625-wait-for-LDO5.patch \
  file://mfd-bd718x7-wait-for-clock.patch \
  file://anx7625-hdmi-audio.patch \
  file://portenta_m8_defconfig \
"
