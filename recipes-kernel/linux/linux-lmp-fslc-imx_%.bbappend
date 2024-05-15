DESCRIPTION = "kernel for Arduino Portenta MX8MM platform"

FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:portenta-mx8mm = " \
  file://portenta-mx8mm.cfg \
  file://0002-drivers-clk-rtc-enable-RTC-as-clock-input-for-the-pm.patch \
  file://mfd-bd718x7-wait-for-clock.patch \
"

SRC_URI:append:portenta-m8 = " \
  file://portenta-m8-standard.scc \
  file://portenta-m8.cfg \
"

SRC_URI:append:portenta-x8 = " \
  file://portenta-x8-standard.scc \
  file://portenta-x8.cfg \
  file://0001-rohm-bd718xx-add-dt-configurable-button-keycode.patch \
  file://QUECTEL-0001-support-ec200a.patch \
  file://DVFS-0001-Add_400MHz_200MHz_100MHz.patch \
"
