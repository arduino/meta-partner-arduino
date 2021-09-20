DESCRIPTION = "kernel for Arduino Portenta M8 platform"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
  file://portenta-m8-standard.scc \
  file://portenta-m8.cfg \
  file://0002-drivers-clk-rtc-enable-RTC-as-clock-input-for-the-pm.patch \
  file://0002-Extcon-GPIO_Add_DT_bindings.patch \
  file://1000-TEMP-FIX_PCIe_reset_after_clock.patch \
  file://mfd-bd718x7-wait-for-clock.patch \
  file://uart-imx-0001-Bug-rx-uart1-doesn-t-work-with-DMA-channels.patch \
"

# file://rtc-pcf8563-remove-workaround.patch
