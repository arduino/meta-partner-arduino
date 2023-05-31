DESCRIPTION = "kernel for Arduino Portenta MX8MM platform"

FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:portenta-mx8mm = " \
  file://portenta-mx8mm.cfg \
  file://0002-drivers-clk-rtc-enable-RTC-as-clock-input-for-the-pm.patch \
  file://0002-Extcon-GPIO_Add_DT_bindings.patch \
  file://1000-TEMP-FIX_PCIe_reset_after_clock.patch \
  file://mfd-bd718x7-wait-for-clock.patch \
  file://uart-imx-0001-Bug-rx-uart1-doesn-t-work-with-DMA-channels.patch \
  file://change-trip0-to-75.patch \
  file://0002-fix-ov5647_mipi-build.patch \
  file://0003-add-debug-prints-to-imx219.patch \
  file://0004-porting-from-linux-rpi-5.10.patch \
  file://0005-adding-debug-read-write-regs-to-ov5647_mipi.patch \
  file://0006-pwdn-polarity-is-wrong.patch \
"

SRC_URI:append:portenta-m8 = " \
  file://portenta-m8-standard.scc \
  file://portenta-m8.cfg \
"

SRC_URI:append:portenta-x8 = " \
  file://portenta-x8-standard.scc \
  file://portenta-x8.cfg \
  file://0001-rohm-bd718xx-add-dt-configurable-button-keycode.patch \
  file://V4L2-0001-LF-3760-media-csi-fill-in-colorspace.patch \
  file://V4L2-0002-LF-7107-1-media-capture-csi-Fix-intermittent-camera-.patch \
  file://V4L2-imx219.patch \
  file://V4L2-imx219-def-gain-exp.patch \
"

# file://rtc-pcf8563-remove-workaround.patch
