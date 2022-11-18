DESCRIPTION = "Preempt-RT kernel for Arduino Portenta MX8MM platform"

FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

KERNEL_META_REPO = "git://github.com/bcmi-labs/lmp-kernel-cache.git"
KERNEL_META_REPO_PROTOCOL = "https"
KERNEL_META_BRANCH = "linux-v5.10.y-arduino"
KERNEL_META_COMMIT = "f62d0db5bd67df5b8e6c0082960048cda20b2793"

SRC_URI:append:portenta-mx8mm = " \
  file://portenta-mx8mm.cfg \
  file://0002-drivers-clk-rtc-enable-RTC-as-clock-input-for-the-pm.patch \
  file://0002-Extcon-GPIO_Add_DT_bindings.patch \
  file://1000-TEMP-FIX_PCIe_reset_after_clock.patch \
  file://mfd-bd718x7-wait-for-clock.patch \
  file://uart-imx-0001-Bug-rx-uart1-doesn-t-work-with-DMA-channels.patch \
  file://change-trip0-to-75.patch \
"

SRC_URI:append:portenta-x8-preempt-rt = " \
  file://portenta-x8-preempt-rt.scc \
  file://portenta-x8.cfg \
  file://0001-rohm-bd718xx-add-dt-configurable-button-keycode.patch \
"

# file://rtc-pcf8563-remove-workaround.patch
