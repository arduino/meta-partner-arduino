#
#
#

DESCRIPTION = "kernel for Arduino Portenta M8 platform"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append_portenta-m8 = " \
  file://0001-Add_Portenta-M8_Board.patch \
  file://0002-drivers-clk-rtc-enable-RTC-as-clock-input-for-the-pm.patch \
  file://0002-Extcon-GPIO_Add_DT_bindings.patch \
  file://0003-DTB_enable_pci.patch \
  file://1000-TEMP-FIX_PCIe_reset_after_clock.patch \
  file://0005-DTB-Added_GPIO1_9_11_to_hog.patch \
  file://0006-DTB_ANX7625_Audio.patch \
  file://0007-ANX7625_Audio-DTB.patch \
  file://portenta_m8_defconfig \
"
