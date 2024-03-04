# common Arduino u-boot-fio settings
SRC_URI:append:portenta-mx8mm = " \
    file://0001-Added-Portenta-X8-hw-support-need-to-test.patch \
    file://0002-portenta-x8-add-minimal-anx7625-setup.patch \
    file://0003-portenta-x8-enable-anx7625.patch \
    file://0004-portenta-x8-cleanup-left-over-settings-in-x8-dts.patch \
    file://0005-portenta-common-anx7625-add-portenta-x8-HW-settings.patch \
    file://0006-portenta-common-anx7625-additional-changes-to-u-boot.patch \
    file://0007-arch-arm-dts-portenta-x8-fix-i2c-booting-errors.patch \
    file://0008-Bus-num.-i2c-changes-between-machines.patch \
    file://0009-Implemented-PD2.0-downgrade-to-solve-issue-of-video-.patch \
    file://0010-Reviewed-timeout-value-for-GOT_POWER.patch \
    file://0011-Fix-compilation-bug.patch \
    file://0012-A-few-adjs-to-anx-driver-and-doc.patch \
    file://0013-Implemented-carrier-detection-on-portenta-x8.patch \
    file://0014-Refactored-variable-names-for-board-model-and-carrie.patch \
    file://0015-Changed-alert-message-when-power-negotiation-fails.patch \
    file://0016-Added-hw-changes-in-anx-for-x8-hw-v4.1.patch \
    file://0017-Update-lpddr4-timings-with-DBI-enabled.patch \
    file://0018-Forcefully-restore-FEC-reset-with-gpio.patch \
    file://0019-Updated-ddr-ram-timings-according-to-manufacturing.patch \
    file://0020-Added-Rasp-Tenta-eeprom-detect.patch \
    file://0021-Added-VBUS-switch-management.patch \
    file://0022-Review-pinctrl_hog-groups.patch \
    file://0023-Fixed-STM32H7-pins-in-u-boot.patch \
    file://0024-Force-Breakout-carrier-as-default.patch \
    file://0052-Modify-env-in-ram-only-if-carrier_custom-is-not-defi.patch \
"
