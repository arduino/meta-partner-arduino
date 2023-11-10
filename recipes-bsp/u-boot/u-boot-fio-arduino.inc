# common Arduino u-boot-fio settings
SRC_URI:append:portenta-mx8mm = " \
    file://0001-Added-Portenta-X8-hw-support-need-to-test.patch \
    file://0002-portenta-x8-add-minimal-anx7625-setup.patch \
    file://0023-portenta-x8-enable-anx7625.patch \
    file://0004-portenta-x8-cleanup-left-over-settings-in-x8-dts.patch \
    file://0026-portenta-common-anx7625-add-portenta-x8-HW-settings.patch \
    file://0006-portenta-common-anx7625-additional-changes-to-u-boot.patch \
    file://0007-arch-arm-dts-portenta-x8-fix-i2c-booting-errors.patch \
    file://0044-Forcefully-restore-FEC-reset-with-gpio.patch \
"
