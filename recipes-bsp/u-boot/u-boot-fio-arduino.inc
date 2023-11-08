# common Arduino u-boot-fio settings
SRC_URI:append:portenta-mx8mm = " \
    file://0001-Added-Portenta-X8-hw-support-need-to-test.patch \
    file://0044-Forcefully-restore-FEC-reset-with-gpio.patch \
"
