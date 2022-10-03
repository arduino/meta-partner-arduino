FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append:portenta-mx8mm = " \
    file://speaker-test-supporting-s24le-format.patch \
"
