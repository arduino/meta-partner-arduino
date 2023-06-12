DESCRIPTION = "udev rules for Arduino Portenta Boards"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
    file://65-apex.rules \
"

S = "${WORKDIR}"

INHIBIT_DEFAULT_DEPS = "1"

do_install () {
    install -d ${D}${sysconfdir}/udev/rules.d
    install -m 0644 ${WORKDIR}/65-apex.rules ${D}${sysconfdir}/udev/rules.d/
}

# Required group configuration for udev rules
EXTRA_USERS_PARAMS:append = " \
groupadd apex; \
usermod -a -G apex ${LMP_USER}; \
"
