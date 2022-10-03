FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += "\
    file://sftp-ssh.service \
"

do_install:append () {
    # Install mDNS broadcasting service for avahi
    install -d ${D}/${sysconfdir}/avahi/services
    install -m 0644 ${WORKDIR}/sftp-ssh.service ${D}${sysconfdir}/avahi/services/sftp-ssh.service
}
