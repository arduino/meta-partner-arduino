INITRAMFS_MAXSIZE_portenta-mx8mm = "524288"

IMAGE_INSTALL += " \
  hdparm \
"

IMAGE_FEATURES += "ssh-server-dropbear"
