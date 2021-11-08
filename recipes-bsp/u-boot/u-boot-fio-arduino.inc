# common Arduino u-boot-fio settings
LIC_FILES_CHKSUM = "file://Licenses/README;md5=5a7450c57ffe5ae63fd732446b988025"

SRCREV_portenta-mx8mm = "eabd7758979d633a0a9d00482bc29edcadaa1841"
SRCBRANCH_portenta-mx8mm = "2021.04+imx_5.10.35_2.0.0-arduino"

SRC_URI_portenta-mx8mm = "git://source.foundries.io/factories/arduino/u-boot.git;protocol=https;branch=${SRCBRANCH} \
    file://fw_env.config \
    file://lmp.cfg \
"
