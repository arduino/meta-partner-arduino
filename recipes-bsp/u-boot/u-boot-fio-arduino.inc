# common Arduino u-boot-fio settings
LIC_FILES_CHKSUM = "file://Licenses/README;md5=5a7450c57ffe5ae63fd732446b988025"

SRCREV_portenta-mx8mm = "e08064a9524b65a027b146dd841a29b2cfeeda56"
SRCBRANCH_portenta-mx8mm = "2021.04+imx_5.10.35_2.0.0-arduino"

SRC_URI_portenta-mx8mm = "git://source.foundries.io/factories/arduino/u-boot.git;protocol=https;branch=${SRCBRANCH} \
    file://fw_env.config \
    file://lmp.cfg \
"
