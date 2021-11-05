# common Arduino u-boot-fio settings

SRCREV_portenta-mx8mm = "acdcb0bffa3ae18a9886044b4e29a9a5ff2a90a2"
SRCBRANCH_portenta-mx8mm = "2020.04+imx_5.4.70_2.3.0-arduino"

SRC_URI_portenta-mx8mm = "git://source.foundries.io/factories/arduino/u-boot.git;protocol=https;branch=${SRCBRANCH} \
    file://fw_env.config \
    file://lmp.cfg \
"
