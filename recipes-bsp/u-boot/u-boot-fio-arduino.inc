# common Arduino u-boot-fio settings
LIC_FILES_CHKSUM = "file://Licenses/README;md5=5a7450c57ffe5ae63fd732446b988025"

SRCREV_portenta-mx8mm = "feature/add-hw-4.1-x8-support"
SRCBRANCH_portenta-mx8mm = "feature/add-hw-4.1-x8-support"

SRC_URI_portenta-mx8mm = "git://github.com/MaxPayne86Arduino/u-boot.git;protocol=https;branch=${SRCBRANCH} \
    file://fw_env.config \
    file://lmp.cfg \
"
