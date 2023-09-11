DESCRIPTION = "Library for decoding images in bayer format. It supports 8, 10, 12 and 16 bpp formats with NEON acceleration on ARM platforms"
HOMEPAGE = "https://gitlab-ext.sigma-chemnitz.de/ensc/bayer2rgb"

SRC_URI = "git://gitlab-ext.sigma-chemnitz.de/ensc/bayer2rgb.git;protocol=https;branch=master"

SRCREV = "bc950b3398ba034fe5cc39f625796a6111cdb87f"

S = "${WORKDIR}/git"

LICENSE = "GPL-3.0-only"
LIC_FILES_CHKSUM = "file://COPYING;md5=d32239bcb673463ab874e80d47fae504"

INSANE_SKIP:${PN} += "file-rdeps"

inherit autotools pkgconfig

DEPENDS += "gengetopt-native"

RPROVIDES:${PN} = "libbayer2rgb3"

FILES:${PN} += "\
    /usr/bin \
    /usr/lib \
    /usr/include \
"