DESCRIPTION = "A gstreamer plugin for the bayer2rgb-neon conversion library"
HOMEPAGE = "https://gitlab-ext.sigma-chemnitz.de/ensc/gst-bayer2rgb-neon"

SRC_URI = "git://gitlab-ext.sigma-chemnitz.de/ensc/gst-bayer2rgb-neon.git;protocol=https;branch=master"

SRCREV = "4fe7bba3f0a84db89e1412fefe6adfcdb515761f"

S = "${WORKDIR}/git"

LICENSE = "GPL-3.0-only"
LIC_FILES_CHKSUM = "file://COPYING;md5=d32239bcb673463ab874e80d47fae504"

inherit autotools pkgconfig

DEPENDS += "gstreamer1.0-plugins-base bayer2rgb"

RDEPENDS:${PN} += "bayer2rgb"

FILES:${PN} += "\
    /usr/lib \
"
