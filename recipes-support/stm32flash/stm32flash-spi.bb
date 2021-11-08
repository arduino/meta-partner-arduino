SUMMARY = "Open source flash program for STM32 using the ST serial bootloader, SPI version"
HOMEPAGE = "https://sourceforge.net/projects/stm32flash/"
BUGTRACKER = "https://sourceforge.net/p/stm32flash/tickets/"
LICENSE = "GPL-2.0+"
LIC_FILES_CHKSUM = "file://gpl-2.0.txt;md5=b234ee4d69f5fce4486a80fdaf4a4263"

S = "${WORKDIR}/git"

SRC_URI = "git://github.com/facchinm/stm32flash.git;protocol=git;branch=spi"
SRCREV = "b0e60d75edebf47b2a15fa1426199b736a6e0564"

do_install() {
	oe_runmake install DESTDIR=${D} PREFIX=${prefix}
}
