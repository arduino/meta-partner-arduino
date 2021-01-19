# Add symlink to for brcmfmac43430-sdio.txt
do_install_append() {
    ln -s brcmfmac43430-sdio.MUR1DX.txt ${D}${nonarch_base_libdir}/firmware/brcm/brcmfmac43430-sdio.txt
}
