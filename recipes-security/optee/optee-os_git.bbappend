OPTEEMACHINE_portenta-m8 = "imx-imx8mmevk"

EXTRA_OEMAKE_append_portenta-m8 = " \
    CFG_CORE_DYN_SHM=n CFG_DT=y CFG_OVERLAY_ADDR=0x43600000 \
    CFG_UART_BASE=UART3_BASE \
"
