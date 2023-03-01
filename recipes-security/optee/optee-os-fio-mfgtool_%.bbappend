EXTRA_OEMAKE:append:portenta-x8 = " \
    CFG_UART_BASE=UART3_BASE \
"

EXTRA_OEMAKE:append:portenta-x8 = " \
    ${@bb.utils.contains('MACHINE_FEATURES', 'se05x', 'CFG_IMX_I2C=y CFG_CORE_SE05X_I2C_BUS=0', '', d)} \
    CFG_TEE_CORE_LOG_LEVEL=1 \
    CFG_TEE_TA_LOG_LEVEL=1 \
"

EXTRA_OEMAKE:remove:portenta-x8 = " \
    CFG_CORE_SE05X_I2C_BUS=2 \
    CFG_TEE_CORE_LOG_LEVEL=2 \
    CFG_TEE_TA_LOG_LEVEL=2 \
"
