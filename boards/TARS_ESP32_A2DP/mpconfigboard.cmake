set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    ${MICROPY_BOARD_DIR}/sdkconfig.bluetooth
)

set(PARTITION_TABLE_CUSTOM_FILENAME
    "${MICROPY_BOARD_DIR}/partitions.csv"
)
