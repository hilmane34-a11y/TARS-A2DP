add_library(usermod_tars_a2dp INTERFACE)

target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp.c
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Ambil include directory dari komponen Bluetooth ESP-IDF.
# Ini diperlukan agar proses QSTR MicroPython dapat menemukan
# header Bluetooth seperti esp_bt_main.h.
idf_component_get_property(
    TARS_BT_INCLUDE_DIRS
    bt
    INCLUDE_DIRS
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${TARS_BT_INCLUDE_DIRS}
)

# Link ke library Bluetooth ESP-IDF.
target_link_libraries(usermod_tars_a2dp INTERFACE
    __idf_bt
)

target_link_libraries(usermod INTERFACE
    usermod_tars_a2dp
)
