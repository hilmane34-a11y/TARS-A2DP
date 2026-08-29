add_library(usermod_tars_a2dp INTERFACE)

# File MicroPython binding.
# File ini diproses oleh qstr scanner.
target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp.c
)

# Implementasi Bluetooth.
# Jangan dijadikan sumber qstr utama.
target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_bt.c
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Hubungkan ke komponen Bluetooth ESP-IDF
target_link_libraries(usermod_tars_a2dp INTERFACE
    __idf_bt
)

target_link_libraries(usermod INTERFACE
    usermod_tars_a2dp
)
