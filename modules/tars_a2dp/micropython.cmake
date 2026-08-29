add_library(usermod_tars_a2dp INTERFACE)

target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp.c
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Hubungkan custom module ke komponen Bluetooth ESP-IDF.
# Jangan memakai "bt" karena itu menyebabkan error: cannot find -lbt.
target_link_libraries(usermod_tars_a2dp INTERFACE
    __idf_bt
)

target_link_libraries(usermod INTERFACE
    usermod_tars_a2dp
)
