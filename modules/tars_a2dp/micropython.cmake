add_library(usermod_tars_a2dp INTERFACE)

target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp.c
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}

    # Bluetooth ESP-IDF headers
    $ENV{IDF_PATH}/components/bt/include/esp32/include
    $ENV{IDF_PATH}/components/bt/host/bluedroid/api/include/api
    $ENV{IDF_PATH}/components/bt/host/bluedroid/common/include
)

target_link_libraries(usermod INTERFACE
    usermod_tars_a2dp
)
