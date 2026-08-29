add_library(usermod_tars_a2dp INTERFACE)

target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp.c
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}

    # ESP-IDF Bluetooth headers
    $ENV{IDF_PATH}/components/bt/include/esp32/include

    # Bluedroid API headers
    $ENV{IDF_PATH}/components/bt/host/bluedroid/api/include/api

    # Bluedroid common headers
    $ENV{IDF_PATH}/components/bt/host/bluedroid/common/include
)

target_link_libraries(usermod_tars_a2dp INTERFACE
    bt
)

target_link_libraries(usermod INTERFACE
    usermod_tars_a2dp
)
