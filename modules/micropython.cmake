target_sources(usermod INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp/tars_a2dp.c
    ${CMAKE_CURRENT_LIST_DIR}/tars_bluetooth/tars_bluetooth.c
)

target_include_directories(usermod INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp
    ${CMAKE_CURRENT_LIST_DIR}/tars_bluetooth
)
