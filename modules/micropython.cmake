add_library(usermod_tars INTERFACE)

target_sources(usermod_tars INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp/tars_a2dp.c
    ${CMAKE_CURRENT_LIST_DIR}/tars_bluetooth/tars_bluetooth.c
)

target_include_directories(usermod_tars INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp
    ${CMAKE_CURRENT_LIST_DIR}/tars_bluetooth
)

target_link_libraries(usermod INTERFACE
    usermod_tars
)
