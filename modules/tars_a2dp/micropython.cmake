add_library(usermod_tars_bluetooth INTERFACE)

target_sources(usermod_tars_bluetooth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_bluetooth.c
)

target_include_directories(usermod_tars_bluetooth INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_tars_bluetooth)
