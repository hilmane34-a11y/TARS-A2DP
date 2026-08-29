add_library(usermod_tars_a2dp INTERFACE)

target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp.c
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(usermod_tars_a2dp INTERFACE
    MODULE_TARS_A2DP_ENABLED=1
)

target_link_libraries(usermod INTERFACE
    usermod_tars_a2dp
)
