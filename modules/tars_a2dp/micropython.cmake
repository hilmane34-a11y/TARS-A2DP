add_library(usermod_tars_a2dp INTERFACE)

target_sources(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tars_a2dp.c
)

target_include_directories(usermod_tars_a2dp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod_tars_a2dp INTERFACE
    __idf_bt
    __idf_esp_http_client
    __idf_esp_tls
    __idf_tcp_transport
    __idf_mbedtls
)

target_link_libraries(usermod INTERFACE
    usermod_tars_a2dp
)
