add_executable(euroc_mono
    ${CMAKE_CURRENT_SOURCE_DIR}/euroc_mono.cpp
)

target_link_libraries(euroc_mono
    PRIVATE
        ${PROJECT_NAME}
)