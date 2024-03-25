# EuRoC Mono
add_executable(euroc_mono 
    ${CMAKE_CURRENT_SOURCE_DIR}/euroc_mono.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dataset_euroc.cpp
)

target_link_libraries(euroc_mono
    PRIVATE ${PROJECT_NAME}
)


# EuRoC Mono IMU
add_executable(euroc_mono_imu 
    ${CMAKE_CURRENT_SOURCE_DIR}/euroc_mono_imu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dataset_euroc.cpp
)

target_link_libraries(euroc_mono_imu
    PRIVATE ${PROJECT_NAME}
)