# EuRoC Mono
add_executable(euroc_mono 
    ${CMAKE_CURRENT_SOURCE_DIR}/euroc_mono.cpp
)
target_link_libraries(euroc_mono PRIVATE ${PROJECT_NAME})


# EuRoC Mono IMU
add_executable(euroc_mono_imu 
    ${CMAKE_CURRENT_SOURCE_DIR}/euroc_mono_imu.cpp
)
target_link_libraries(euroc_mono_imu PRIVATE ${PROJECT_NAME})


# Images Mono
add_executable(image_mono
    ${CMAKE_CURRENT_SOURCE_DIR}/image_mono.cpp
)
target_link_libraries(image_mono PRIVATE ${PROJECT_NAME})


# Video Mono
add_executable(video_mono
    ${CMAKE_CURRENT_SOURCE_DIR}/video_mono.cpp
)
target_link_libraries(video_mono PRIVATE ${PROJECT_NAME})


# Webcam
add_executable(webcam
    ${CMAKE_CURRENT_SOURCE_DIR}/webcam.cpp
)
target_link_libraries(webcam PRIVATE ${PROJECT_NAME})