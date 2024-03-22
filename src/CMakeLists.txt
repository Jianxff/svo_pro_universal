add_library(${PROJECT_NAME} SHARED "")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/3rd/minkindr/include/
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        Threads::Threads
        yaml-cpp
        Eigen3::Eigen
        Ceres::ceres
        ${OpenCV_LIBS}

)

## subdirectories
add_subdirectory(svo_core)
add_subdirectory(fast_neon)
add_subdirectory(vikit)