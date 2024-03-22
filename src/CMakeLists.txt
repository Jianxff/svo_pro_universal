add_library(${PROJECT_NAME} SHARED "")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/3rd/minkindr/include/
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        Threads::Threads
        ${OpenCV_LIBS}
        Eigen3::Eigen
        opengv
        Ceres::ceres
        yaml-cpp
        glog::glog
)

## subdirectories
add_subdirectory(svo_core)
add_subdirectory(fast_neon)
add_subdirectory(vikit)
add_subdirectory(visualize)