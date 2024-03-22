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
# svo core
add_subdirectory(rpg_common)
add_subdirectory(svo)
add_subdirectory(svo_common)
add_subdirectory(svo_direct)
add_subdirectory(svo_img_align)
add_subdirectory(svo_pgo)
add_subdirectory(svo_tracker)
add_subdirectory(svo_vio_common)
# svo kits
add_subdirectory(fast_neon)
add_subdirectory(vikit)
add_subdirectory(visualize)