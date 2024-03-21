add_library(${PROJECT_NAME} SHARED "")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/3rd/spdlog/include/
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
add_subdirectory(rpg_common)
add_subdirectory(svo)
# add_subdirectory(svo_ceres_backend)
add_subdirectory(svo_common)
add_subdirectory(svo_direct)
add_subdirectory(svo_img_align)
add_subdirectory(svo_pgo)
add_subdirectory(svo_tracker)
add_subdirectory(svo_vio_common)
add_subdirectory(fast_neon)
add_subdirectory(vikit)