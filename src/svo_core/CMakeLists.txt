target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/3rd/minkindr/include/
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