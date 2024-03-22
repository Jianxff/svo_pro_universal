# svo img align
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/svo/img_align/sparse_img_align_base.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/svo/img_align/sparse_img_align.h

        ${CMAKE_CURRENT_SOURCE_DIR}/src/sparse_img_align_base.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sparse_img_align.cpp
        
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
