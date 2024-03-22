# svo pgo
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/svo/pgo/pgo.h

        ${CMAKE_CURRENT_SOURCE_DIR}/src/pgo.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
