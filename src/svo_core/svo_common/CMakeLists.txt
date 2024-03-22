# svo common
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/empty.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/frame.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/point.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)