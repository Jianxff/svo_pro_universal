# vikit cameras
FILE(GLOB_RECURSE SOURCES_VIKIT_CAMERAS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
target_sources(${PROJECT_NAME}
    PRIVATE
        ${SOURCES_VIKIT_CAMERAS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
