# svo ceres backend
FILE(GLOB_RECURSE SOURCES_SVO_CERES_BACKEND "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
target_sources(${PROJECT_NAME}
    PRIVATE
        ${SOURCES_SVO_CERES_BACKEND}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
