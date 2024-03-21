# svo tracker
FILE(GLOB_RECURSE SOURCES_SVO_TRACKER "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
target_sources(${PROJECT_NAME}
    PRIVATE
        ${SOURCES_SVO_TRACKER}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
