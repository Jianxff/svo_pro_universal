# vikit common
FILE(GLOB_RECURSE SOURCES_VIKIT_COMMON "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
target_sources(${PROJECT_NAME}
    PRIVATE
        ${SOURCES_VIKIT_COMMON}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
