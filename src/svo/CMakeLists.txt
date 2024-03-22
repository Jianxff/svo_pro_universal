# svo
FILE(GLOB_RECURSE SOURCES_SVO "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
target_sources(${PROJECT_NAME}
    PRIVATE
        ${SOURCES_SVO}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
