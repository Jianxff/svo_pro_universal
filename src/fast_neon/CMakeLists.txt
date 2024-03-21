# fast neon
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fast_9.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fast_10.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fast_10_score.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/nonmax_3x3.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# IF(DEFINED ENV{ARM_ARCHITECTURE})
# 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a")
# ELSE()
# 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
# ENDIF()

# IF(DEFINED ENV{ARM_ARCHITECTURE})
#   LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
# ELSE()
#   LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
# ENDIF()
