cmake_minimum_required(VERSION 3.8)
project(svo_pro LANGUAGES CXX C)

cmake_policy(SET CMP0072 NEW)
cmake_policy(SET CMP0079 NEW)

## cxx standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

## cmake warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msimd128 -Wno-unused-result -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -msimd128 -Wno-unused-result -Wno-ignored-attributes")

## cmake optimize
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")

## emscripten settings
set(COMPILE_FLAG_TEXT "-O3 -fexceptions -mbulk-memory -s USE_PTHREADS=1")
set(LINK_FLAG_TEXT " \
    -s USE_ZLIB=1 \
    -s DYNAMIC_EXECUTION=0 \
    -s INITIAL_MEMORY=256MB \
    -s NO_DISABLE_EXCEPTION_CATCHING=1 \
    -s ERROR_ON_UNDEFINED_SYMBOLS=0 \
    -s MODULARIZE=1 \
    -s EXPORT_NAME=WorldTracking \
    -s SINGLE_FILE=1 \
    -s ENVIRONMENT=web,worker,shell \
    -s USE_PTHREADS=1 \
    -s SHARED_MEMORY=1 \
    -mbulk-memory -fexceptions \
    -O3 --no-entry --no-check-features --bind"
)

# -s ALLOW_MEMORY_GROWTH=1 \
# -s ALLOW_TABLE_GROWTH=1 \

## emscripten debug
# debug on
# set(LINK_FLAG_TEXT "${LINK_FLAG_TEXT} -s ASSERTIONS=1")
# debug off
set(COMPILE_FLAG_TEXT "${COMPILE_FLAG_TEXT} -g0")
set(LINK_FLAG_TEXT "${LINK_FLAG_TEXT} -g0 -s ASSERTIONS=0")

## ccache
find_program(CCACHE_EXE ccache)
if(CCACHE_EXE)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_EXE}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_EXE}")
    message(STATUS "ccache: Enabled")
endif()

## OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    message(STATUS "OpenMP found")
    add_definitions(-DUSE_OPENMP)
endif()

## OpenGV
find_package(opengv REQUIRED)
if(opengv_FOUND)
    message(STATUS "OpenGV found")
    add_definitions(-DSVO_USE_OPENGV)
endif()

## dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)


add_executable(${PROJECT_NAME} "")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/3rd/minkindr/include/
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        ${OpenCV_LIBS}
        Eigen3::Eigen
        opengv
        Ceres::ceres
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${LINK_FLAG_TEXT}")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${COMPILE_FLAG_TEXT}")

## source
add_subdirectory(src)

# add export
add_custom_command(TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND echo "export default WorldTracking;" >> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.js
  VERBATIM
)