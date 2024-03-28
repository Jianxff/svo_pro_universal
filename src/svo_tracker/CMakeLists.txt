# svo tracker
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/feature_tracker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/feature_tracking_types.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/feature_tracking_utils.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
