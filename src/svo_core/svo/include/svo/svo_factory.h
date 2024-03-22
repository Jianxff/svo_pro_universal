#pragma once

#include <memory>

#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>


#include <svo/common/camera_fwd.h>
#include <svo/frame_handler_base.h>
#include <svo/imu_handler.h>
#include <svo/initialization.h>
#include <svo/reprojector.h>
#include <svo/stereo_triangulation.h>

#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection_types.h>
#include <svo/tracker/feature_tracking_types.h>

#include <yaml-cpp/yaml.h>

namespace svo{
namespace factory{

/// Options Loading
BaseOptions 
    loadBaseOptions(const std::string& filepath);

IMUHandlerOptions 
    loadIMUHandlerOptions(const std::string& filepath);

InitializationOptions 
    loadInitializationOptions(const std::string& filepath);

ReprojectorOptions 
    loadReprojectorOptions(const std::string& filepath);

StereoTriangulationOptions 
    loadStereoTriangulationOptions(const std::string& filepath);

DepthFilterOptions 
    loadDepthFilterOptions(const std::string& filepath);

DetectorOptions 
    loadDetectorOptions(const std::string& filepath);

FeatureTrackerOptions 
    loadFeatureTrackerOptions(const std::string& filepath);


/// Factory for Mono-SVO.
std::shared_ptr<FrameHandlerMono> makeMono(
    const CameraBundlePtr& cam = nullptr,
    const std::string& config_file_path = ""
);

/// Factory for Stereo-SVO.
std::shared_ptr<FrameHandlerStereo> makeStereo(
    const CameraBundlePtr& cam = nullptr,
    const std::string& config_file_path = ""
);

/// Factory for Camera-Array-SVO.
std::shared_ptr<FrameHandlerArray> makeArray(
    const CameraBundlePtr& cam = nullptr,
    const std::string& config_file_path = ""
);

} // namespace factory
} // namespace svo


