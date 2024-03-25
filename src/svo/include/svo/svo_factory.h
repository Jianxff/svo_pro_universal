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

/// Factory for IMU
std::shared_ptr<ImuHandler> makeIMU(
    const std::string& imu_calib_file = "",
    const std::string& config_file_path = ""
);

/// Factory for camera bunlde
std::shared_ptr<CameraBundle> makeCamera(
    const std::string& calib_file = ""
);

} // namespace factory

/// Full system
class Odometry {
public:
    enum Type {
        kMono,
        kStereo,
        kMonoIMU,
        kStereoIMU,
    };

    Odometry(
        const Type type,
        const std::string& calib_file,
        const std::string& svo_config_file,
        const bool set_initial_attitude_from_gravity = true
    );

    FrameHandlerMono::Ptr frame_handler_mono() const;
    FrameHandlerStereo::Ptr frame_handler_stereo() const;
    std::shared_ptr<FrameHandlerBase> frame_handler() const;
    ImuHandler::Ptr imu_handler() const;

    void start() const;

    const Stage stage() const;

    void addImageBundle(
        const std::vector<cv::Mat>& imgs,
        const uint64_t timestamp
    ) const;

    void addImuMeasurement(
        const uint64_t timestamp,
        const Eigen::Vector3d &gyro,
        const Eigen::Vector3d &acc
    ) const;

private:
    bool _set_imu_prior(const uint64_t timestamp) const;

    ImuHandlerPtr imu_handler_;
    FrameHandlerMono::Ptr mono_frame_handler_;
    FrameHandlerStereo::Ptr stereo_frame_handler_;
    std::shared_ptr<FrameHandlerBase> frame_handler_;
    bool set_initial_attitude_from_gravity_;

    const Type type_;
};

} // namespace svo


