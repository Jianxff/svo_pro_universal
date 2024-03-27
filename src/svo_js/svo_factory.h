#pragma once

#include <memory>

#include <svo/svo.h>
#include <svo/frame_handler_mono.h>

#include <svo/common/camera_fwd.h>
#include <svo/frame_handler_base.h>
#include <svo/imu_handler.h>
#include <svo/initialization.h>
#include <svo/reprojector.h>
#include <svo/stereo_triangulation.h>

#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection_types.h>
#include <svo/tracker/feature_tracking_types.h>

#include <vikit/cameras/ncamera.h>
#include <vikit/cameras/camera_factory.h>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <opencv2/opencv.hpp>


/// Options Loading
svo::BaseOptions 
    loadBaseOptions(const emscripten::val json);

svo::IMUHandlerOptions 
    loadIMUHandlerOptions(const emscripten::val json);

svo::InitializationOptions 
    loadInitializationOptions(const emscripten::val json);

svo::ReprojectorOptions 
    loadReprojectorOptions(const emscripten::val json);

svo::StereoTriangulationOptions 
    loadStereoTriangulationOptions(const emscripten::val json);

svo::DepthFilterOptions 
    loadDepthFilterOptions(const emscripten::val json);

svo::DetectorOptions 
    loadDetectorOptions(const emscripten::val json);

svo::FeatureTrackerOptions 
    loadFeatureTrackerOptions(const emscripten::val json);


/// Factory for Mono-SVO.
std::shared_ptr<svo::FrameHandlerMono> makeMono(
    const CameraBundlePtr& cam,
    const emscripten::val config
);

/// Factory for IMU
svo::ImuCalibration loadImuCalibration(const emscripten::val json);
svo::ImuInitialization loadImuInitialization(const emscripten::val json);
std::shared_ptr<svo::ImuHandler> makeIMU(
    const emscripten::val imu_calib,
    const emscripten::val config
);

/// Factory for camera bunlde
std::shared_ptr<svo::CameraBundle> makeCamera(
    const emscripten::val calib
);


/// Full system
class Odometry {
public:
    Odometry(
        const emscripten::val calib,
        const emscripten::val config
    );

    void start() const;

    const Stage stage() const;
    const uint32_t imwidth() const;
    const uint32_t imheight() const;

    const Eigen::Matrix4d transform_world_cam() const;
    emscripten::val world_pose_gl() const;
    emscripten::val world_viewpose_gl() const;

    void addImageBundle(
        const emscripten::val timestamp,
        int arraybuffer
    ) const;

    void addImuMeasurement(
        const emscripten::val timestamp,
        const emscripten::val gyro_array,
        const emscripten::val acc_array
    ) const;

private:
    bool _set_imu_prior(const uint64_t timestamp) const;
    cv::Mat _emscripten_arraybuffer_to_cvmat(int data) const;

    uint32_t imwidth_;
    uint32_t imheight_;

    ImuHandlerPtr imu_handler_;
    FrameHandlerMono::Ptr frame_handler_;

    bool set_initial_attitude_from_gravity_ = true;
};


