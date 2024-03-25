#pragma once

#include <vector>
#include <fstream>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>


namespace fs = std::filesystem;

namespace dataset {
// assertion
void _assert_filepath_(const fs::path& p);

// EuRoC dataset
class Euroc{
public:
    // Camera data pack
    struct FrameData{
        const uint64_t ts = -1;
        const cv::Mat cam0;
        const cv::Mat cam1;
        const bool valid() const { return ts != -1;}
    };

    // Imu data pack
    struct ImuData{
        const uint64_t ts = -1;
        const Eigen::Vector3d gyro; // gyroscope
        const Eigen::Vector3d acc;  // accelerometer
        const bool valid() const { return ts != -1; }
    };


    // Constructor
    Euroc(const std::string& mav_dir);

    // Reset pointer
    void reset() const;

    // Get next frame data
    const FrameData nextFrame() const;

    // Get next imu data
    const ImuData nextImu() const;

    // Get next imu data before next frame
    const std::vector<ImuData> nextImuAll() const;

    // Align initial frames and imu data
    void align() const;

    // static creation
    static Euroc create(
        const std::string& mav_dir,
        const bool load_imu = false,
        const bool preload_image = false,
        const bool load_config = false
    );
    
    /////////////////////////////////////////////////////////////
    // Get current frame timestamp
    const uint64_t cur_frame_ts(int bias = 0) const;
    // Get current imu timestamp
    const uint64_t cur_imu_ts(int bias = 0) const;
    // get camera frame
    const cv::Mat frame(const size_t idx, const int cam_id) const;
    // get imu data
    const Eigen::Vector3d gyro(const size_t idx) const;
    const Eigen::Vector3d acc(const size_t idx) const;
    // get timestamp
    const uint64_t frame_ts(const size_t idx) const;
    const uint64_t imu_ts(const size_t idx) const;
    // get length
    const size_t len_frames() const;
    const size_t len_imu() const;
    // get config
    const YAML::Node& config() const;
    const YAML::Node operator[](const std::string& key) const;
    /////////////////////////////////////////////////////////////

private:
    // clear all data
    void _clear();
    // load yaml config file
    void _read_yaml_config();
    // load camera frame data
    size_t _read_cam_data();
    // load imu data
    size_t _read_imu_data();
    // read all images
    std::vector<cv::Mat> load_image(const std::vector<fs::path>& files);

    bool preload_;                  // preload all images
    fs::path mav_dir_;              // base MAV dir for EuRoC dataset

    mutable size_t image_pt_;       // frame reader pointer
    mutable size_t imu_pt_;         // imu reader pointer

    std::vector<uint64_t> cam_ts_;  // camera timestamps
    std::vector<fs::path> cam0_f_;  // camera0 image filepath
    std::vector<fs::path> cam1_f_;  // camera1 image filepath
    std::vector<cv::Mat> cam1_;     // camera0 rgb images
    std::vector<cv::Mat> cam0_;     // camera1 rgb images

    YAML::Node config_;             // yaml sensor config

    std::vector<uint64_t> imu_ts_;               // imu timestamps
    std::vector<Eigen::Vector3d> imu_gyro_;      // imu gyroscope data
    std::vector<Eigen::Vector3d> imu_acc_;       // imu accelerometer data
};



} // namespace dataset
