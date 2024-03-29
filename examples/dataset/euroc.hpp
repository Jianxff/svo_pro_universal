#pragma once

#include <vector>
#include <fstream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#ifdef __READ_YAML__
#include <yaml-cpp/yaml.h>
#endif


namespace fs = std::filesystem;

namespace dataset {

// EuRoC dataset
class Euroc{
public:
    static std::vector<cv::Mat> _load_images_(const std::vector<fs::path>& files) {
        std::vector<cv::Mat> imgs;
        for(const auto& f : files) {
            cv::Mat img = cv::imread(f.string());
            if(img.empty()) {
                throw std::runtime_error("Cannot open image: " + f.string());
            }
            imgs.push_back(img);
        }
        return imgs;
    }

    static void _assert_filepath_(const fs::path& p) {
        if(!fs::exists(p)) {
            throw std::runtime_error("Cannot open file: " + p.string());
        }
    }
// basic
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
    Euroc(const std::string& mav_dir){
        mav_dir_ = fs::canonical(fs::absolute(mav_dir));
        _clear();
        reset();
    }

    // Reset pointer
    void reset() const {
        image_pt_ = 0;
        imu_pt_ = 0;
    }

    // Get next frame data
    const FrameData nextFrame() const {
        if(image_pt_ >= cam_ts_.size()) {
            return FrameData();
        }
        FrameData data {
            .ts = cam_ts_[image_pt_],
            .cam0 = frame(image_pt_, 0),
            .cam1 = frame(image_pt_, 1)
        };
        image_pt_++;
        return data;
    }

    // Get next imu data
    const ImuData nextImu() const {
        if(imu_pt_ >= imu_ts_.size()) {
            return ImuData();
        }
        ImuData data {
            .ts = imu_ts_[imu_pt_],
            .gyro = gyro(imu_pt_),
            .acc = acc(imu_pt_)
        };
        imu_pt_++;
        return data;
    }

    // Get next imu data before next frame
    const std::vector<ImuData> nextImuAll() const {
        std::vector<ImuData> data;
        const auto next_ts = cur_frame_ts(1);
        while(cur_imu_ts() < next_ts && imu_pt_ < imu_ts_.size()) {
            const auto d = nextImu();
            data.push_back(d);
        }
        return data;
    }

    // Align initial frames and imu data
    void align() const {
        if(imu_ts_.size() == 0 || cam_ts_.size() == 0) return;
        const auto first_ts = std::min(imu_ts_.front(), cam_ts_.front());
        while(imu_ts_[imu_pt_] < first_ts)
            imu_pt_++;
        while(cam_ts_[image_pt_] < first_ts)
            image_pt_++;
    }

    // static creation
    static Euroc create(
        const std::string& mav_dir,
        const bool load_imu = false,
        const bool preload_image = false,
        const bool load_config = false
    ) {
        Euroc euroc(mav_dir);
        euroc._read_cam_data();
        if(preload_image) {
            euroc.cam0_ = std::move(_load_images_(euroc.cam0_f_));
            euroc.cam1_ = std::move(_load_images_(euroc.cam1_f_));
        }
        euroc.preload_ = preload_image;
        if(load_imu) euroc._read_imu_data();
        
#ifdef __READ_YAML__
        if(load_config) euroc._read_yaml_config();
#endif        
        return euroc;
    }
    
    /////////////////////////////////////////////////////////////
    // Get current frame timestamp
    const uint64_t cur_frame_ts(int bias = 0) const { return cam_ts_[image_pt_ + bias]; }
    // Get current imu timestamp
    const uint64_t cur_imu_ts(int bias = 0) const { return imu_ts_[imu_pt_ + bias]; }
    // get camera frame
    const cv::Mat frame(const size_t idx, const int cam_id) const {
        if(preload_) return (cam_id == 0 ? cam0_[idx] : cam1_[idx]);
        return cv::imread((cam_id == 0 ? cam0_f_[idx] : cam1_f_[idx]).string());
    }
    // get imu data
    const Eigen::Vector3d gyro(const size_t idx) const{ return imu_gyro_[idx]; }
    const Eigen::Vector3d acc(const size_t idx) const { return imu_acc_[idx]; }
    // get timestamp
    const uint64_t frame_ts(const size_t idx) const { return cam_ts_[idx]; }
    const uint64_t imu_ts(const size_t idx) const { return imu_ts_[idx]; }
    // get length
    const size_t len_frames() const { return cam_ts_.size(); }
    const size_t len_imu() const { return imu_ts_.size(); }
    // get config
#ifdef __READ_YAML__
    const YAML::Node& config() const { return config_; }
    const YAML::Node operator[](const std::string& key) const { return config_[key]; }
#endif
    /////////////////////////////////////////////////////////////

private:
    // clear all data
    void _clear() {
        cam_ts_.clear();
        cam0_.clear();
        cam1_.clear();
        cam0_f_.clear();
        cam1_f_.clear();
        imu_ts_.clear();
        imu_gyro_.clear();
        imu_acc_.clear();
    }

#ifdef __READ_YAML__
    // load yaml config file
    void _read_yaml_config() {
        fs::path cam0_yaml = mav_dir_ / "cam0" / "sensor.yaml";
        fs::path cam1_yaml = mav_dir_ / "cam1" / "sensor.yaml";
        fs::path imu_yaml = mav_dir_ / "imu0" / "sensor.yaml";
        // read yaml
        config_["cam0"] = YAML::LoadFile(cam0_yaml);
        config_["cam1"] = YAML::LoadFile(cam1_yaml);
        config_["imu"] = YAML::LoadFile(imu_yaml);
    }
#endif

    // load camera frame data
    size_t _read_cam_data() {
        fs::path csv_filepath = mav_dir_ / "cam0" / "data.csv";
        fs::path cam0_dir = mav_dir_ / "cam0" / "data";
        fs::path cam1_dir = mav_dir_ / "cam1" / "data";
        
        _assert_filepath_(csv_filepath);

        std::ifstream ifs(csv_filepath);
        // read csv
        std::string s;
        while(!ifs.eof()) {
            std::getline(ifs, s);
            std::replace(s.begin(), s.end(), ',', ' ');
            if(s.empty()) continue;
            if(*s.begin() == '#') continue;
            //
            std::stringstream ss;
            ss << s;
            uint64_t ts_u64;
            ss >> ts_u64;
            cam_ts_.push_back(ts_u64);
            cam0_f_.push_back(cam0_dir / (std::to_string(ts_u64) + ".png"));
            cam1_f_.push_back(cam1_dir / (std::to_string(ts_u64) + ".png"));
        }
        return cam_ts_.size();
    }

    // load imu data
    size_t _read_imu_data() {
        fs::path imu_dir = mav_dir_ / "imu0";
        fs::path csv_filepath = imu_dir / "data.csv";
        
        _assert_filepath_(csv_filepath);

        std::ifstream ifs(csv_filepath);
        // read csv
        std::string s;
        while(!ifs.eof()) {
            std::getline(ifs, s);
            std::replace(s.begin(), s.end(), ',', ' ');
            if(s.empty()) continue;
            if(*s.begin() == '#') continue;
            //
            std::stringstream ss;
            ss << s;
            uint64_t ts_u64;
            ss >> ts_u64;
            imu_ts_.push_back(ts_u64);
            double x, y, z;
            ss >> x >> y >> z;
            imu_gyro_.push_back(Eigen::Vector3d(x, y, z));
            ss >> x >> y >> z;
            imu_acc_.push_back(Eigen::Vector3d(x, y, z));
        }
        return imu_ts_.size();
    }

    bool preload_;                  // preload all images
    fs::path mav_dir_;              // base MAV dir for EuRoC dataset

    mutable size_t image_pt_;       // frame reader pointer
    mutable size_t imu_pt_;         // imu reader pointer

    std::vector<uint64_t> cam_ts_;  // camera timestamps
    std::vector<fs::path> cam0_f_;  // camera0 image filepath
    std::vector<fs::path> cam1_f_;  // camera1 image filepath
    std::vector<cv::Mat> cam1_;     // camera0 rgb images
    std::vector<cv::Mat> cam0_;     // camera1 rgb images

#ifdef __READ_YAML__
    YAML::Node config_;             // yaml sensor config
#endif

    std::vector<uint64_t> imu_ts_;               // imu timestamps
    std::vector<Eigen::Vector3d> imu_gyro_;      // imu gyroscope data
    std::vector<Eigen::Vector3d> imu_acc_;       // imu accelerometer data
};


} // namespace dataset