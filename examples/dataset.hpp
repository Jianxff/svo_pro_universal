#pragma once

#include <vector>
#include <fstream>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>


namespace fs = std::filesystem;

namespace dataset {
class Euroc{
public:
    struct Imu {
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;
    };

    struct CamData{
        uint64_t ts = -1;
        cv::Mat cam0;
        cv::Mat cam1;
        const bool valid() const { return ts != -1;}
    };

    struct ImuData{
        uint64_t ts;
        Imu imu;
        const bool valid() const { return ts != -1; }
    };

    Euroc(const std::string& mav_dir) {
        mav_dir_ = fs::canonical(fs::absolute(mav_dir));
        _clear();
        reset();
    }

    void load_image(const bool preload = false){
        _read_cam_data();
        if(preload) {
            preload_ = true;
            cam0 = std::move(load_image(cam0_f));
            cam1 = std::move(load_image(cam1_f));
        }
        if(cam_ts.size() != cam0_f.size() || cam_ts.size() != cam1_f.size()) {
            throw std::runtime_error("length of images and timestamps are not equal");
        }
    }
    
    void load_config() {
        _read_yaml_config();
    }

    void load_imu() {
        _read_imu_data();
    }

    void reset() const {
        image_pt_ = 0;
        imu_pt_ = 0;
    }

    const CamData next_cam() const{
        if(image_pt_ >= cam_ts.size()) {
            return CamData();
        }
        CamData data;
        data.ts = cam_ts[image_pt_];
        data.cam0 = get_cam(image_pt_, 0);
        data.cam1 = get_cam(image_pt_, 1);
        image_pt_++;
        return data;
    }

    const ImuData next_imu() const{
        if(imu_pt_ >= imu_ts.size()) {
            return ImuData();
        }
        ImuData data;
        data.ts = imu_ts[imu_pt_];
        data.imu = get_imu(imu_pt_);
        imu_pt_++;
        return data;
    }

    const std::vector<ImuData> next_imu_all() const {
        std::vector<ImuData> data;
        const auto next_ts = cur_cam_ts(1);
        while(cur_imu_ts() < next_ts && imu_pt_ < imu_ts.size()) {
            const auto d = next_imu();
            data.push_back(d);
        }
        return data;
    }

    const uint64_t cur_cam_ts(int bias = 0) const{
        return cam_ts[image_pt_ + bias];
    }

    const uint64_t cur_imu_ts(int bias = 0) const{
        return imu_ts[imu_pt_ + bias];
    }

    void back_cam(const size_t n = 1) const{
        image_pt_ = (image_pt_ < n ? 0 : image_pt_ - n);
    }

    void back_imu(const size_t n = 1) const{
        imu_pt_ = (imu_pt_ < n ? 0 : imu_pt_ - n);
    }
    
    const cv::Mat get_cam(const size_t idx, const int cam_id) const{
        if(preload_)
            return (cam_id == 0 ? cam0[idx] : cam1[idx]);
        return cv::imread((cam_id == 0 ? cam0_f[idx] : cam1_f[idx]).string());
    }

    const Imu& get_imu(const size_t idx) const{
        return imu0[idx];
    }

    const uint64_t get_cam_ts(const size_t idx) const{
        return cam_ts[idx];
    }

    const uint64_t get_imu_ts(const size_t idx) const{
        return imu_ts[idx];
    }

    const YAML::Node& get_config() const{
        return config_;
    }

    const YAML::Node operator[](const std::string& key) const{
        return config_[key];
    }

    const size_t size_cam() const{
        return cam_ts.size();
    }

    const size_t size_imu() const{
        return imu_ts.size();
    }

    void align() const {
        if(imu_ts.size() == 0 || cam_ts.size() == 0) return;
        const auto first_ts = std::min(imu_ts.front(), cam_ts.front());
        while(imu_ts[imu_pt_] < first_ts)
            imu_pt_++;
        while(cam_ts[image_pt_] < first_ts)
            image_pt_++;
    }

    static Euroc create(const std::string& mav_dir,
        const bool load_imu = false,
        const bool preload_image = false,
        const bool load_config = false
    ) {
        Euroc euroc(mav_dir);
        euroc.load_image(preload_image);
        if(load_imu) euroc.load_imu();
        if(load_config) euroc.load_config();
        return euroc;
    }


private:
    void _clear() {
        cam_ts.clear();
        cam0.clear();
        cam1.clear();
        imu_ts.clear();
        imu0.clear();
    }

    void _assert_filepath(const fs::path& p) {
        if(!fs::exists(p)) {
            throw std::runtime_error("Cannot open file: " + p.string());
        }
    }

    void _read_yaml_config() {
        fs::path cam0_yaml = mav_dir_ / "cam0" / "sensor.yaml";
        fs::path cam1_yaml = mav_dir_ / "cam1" / "sensor.yaml";
        fs::path imu_yaml = mav_dir_ / "imu0" / "sensor.yaml";
        // read yaml
        config_["cam0"] = YAML::LoadFile(cam0_yaml);
        config_["cam1"] = YAML::LoadFile(cam1_yaml);
        config_["imu"] = YAML::LoadFile(imu_yaml);
    }

    size_t _read_cam_data() {
        fs::path csv_filepath = mav_dir_ / "cam0" / "data.csv";
        fs::path cam0_dir = mav_dir_ / "cam0" / "data";
        fs::path cam1_dir = mav_dir_ / "cam1" / "data";
        
        _assert_filepath(csv_filepath);

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
            cam_ts.push_back(ts_u64);
            cam0_f.push_back(cam0_dir / (std::to_string(ts_u64) + ".png"));
            cam1_f.push_back(cam1_dir / (std::to_string(ts_u64) + ".png"));
        }

        return cam_ts.size();
    }

    size_t _read_imu_data() {
        fs::path imu_dir = mav_dir_ / "imu0";
        fs::path csv_filepath = imu_dir / "data.csv";
        
        _assert_filepath(csv_filepath);

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
            imu_ts.push_back(ts_u64);
            Imu imu;
            double x, y, z;
            ss >> x >> y >> z;
            imu.gyro = Eigen::Vector3d(x, y, z);
            ss >> x >> y >> z;
            imu.acc = Eigen::Vector3d(x, y, z);
            imu0.push_back(imu);
        }

        return imu_ts.size();
    }
    
    std::vector<cv::Mat> load_image(const std::vector<fs::path>& files) {
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


    bool preload_;
    fs::path mav_dir_;

    mutable size_t image_pt_;
    mutable size_t imu_pt_;

    std::vector<uint64_t> cam_ts;
    std::vector<fs::path> cam0_f;
    std::vector<fs::path> cam1_f;
    std::vector<cv::Mat> cam1;
    std::vector<cv::Mat> cam0;

    YAML::Node config_;

    std::vector<uint64_t> imu_ts;
    std::vector<Imu> imu0;
};
} // namespace dataset
