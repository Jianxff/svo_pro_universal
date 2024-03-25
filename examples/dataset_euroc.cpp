#include "dataset.hpp"

namespace dataset {
// file assertion
void _assert_filepath_(const fs::path& p) {
    if(!fs::exists(p)) {
        throw std::runtime_error("Cannot open file: " + p.string());
    }
}

// Constructor
Euroc::Euroc(const std::string& mav_dir) {
    mav_dir_ = fs::canonical(fs::absolute(mav_dir));
    _clear();
    reset();
}

Euroc Euroc::create(const std::string& mav_dir,
    const bool load_imu,
    const bool preload_image,
    const bool load_config 
) {
    Euroc euroc(mav_dir);
    euroc._read_cam_data();
    if(preload_image) {
        euroc.cam0_ = std::move(euroc.load_image(euroc.cam0_f_));
        euroc.cam1_ = std::move(euroc.load_image(euroc.cam1_f_));
    }
    if(load_imu) euroc._read_imu_data();
    if(load_config) euroc._read_yaml_config();
    return euroc;
}

void Euroc::reset() const {
    image_pt_ = 0;
    imu_pt_ = 0;
}

const Euroc::FrameData Euroc::nextFrame() const{
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

const Euroc::ImuData Euroc::nextImu() const{
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

const std::vector<Euroc::ImuData> Euroc::nextImuAll() const {
    std::vector<ImuData> data;
    const auto next_ts = cur_frame_ts(1);
    while(cur_imu_ts() < next_ts && imu_pt_ < imu_ts_.size()) {
        const auto d = nextImu();
        data.push_back(d);
    }
    return data;
}

const uint64_t Euroc::cur_frame_ts(int bias) const{
    return cam_ts_[image_pt_ + bias];
}

const uint64_t Euroc::cur_imu_ts(int bias) const{
    return imu_ts_[imu_pt_ + bias];
}

const cv::Mat Euroc::frame(const size_t idx, const int cam_id) const{
    if(preload_)
        return (cam_id == 0 ? cam0_[idx] : cam1_[idx]);
    return cv::imread((cam_id == 0 ? cam0_f_[idx] : cam1_f_[idx]).string());
}

const Eigen::Vector3d Euroc::gyro(const size_t idx) const{
    return imu_gyro_[idx];
}

const Eigen::Vector3d Euroc::acc(const size_t idx) const{
    return imu_acc_[idx];
}

const uint64_t Euroc::frame_ts(const size_t idx) const{
    return cam_ts_[idx];
}

const uint64_t Euroc::imu_ts(const size_t idx) const{
    return imu_ts_[idx];
}

const YAML::Node& Euroc::config() const{
    return config_;
}

const YAML::Node Euroc::operator[](const std::string& key) const{
    return config_[key];
}

const size_t Euroc::len_frames() const{
    return cam_ts_.size();
}

const size_t Euroc::len_imu() const{
    return imu_ts_.size();
}

void Euroc::align() const {
    if(imu_ts_.size() == 0 || cam_ts_.size() == 0) return;
    const auto first_ts = std::min(imu_ts_.front(), cam_ts_.front());
    while(imu_ts_[imu_pt_] < first_ts)
        imu_pt_++;
    while(cam_ts_[image_pt_] < first_ts)
        image_pt_++;
}

void Euroc::_clear() {
    cam_ts_.clear();
    cam0_.clear();
    cam1_.clear();
    cam0_f_.clear();
    cam1_f_.clear();
    imu_ts_.clear();
    imu_gyro_.clear();
    imu_acc_.clear();
}

void Euroc::_read_yaml_config() {
    fs::path cam0_yaml = mav_dir_ / "cam0" / "sensor.yaml";
    fs::path cam1_yaml = mav_dir_ / "cam1" / "sensor.yaml";
    fs::path imu_yaml = mav_dir_ / "imu0" / "sensor.yaml";
    // read yaml
    config_["cam0"] = YAML::LoadFile(cam0_yaml);
    config_["cam1"] = YAML::LoadFile(cam1_yaml);
    config_["imu"] = YAML::LoadFile(imu_yaml);
}

size_t Euroc::_read_cam_data() {
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

size_t Euroc::_read_imu_data() {
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

std::vector<cv::Mat> Euroc::load_image(const std::vector<fs::path>& files) {
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

} // namespace dataset
