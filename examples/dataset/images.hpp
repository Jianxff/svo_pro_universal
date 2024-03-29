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

// Image Sequence Dataset
class Images{
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
    struct FrameData{
        const uint64_t ts = -1;
        const cv::Mat cam;
        const bool valid() const { return ts != -1;}
    };

    Images(const std::string& image_dir, const int fps) {
        image_dir_ = fs::canonical(fs::absolute(image_dir));
        fps_ = fps;
        _clear();
    }

    // Get next frame data
    const FrameData nextFrame() const {
        if(image_pt_ >= cam_ts_.size()) {
            return FrameData();
        }
        FrameData data {
            .ts = cam_ts_[image_pt_],
            .cam = frame(image_pt_)
        };
        image_pt_++;
        return data;
    }

    // Reset pointer
    void reset() const {
        image_pt_ = 0;
    }

    /////////////////////////////////////////////////////////////
    // Get current frame timestamp
    const uint64_t cur_frame_ts(int bias = 0) const { return cam_ts_[image_pt_ + bias]; }
    // get camera frame
    const cv::Mat frame(const size_t idx) const {
        if(preload_) return cam_[idx];
        return cv::imread(cam_f_[idx].string());
    }
    // get timestamp
    const uint64_t frame_ts(const size_t idx) const { return cam_ts_[idx]; }
    // get length
    const size_t len_frames() const { return cam_ts_.size(); }
    /////////////////////////////////////////////////////////////

    static Images create(
        const std::string& img_dir, 
        const int fps,
        const bool preload_images = false
    ) {
        Images images(img_dir, fps);
        images._read_cam_data();
        if(preload_images) {
            images.cam_ = std::move(
                _load_images_(images.cam_f_)
            );
        }
        images.preload_ = preload_images;
        return images;
    }

protected:
    // clear all data
    void _clear() {
        cam_f_.clear();
        cam_ts_.clear();
        cam_.clear();
        image_pt_ = 0;  
    }

    // load camera frame data
    size_t _read_cam_data() {
        fs::path cam_dir = image_dir_;
        size_t timestamp = 1;
        const size_t ts_step = 1e9 / fps_;
        for(const auto& entry : fs::directory_iterator(cam_dir)) {
            if(entry.is_regular_file()) {
                cam_f_.push_back(entry.path());
                cam_ts_.push_back(timestamp);
                timestamp += ts_step;
            }
        }
        std::sort(cam_f_.begin(), cam_f_.end());
        return cam_f_.size();
    }

    bool preload_;                  // preload all images
    fs::path image_dir_;            // base images dir
    int fps_;                       // fps

    mutable size_t image_pt_;       // frame reader pointer

    std::vector<fs::path> cam_f_;   // camera0 image filepath
    std::vector<uint64_t> cam_ts_;  // camera timestamps
    std::vector<cv::Mat> cam_;      // camera image

};



} // namespace dataset