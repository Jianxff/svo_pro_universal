#pragma once

#include <vector>
#include <fstream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace fs = std::filesystem;

namespace dataset {

// Video Dataset
class Video {
public:
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

    Video(const std::string& video_path) {
        video_path_ = fs::canonical(fs::absolute(video_path));
        _assert_filepath_(video_path_);
        //
        cap_ = cv::VideoCapture(video_path_.string());
        if(!cap_.isOpened()) {
            throw std::runtime_error("Cannot open video: " + video_path_.string());
        }
        // settings
        fps_ = cap_.get(cv::CAP_PROP_FPS);
        width_ = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        height_ = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        // 
        timestamp_ = 1;
    }

    // Get next frame data
    const FrameData nextFrame() const{
        if(!cap_.isOpened()) {
            return FrameData();
        }

        cv::Mat frame;
        bool res = cap_.read(frame);
        if(!res || frame.empty()) {
            return FrameData();
        }

        FrameData data {
            .ts = timestamp_,
            .cam = frame
        };
        timestamp_ += 1e9 / fps_;
        return data;
    }

    void reset() const{
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        timestamp_ = 1;
    }

    const size_t len_frames() const {
        return cap_.get(cv::CAP_PROP_FRAME_COUNT);
    }

    static Video create(const std::string& video_path) {
        Video video(video_path);
        return video;
    }


protected:
    fs::path video_path_;
    mutable cv::VideoCapture cap_;
    mutable uint64_t timestamp_;
    uint32_t fps_;
    uint32_t width_;
    uint32_t height_;
    

};




} // namespace dataset