#pragma once

#include <memory>
#include <vector>
#include <atomic>

#include <svo/svo.h>

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace svo {
namespace viewer{

class Viewer {
public:
    Viewer(const std::shared_ptr<FrameHandlerBase>& vo);

    void run();
    void exit();

    void update_vo();

    void drawCamera();
    void drawMapRegion();
    void drawTrajectory();

private:
    void update_vo_tracking_();
    void update_vo_initializing_();
    void update_frame_();

    std::shared_ptr<FrameHandlerBase> pvo_;
    pangolin::OpenGlMatrix transform_Twc_;

    std::vector<Eigen::Vector3d> traj_cam_;
    std::vector<Eigen::Vector3d> traj_imu_;
    std::vector<Eigen::Vector3d> region_lm_;
    // std::shared_ptr<std::vector<

    std::atomic<bool> sig_exit_ = false;

};

} // namespace viewer
} // namespace svo