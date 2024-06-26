#include <thread>
#include <vector>
#include <memory>
#include <fstream>

#include <svo/svo.h>
#include <svo/viewer/viewer.h>

#include "dataset/euroc.hpp"

int main(int argc, char* argv[]) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <svo_config_file> <euroc_mav_directory>" << std::endl;
        return 1;
    }

    // set glog level
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_minloglevel = google::INFO;

    const std::string calib_file = argv[1];
    const std::string svo_config_file = argv[2];
    const std::string euroc_mav_dir = argv[3];

    const auto vio = svo::Odometry(
        svo::Odometry::kMonoIMU,
        calib_file,
        svo_config_file
    );

    auto svo_viewer_ = std::make_shared<svo::viewer::Viewer>(vio.frame_handler());

    // load frames
    const dataset::Euroc euroc_ = dataset::Euroc::create(euroc_mav_dir, true);
    euroc_.align();

    std::thread viz_thread(&svo::viewer::Viewer::run, svo_viewer_);

    // loop
    vio.start();
    for(;;) {
        // add imu data
        const auto imu_data = euroc_.nextImuAll();
        for(const auto& imu : imu_data) {
            if(imu.valid()) {
                vio.addImuMeasurement(
                    imu.ts, imu.gyro, imu.acc
                );
            }
        }
        // add frame data
        const auto data = euroc_.nextFrame();
        if(!data.valid()) {
            break;
        }
        vio.addImageBundle({data.cam0}, data.ts);
        // check frame status
        if(vio.stage() == svo::Stage::kPaused) {
            cv::waitKey(0);
            svo_viewer_->reset();
            vio.start();
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    cv::waitKey(0);

    svo_viewer_->exit();
    viz_thread.join();

    return 0;
}