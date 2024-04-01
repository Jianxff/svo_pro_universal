#include <thread>
#include <vector>

#include <svo/svo.h>
#include <svo/viewer/viewer.h>

#include "dataset/video.hpp"

int main(int argc, char* argv[]) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <camera_calib_file> <svo_config_file> <video_filepath>" << std::endl;
        return 1;
    }

    // set glog level
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_minloglevel = google::INFO;

    const std::string camera_calib_file = argv[1];
    const std::string svo_config_file = argv[2];
    const std::string video_filepath = argv[3];

    const auto vo = svo::Odometry(
        svo::Odometry::Type::kMono,
        camera_calib_file,
        svo_config_file
    );

    // load frames
    const dataset::Video vid_ = dataset::Video::create(video_filepath);

    auto svo_viewer_ = std::make_shared<svo::viewer::Viewer>(vo.frame_handler());
    std::thread viz_thread(&svo::viewer::Viewer::run, svo_viewer_);

    // loop
    vo.start();
    for(;;) {
        const auto data = vid_.nextFrame();
        if(!data.valid()) {
            break;
        }
        cv::imshow("inputframe", data.cam);
        vo.addImageBundle({data.cam}, data.ts);

        if(vo.stage() == svo::Stage::kPaused) {
            cv::waitKey(0);
            svo_viewer_->reset();
            vo.start();
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
        cv::waitKey(5);
    }

    cv::waitKey(0);

    svo_viewer_->exit();
    viz_thread.join();

    return 0;
}