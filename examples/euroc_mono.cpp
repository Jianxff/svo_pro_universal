#include <thread>
#include <vector>
#include <memory>
#include <fstream>

#include <svo/svo.h>
#include <svo/viewer/viewer.h>

#include "dataset.hpp"



// void load_image(const std::string &img_foler, const std::string &timestamp_file,
//     std::vector<std::string>& images, std::vector<uint64_t>& times)
// {
//     std::ifstream in;
//     in.open(timestamp_file.c_str());
//     if(!in.is_open()) {
//         std::cerr << "Failed to open timestamp file: " << timestamp_file << std::endl;
//         return;
//     }

//     images.reserve(5000);
//     times.reserve(5000);

//     while(!in.eof()) {
//         std::string s;
//         std::getline(in, s);
//         if(!s.empty()) {
//             std::stringstream ss;
//             ss << s;
//             std::string image_name = img_foler + "/" + ss.str() + ".png";
//             images.push_back(image_name);
//             uint64_t t;
//             ss >> t;
//             times.push_back(t);
//         }
//     }
// }

int main(int argc, char* argv[]) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <camera_calib_file> <svo_config_file> <euroc_mav_directory>" << std::endl;
        return 1;
    }

    // set glog level
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_minloglevel = google::INFO;

    const std::string camera_calib_file = argv[1];
    const std::string svo_config_file = argv[2];
    const std::string euroc_mav_dir = argv[3];

    auto camera = svo::factory::makeCamera(camera_calib_file);
    auto svo_ = svo::factory::makeMono(camera, svo_config_file);
    const bool auto_reinitialize = true;

    auto svo_viewer_ = std::make_shared<svo::viewer::Viewer>(svo_);

    // load frames
    const dataset::Euroc euroc_ = dataset::Euroc::create(euroc_mav_dir);

    std::thread viz_thread(&svo::viewer::Viewer::run, svo_viewer_);

    // loop
    svo_->start();
    for(;;) {
        const auto data = euroc_.next_cam();
        if(!data.valid()) {
            break;
        }
        svo_->addImageBundle({data.cam0}, data.ts);
        if(svo_->stage() == svo::Stage::kPaused && auto_reinitialize) {
            cv::waitKey(0);
            svo_->start();
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    cv::waitKey(0);

    svo_viewer_->exit();
    viz_thread.join();

    return 0;
}