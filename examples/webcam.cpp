#include <thread>
#include <vector>

#include <svo/svo.h>
#include <svo/viewer/viewer.h>

template<typename T>
uint64_t get_timestamp() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<T>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
}

int main(int argc, char* argv[]) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <camera_calib_file> <svo_config_file> <camera_id>" << std::endl;
        return 1;
    }

    // set glog level
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_minloglevel = google::INFO;

    const std::string camera_calib_file = argv[1];
    const std::string svo_config_file = argv[2];
    const int camera_id = std::stoi(argv[3]);

    const auto vo = svo::Odometry(
        svo::Odometry::Type::kMono,
        camera_calib_file,
        svo_config_file
    );

    // load frames
    cv::VideoCapture cap_ = cv::VideoCapture(camera_id);
    if(!cap_.isOpened()) {
        throw std::runtime_error("Cannot open camera: " + std::to_string(camera_id));
    }

    auto svo_viewer_ = std::make_shared<svo::viewer::Viewer>(vo.frame_handler());
    std::thread viz_thread(&svo::viewer::Viewer::run, svo_viewer_);

    // loop
    vo.start();
    for(;;) {
        uint64_t ts = get_timestamp<std::chrono::nanoseconds>();
        
        cv::Mat image;
        bool res = cap_.read(image);
        if(!res || image.empty()) {
            break;
        }

        vo.addImageBundle({image}, ts);

        if(vo.stage() == svo::Stage::kPaused) {
            cv::waitKey(0);
            svo_viewer_->reset();
            vo.start();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    cv::waitKey(0);

    svo_viewer_->exit();
    viz_thread.join();

    return 0;
}