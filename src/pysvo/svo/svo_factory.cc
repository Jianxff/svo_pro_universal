#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <svo/svo_factory.h>

namespace py = pybind11;

void bind_svo_factory(py::module &m) {
    py::enum_<svo::Odometry::Type>(m, "OdometryType")
        .value("Mono", svo::Odometry::Type::kMono)
        .value("Stereo", svo::Odometry::Type::kStereo)
        .value("MonoIMU", svo::Odometry::Type::kMonoIMU)
        .value("StereoIMU", svo::Odometry::Type::kStereoIMU);
    
    py::enum_<svo::Stage>(m, "Stage")
        .value("Initializing", svo::Stage::kInitializing)
        .value("Paused", svo::Stage::kPaused)
        .value("Tracking", svo::Stage::kTracking)
        .value("Relocalization", svo::Stage::kRelocalization);

    py::class_<svo::Odometry, std::shared_ptr<svo::Odometry>> PyOdometry(m, "Odometry");
    PyOdometry
        .def(py::init<svo::Odometry::Type, const std::string&, const std::string&>(),
            py::arg("type"),
            py::arg("calib_file"),
            py::arg("config_file")
        )
        .def_property_readonly("stage", &svo::Odometry::stage)
        .def_property_readonly("transform_world_cam", &svo::Odometry::transform_world_cam)
        .def_property_readonly("rotation_world_cam", &svo::Odometry::rotation_world_cam)
        .def_property_readonly("translation_world_cam", &svo::Odometry::translation_world_cam)
        .def("start", &svo::Odometry::start)
        .def("addImageBundle", [](
                svo::Odometry &self, 
                const py::array_t<uint8_t> &image_bundle, 
                const uint64_t timestamp
            ) {
                py::buffer_info buf = image_bundle.request();
                if (buf.ndim != 4) {
                    throw std::runtime_error("Number of dimensions must be 3");
                }
                // to vector<Mat>
                std::vector<cv::Mat> imgs;
            })

}



