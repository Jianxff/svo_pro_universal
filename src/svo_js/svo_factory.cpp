#include <map>

#include "svo_factory.h"


using namespace svo;

const Eigen::Matrix4d CONVERT_MAT44D_FLIP_YZ = 
  (Eigen::Matrix4d() << 
    1, 0, 0, 0,
    0, -1, 0, 0,
    0, 0, -1, 0,
    0, 0, 0, 1).finished();


template<typename T>
Eigen::MatrixX<T> convert_emarray_matrix(const emscripten::val array, const int rows, const int cols) {
    Eigen::MatrixX<T> matrix(rows, cols);
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            matrix(i, j) = array[i][j].as<T>();
        }
    }
    return matrix;
}

template<typename T>
Eigen::VectorX<T> convert_emarray_vector(const emscripten::val array, const int size) {
    Eigen::VectorX<T> vector(size);
    for(int i = 0; i < size; i++) {
        vector(i) = array[i].as<T>();
    }
    return vector;
}

template<typename T>
emscripten::val convert_eigen_emarray(const Eigen::MatrixX<T> matrix) {
    emscripten::val array = emscripten::val::array();
    for(int i = 0; i < matrix.rows(); i++) {
        emscripten::val row = emscripten::val::array();
        for(int j = 0; j < matrix.cols(); j++)
            row.set(j, matrix(i, j));
        array.set(i, row);
    }
    return array;
}

template<typename T>
T read_val(const emscripten::val value, const T default_val = T()) {
    if(value == emscripten::val::undefined()) return default_val;
    return value.as<T>();
}

std::map<svo::Stage, int> stage2int = {
    {svo::Stage::kTracking, 0},
    {svo::Stage::kInitializing, 1},
    {svo::Stage::kPaused, 2},
    {svo::Stage::kRelocalization, 3}
};

void setInitialPose(FrameHandlerBase& vo) {
    Transformation T_world_imuinit(
        Eigen::Quaternion(1.0,0.0,0.0,0.0),
        Eigen::Vector3d(0.0,0.0,0.0)
    );
    vo.setInitialImuPose(T_world_imuinit);
}

std::shared_ptr<CameraBundle> makeCamera(
    const emscripten::val calib
) {
    // Cameras
    const uint32_t imwidth = read_val<uint32_t>(calib["width"]);
    const uint32_t imheight = read_val<uint32_t>(calib["height"]);
    Eigen::Vector4d intrinsics;
    if(calib["intrinsics"] == emscripten::val::undefined()) {
        const double focal = std::max(imwidth, imheight) * 1.2;
        const double cx = (double)imwidth / 2.0;
        const double cy = (double)imheight / 2.0;
        intrinsics << focal, focal, cx, cy;
    } else {
        intrinsics = convert_emarray_vector<double>(calib["intrinsics"], 4);
    }
    // SVO_INFO_STREAM("Using default intrinsics: " << intrinsics.transpose());
    CameraPtr camera = vk::cameras::factory::makePinholeCamera(
        intrinsics, imwidth, imheight
    );

    Eigen::Matrix4d T_B_C_raw = Eigen::Matrix4d::Identity();
    if(calib["T_B_C"] != emscripten::val::undefined()) 
        T_B_C_raw = convert_emarray_matrix<double>(calib["T_B_C"], 4, 4);
    vk::cameras::Quaternion q_B_C = vk::cameras::Quaternion(
          static_cast<Eigen::Matrix3d>(T_B_C_raw.block<3,3>(0,0)));
    vk::cameras::Transformation T_B_C(q_B_C, T_B_C_raw.block<3,1>(0,3));

    const vk::cameras::TransformationVector T_C_B_vec = {T_B_C.inverse()};
    const std::vector<Camera::Ptr> cameras = {camera};

    CameraBundle::Ptr ncam = std::make_shared<CameraBundle>(
        T_C_B_vec, cameras, "camera_bundle"
    );

    return ncam;
}

std::shared_ptr<FrameHandlerMono> makeMono(
    const CameraBundlePtr& ncam,
    const emscripten::val config)
{
    if (ncam->numCameras() > 1)
        ncam->keepFirstNCams(1);

    FrameHandlerMono::Ptr vo = 
        std::make_shared<FrameHandlerMono>(
            ncam, 
            loadBaseOptions(config),
            loadDepthFilterOptions(config),
            loadDetectorOptions(config),
            loadInitializationOptions(config),
            loadReprojectorOptions(config),
            loadFeatureTrackerOptions(config)
        );

    // Get initial position and orientation of IMU
    setInitialPose(*vo);
    return vo;
}


std::shared_ptr<ImuHandler> makeIMU(
    const emscripten::val imu_calib,
    const emscripten::val config
) {
    ImuHandler::Ptr imu = std::make_shared<ImuHandler>(
        loadImuCalibration(imu_calib),
        loadImuInitialization(imu_calib),
        loadIMUHandlerOptions(config)
    );
    return imu;
}

ImuCalibration loadImuCalibration(const emscripten::val data){
    ImuCalibration calib;
    calib.delay_imu_cam = read_val<double>(data["imu_params"]["delay_imu_cam"]);
    calib.max_imu_delta_t = read_val<double>(data["imu_params"]["max_imu_delta_t"]);
    calib.saturation_accel_max = read_val<double>(data["imu_params"]["acc_max"]);
    calib.saturation_omega_max = read_val<double>(data["imu_params"]["omega_max"]);
    calib.gyro_noise_density = read_val<double>(data["imu_params"]["sigma_omega_c"]);
    calib.acc_noise_density = read_val<double>(data["imu_params"]["sigma_acc_c"]);
    calib.gyro_bias_random_walk_sigma = read_val<double>(data["imu_params"]["sigma_omega_bias_c"]);
    calib.acc_bias_random_walk_sigma = read_val<double>(data["imu_params"]["sigma_acc_bias_c"]);
    calib.gravity_magnitude = read_val<double>(data["imu_params"]["g"]);
    calib.imu_rate = read_val<double>(data["imu_params"]["imu_rate"]);
    return calib;
}


ImuInitialization loadImuInitialization(const emscripten::val data){
    ImuInitialization init;
    init.velocity = Eigen::Vector3d(
        read_val<double>(data["imu_initialization"]["velocity"][0]),
        read_val<double>(data["imu_initialization"]["velocity"][1]),
        read_val<double>(data["imu_initialization"]["velocity"][2])
    );
    init.omega_bias = Eigen::Vector3d(
        read_val<double>(data["imu_initialization"]["omega_bias"][0]),
        read_val<double>(data["imu_initialization"]["omega_bias"][1]),
        read_val<double>(data["imu_initialization"]["omega_bias"][2])
    );
    init.acc_bias = Eigen::Vector3d(
        read_val<double>(data["imu_initialization"]["acc_bias"][0]),
        read_val<double>(data["imu_initialization"]["acc_bias"][1]),
        read_val<double>(data["imu_initialization"]["acc_bias"][2])
    );
    init.velocity_sigma = read_val<double>(data["imu_initialization"]["velocity_sigma"]);
    init.omega_bias_sigma = read_val<double>(data["imu_initialization"]["omega_bias_sigma"]);
    init.acc_bias_sigma = read_val<double>(data["imu_initialization"]["acc_bias_sigma"]);
    return init;
}


BaseOptions loadBaseOptions(const emscripten::val node) {
    BaseOptions o;
    o.max_n_kfs = (size_t)read_val<int>(node["max_n_kfs"], 5);
    o.use_imu = read_val<bool>(node["use_imu"], false);
    o.quality_min_fts = read_val<int>(node["quality_min_fts"], 50);
    o.quality_max_fts_drop = read_val<int>(node["quality_max_drop_fts"], 40);
    o.relocalization_max_trials = read_val<int>(node["relocalization_max_trials"], 50);
    o.poseoptim_prior_lambda = read_val<double>(node["poseoptim_prior_lambda"], 0.0);
    o.poseoptim_using_unit_sphere = read_val<bool>(node["poseoptim_using_unit_sphere"], false);
    o.img_align_prior_lambda_rot = read_val<double>(node["img_align_prior_lambda_rot"], 0.0);
    o.img_align_prior_lambda_trans = read_val<double>(node["img_align_prior_lambda_trans"], 0.0);
    o.structure_optimization_max_pts = read_val<int>(node["structure_optimization_max_pts"], 20);
    o.init_map_scale = read_val<double>(node["map_scale"], 1.0);
    const std::string kfselect_criterion = read_val<std::string>(node["kfselect_criterion"], "FORWARD");
    if(kfselect_criterion == "FORWARD")
        o.kfselect_criterion = KeyframeCriterion::FORWARD;
    else
        o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;
    o.kfselect_min_dist = read_val<double>(node["kfselect_min_dist"], 0.12);
    o.kfselect_numkfs_upper_thresh = read_val<int>(node["kfselect_numkfs_upper_thresh"], 120);
    o.kfselect_numkfs_lower_thresh = read_val<int>(node["kfselect_numkfs_lower_thresh"], 70);
    o.kfselect_min_dist_metric = read_val<double>(node["kfselect_min_dist_metric"], 0.01);
    o.kfselect_min_angle = read_val<double>(node["kfselect_min_angle"], 20.0);
    o.kfselect_min_disparity = read_val<double>(node["kfselect_min_disparity"], 40.0);
    o.kfselect_min_num_frames_between_kfs = read_val<int>(node["kfselect_min_num_frames_between_kfs"], 2);
    o.kfselect_backend_max_time_sec = read_val<double>(node["kfselect_backend_max_time_sec"], 3.0);
    o.img_align_max_level = read_val<int>(node["img_align_max_level"], 4);
    o.img_align_min_level = read_val<int>(node["img_align_min_level"], 2);
    o.img_align_robustification = read_val<bool>(node["img_align_robustification"], false);
    o.img_align_use_distortion_jacobian =
        read_val<bool>(node["img_align_use_distortion_jacobian"], false);
    o.img_align_est_illumination_gain =
        read_val<bool>(node["img_align_est_illumination_gain"], false);
    o.img_align_est_illumination_offset =
        read_val<bool>(node["img_align_est_illumination_offset"], false);
    o.poseoptim_thresh = read_val<double>(node["poseoptim_thresh"], 2.0);
    o.update_seeds_with_old_keyframes =
        read_val<bool>(node["update_seeds_with_old_keyframes"], true);
    o.use_async_reprojectors = read_val<bool>(node["use_async_reprojectors"], false);
    // o.trace_statistics = read_val<bool>(node["trace_statistics"], false);
    o.backend_scale_stable_thresh =
        read_val<double>(node["backend_scale_stable_thresh"], 0.02);
    o.global_map_lc_timeout_sec_ =
        read_val<double>(node["global_map_timeout_sec"], 2.0);
    return o;
}

IMUHandlerOptions loadIMUHandlerOptions(const emscripten::val node) {
    IMUHandlerOptions o;

    o.temporal_stationary_check = read_val<bool>(node["temporal_stationary_check"], false);
    o.temporal_window_length_sec_ = read_val<double>(node["temporal_window_length_sec"], 0.5);
    o.stationary_acc_sigma_thresh_ = read_val<double>(node["stationary_acc_sigma_thresh"], 10e-4);
    o.stationary_gyr_sigma_thresh_ = read_val<double>(node["stationary_gyr_sigma_thresh"], 6e-5);

    return o;
}

InitializationOptions loadInitializationOptions(const emscripten::val node) {
    InitializationOptions o;

    o.init_min_features = read_val<int>(node["init_min_features"], 100);
    o.init_min_tracked = read_val<int>(node["init_min_tracked"], 80);
    o.init_min_inliers = read_val<int>(node["init_min_inliers"], 70);
    o.init_min_disparity = read_val<double>(node["init_min_disparity"], 40.0);
    o.init_min_features_factor = read_val<double>(node["init_min_features_factor"], 2.0);
    o.reproj_error_thresh = read_val<double>(node["reproj_err_thresh"], 2.0);
    o.init_disparity_pivot_ratio = read_val<double>(node["init_disparity_pivot_ratio"], 0.5);
    const std::string init_method = read_val<std::string>(node["init_method"], "FivePoint");
    if(init_method == "Homography")
        o.init_type = InitializerType::kHomography;
    else if(init_method == "TwoPoint")
        o.init_type = InitializerType::kTwoPoint;
    else if(init_method == "FivePoint")
        o.init_type = InitializerType::kFivePoint;
    else if(init_method == "OneShot")
        o.init_type = InitializerType::kOneShot;
    else
        SVO_ERROR_STREAM("Initialization Method not supported: " << init_method);

    return o;
}

ReprojectorOptions loadReprojectorOptions(const emscripten::val node) {
    ReprojectorOptions o;

    o.max_n_kfs = read_val<int>(node["reprojector_max_n_kfs"], 5);
    o.max_n_features_per_frame = read_val<int>(node["max_fts"], 160);
    o.cell_size = read_val<int>(node["grid_size"], 35);
    o.reproject_unconverged_seeds =
        read_val<bool>(node["reproject_unconverged_seeds"], true);
    o.max_unconverged_seeds_ratio =
        read_val<double>(node["max_unconverged_seeds_ratio"], -1.0);
    o.min_required_features =
        read_val<int>(node["quality_min_fts"], 50);
    o.seed_sigma2_thresh =
        read_val<double>(node["seed_convergence_sigma2_thresh"], 200.0);

    o.affine_est_offset =
        read_val<bool>(node["reprojector_affine_est_offset"], true);
    o.affine_est_gain =
        read_val<bool>(node["reprojector_affine_est_gain"], false);
    o.max_fixed_landmarks =
        read_val<int>(node["reprojector_max_fixed_landmarks"], 50);
    o.max_n_global_kfs =
        read_val<int>(node["reprojector_max_n_global_kfs"], 20);
    o.use_kfs_from_global_map =
        read_val<bool>(node["reprojector_use_kfs_from_global_map"], false);
    o.fixed_lm_grid_size =
        read_val<int>(node["reprojector_fixed_lm_grid_size"], 50);

    return o;
}

StereoTriangulationOptions loadStereoTriangulationOptions(const emscripten::val node) {
    StereoTriangulationOptions o;

    o.triangulate_n_features = read_val<int>(node["max_fts"], 120);
    o.max_depth_inv = read_val<double>(node["max_depth_inv"], 1.0/50.0);
    o.min_depth_inv = read_val<double>(node["min_depth_inv"], 1.0/0.5);
    o.mean_depth_inv = read_val<double>(node["mean_depth_inv"], 1.0/2.0);

    return o;
}

DepthFilterOptions loadDepthFilterOptions(const emscripten::val node) {
    DepthFilterOptions o;

    o.max_search_level = read_val<int>(node["n_pyr_levels"], 3) - 1;
    o.use_threaded_depthfilter =
        read_val<bool>(node["use_threaded_depthfilter"], true);
    o.seed_convergence_sigma2_thresh =
        read_val<double>(node["seed_convergence_sigma2_thresh"], 200.0);
    o.mappoint_convergence_sigma2_thresh =
        read_val<double>(node["mappoint_convergence_sigma2_thresh"], 500.0);
    o.scan_epi_unit_sphere = read_val<bool>(node["scan_epi_unit_sphere"], false);
    o.affine_est_offset= read_val<bool>(node["depth_filter_affine_est_offset"], true);
    o.affine_est_gain = read_val<bool>(node["depth_filter_affine_est_gain"], false);
    o.max_n_seeds_per_frame = static_cast<size_t>(
            static_cast<double>(read_val<int>(node["max_fts"], 120))
            * read_val<double>(node["max_seeds_ratio"], 3.0));
    o.max_map_seeds_per_frame = static_cast<size_t>(
            static_cast<double>(read_val<int>(node["max_map_fts"], 120)));
    o.extra_map_points =
        read_val<bool>(node["depth_filter_extra_map_points"], false);
    if(read_val<bool>(node["runlc"], false) && !o.extra_map_points)
    {
        LOG(WARNING) << "Loop closure requires extra map points, "
                    << " but the option is not set, overriding to true.";
        o.extra_map_points = true;
    }

    return o;
}


DetectorOptions loadDetectorOptions(const emscripten::val node) {
    DetectorOptions o;
    o.cell_size = read_val<int>(node["grid_size"], 35);
    o.max_level = read_val<int>(node["n_pyr_levels"], 3) - 1;
    o.threshold_primary = read_val<int>(node["detector_threshold_primary"], 10);
    o.threshold_secondary = read_val<int>(node["detector_threshold_secondary"], 200);
    o.threshold_shitomasi = read_val<int>(node["detector_threshold_shitomasi"], 100);
    if(read_val<bool>(node["use_edgelets"], true))
        o.detector_type = DetectorType::kFastGrad;
    else
        o.detector_type = DetectorType::kFast;
    return o;
}

FeatureTrackerOptions loadFeatureTrackerOptions(const emscripten::val node) {
    FeatureTrackerOptions o;

    o.klt_max_level = read_val<int>(node["klt_max_level"], 4);
    o.klt_min_level = read_val<int>(node["klt_min_level"], 0);

    return o;
}

FrameRGBA::FrameRGBA(int width, int height) {
    mat_ = cv::Mat(height, width, CV_8UC4);
}

const cv::Mat& FrameRGBA::mat() const {
    return mat_;
}

const emscripten::val FrameRGBA::matData() const{
    return emscripten::val(emscripten::memory_view<unsigned char>((mat_.total()*mat_.elemSize())/sizeof(unsigned char),
                            (unsigned char*)mat_.data));
}

Odometry::Odometry(
    const emscripten::val calib,
    const emscripten::val config  
) {
    imwidth_ = read_val<uint32_t>(calib["width"]);
    imheight_ = read_val<uint32_t>(calib["height"]);
    auto_reset_ = read_val<bool>(config["auto_reset"], false);

    // SVO_INFO_STREAM("Initializing SVO with " << imwidth_ << "x" << imheight_ << " resolution.");

    const bool use_imu = read_val<bool>(calib["use_imu"], false);

    auto camera = makeCamera(calib);
    frame_handler_ = makeMono(camera, config);
    if(use_imu) {
        imu_handler_ = makeIMU(calib, config);
        frame_handler_->imu_handler_ = imu_handler_;
    }
}


const Eigen::Matrix4d Odometry::transform_world_cam() const{
    Eigen::Matrix4d T_W_C = Eigen::Matrix4d::Identity();
    auto last_frame = frame_handler_->getLastFrames();
    if(last_frame && last_frame->size() > 0) {
        T_W_C = last_frame->at(0)->T_world_cam().getTransformationMatrix();
    }
    return T_W_C;
}

emscripten::val Odometry::world_pose_gl() const{
    Eigen::Matrix4d T_W_C = transform_world_cam();
    T_W_C = CONVERT_MAT44D_FLIP_YZ * T_W_C;
    emscripten::val matrix = convert_eigen_emarray<double>(T_W_C);
    return matrix;
}

emscripten::val Odometry::world_viewpose_gl() const{
    Eigen::Matrix4d T_W_C = transform_world_cam();
    T_W_C = CONVERT_MAT44D_FLIP_YZ * T_W_C * CONVERT_MAT44D_FLIP_YZ;
    emscripten::val matrix = convert_eigen_emarray<double>(T_W_C);
    return matrix;
}

void Odometry::start() const {
    frame_handler_->start();
}

void Odometry::reset() const {
    frame_handler_->resetAll();
    start();
}

const int Odometry::stage() const {
    return stage2int[frame_handler_->stage()];
}

const uint32_t Odometry::imwidth() const {
    return imwidth_;
}

const uint32_t Odometry::imheight() const {
    return imheight_;
}

bool Odometry::_set_imu_prior(const uint64_t timestamp) const{
    if(imu_handler_ == nullptr) return true;
    auto vo = frame_handler_;
    ///
    if(!vo->hasStarted() && set_initial_attitude_from_gravity_) {
        Quaternion R_imu_world;
        if(imu_handler_->getInitialAttitude(
            (double)timestamp * common::conversions::kNanoSecondsToSeconds,
            R_imu_world
        )) {
            // VLOG(3) << "Set initial orientation from accelerometer measurements.";
            vo->setRotationPrior(R_imu_world);
        } else {
            return false;
        }
    } else if(vo->getLastFrames()) {
        // set incremental rotation prior
        Quaternion R_lastimu_newimu;
        if(imu_handler_->getRelativeRotationPrior(
            vo->getLastFrames()->getMinTimestampNanoseconds() * common::conversions::kNanoSecondsToSeconds,
            timestamp * common::conversions::kNanoSecondsToSeconds,
            false, R_lastimu_newimu
        )) {
        // VLOG(3) << "Set incremental rotation prior from IMU.";
        vo->setRotationIncrementPrior(R_lastimu_newimu);
        }
    }
    return true;
}

void Odometry::addImageBundle(
    const emscripten::val timestamp,
    const FrameRGBA& frame
) const {
    const uint64_t timestamp_u64 = static_cast<uint64_t>(timestamp.as<double>());
    if(!_set_imu_prior(timestamp_u64)){
        // VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
        return;
    }
    cv::Mat rgb;
    cv::cvtColor(frame.mat().clone(), rgb, cv::COLOR_RGBA2BGR);
    frame_handler_->addImageBundle({rgb}, timestamp_u64);
    // check stage
    if(auto_reset_ && stage() == stage2int[Stage::kPaused]) {
        reset();
    }
}

void Odometry::addImuMeasurement(
    const emscripten::val timestamp,
    const emscripten::val gyro_array,
    const emscripten::val acc_array
) const {
    if(imu_handler_ == nullptr) return;
    if(gyro_array[0] == emscripten::val::null() || acc_array[0] == emscripten::val::null()) 
        return;
    ///
    const uint64_t timestamp_u64 = static_cast<uint64_t>(timestamp.as<double>());
    Eigen::Vector3d gyro, acc;
    gyro << gyro_array[0].as<double>(), gyro_array[1].as<double>(), gyro_array[2].as<double>();
    acc << acc_array[0].as<double>(), acc_array[1].as<double>(), acc_array[2].as<double>();
    const ImuMeasurement m(
        (double)timestamp_u64 * common::conversions::kNanoSecondsToSeconds,
        gyro,
        acc
    );
    imu_handler_->addImuMeasurement(m);
}



EMSCRIPTEN_BINDINGS(svojs)
{
    emscripten::constant("S_ERROR", -1);
    emscripten::constant("S_TRACK", 0);
    emscripten::constant("S_INIT", 1);
    emscripten::constant("S_PASUED", 2);
    emscripten::constant("S_RELOC", 3);

    emscripten::class_<FrameRGBA>("Frame")
        .constructor<int, int>()
        .property("data", &FrameRGBA::matData);

    emscripten::class_<Odometry>("Instance")
        .constructor<emscripten::val, emscripten::val>()
        
        .property("state", &Odometry::stage)
        .property("width", &Odometry::imwidth)
        .property("height", &Odometry::imheight)

        .function("start", &Odometry::start)
        .function("reset", &Odometry::reset)
        .function("addFrame", &Odometry::addImageBundle, emscripten::allow_raw_pointers())
        .function("addMotion", &Odometry::addImuMeasurement, emscripten::allow_raw_pointers())
        .function("getGLPose", &Odometry::world_pose_gl)
        .function("getViewPose", &Odometry::world_viewpose_gl);
}