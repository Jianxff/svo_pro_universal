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
    const uint32_t imwidth = calib["width"].as<uint32_t>();
    const uint32_t imheight = calib["height"].as<uint32_t>();
    Eigen::Vector4d intrinsics;
    if(calib["intrinsics"] == emscripten::val::undefined()) {
        const double focal = std::max(imwidth, imheight) * 1.2;
        const double cx = (double)imwidth / 2.0;
        const double cy = (double)imheight / 2.0;
        intrinsics << focal, focal, cx, cy;
    } else {
        intrinsics = convert_emarray_vector<double>(calib["intrinsics"], 4);
    }
    
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
    calib.delay_imu_cam = data["imu_params"]["delay_imu_cam"].as<double>();
    calib.max_imu_delta_t = data["imu_params"]["max_imu_delta_t"].as<double>();
    calib.saturation_accel_max = data["imu_params"]["acc_max"].as<double>();
    calib.saturation_omega_max = data["imu_params"]["omega_max"].as<double>();
    calib.gyro_noise_density = data["imu_params"]["sigma_omega_c"].as<double>();
    calib.acc_noise_density = data["imu_params"]["sigma_acc_c"].as<double>();
    calib.gyro_bias_random_walk_sigma = data["imu_params"]["sigma_omega_bias_c"].as<double>();
    calib.acc_bias_random_walk_sigma = data["imu_params"]["sigma_acc_bias_c"].as<double>();
    calib.gravity_magnitude = data["imu_params"]["g"].as<double>();
    calib.imu_rate = data["imu_params"]["imu_rate"].as<double>();
    return calib;
}


ImuInitialization loadImuInitialization(const emscripten::val data){
    ImuInitialization init;
    init.velocity = Eigen::Vector3d(
          data["imu_initialization"]["velocity"][0].as<double>(),
          data["imu_initialization"]["velocity"][1].as<double>(),
          data["imu_initialization"]["velocity"][2].as<double>());
    init.omega_bias = Eigen::Vector3d(
          data["imu_initialization"]["omega_bias"][0].as<double>(),
          data["imu_initialization"]["omega_bias"][1].as<double>(),
          data["imu_initialization"]["omega_bias"][2].as<double>());
    init.acc_bias = Eigen::Vector3d(
          data["imu_initialization"]["acc_bias"][0].as<double>(),
          data["imu_initialization"]["acc_bias"][1].as<double>(),
          data["imu_initialization"]["acc_bias"][2].as<double>());
    init.velocity_sigma = data["imu_initialization"]["velocity_sigma"].as<double>();
    init.omega_bias_sigma = data["imu_initialization"]["omega_bias_sigma"].as<double>();
    init.acc_bias_sigma = data["imu_initialization"]["acc_bias_sigma"].as<double>();
    return init;
}


BaseOptions loadBaseOptions(const emscripten::val node) {
    BaseOptions o;
    o.max_n_kfs = node["max_n_kfs"].as<int>(5);
    o.use_imu = node["use_imu"].as<bool>(false);
    o.trace_dir = node["trace_dir"].as<std::string>("./");
    o.quality_min_fts = node["quality_min_fts"].as<int>(50);
    o.quality_max_fts_drop = node["quality_max_drop_fts"].as<int>(40);
    o.relocalization_max_trials = node["relocalization_max_trials"].as<int>(50);
    o.poseoptim_prior_lambda = node["poseoptim_prior_lambda"].as<double>(0.0);
    o.poseoptim_using_unit_sphere = node["poseoptim_using_unit_sphere"].as<bool>(false);
    o.img_align_prior_lambda_rot = node["img_align_prior_lambda_rot"].as<double>(0.0);
    o.img_align_prior_lambda_trans = node["img_align_prior_lambda_trans"].as<double>(0.0);
    o.structure_optimization_max_pts = node["structure_optimization_max_pts"].as<int>(20);
    o.init_map_scale = node["map_scale"].as<double>(1.0);
    const std::string kfselect_criterion = node["kfselect_criterion"].as<std::string>("FORWARD");
    if(kfselect_criterion == "FORWARD")
        o.kfselect_criterion = KeyframeCriterion::FORWARD;
    else
        o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;
    o.kfselect_min_dist = node["kfselect_min_dist"].as<double>(0.12);
    o.kfselect_numkfs_upper_thresh = node["kfselect_numkfs_upper_thresh"].as<int>(120);
    o.kfselect_numkfs_lower_thresh = node["kfselect_numkfs_lower_thresh"].as<double>(70);
    o.kfselect_min_dist_metric = node["kfselect_min_dist_metric"].as<double>(0.01);
    o.kfselect_min_angle = node["kfselect_min_angle"].as<double>(20);
    o.kfselect_min_disparity = node["kfselect_min_disparity"].as<double>(40);
    o.kfselect_min_num_frames_between_kfs = node["kfselect_min_num_frames_between_kfs"].as<int>(2);
    o.kfselect_backend_max_time_sec = node["kfselect_backend_max_time_sec"].as<double>(3.0);
    o.img_align_max_level = node["img_align_max_level"].as<int>(4);
    o.img_align_min_level = node["img_align_min_level"].as<int>(2);
    o.img_align_robustification = node["img_align_robustification"].as<bool>(false);
    o.img_align_use_distortion_jacobian =
        node["img_align_use_distortion_jacobian"].as<bool>(false);
    o.img_align_est_illumination_gain =
        node["img_align_est_illumination_gain"].as<bool>(false);
    o.img_align_est_illumination_offset =
        node["img_align_est_illumination_offset"].as<bool>(false);
    o.poseoptim_thresh = node["poseoptim_thresh"].as<double>(2.0);
    o.update_seeds_with_old_keyframes =
        node["update_seeds_with_old_keyframes"].as<bool>(true);
    o.use_async_reprojectors = node["use_async_reprojectors"].as<bool>(false);
    o.trace_statistics = node["trace_statistics"].as<bool>(false);
    o.backend_scale_stable_thresh =
        node["backend_scale_stable_thresh"].as<double>(0.02);
    o.global_map_lc_timeout_sec_ =
        node["global_map_timeout_sec"].as<double>(2.0);
    return o;
}

IMUHandlerOptions loadIMUHandlerOptions(const emscripten::val node) {
    IMUHandlerOptions o;

    o.temporal_stationary_check = node["temporal_stationary_check"].as<bool>(false);
    o.temporal_window_length_sec_ = node["temporal_window_length_sec"].as<double>(0.5);
    o.stationary_acc_sigma_thresh_ = node["stationary_acc_sigma_thresh"].as<double>(10e-4);
    o.stationary_gyr_sigma_thresh_ = node["stationary_gyr_sigma_thresh"].as<double>(6e-5);

    return o;
}

InitializationOptions loadInitializationOptions(const emscripten::val node) {
    InitializationOptions o;

    o.init_min_features = node["init_min_features"].as<int>(100);
    o.init_min_tracked = node["init_min_tracked"].as<int>(80);
    o.init_min_inliers = node["init_min_inliers"].as<int>(70);
    o.init_min_disparity = node["init_min_disparity"].as<double>(40.0);
    o.init_min_features_factor = node["init_min_features_factor"].as<double>(2.0);
    o.reproj_error_thresh = node["reproj_err_thresh"].as<double>(2.0);
    o.init_disparity_pivot_ratio = node["init_disparity_pivot_ratio"].as<double>(0.5);
    std::string init_method = node["init_method"].as<std::string>("FivePoint");
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

    o.max_n_kfs = node["reprojector_max_n_kfs"].as<int>(5);
    o.max_n_features_per_frame = node["max_fts"].as<int>(160);
    o.cell_size = node["grid_size"].as<int>(35);
    o.reproject_unconverged_seeds =
        node["reproject_unconverged_seeds"].as<bool>(true);
    o.max_unconverged_seeds_ratio =
        node["max_unconverged_seeds_ratio"].as<double>(-1.0);
    o.min_required_features =
        node["quality_min_fts"].as<int>(50);
    o.seed_sigma2_thresh =
        node["seed_convergence_sigma2_thresh"].as<double>(200.0);

    o.affine_est_offset =
        node["reprojector_affine_est_offset"].as<bool>(true);
    o.affine_est_gain =
        node["reprojector_affine_est_gain"].as<bool>(false);
    o.max_fixed_landmarks =
        node["reprojector_max_fixed_landmarks"].as<int>(50);
    o.max_n_global_kfs =
        node["reprojector_max_n_global_kfs"].as<int>(20);
    o.use_kfs_from_global_map =
        node["reprojector_use_kfs_from_global_map"].as<bool>(false);
    o.fixed_lm_grid_size =
        node["reprojector_fixed_lm_grid_size"].as<int>(50);

    return o;
}

StereoTriangulationOptions loadStereoTriangulationOptions(const emscripten::val node) {
    StereoTriangulationOptions o;

    o.triangulate_n_features = node["max_fts"].as<int>(120);
    o.max_depth_inv = node["max_depth_inv"].as<double>(1.0/50.0);
    o.min_depth_inv = node["min_depth_inv"].as<double>(1.0/0.5);
    o.mean_depth_inv = node["mean_depth_inv"].as<double>(1.0/2.0);

    return o;
}

DepthFilterOptions loadDepthFilterOptions(const emscripten::val node) {
    DepthFilterOptions o;

    o.max_search_level = node["n_pyr_levels"].as<int>(3) - 1;
    o.use_threaded_depthfilter =
        node["use_threaded_depthfilter"].as<bool>(true);
    o.seed_convergence_sigma2_thresh =
        node["seed_convergence_sigma2_thresh"].as<double>(200.0);
    o.mappoint_convergence_sigma2_thresh =
        node["mappoint_convergence_sigma2_thresh"].as<double>(500.0);
    o.scan_epi_unit_sphere = node["scan_epi_unit_sphere"].as<bool>(false);
    o.affine_est_offset= node["depth_filter_affine_est_offset"].as<bool>(true);
    o.affine_est_gain = node["depth_filter_affine_est_gain"].as<bool>(false);
    o.max_n_seeds_per_frame = static_cast<size_t>(
            static_cast<double>(node["max_fts"].as<int>(120))
            * node["max_seeds_ratio"].as<double>(3.0));
    o.max_map_seeds_per_frame = static_cast<size_t>(
            static_cast<double>(node["max_map_fts"].as<int>(120)));
    o.extra_map_points =
        node["depth_filter_extra_map_points"].as<bool>(false);
    if(node["runlc"].as<bool>(false) && !o.extra_map_points)
    {
        LOG(WARNING) << "Loop closure requires extra map points, "
                    << " but the option is not set, overriding to true.";
        o.extra_map_points = true;
    }

    return o;
}


DetectorOptions loadDetectorOptions(const emscripten::val node) {
    DetectorOptions o;
    o.cell_size = node["grid_size"].as<int>(35);
    o.max_level = node["n_pyr_levels"].as<int>(3) - 1;
    o.threshold_primary = node["detector_threshold_primary"].as<int>(10);
    o.threshold_secondary = node["detector_threshold_secondary"].as<int>(200);
    o.threshold_shitomasi = node["detector_threshold_shitomasi"].as<int>(100);
    if(node["use_edgelets"].as<bool>(true))
        o.detector_type = DetectorType::kFastGrad;
    else
        o.detector_type = DetectorType::kFast;
    return o;
}

FeatureTrackerOptions loadFeatureTrackerOptions(const emscripten::val node) {
    FeatureTrackerOptions o;

    o.klt_max_level = node["klt_max_level"].as<int>(4);
    o.klt_min_level = node["klt_min_level"].as<int>(0);

    return o;
}

Odometry::Odometry(
    const emscripten::val calib,
    const emscripten::val config  
) {
    imwidth_ = calib["width"].as<uint32_t>();
    imheight_ = calib["height"].as<uint32_t>();
    const bool use_imu = calib["use_imu"].as<bool>(false);

    auto camera = makeCamera(calib);
    frame_handler_ = makeMono(camera, config);
    if(use_imu) {
        imu_handler_ = makeIMU(calib, config);
        frame_handler_->imu_handler_ = imu_handler_;
    }
    //
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

const Stage Odometry::stage() const {
    return frame_handler_->stage();
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
            VLOG(3) << "Set initial orientation from accelerometer measurements.";
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
        VLOG(3) << "Set incremental rotation prior from IMU.";
        vo->setRotationIncrementPrior(R_lastimu_newimu);
        }
    }
    return true;
}

cv::Mat Odometry::_emscripten_arraybuffer_to_cvmat(int data) const{
    cv::Mat image_rgba(
        imwidth_, imheight_, CV_8UC4, 
        reinterpret_cast<void*>(data)
    );
    cv::Mat image_bgr;
    // convert
    cv::cvtColor(image_rgba, image_bgr, cv::COLOR_RGBA2BGR);
    return image_bgr;
}

void Odometry::addImageBundle(
    const emscripten::val timestamp,
    const int arraybuffer
) const {
    const uint64_t timestamp_u64 = timestamp.as<uint64_t>();
    if(!_set_imu_prior(timestamp_u64)){
        VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
        return;
    }
    cv::Mat img = _emscripten_arraybuffer_to_cvmat(arraybuffer);
    frame_handler_->addImageBundle({img}, timestamp_u64);
}

void Odometry::addImuMeasurement(
    const emscripten::val timestamp,
    const emscripten::val gyro_array,
    const emscripten::val acc_array
) const {
    if(imu_handler_ == nullptr) return;
    const uint64_t timestamp_u64 = timestamp.as<uint64_t>();
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
    emscripten::constant("S_INIT", Stage::kInitializing);
    emscripten::constant("S_PASUE", Stage::kPaused);
    emscripten::constant("S_TRACK", Stage::kTracking);
    emscripten::constant("S_RELOC", Stage::kRelocalization);

    emscripten::class_<Odometry>("Odometry")
        .constructor<emscripten::val, emscripten::val>()
        
        .property("stage", &Odometry::stage)
        .property("width", &Odometry::imwidth)
        .property("height", &Odometry::imheight)

        .function("start", &Odometry::start)
        .function("getViewPose", &Odometry::world_viewpose_gl)
}