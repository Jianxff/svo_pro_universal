#include <svo/svo_factory.h>

namespace svo{
namespace factory{

void setInitialPose(FrameHandlerBase& vo) {
    Transformation T_world_imuinit(
        Eigen::Quaternion(1.0,0.0,0.0,0.0),
        Eigen::Vector3d(0.0,0.0,0.0)
    );
    vo.setInitialImuPose(T_world_imuinit);
}

std::shared_ptr<CameraBundle> makeCamera(
    const std::string& calib_file
) {
    return CameraBundle::loadFromYaml(calib_file);
}

std::shared_ptr<FrameHandlerMono> makeMono(
    const CameraBundlePtr& ncam,
    const std::string& config_file_path)
{
    if (ncam->numCameras() > 1) {
        LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
        ncam->keepFirstNCams(1);
    }

    FrameHandlerMono::Ptr vo = 
        std::make_shared<FrameHandlerMono>(
            ncam, 
            loadBaseOptions(config_file_path),
            loadDepthFilterOptions(config_file_path),
            loadDetectorOptions(config_file_path),
            loadInitializationOptions(config_file_path),
            loadReprojectorOptions(config_file_path),
            loadFeatureTrackerOptions(config_file_path)
        );

    // Get initial position and orientation of IMU
    setInitialPose(*vo);
    return vo;
}


std::shared_ptr<FrameHandlerStereo> makeStereo(
    const CameraBundlePtr& ncam,
    const std::string& config_file_path)
{
    if (ncam->numCameras() > 2) {
        LOG(ERROR) << "Stereo camera requires exactly two cameras.";
        return nullptr;
    }

    FrameHandlerStereo::Ptr vo = 
        std::make_shared<FrameHandlerStereo>(
            ncam, 
            loadBaseOptions(config_file_path),
            loadDepthFilterOptions(config_file_path),
            loadDetectorOptions(config_file_path),
            loadInitializationOptions(config_file_path),
            loadStereoTriangulationOptions(config_file_path),
            loadReprojectorOptions(config_file_path),
            loadFeatureTrackerOptions(config_file_path)
        );

    // Get initial position and orientation of IMU
    setInitialPose(*vo);
    return vo;
}


std::shared_ptr<FrameHandlerArray> makeArray(
    const CameraBundlePtr& ncam,
    const std::string& config_file_path)
{
    FrameHandlerArray::Ptr vo = 
        std::make_shared<FrameHandlerArray>(
            ncam, 
            loadBaseOptions(config_file_path),
            loadDepthFilterOptions(config_file_path),
            loadDetectorOptions(config_file_path),
            loadInitializationOptions(config_file_path),
            loadReprojectorOptions(config_file_path),
            loadFeatureTrackerOptions(config_file_path)
        );

    // Get initial position and orientation of IMU
    setInitialPose(*vo);
    return vo;
}

std::shared_ptr<ImuHandler> makeIMU(
    const std::string& calib_file_path,
    const std::string& config_file_path
) {
    ImuHandler::Ptr imu = std::make_shared<ImuHandler>(
        ImuHandler::loadCalibrationFromFile(calib_file_path),
        ImuHandler::loadInitializationFromFile(calib_file_path),
        loadIMUHandlerOptions(config_file_path)
    );

    return imu;
}


BaseOptions loadBaseOptions(const std::string& filepath) {
    if(filepath.size() == 0) return BaseOptions();

    YAML::Node node = YAML::LoadFile(filepath);
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

IMUHandlerOptions loadIMUHandlerOptions(const std::string& filepath) {
    if(filepath.size() == 0) return IMUHandlerOptions();

    YAML::Node node = YAML::LoadFile(filepath);
    IMUHandlerOptions o;

    o.temporal_stationary_check = node["temporal_stationary_check"].as<bool>(false);
    o.temporal_window_length_sec_ = node["temporal_window_length_sec"].as<double>(0.5);
    o.stationary_acc_sigma_thresh_ = node["stationary_acc_sigma_thresh"].as<double>(10e-4);
    o.stationary_gyr_sigma_thresh_ = node["stationary_gyr_sigma_thresh"].as<double>(6e-5);

    return o;
}

InitializationOptions loadInitializationOptions(const std::string& filepath) {
    if(filepath.size() == 0) return InitializationOptions();

    YAML::Node node = YAML::LoadFile(filepath);
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

ReprojectorOptions loadReprojectorOptions(const std::string& filepath) {
    if(filepath.size() == 0) return ReprojectorOptions();

    YAML::Node node = YAML::LoadFile(filepath);
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

StereoTriangulationOptions loadStereoTriangulationOptions(const std::string& filepath) {
    if(filepath.size() == 0) return StereoTriangulationOptions();

    YAML::Node node = YAML::LoadFile(filepath);
    StereoTriangulationOptions o;

    o.triangulate_n_features = node["max_fts"].as<int>(120);
    o.max_depth_inv = node["max_depth_inv"].as<double>(1.0/50.0);
    o.min_depth_inv = node["min_depth_inv"].as<double>(1.0/0.5);
    o.mean_depth_inv = node["mean_depth_inv"].as<double>(1.0/2.0);

    return o;
}

DepthFilterOptions loadDepthFilterOptions(const std::string& filepath) {
    if(filepath.size() == 0) return DepthFilterOptions();

    YAML::Node node = YAML::LoadFile(filepath);
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


DetectorOptions loadDetectorOptions(const std::string& filepath) {
    if(filepath.size() == 0) return DetectorOptions();

    YAML::Node node = YAML::LoadFile(filepath);
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

FeatureTrackerOptions loadFeatureTrackerOptions(const std::string& filepath) {
    if(filepath.size() == 0) return FeatureTrackerOptions();

    YAML::Node node = YAML::LoadFile(filepath);
    FeatureTrackerOptions o;

    o.klt_max_level = node["klt_max_level"].as<int>(4);
    o.klt_min_level = node["klt_min_level"].as<int>(0);

    return o;
}


} // namespace factory


Odometry::Odometry(
    const Type type,
    const std::string& calib_file,
    const std::string& svo_config_file,
    const bool set_initial_attitude_from_gravity
) : type_(type), set_initial_attitude_from_gravity_(set_initial_attitude_from_gravity)
{
    auto camera = factory::makeCamera(calib_file);
    if(type == Type::kMono || type == Type::kMonoIMU)
        frame_handler_ = mono_frame_handler_ = factory::makeMono(camera, svo_config_file);
    else if(type == Type::kStereo || type == Type::kStereoIMU)
        frame_handler_ = stereo_frame_handler_ = factory::makeStereo(camera, svo_config_file); 
    if(type == Type::kMonoIMU || type == Type::kStereoIMU) {
        imu_handler_ = factory::makeIMU(calib_file, svo_config_file);
        frame_handler_->imu_handler_ = imu_handler_;
    }
    //
}

FrameHandlerMono::Ptr Odometry::frame_handler_mono() const {
    return mono_frame_handler_;
}

FrameHandlerStereo::Ptr Odometry::frame_handler_stereo() const {
    return stereo_frame_handler_;
}

std::shared_ptr<FrameHandlerBase> Odometry::frame_handler() const {
    return frame_handler_;
}

ImuHandler::Ptr Odometry::imu_handler() const {
    return imu_handler_;
}

void Odometry::start() const {
    frame_handler_->start();
}

const Stage Odometry::stage() const {
    return frame_handler_->stage();
}


bool Odometry::_set_imu_prior(const uint64_t timestamp) const{
    if(imu_handler_ == nullptr) return true;
    std::shared_ptr<FrameHandlerBase> vo = frame_handler_;
    ///
    if(!vo->hasStarted() && set_initial_attitude_from_gravity_) {
        Quaternion R_imu_world;
        if(imu_handler_->getInitialAttitude(
            timestamp * common::conversions::kNanoSecondsToSeconds,
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

void Odometry::addImageBundle(
    const std::vector<cv::Mat>& imgs,
    const uint64_t timestamp
) const {
    if(!_set_imu_prior(timestamp)){
        VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
        return;
    }

    frame_handler_->addImageBundle(imgs, timestamp);
}

void Odometry::addImuMeasurement(
    const uint64_t timestamp,
    const Eigen::Vector3d &gyro,
    const Eigen::Vector3d &acc
) const {
    if(imu_handler_ == nullptr) return;
    const ImuMeasurement m(
        timestamp * common::conversions::kNanoSecondsToSeconds,
        gyro,
        acc
    );

    imu_handler_->addImuMeasurement(m);
}


} // namespace svo