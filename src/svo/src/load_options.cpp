#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include "svo/frame_handler_base.h"

namespace svo {

YAML::Node try_load_yaml(const std::string& filepath) {
    YAML::Node node;
    try {
        node = YAML::LoadFile(filepath);
    } catch (YAML::Exception& e) {
        LOG(ERROR) << "Error parsing config file: " << e.what();
    }
    return node;
}

BaseOptions BaseOptions::from_yaml(const std::string& filepath) {
    BaseOptions o;
    YAML::Node node = try_load_yaml(filepath);
    if (node.IsNull()) {
        return o;
    }
    node = node["base"];
    
    o.max_n_kfs = node["max_n_kfs"].as<int>(5);
    o.use_imu = node["use_imu"].as<bool>(false);
    o.trace_dir = node["trace_dir"].as<std::string>("");
    o.quality_min_fts = node["quality_min_fts"].as<int>(50);
    o.quality_max_fts_drop = node["quality_max_fts_drop"].as<int>(40);
    o.relocalization_max_trials = node["relocalization_max_trials"].as<int>(50);
    o.poseoptim_prior_lambda = node["poseoptim_prior_lambda"].as<double>(0.0);
    o.poseoptim_using_unit_sphere = node["poseoptim_using_unit_sphere"].as<bool>(false);
    o.img_align_prior_lambda_rot = node["img_align_prior_lambda_rot"].as<double>(0.0);
    o.img_align_prior_lambda_trans = node["img_align_prior_lambda_trans"].as<double>(0.0);
    o.structure_optimization_max_pts = node["structure_optimization_max_pts"].as<int>(20);
    
}





} // namespace svo