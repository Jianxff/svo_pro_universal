const svo_config = {
    auto_reset: true,
    max_fts: 180,
    max_n_kfs: 30,
    kfselect_numkfs_upper_thresh: 180,
    kfselect_numkfs_lower_thresh: 90,
    kfselect_min_dist_metric: 0.001,
    kfselect_min_angle: 6,
    kfselect_min_disparity: 40,
    update_seeds_with_old_keyframes: true,
    kfselect_min_num_frames_between_kfs: 1,
    img_align_est_illumination_gain: true,
    img_align_est_illumination_offset: true,
    depth_filter_affine_est_offset: true,
    depth_filter_affine_est_gain: true,
    reprojector_affine_est_offset: true,
    reprojector_affine_est_gain: true,
    init_min_disparity: 30,
    quality_min_fts: 40,
    quality_max_drop_fts: 80
}

const calib = {
    width: 640,
    height: 480
}

export { svo_config, calib }