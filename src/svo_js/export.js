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
    quality_max_drop_fts: 80,
    use_imu: false
}

const calib = {
    width: 640,
    height: 480,
    use_imu: false,
    imu_rate: 60,
    imu_g: 9.8,
    T_B_C: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
}

let Module_, Odometry_, Frame_, Context_;
let onWorker_ = false;

// init function
async function initialize(onworker, settings={}) {
    onWorker_ = onworker;
    for(const key in settings) {
        calib[key] = settings[key];
        svo_config[key] = settings[key];
    }
    // init loading
    return new Promise(async (resolve, reject) => {
        try{
            Module_ = await initModule();
            Odometry_ = new Module_.Instance(calib, svo_config);
            Frame_ = new Module_.Frame(calib.width, calib.height);
            Context_ = new OffscreenCanvas(calib.width, calib.height).getContext('2d', { willReadFrequently: true });
            if(onWorker_) postMessage({name: "init", data: true, msg: "success"});
            resolve(true);
        }
        catch(e) {
            if(onWorker_) postMessage({name: "init", data: false, msg: e});
            reject(e);
        }
    })
}

function addFrameBitmap(data) {
    const timestamp = data.timestamp;
    const bitmap = data.data;
    Context_.drawImage(bitmap, 0, 0);
    Frame_.data.set(
        Context_.getImageData(0, 0, bitmap.width, bitmap.height).data
    );
    Odometry_.addFrame(timestamp, Frame_);  
    return getViewPose();  
}

function addFrame(data) {
    const timestamp = data.timestamp;
    const ImageData = data.data;
    Frame_.data.set(ImageData.data);
    Odometry_.addFrame(timestamp, Frame_);
    return getViewPose();
}

function addMotion(data) {
    const timestamp = data.timestamp;
    const gyro = data.gyroscope;
    const acc = data.accelerate;
    Odometry_.addMotion(timestamp, gyro, acc);
    return getViewPose();
}

function getViewPose() {
    const pose = Odometry_.getViewPose();
    if(onWorker_) postMessage({name: "setViewPose", data: pose});
    return pose;
}

function getState() {
    const state = Odometry_.state;
    if(onWorker_) postMessage({name: "setState", data: state});
    return state;
}

function getLocalMap() {
    const mapdata = Odometry_.getLocalMap();
    if(onWorker_) postMessage({name: "setLoacalMap", data: mapdata});
    return mapdata
}

function reset() {
    Odometry_.reset();
}

onmessage = e => {
    switch (e.data.name) {
        case "init": initialize(true, e.data.data); break;
        case "addFrameBitmap": addFrameBitmap(e.data); break;
        case "addFrame": addFrame(e.data); break;
        case "addMotion": addMotion(e.data); break;
        case "getViewPose": getViewPose(); break;
        case "getState": getState(); break;
        case "getLocalmap": getLocalMap(); break;
        case "reset": reset(); break;
    }
}
onerror = e => console.error(e);

export default {
    initialize,
    addFrameBitmap,
    addFrame,
    addMotion,
    getViewPose,
    getLocalMap,
    getState,
    reset
}