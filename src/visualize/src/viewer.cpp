#include <svo/viewer/viewer.h>

namespace svo {
namespace viewer{

Viewer::Viewer(const std::shared_ptr<FrameHandlerBase>& pvo) {
    pvo_ = pvo;
}

void Viewer::run() {
    // Window Create 
    pangolin::CreateWindowAndBind("svo_pro_trajectory",1024,768);
    
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
        pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    transform_Twc_.SetIdentity();

    cv::namedWindow("svo_pro_frame", cv::WINDOW_AUTOSIZE);

    while( !sig_exit_ ) {
        // Clear GL Buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //  Update VO System
        update_vo();
        // Follow Main Camera
        s_cam.Follow(transform_Twc_);
        d_cam.Activate(s_cam);
        // Drawing 
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        drawCamera();
        drawMapRegion();
        drawTrajectory();
        // Finish
        pangolin::FinishFrame();
    }

    pangolin::DestroyWindow("svo_pro_trajectory");
    cv::destroyAllWindows();
}

void Viewer::exit() {
    sig_exit_ = true;
}

void Viewer::drawCamera() {
    const float w = 0.08; // camera size;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();
    glMultMatrixd(transform_Twc_.m);
    glLineWidth(3); // camera line width

    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void Viewer::drawMapRegion() {
    if(region_lm_.empty())
        return;
    glPointSize(4);
    glBegin(GL_POINTS);
    
    for(const auto& lm : region_lm_) {
        glColor3f(0.0,1.0,0.0);
        float x = lm(0), y = lm(1), z = lm(2);
        glVertex3f(x, y, z);
    }

    glEnd();
}


void Viewer::drawTrajectory() {
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    for(const auto& pos : traj_cam_) {
        glColor3f(1.0,0.0,0.0);
        glVertex3d(pos(0), pos(1), pos(2));
    }
    glEnd();

    glBegin(GL_LINE_STRIP);
    for(const auto& pos : traj_imu_) {
        glColor3f(0.0,0.0,1.0);
        glVertex3d(pos(0), pos(1), pos(2));
    }
    glEnd();

}


void Viewer::update_vo() {
    switch(pvo_->stage()) {
        case Stage::kTracking: {
            update_vo_tracking_();
            break;
        }
        case Stage::kInitializing: {
            traj_cam_.clear();
            traj_imu_.clear();
            region_lm_.clear();
            transform_Twc_.SetIdentity();
            update_vo_initializing_();
            break;
        }
        default:
            update_frame_();
            break;
    }
}

void Viewer::update_vo_tracking_() {
    auto last_frame = pvo_->getLastFrames();
    auto close_kfs = pvo_->closeKeyframes();
    auto map = pvo_->map();

    // update trajectory
    auto trans_cam = last_frame->at(0)->T_world_cam();
    auto pose_imu = last_frame->get_T_W_B();
    traj_cam_.push_back(trans_cam.getPosition());
    traj_imu_.push_back(pose_imu.getPosition());

    // update pose
    Eigen::Matrix4d Twc = trans_cam.getTransformationMatrix();
    transform_Twc_ = pangolin::OpenGlMatrix(Twc);

    // update map points
    std::vector<FramePtr> viz_frames = close_kfs;
    viz_frames.push_back(last_frame->at(0));
    region_lm_.clear();
    region_lm_.reserve(viz_frames.size() * 150);

    for(const auto& vf : viz_frames) {
        auto p_lm_vec_ = &(vf->landmark_vec_);
        for(size_t i = 0; i < vf->num_features_; ++i) {
            PointPtr p_lm = p_lm_vec_->at(i);
            if(p_lm == nullptr)
                continue;
            Eigen::Vector3d pos = p_lm->pos();
            region_lm_.push_back(pos);
        }
    }

    /// image with feature
    FramePtr frame = last_frame->at(0);
    cv::Mat img = frame->img().clone();
    feature_detection_utils::drawFeatures(*frame, 0, true, &img);
    cv::imshow("svo_pro_frame", img);
    cv::waitKey(5);
}

void Viewer::update_vo_initializing_() {
    auto ref_frames = pvo_->initializer_->frames_ref_;
    auto last_frames = pvo_->getLastFrames();

    if(!last_frames ||last_frames->size() == 0) return;
    
    cv::Mat img = last_frames->at(0)->img().clone();

    if (ref_frames) {
        auto ref = ref_frames->at(0);
        auto cur = last_frames->at(0);
        std::vector<std::pair<size_t, size_t>> matches_ref_cur;
        feature_tracking_utils::getFeatureMatches(
            *ref, *cur, &matches_ref_cur);
        const Keypoints& px_ref = ref->px_vec_;
        const Keypoints& px_cur = cur->px_vec_;
        for (size_t i = 0; i < matches_ref_cur.size(); ++i){
            size_t i_ref = matches_ref_cur[i].first;
            size_t i_cur = matches_ref_cur[i].second;
            cv::line(img,
                cv::Point2f(px_cur(0, i_cur), px_cur(1, i_cur)),
                cv::Point2f(px_ref(0, i_ref), px_ref(1, i_ref)),
                cv::Scalar(0, 255, 0), 2
            );
        }
    }

    if(!img.empty()) {
        cv::imshow("svo_pro_frame", img);
        cv::waitKey(5);        
    }
}

void Viewer::update_frame_() {
    auto last_frames = pvo_->getLastFrames();
    if(!last_frames || last_frames->empty())
        return;
    cv::Mat img = last_frames->at(0)->img().clone();
    cv::imshow("svo_pro_frame", img);
    cv::waitKey(5);
}

} // namespace viewer
} // namespace svo