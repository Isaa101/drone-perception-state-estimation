/*
 * Recovered source from a university robotics project.
 * This estimator was completed inside a course-provided stereo-VO skeleton.
 * The assignment explicitly states that generate3dPoints() and undistortedPts() were provided functionality.
 * Implementation logic below is preserved as recovered; see SOURCE_RECOVERY.md for provenance.
 */

#include "estimator.h"

void drawImage(const cv::Mat& img, const vector<cv::Point2f>& pts, string name) {
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++) {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}

vector<uchar> geometricVerification(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2) {
  if(pts2.size() > 8) {
    vector<uchar> mask;
    cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.01, 0.99, mask);
    return mask;
  }
  return vector<uchar>();
}

Estimator::Estimator() {
  ROS_INFO("Estimator initialization started");
  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = Eigen::Vector3d(0, 0, 0);
  prev_frame.w_R_c = Eigen::Matrix3d::Identity();
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::reset() {
  ROS_WARN("System reset triggered");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::setParameter() {
  for (int i = 0; i < 2; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    ROS_INFO_STREAM("Camera " << i << " extrinsics:\n" << ric[i] << "\nTranslation: " << tic[i].transpose());
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  readIntrinsicParameter(CAM_NAMES);

  Matrix4d Tl = Matrix4d::Identity();
  Tl.block(0, 0, 3, 3) = ric[0];
  Tl.block(0, 3, 3, 1) = tic[0];
  
  Matrix4d Tr = Matrix4d::Identity();
  Tr.block(0, 0, 3, 3) = ric[1];
  Tr.block(0, 3, 3, 1) = tic[1];
  
  Tlr = Tl.inverse() * Tr;
}

void Estimator::readIntrinsicParameter(const vector<string>& calib_file) {
  for (size_t i = 0; i < calib_file.size(); i++) {
    ROS_INFO("Loading calibration for camera %s", calib_file[i].c_str());
    m_camera.push_back(
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i])
    );
  }
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1) {
  if(fail_cnt > 20) {
    reset();
  }
  ROS_DEBUG("Processing new image frame");

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img = _img;

  vector<cv::Point2f> left_features, right_features;
  vector<cv::Point3f> map_points;
  
  c_R_k.setIdentity();
  c_t_k.setZero();

  if (init_finish) {
    vector<cv::Point2f> tracked_features;
    
    trackFeatureBetweenFrames(key_frame, _img, map_points, tracked_features);
    estimateTBetweenFrames(map_points, tracked_features, c_R_k, c_t_k);
  }

  extractNewFeatures(_img, left_features);
  trackFeatureLeftRight(_img, _img1, left_features, right_features);
  cur_frame.uv = left_features;

  vector<cv::Point2f> norm_left = undistortedPts(left_features, m_camera[0]);
  vector<cv::Point2f> norm_right = undistortedPts(right_features, m_camera[1]);
  
  vector<uchar> mask = geometricVerification(norm_left, norm_right);
  reduceVector(norm_left, mask);
  reduceVector(norm_right, mask);
  reduceVector(cur_frame.uv, mask);
  
  cur_frame.xyz.clear();
  generate3dPoints(norm_left, norm_right, cur_frame.xyz, cur_frame.uv);

  Matrix3d relative_R = c_R_k.transpose();
  Vector3d relative_t = -relative_R * c_t_k;
  cur_frame.w_R_c = key_frame.w_R_c * relative_R;
  cur_frame.w_t_c = key_frame.w_t_c + key_frame.w_R_c * relative_t;

  if(c_t_k.norm() > TRANSLATION_THRESHOLD || 
     acos(Quaterniond(c_R_k).w()) * 2.0 > ROTATION_THRESHOLD || 
     map_points.size() < FEATURE_THRESHOLD || 
     !init_finish) {
    key_frame = cur_frame;
    ROS_INFO("New keyframe selected");
  }

  prev_frame = cur_frame;
  updateLatestStates(cur_frame);
  init_finish = true;

  return true;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame& keyframe, 
                                        const cv::Mat& current_img,
                                        vector<cv::Point3f>& map_points,
                                        vector<cv::Point2f>& current_features) {
  vector<uchar> track_status;
  vector<float> track_error;
  map_points = key_frame.xyz;
  vector<cv::Point2f> key_features = key_frame.uv;
  
  cv::calcOpticalFlowPyrLK(key_frame.img, current_img, key_features, 
                          current_features, track_status, track_error,
                          cv::Size(19,19), 2);

  for (size_t i = 0; i < current_features.size(); i++) {
    if (track_status[i] && !inBorder(current_features[i], ROW, COL)) {
      track_status[i] = 0;
    }
  }
  
  reduceVector(key_features, track_status);
  reduceVector(map_points, track_status);
  reduceVector(current_features, track_status);

  vector<cv::Point2f> norm_key = undistortedPts(key_features, m_camera[0]);
  vector<cv::Point2f> norm_current = undistortedPts(current_features, m_camera[0]);
  vector<uchar> geo_mask = geometricVerification(norm_key, norm_current);
  
  reduceVector(map_points, geo_mask);
  reduceVector(current_features, geo_mask);

  return true;
}

bool Estimator::estimateTBetweenFrames(vector<cv::Point3f>& world_points,
                                     vector<cv::Point2f>& image_points, 
                                     Matrix3d& rotation,
                                     Vector3d& translation) {
  if(image_points.size() < 5) {
    ROS_WARN("Insufficient points for pose estimation");
    return false;
  }
  
  cv::Mat rvec, tvec, inliers;
  vector<cv::Point2f> norm_points = undistortedPts(image_points, m_camera[0]);

  cv::solvePnPRansac(world_points, norm_points,
                    cv::Mat::eye(3,3,CV_64F), cv::noArray(),
                    rvec, tvec, false, 150, 3.0, 0.98, inliers);

  cv::Mat rot_mat;
  cv::Rodrigues(rvec, rot_mat);
  
  for(int i = 0; i < 3; i++) {
    translation(i) = tvec.at<double>(i);
    for(int j = 0; j < 3; j++) {
      rotation(i,j) = rot_mat.at<double>(i,j);
    }
  }

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat& image, vector<cv::Point2f>& features) {
  int max_features = 180;
  double quality = 0.015;
  double min_dist = 9.0;
  int block_size = 5;
  
  cv::goodFeaturesToTrack(image, features, max_features, quality, min_dist, cv::Mat(), block_size);
}

bool Estimator::trackFeatureLeftRight(const cv::Mat& left_img, 
                                    const cv::Mat& right_img,
                                    vector<cv::Point2f>& left_points,
                                    vector<cv::Point2f>& right_points) {
  vector<uchar> track_status;
  vector<float> track_error;
  
  cv::TermCriteria criteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 25, 0.02);
  cv::calcOpticalFlowPyrLK(left_img, right_img, left_points, right_points,
                         track_status, track_error, cv::Size(27,27), 2, criteria);

  for(size_t i = 0; i < right_points.size(); i++) {
    if(track_status[i] && !inBorder(right_points[i], ROW, COL)) {
      track_status[i] = 0;
    }
  }
  
  reduceVector(left_points, track_status);
  reduceVector(right_points, track_status);

  drawImage(right_img, right_points, "Right Image Features");

  return true;
}

void Estimator::generate3dPoints(const vector<cv::Point2f>& left_pts,
                               const vector<cv::Point2f>& right_pts, 
                               vector<cv::Point3f>& points_3d,
                               vector<cv::Point2f>& points_2d) {
  Matrix<double,3,4> P_left, P_right;

  P_left << Matrix3d::Identity(), Vector3d::Zero();
  P_right.block(0,0,3,3) = Tlr.block(0,0,3,3).transpose();
  P_right.block(0,3,3,1) = -P_right.block(0,0,3,3) * Tlr.block(0,3,3,1);

  vector<uchar> valid_points;

  for (size_t i = 0; i < left_pts.size(); ++i) {
    Vector2d left(left_pts[i].x, left_pts[i].y);
    Vector2d right(right_pts[i].x, right_pts[i].y);
    Vector3d point_3d;
    
    triangulatePoint(P_left, P_right, left, right, point_3d);

    if (point_3d[2] > 0) {
      points_3d.push_back(cv::Point3f(point_3d[0], point_3d[1], point_3d[2]));
      valid_points.push_back(1);
    } else {
      valid_points.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(points_2d, valid_points);
}

bool Estimator::inBorder(const cv::Point2f& pt, const int& row, const int& col) {
  const int BORDER = 2;
  int x = cvRound(pt.x);
  int y = cvRound(pt.y);
  return BORDER <= x && x < col - BORDER && BORDER <= y && y < row - BORDER;
}

double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2) {
  cv::Point2f diff = pt1 - pt2;
  return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

template <typename T>
void Estimator::reduceVector(vector<T>& vec, vector<uchar> mask) {
  size_t index = 0;
  for (size_t i = 0; i < vec.size(); i++) {
    if(mask[i]) vec[index++] = vec[i];
  }
  vec.resize(index);
}

void Estimator::updateLatestStates(frame &current_frame) {
  latest_time = current_frame.frame_time;
  latest_pointcloud = current_frame.xyz;
  
  latest_P = current_frame.w_R_c * (-ric[0].transpose()*tic[0]) + current_frame.w_t_c;
  latest_Q = Quaterniond(current_frame.w_R_c * ric[0].transpose());
}

void Estimator::triangulatePoint(Matrix<double,3,4>& cam0, 
                               Matrix<double,3,4>& cam1,
                               Vector2d& pt0, 
                               Vector2d& pt1,
                               Vector3d& pt3d) {
  Matrix4d A = Matrix4d::Zero();
  A.row(0) = pt0[0] * cam0.row(2) - cam0.row(0);
  A.row(1) = pt0[1] * cam0.row(2) - cam0.row(1);
  A.row(2) = pt1[0] * cam1.row(2) - cam1.row(0);
  A.row(3) = pt1[1] * cam1.row(2) - cam1.row(1);
  
  Vector4d pt_homo = A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  pt3d = pt_homo.head<3>() / pt_homo(3);
}

double Estimator::reprojectionError(Matrix3d &R, Vector3d &t, 
                                  cv::Point3f &pt3d, cv::Point2f &pt2d) {
  Vector3d pt(pt3d.x, pt3d.y, pt3d.z);
  pt = R * pt + t;
  pt /= pt[2];
  return sqrt(pow(pt[0] - pt2d.x, 2) + pow(pt[1] - pt2d.y, 2));
}

vector<cv::Point2f> Estimator::undistortedPts(vector<cv::Point2f>& pts, 
                                            camodocal::CameraPtr cam) {
  vector<cv::Point2f> normalized_pts;
  for (auto& pt : pts) {
    Eigen::Vector3d p;
    cam->liftProjective(Eigen::Vector2d(pt.x, pt.y), p);
    normalized_pts.push_back(cv::Point2f(p.x()/p.z(), p.y()/p.z()));
  }
  return normalized_pts;
}