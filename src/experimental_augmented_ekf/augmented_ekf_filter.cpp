/*
 * Recovered experimental source from a later team integration stage of a university robotics project.
 * The augmented EKF did not validate reliably in the final integration.
 * Implementation logic below is preserved as recovered; see docs/failure_analysis.md and SOURCE_RECOVERY.md.
 */

#include <ekf_filter.h>
#include <Eigen/Dense>  
using Eigen::MatrixXd; 





namespace ekf_imu_vision {
EKFImuVision::EKFImuVision(/* args */) {}

EKFImuVision::~EKFImuVision() {}


void EKFImuVision::init(ros::NodeHandle& nh) {
  node_ = nh;

  /* ---------- parameter ---------- */
  Qt_.setZero();
  Rt1_.setZero();
  Rt2_.setZero();

  // addition and removel of augmented state
  
  // TODO
  // set M_a_ and M_r_
  M_r_.setZero();
  M_r_.topLeftCorner<15,15>() = MatrixXd::Identity(15,15); 

  M_a_.setZero();
  M_a_.topLeftCorner<15,15>() = MatrixXd::Identity(15,15);
  M_a_.block<6,6>(15,0) = MatrixXd::Identity(6,6);

  for (int i = 0; i < 3; i++) {
    /* process noise */
    node_.param("aug_ekf/ng", Qt_(i, i), -1.0);
    node_.param("aug_ekf/na", Qt_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/nbg", Qt_(i + 6, i + 6), -1.0);
    node_.param("aug_ekf/nba", Qt_(i + 9, i + 9), -1.0);
    node_.param("aug_ekf/pnp_p", Rt1_(i, i), -1.0);
    node_.param("aug_ekf/pnp_q", Rt1_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/vo_pos", Rt2_(i, i), -1.0);
    node_.param("aug_ekf/vo_rot", Rt2_(i + 3, i + 3), -1.0);
  }
  cout << "Qt_:\n" << Qt_ << endl;
  cout << "Rt1_:\n" << Rt1_ << endl;
  cout << "Rt2_:\n" << Rt2_ << endl;
  cout << "deque 0: " << aug_state_hist_.begin()->mean << endl;

  init_        = false;

  for(int i = 0; i < 4; i++)
    latest_idx[i] = 0;

  /* ---------- subscribe and publish ---------- */
  imu_sub_ =
      node_.subscribe<sensor_msgs::Imu>("/dji_sdk_1/dji_sdk/imu", 100, &EKFImuVision::imuCallback, this);
  ROS_WARN("Hi");
  pnp_sub_     = node_.subscribe<nav_msgs::Odometry>("tag_odom", 10, &EKFImuVision::PnPCallback, this);
  // opti_tf_sub_ = node_.subscribe<geometry_msgs::PointStamped>("opti_tf_odom", 10,
                                                              // &EKFImuVision::opticalCallback, this);
  stereo_sub_  = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 10,
                                                          &EKFImuVision::stereoVOCallback, this);
  fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("ekf_fused_odom", 10);
  path_pub_         = node_.advertise<nav_msgs::Path>("/aug_ekf/Path", 100);

  ros::Duration(0.5).sleep();

  ROS_INFO("Start ekf.");
}

void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr& msg) {

  // TODO
  // construct a new state using the absolute measurement from marker PnP and process the new state
  ROS_WARN("Hi again");
  if (std::isnan(msg->pose.pose.position.x) || 
      std::isnan(msg->pose.pose.orientation.w)) {
    ROS_WARN("Invalid PnP measurement received");
    return;
  }
  ROS_WARN("NOT Invalid PnP measurement received");


  bool pnp_lost = fabs(msg->pose.pose.position.x) < 1e-4 && fabs(msg->pose.pose.position.y) < 1e-4 &&
      fabs(msg->pose.pose.position.z) < 1e-4;
  if (pnp_lost) return;

  

  Mat3x3 R_w_b = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                  .toRotationMatrix();
  Vec3 t_w_b;
  t_w_b[0] = msg->pose.pose.position.x;
  t_w_b[1] = msg->pose.pose.position.y;
  t_w_b[2] = msg->pose.pose.position.z;

  AugState         new_state;

  new_state.time_stamp = msg->header.stamp;
  new_state.type = pnp;
  new_state.ut.head(3) = t_w_b;
  new_state.ut.segment(3, 3) = rotation2Euler(R_w_b);

  new_state.mean = Vec21::Zero(21);
  new_state.covariance = 0.8*Mat21x21::Identity(21,21);
  new_state.mean.segment(0,6) = new_state.ut;
  new_state.mean.segment(15,6) = new_state.ut;
  
  
  deque<AugState>::iterator state_iterator;


  // first state
  if (!init_){
    state_iterator = insertNewState(new_state);
    if (initUsingPnP(state_iterator)) {
      init_ = true;
      ROS_INFO("EKF initialized successfully with first PnP measurement");
      changeAugmentedState(*state_iterator);
    } 
    
    ROS_INFO("first state:\n%s", state_iterator->mean.transpose());
    latest_idx[keyframe] = distance(aug_state_hist_.begin(),state_iterator);
    ROS_INFO("keyframe index: %d", latest_idx[keyframe]);
    return;
  }

  state_iterator = insertNewState(new_state);
  updatePnP(*state_iterator,*(state_iterator-1));
  repropagate(state_iterator, init_);
  removeOldState();

  publishFusedOdom();
  

  if (!processNewState(new_state, false)) {
    return;
  }

}

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg) {

  // TODO
  // label the previous keyframe
  // construct a new state using the relative measurement from VO and process the new state
  if(!init_){
    ROS_WARN("EKF not initialized.");
    return;
  }

  //new state
  ros::Time time_keyframe = msg->key_stamp, time_vo = msg->header.stamp;
  Mat3x3 R_i_k;
  Vec3 t_i_k;
  R_i_k = Eigen::Quaterniond(msg->relative_pose.orientation.w, msg->relative_pose.orientation.x,
                                  msg->relative_pose.orientation.y, msg->relative_pose.orientation.z)
                    .toRotationMatrix();
  t_i_k[0] = msg->relative_pose.position.x;
  t_i_k[1] = msg->relative_pose.position.y;
  t_i_k[2] = msg->relative_pose.position.z;

  AugState         new_state;
  
  new_state.time_stamp = time_vo;
  new_state.type = vo;
  new_state.ut.segment(0,3) = t_i_k;
  new_state.ut.segment(3,3) = rotation2Euler(R_i_k);
  new_state.key_frame_time_stamp = time_keyframe;

  // put new state into the queue and process it
  deque<AugState>::iterator state_iterator;
  state_iterator = insertNewState(new_state);
  deque<AugState>::iterator keyframe_it = aug_state_hist_.end();

  if(time_keyframe != aug_state_hist_.at(latest_idx[keyframe]).time_stamp){

    while (keyframe_it != aug_state_hist_.begin() && keyframe_it->time_stamp !=time_keyframe){
      keyframe_it--;
    }

    if (keyframe_it->time_stamp.toSec() ==0){
      updateVO(*state_iterator, *(state_iterator-1));
      repropagate(state_iterator, init_);
      removeOldState();
      return;
    }

    ROS_INFO("keyframe index: %d", distance(aug_state_hist_.begin(),keyframe_it));

    changeAugmentedState(*keyframe_it);
    latest_idx[keyframe] = distance(aug_state_hist_.begin(), keyframe_it);
    updateVO(*state_iterator,*(state_iterator-1));
    repropagate(keyframe_it, init_);

  } else{
    updateVO(*state_iterator, *(state_iterator-1));
    repropagate(state_iterator, init_);

  }
  removeOldState();

}

void EKFImuVision::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

  // TODO
  // construct a new state using the IMU input and process the new state
  if(!init_){
    ROS_WARN("EKF not initialized.");
    return;
  }
  //new state
  Vec3 acceleration(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
  Vec3 angular_vel(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
  AugState new_state;
  new_state.time_stamp = imu_msg->header.stamp;
  new_state.type = imu;
  new_state.ut.segment(0,3) = angular_vel;
  new_state.ut.segment(3,3) = acceleration;

  //process it
  Vec6 tmp_ut = Vec6::Zero(6);
  aug_state_hist_.push_back(new_state);
  latest_idx[imu] = aug_state_hist_.size()-1;
  predictIMU(aug_state_hist_.at(latest_idx[imu]), aug_state_hist_.at(latest_idx[imu]-1),tmp_ut);
  Mat3x3 R_EKF;
  
  Vec21 x =aug_state_hist_.at(max(latest_idx[pnp],latest_idx[vo])).mean; 

  R_EKF << cos(x(5))*cos(x(4))- sin(x(3))*sin(x(5))*sin(x(4)), -cos(x(3))*sin(x(5)), cos(x(5))*sin(x(4))+cos(x(4))*sin(x(3))*sin(x(5)),
        cos(x(4))*sin(x(5))+cos(x(5))*(sin(x(3)))*sin(x(4)), cos(x(3))*cos(x(5)), sin(x(5))*sin(x(4))-cos(x(5))*sin(x(3))*cos(x(4)),
        -cos(x(3))*sin(x(4)), sin(x(3)), cos(x(3))*cos(x(4));
  

  Eigen::Quaterniond Q_EKF(R_EKF);
  nav_msgs::Odometry odom_EKF;

  odom_EKF.header.stamp = aug_state_hist_.at(latest_idx[imu]).time_stamp;
  odom_EKF.header.frame_id = "world";

  odom_EKF.pose.pose.position.x = x(0);
  odom_EKF.pose.pose.position.y = x(1);
  odom_EKF.pose.pose.position.z = x(2);
  odom_EKF.twist.twist.linear.x = x(6);
  odom_EKF.twist.twist.linear.y = x(7);
  odom_EKF.twist.twist.linear.z = x(8);

  odom_EKF.pose.pose.orientation.w = Q_EKF.w();
  odom_EKF.pose.pose.orientation.x = Q_EKF.x();
  odom_EKF.pose.pose.orientation.y = Q_EKF.y();
  odom_EKF.pose.pose.orientation.z = Q_EKF.z();
  fuse_odom_pub_.publish(odom_EKF);

  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x  = x(0);
  path_pose.pose.position.y  = x(1);
  path_pose.pose.position.z  = x(2);
  path_.poses.push_back(path_pose);
  path_pub_.publish(path_);
  
}

void EKFImuVision::predictIMU(AugState& cur_state, AugState& prev_state, Vec6 ut) {
  // TODO
  // construct a new state using the IMU input and process the new state

  double dt = (cur_state.time_stamp - prev_state.time_stamp).toSec();

  Vec15 state_derivative = modelF(prev_state.mean.segment(0,15), cur_state.ut, Vec12::Zero(12));
  cur_state.mean.segment(15,6) = prev_state.mean.segment(15,6);
  cur_state.mean.segment(0,15) = prev_state.mean.segment(0,15) + dt * state_derivative;

  // Jacobians
  Mat15x15 state_jacobian = jacobiFx(prev_state.mean.segment(0,15), cur_state.ut, Vec12::Zero(12));
  Mat15x15 state_transition = Mat15x15::Identity() + dt * state_jacobian;

  Mat15x12 noise_jacobian = jacobiFn(prev_state.mean.segment(0,15), cur_state.ut, Vec12::Zero(12));
  Mat15x12 discrete_noise_influence = dt * noise_jacobian;

  // Covariance propagation
  cur_state.covariance.block<15,15>(0,0) = state_transition * prev_state.covariance.block<15,15>(0,0) * state_transition.transpose() + discrete_noise_influence * Qt_ * discrete_noise_influence.transpose();
  cur_state.covariance.block<6,15>(15,0) = prev_state.covariance.block<6,15>(15,0) * state_transition.transpose();
  cur_state.covariance.block<15,6>(0,15) = state_transition * prev_state.covariance.block<15,6>(0,15);
  cur_state.covariance.block<6,6>(15,15) = prev_state.covariance.block<6,6>(15,15);

}



void EKFImuVision::updatePnP(AugState& cur_state, AugState& prev_state) {

  // TODO
  // update by marker PnP measurements
  
  Vec6 predicted_measurement, innovation	, measurement_noise_sample = Vec6::Zero(6);
  predicted_measurement = modelG1(prev_state.mean.segment(0,15), measurement_noise_sample);
  Mat6x21 measurement_jacobian ;
  measurement_jacobian .setZero();
  measurement_jacobian .block<6,15>(0,0)=jacobiG1x(prev_state.mean.segment(0,15), measurement_noise_sample);
  innovation = cur_state.ut - measurement_jacobian  * prev_state.mean;
  innovation(3) = atan2(sin(innovation(3)), cos(innovation(3)));
  innovation(4) = atan2(sin(innovation(4)), cos(innovation(4)));
  innovation(5) = atan2(sin(innovation(5)), cos(innovation(5)));

  Mat21x6 K;
  K = prev_state.covariance*measurement_jacobian .transpose()*(measurement_jacobian *prev_state.covariance*measurement_jacobian .transpose()+Rt1_).inverse();
  cur_state.mean = prev_state.mean + K*innovation;
  cur_state.covariance = prev_state.covariance - K * measurement_jacobian  * prev_state.covariance;
  innovation(3) = atan2(sin(innovation(3)), cos(innovation(3)));
  innovation(4) = atan2(sin(innovation(4)), cos(innovation(4)));
  innovation(5) = atan2(sin(innovation(5)), cos(innovation(5)));
  ROS_INFO("PNP update result: %f %f %f %f %f %f", cur_state.mean(0), cur_state.mean(1), cur_state.mean(2), cur_state.mean(3), cur_state.mean(4), cur_state.mean(5));
  
}

void EKFImuVision::updateVO(AugState& cur_state, AugState& prev_state) {
  
  // TODO
  // update by relative pose measurements

  Vec6 predicted_measurement, innovation, measurement_noise_sample = Vec6::Zero(6); 
  predicted_measurement = modelG2(prev_state.mean, measurement_noise_sample);
  Mat6x21 measurement_jacobian ;
  measurement_jacobian  = jacobiG2x(prev_state.mean, measurement_noise_sample);
  innovation = cur_state.ut - measurement_jacobian  * prev_state.mean;
  innovation(3) = atan2(sin(innovation(3)), cos(innovation(3)));
  innovation(4) = atan2(sin(innovation(4)), cos(innovation(4)));
  innovation(5) = atan2(sin(innovation(5)), cos(innovation(5)));
  Mat21x6 K;
  Mat6x6 W_t;
  W_t = jacobiG2v(prev_state.mean, measurement_noise_sample);
  K = prev_state.covariance*measurement_jacobian .transpose()*(measurement_jacobian *prev_state.covariance*measurement_jacobian .transpose()+W_t* Rt2_*W_t.transpose()).inverse();
  cur_state.mean = prev_state.mean + K*innovation;
  cur_state.covariance = prev_state.covariance - K*measurement_jacobian *prev_state.covariance;
  innovation(3) = atan2(sin(innovation(3)), cos(innovation(3)));
  innovation(4) = atan2(sin(innovation(4)), cos(innovation(4)));
  innovation(5) = atan2(sin(innovation(5)), cos(innovation(5)));

}

void EKFImuVision::changeAugmentedState(AugState& state) {
  ROS_ERROR("----------------change keyframe------------------------");

  // TODO
  // change augmented state

  state.covariance = M_a_ * M_r_ * state.covariance * M_r_.transpose() * M_a_.transpose();
  state.mean = M_a_ * M_r_ * state.mean;
  state.key_frame_time_stamp = state.time_stamp;
}

bool EKFImuVision::processNewState(AugState& new_state, bool change_keyframe) {

  // TODO
  // process the new state
  // step 1: insert the new state into the queue and get the iterator to start to propagate (be careful about the change of key frame).
  // step 2: try to initialize the filter if it is not initialized.
  // step 3: repropagate from the iterator you extracted.
  // step 4: remove the old states.
  // step 5: publish the latest fused odom

  //deque<AugState>::iterator state_iterator;

  //state_iterator = insertNewState(new_state);
  //updatePnP(*state_iterator,*(state_iterator-1));
  //repropagate(state_iterator, init_);
  //removeOldState();

  
  return true;

}

deque<AugState>::iterator EKFImuVision::insertNewState(AugState& new_state){

  // TODO
  // insert the new state to the queue
  // update the latest_idx of the type of the new state
  // return the iterator point to the new state in the queue 

  ros::Time time = new_state.time_stamp;

  if (aug_state_hist_.empty()){
    aug_state_hist_.push_front(new_state);
    latest_idx[new_state.type] = 0;
    return aug_state_hist_.begin();
  }
  deque<AugState>::iterator state_it = aug_state_hist_.end();

  while (state_it != aug_state_hist_.begin() && time<=(*(state_it-1)).time_stamp){
    state_it--;
  }
  state_it = aug_state_hist_.insert(state_it, new_state);

  latest_idx[new_state.type] = distance(aug_state_hist_.begin(),state_it);

  for (int i = 0; i < 4; ++i) {
    if (i != new_state.type && latest_idx[i] >= latest_idx[new_state.type]) {
      latest_idx[i]++;
    }
  }
  
  return state_it;
}

void EKFImuVision::repropagate(deque<AugState>::iterator& new_input_it, bool& init) {

  // TODO
  // repropagate along the queue from the new input according to the type of the inputs / measurements
  // remember to consider the initialization case 

  ROS_INFO("time: %f", new_input_it->time_stamp.toSec());
  ROS_INFO("type: %d", new_input_it->type);
  ROS_ERROR("%d", aug_state_hist_.size());
  for (deque<AugState>::iterator it = new_input_it+1; it != aug_state_hist_.end(); it++){
    if (it->type == imu){
      Vec6 tmp_ut = Vec6::Zero();
      predictIMU(*it, *(it-1), tmp_ut);

    } else if (it->type ==pnp)
    {
      updatePnP(*it, *(it-1));
    } else if (it->type == vo || it->type == keyframe) {
      updateVO(*it, *(it-1));
      
    }
  }
}

void EKFImuVision::removeOldState() {

  // TODO
  // remove the unnecessary old states to prevent the queue from becoming too long

  unsigned int remove_idx = min(min(latest_idx[imu], latest_idx[pnp]), latest_idx[keyframe]);

  aug_state_hist_.erase(aug_state_hist_.begin(), aug_state_hist_.begin() + remove_idx);

  for(int i = 0; i < 4; i++){
    latest_idx[i] -= remove_idx;
  }

  
}

void EKFImuVision::publishFusedOdom() {
  AugState last_state = aug_state_hist_.back();

  double phi, theta, psi;
  phi   = last_state.mean(3);
  theta = last_state.mean(4);
  psi   = last_state.mean(5);

  if (last_state.mean.head(3).norm() > 20) {
    ROS_ERROR_STREAM("error state: " << last_state.mean.head(3).transpose());
    return;
  }

  // using the zxy euler angle
  Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp    = last_state.time_stamp;

  odom.pose.pose.position.x = last_state.mean(0);
  odom.pose.pose.position.y = last_state.mean(1);
  odom.pose.pose.position.z = last_state.mean(2);

  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  odom.twist.twist.linear.x = last_state.mean(6);
  odom.twist.twist.linear.y = last_state.mean(7);
  odom.twist.twist.linear.z = last_state.mean(8);


  fuse_odom_pub_.publish(odom);

  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x                         = last_state.mean(0);
  path_pose.pose.position.y                         = last_state.mean(1);
  path_pose.pose.position.z                         = last_state.mean(2);
  path_.poses.push_back(path_pose);
  path_pub_.publish(path_);
}



bool EKFImuVision::initFilter() {

  // TODO
  // Initial the filter when a keyframe after marker PnP measurements is available

 
  return false;
}


bool EKFImuVision::initUsingPnP(deque<AugState>::iterator start_it) {

  // TODO
  // Initialize the absolute pose of the state in the queue using marker PnP measurement.
  // This is only step 1 of the initialization.

  if (start_it->type!=pnp){
    ROS_ERROR("it is not PnP state for initialization");
    return false;
  }
  
  AugState start_state;
  start_state.mean = Vec21::Zero(21);
  start_state.covariance = 0.8*Mat21x21::Identity(21,21);
  updatePnP(*start_it, start_state);

  // std::cout << "init PnP state: " << start_it->mean.transpose() << std::endl;
  return true;
}


Vec3 EKFImuVision::rotation2Euler(const Mat3x3& R) {
  double phi   = asin(R(2, 1));
  double theta = atan2(-R(2, 0), R(2, 2));
  double psi   = atan2(-R(0, 1), R(1, 1));
  return Vec3(phi, theta, psi);
}



}  // namespace ekf_imu_vision