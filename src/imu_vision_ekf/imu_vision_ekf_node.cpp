/*
 * Recovered source from an individual university robotics project completed in a provided EKF skeleton.
 * The original ROS workspace, launch/configuration files and datasets are no longer available.
 * Implementation logic below is preserved as recovered; see SOURCE_RECOVERY.md for provenance and code-review notes.
 */

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include<cmath>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);

// Global variables
Vector3d gravity(0, 0, 9.80665);

double end_t = 0;
bool updated = false; 


VectorXd est = VectorXd::Zero(15); 
MatrixXd cov_mat = 0.5*MatrixXd::Identity(15,15); 


void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double dt = msg->header.stamp.toSec() - end_t;
    if(!updated || dt>1){
        end_t = msg->header.stamp.toSec();
        //update=true;
        return;
    }

    // cout << dt << endl;
    //acceleration
    Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    
    //angular velocity
    Vector3d gyro(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);

    Matrix3d G, R, R_dot, G_inv, At_angular ;

    double r = est(3), p = est(4), y = est(5);

    G << cos(p),   0,    -cos(r)*sin(p),
        0,        1,     sin(r),
        sin(p),   0,     cos(r)*cos(p);

    R << cos(y)*cos(p) - sin(r)*sin(y)*sin(p),   -cos(r)*sin(y),    cos(y)*sin(p) + cos(p)*sin(r)*sin(y),
        cos(p)*sin(y) + cos(y)*sin(r)*sin(p),    cos(r)*cos(y),    sin(y)*sin(p) - cos(y)*sin(r)*cos(p),
        -cos(r)*sin(p),                          sin(r),           cos(r)*cos(p);


    G_inv = G.inverse();

    //Noise terms
    Vector3d noise_gyro(0, 0, 0);
    Vector3d noise_accel(0, 0, 0);

    At_angular.setZero();  // Asegura que esté limpia

    At_angular(0,1) = (sin(p)*(-gyro(0) + est(9)) - cos(p)*(-gyro(2) + est(11))) / (cos(p)*cos(p));
    At_angular(1,0) = (cos(r)*cos(p)*(-gyro(2) + est(11)) - cos(r)*sin(p)*(-gyro(0) + est(9))) / (cos(r)*cos(p)*cos(p));
    At_angular(1,1) = -(sin(r)*sin(p)*(-gyro(2) + est(11)) + cos(p)*sin(r)*(-gyro(0) + est(9))) / (cos(r)*cos(p)*cos(p));
    At_angular(2,0) = (sin(p)*(-gyro(0) + est(9)) - cos(p)*(-gyro(2) + est(11))) / (cos(r)*cos(p)*cos(p));
    At_angular(2,1) = (cos(p)*(-gyro(0) + est(9)) + sin(p)*(-gyro(2) + est(11))) / (cos(r)*cos(p)*cos(p));

    //Rotation matrix derivation
    R_dot << acc(1)*sin(r)*sin(y) + acc(2)*cos(r)*cos(p)*sin(y) - acc(0)*cos(r)*sin(p)*sin(y), 
            acc(2)*(cos(p)*cos(y) - sin(r)*sin(p)*sin(y)) - acc(0)*(cos(y)*sin(p) + cos(p)*sin(r)*sin(y)), 
            -acc(0)*(cos(p)*sin(y) + cos(y)*sin(r)*sin(p)) - acc(2)*(sin(p)*sin(y) - cos(p)*cos(y)*sin(r)) - acc(1)*cos(r)*cos(y),
            acc(0)*cos(r)*cos(y)*sin(p) - acc(2)*cos(r)*cos(p)*cos(y) - acc(1)*cos(y)*sin(r), 
            acc(2)*(cos(p)*sin(y) + cos(y)*sin(r)*sin(p)) - acc(0)*(sin(p)*sin(y) - cos(p)*cos(y)*sin(r)),   
            acc(0)*(cos(p)*cos(y) - sin(r)*sin(p)*sin(y)) + acc(2)*(cos(y)*sin(p) + cos(p)*sin(r)*sin(y)) - acc(1)*cos(r)*sin(y),
            acc(1)*cos(r) - acc(2)*cos(p)*sin(r) + acc(0)*sin(r)*sin(p), 
            -acc(0)*cos(r)*cos(p) - acc(2)*cos(r)*sin(p), 
            0;

    
    MatrixXd At = MatrixXd::Zero(15,15);
    At.block<3,3>(0,6) = MatrixXd::Identity(3,3);
    At.block<3,3>(3,3) << At_angular;
    At.block<3,3>(6,3) << R_dot;
    At.block<3,3>(3,9) << - G_inv;
    At.block<3,3>(6,12) << -R;

    MatrixXd Ut = MatrixXd::Zero(15,12);
    Ut.block<3,3>(3,0) << -G_inv;
    Ut.block<3,3>(6,3) << -R;
    Ut.block<6,6>(9,6) << MatrixXd::Identity(6,6);

    MatrixXd Ft = MatrixXd::Identity(15,15) + dt*At;
    Ft += dt * At;
    MatrixXd Vt = dt * Ut;

    cov_mat = Ft * cov_mat * Ft.transpose() + Vt * Q * Vt.transpose();
    
    VectorXd f = VectorXd::Zero(15);
    f.segment<3>(0) = est.segment<3>(6);
    f.segment<3>(3) = G_inv * (gyro - est.segment<3>(9));
    f.segment<3>(6) = gravity + R * (acc - est.segment<3>(12));
    
    //estate update
    est += dt*f;
    //normalizing angle
    for (int i = 3; i <= 5; ++i) {
        est(i) = atan2(sin(est(i)), cos(est(i)));
    }
    
    //time update
    end_t = msg->header.stamp.toSec();
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;

    
    Vector3d world_to_cam, cam_to_imu, imu_to_world;
    Matrix3d cam_rot_world, imu_rot_cam, world_rot_imu;
    Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                     msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    world_to_cam << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    cam_rot_world = quat.toRotationMatrix();
    
    // IMU to camera transform
    cam_to_imu << 0.05, 0.05, 0;
    imu_rot_cam << 1,  0,  0,
                   0, -1,  0,
                   0,  0, -1;
    
    // Calculate world to IMU transform
    world_rot_imu = cam_rot_world.transpose() * imu_rot_cam.transpose();
    imu_to_world = -world_rot_imu * cam_to_imu - cam_rot_world.transpose() * world_to_cam;
    
    // Convert rotation matrix to Euler angles
    double roll = asin(world_rot_imu(2, 1));
    double pitch = atan2(-world_rot_imu(2, 0)/cos(roll), world_rot_imu(2, 2)/cos(roll));
    double yaw = atan2(-world_rot_imu(0, 1)/cos(roll), world_rot_imu(1, 1)/cos(roll));
    
    // EKF measurement update
    VectorXd measurement = VectorXd::Zero(6);
    VectorXd innovation = VectorXd::Zero(6);
    MatrixXd observation_mat = MatrixXd::Zero(6,15);
    MatrixXd kalman_gain, kalman_temp;
    
    measurement << imu_to_world(0), imu_to_world(1), imu_to_world(2), roll, pitch, yaw;
    observation_mat.block(0,0,6,6) = MatrixXd::Identity(6,6);
    
    innovation = measurement - observation_mat * est;
    // Normalize angles
    for(int i=3; i<=5; i++) {
        innovation(i) = atan2(sin(innovation(i)), cos(innovation(i)));
    }
    
    // Calculate Kalman gain (two methods)
    kalman_temp = cov_mat * observation_mat.transpose() * 
                 (observation_mat * cov_mat * observation_mat.transpose() + Rt).inverse();
    kalman_gain = (observation_mat * cov_mat * observation_mat.transpose() + Rt).lu()
                 .solve(observation_mat * cov_mat).transpose();
    
    // Update state and covariance
    est += kalman_gain * innovation;
    cov_mat -= kalman_gain * observation_mat * cov_mat;
    
    // Publish odometry
    Matrix3d ekf_rot;
    double r = est(3), p = est(4), y = est(5);
    ekf_rot << cos(y)*cos(p)-sin(r)*sin(y)*sin(p), -cos(r)*sin(y), cos(y)*sin(p)+cos(p)*sin(r)*sin(y),
               cos(p)*sin(y)+cos(y)*sin(r)*sin(p),  cos(r)*cos(y), sin(y)*sin(p)-cos(y)*sin(r)*cos(p),
              -cos(r)*sin(p), sin(r), cos(r)*cos(p);
    
    Quaterniond ekf_quat(ekf_rot);
    nav_msgs::Odometry ekf_odom;
    ekf_odom.header.stamp = msg->header.stamp;
    ekf_odom.header.frame_id = "world";
    ekf_odom.pose.pose.position.x = est(0);
    ekf_odom.pose.pose.position.y = est(1);
    ekf_odom.pose.pose.position.z = est(2);
    ekf_odom.twist.twist.linear.x = est(6);
    ekf_odom.twist.twist.linear.y = est(7);
    ekf_odom.twist.twist.linear.z = est(8);
    ekf_odom.pose.pose.orientation.w = ekf_quat.w();
    ekf_odom.pose.pose.orientation.x = ekf_quat.x();
    ekf_odom.pose.pose.orientation.y = ekf_quat.y();
    ekf_odom.pose.pose.orientation.z = ekf_quat.z();
    odom_pub.publish(ekf_odom);
    
    updated = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.1 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.04 * Q.bottomRightCorner(6, 6);
    Rt = 0.01*MatrixXd::Identity(6,6);
    // Rt.topLeftCorner(3, 3) = 0.005 * Rt.topLeftCorner(3, 3);
    // Rt.bottomRightCorner(3, 3) = 0.005 * Rt.bottomRightCorner(3, 3);
    // Rt.bottomRightCorner(1, 1) = 0.005 * Rt.bottomRightCorner(1, 1);
    cout << "Rt" << endl << Rt << endl;
    cout << "Q" << endl << Q << endl;
    ros::spin();
}