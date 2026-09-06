# Drone Perception & State Estimation

ROS/C++ project covering camera pose estimation, stereo visual odometry and IMU/vision sensor fusion with an Extended Kalman Filter.

The project was developed in several stages, starting with vision-based pose estimation and ending with an experimental integration of IMU, fiducial markers and stereo visual odometry.

![ROS](https://img.shields.io/badge/ROS-1-22314E?logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?logo=cplusplus&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?logo=opencv&logoColor=white)
![Eigen](https://img.shields.io/badge/Eigen-linear%20algebra-555555)

<p align="center">
  <img src="media/stereo_vo/trajectory_rviz.png" width="750">
</p>

## Overview

The project progressed through four main parts:

**Planar pose estimation → Stereo visual odometry → IMU + vision EKF → Augmented EKF integration**

The main topics I worked with were computer vision, feature tracking, PnP, coordinate transformations, state estimation, multi-rate sensor fusion and ROS visualization.

## Planar Camera Pose Estimation

The first stage estimated the camera pose from a planar fiducial board.

I implemented a pose estimation method based on:

- 2D-3D marker correspondences
- planar homography estimation using DLT
- SVD
- camera intrinsic calibration
- homography decomposition into rotation and translation
- rotation orthogonalization using SVD

The result was compared with an OpenCV `solvePnP` implementation included in the project as a reference.

## Stereo Visual Odometry

The next stage was a stereo visual odometry pipeline implemented on top of a provided ROS skeleton.

The pipeline uses:

```text
Stereo images
    ↓
Shi-Tomasi feature detection
    ↓
Lucas-Kanade stereo tracking
    ↓
Fundamental matrix / RANSAC filtering
    ↓
3D feature points
    ↓
Temporal feature tracking
    ↓
PnP-RANSAC
    ↓
Relative pose and trajectory
