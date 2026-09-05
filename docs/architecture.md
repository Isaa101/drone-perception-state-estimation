# Technical architecture notes

This document records details that are useful for code review but would make the top-level README too dense. All descriptions are based on the surviving source and supplied project documents; missing workspace pieces are not reconstructed.

## 1. Planar fiducial pose

### Data flow

```text
ArUco marker detections
  -> marker corner 2D measurements
  -> board geometry 3D points (all z = 0)
  -> planar DLT matrix A
  -> SVD null-space vector
  -> homography H
  -> K^-1 H decomposition
  -> rotation orthogonalization by SVD
  -> translation scaling
  -> ROS odometry publication
```

The source obtains 3D corners from marker-board indices and pairs them with 2D detected corners. Because every recovered board point lies on `z = 0`, the student-work block constructs a 9-parameter planar homography system rather than a general 3D DLT system.

The homography is decomposed by applying `K^-1`, taking the first two columns as rotation directions and their cross product as the third direction. A second SVD computes `R = U V^T`, which acts as an orthogonal projection. Translation is taken from the third column and divided by the norm of the first column. A sign check forces positive `T_z`.

### Validation path

The same callback executes OpenCV `solvePnP` as a reference and logs:

- `||R_est - R_ref||` (matrix/Frobenius norm in Eigen),
- `||T_est - T_ref||` (Euclidean norm).

This is a consistency comparison against a library reference, not a ground-truth accuracy evaluation.

### Post-recovery code-review observation

The variable `frame` is incremented once before the DLT implementation and again before the average-error log. As preserved, the reported "Average R/T error" denominator therefore grows twice per processed call. The repository does **not** fix this; it treats the running average as questionable.

A second possible edge case is that `R = U V^T` is not followed by an explicit determinant correction. The surviving results are insufficient to show whether an improper rotation ever occurred.

## 2. Stereo visual odometry

### Course-provided versus completed functionality

The stereo assignment explicitly says the ROS package was a skeleton. It requires completion of the main `inputImage` procedure, new-feature extraction, left/right tracking, keyframe/current tracking, PnP relative pose estimation and latest-state update.

The same assignment explicitly states that:

- `generate3dPoints` was already provided,
- `undistortedPts` was already provided.

Those functions remain in the recovered file because the entire `.cpp` survived, but they are not presented as independently implemented from scratch.

### Recovered processing pipeline

1. If the estimator has already initialized, track keyframe features into the current left image.
2. Geometrically filter temporal matches using a fundamental matrix estimated by RANSAC.
3. Estimate keyframe-to-current relative pose with PnP-RANSAC.
4. Detect a fresh set of left-image features with `goodFeaturesToTrack`.
5. Track those features into the right image with pyramidal LK optical flow.
6. Undistort/normalize the stereo correspondences.
7. Geometrically filter the stereo matches with a fundamental matrix.
8. Generate 3D points from the filtered stereo matches.
9. Accumulate the relative transform into the world trajectory.
10. Select a new keyframe when motion/feature-count conditions are met.
11. Update the latest pose and point cloud for ROS publication.

### Verified implementation parameters

From the source:

- Shi-Tomasi maximum features: `180`
- quality level: `0.015`
- minimum distance: `9.0`
- block size: `5`
- stereo LK window: `27 x 27`, max pyramid level argument `2`
- stereo LK termination: 25 iterations or epsilon 0.02
- temporal LK window: `19 x 19`, max pyramid level argument `2`
- fundamental-matrix RANSAC threshold: `0.01`, confidence `0.99` on normalized points
- PnP-RANSAC iterations: `150`
- PnP-RANSAC reprojection threshold argument: `3.0`
- PnP-RANSAC confidence: `0.98`

The PnP implementation first converts the current image points to normalized coordinates and passes an identity camera matrix. Therefore the `3.0` threshold is applied in the coordinate system of those normalized measurements; describing it as a verified "3-pixel" threshold would overstate what the code supports.

The report records keyframe settings of 0.1 m, 0.1 rad and 20 features, while the source reads those values from the missing YAML through `TRANSLATION_THRESHOLD`, `ROTATION_THRESHOLD` and `FEATURE_THRESHOLD`.

### Frame accumulation and ROS output

The relative pose returned by PnP is inverted before accumulation:

```text
relative_R = c_R_k^T
relative_t = -relative_R * c_t_k
world_R_current = world_R_key * relative_R
world_t_current = world_t_key + world_R_key * relative_t
```

`updateLatestStates` then applies camera/body extrinsics so the published latest pose corresponds to the body/IMU convention used by the project. The recovered ROS node publishes odometry, path and a transformed point cloud.

### Post-recovery code-review observations

These are observations, not retroactive fixes:

- `geometricVerification` returns an empty mask when there are 8 or fewer points, while `reduceVector` later indexes the mask without an empty-mask guard. Low feature counts could therefore cause invalid access.
- The return value of `solvePnPRansac` is not checked before `rvec/tvec` are converted.
- The recovered ROS node publishes a later `relative_pose` message using estimator members not assigned in the recovered estimator `.cpp`, which suggests a missing file or revision mismatch.

## 3. 15-state IMU + vision EKF

### State

```text
x = [ p(3), euler(3), v(3), b_g(3), b_a(3) ]^T
```

The source uses roll/pitch/yaw kinematics with a matrix `G`, a body-to-world rotation matrix `R`, a gravity vector, gyroscope bias and accelerometer bias.

### Prediction

The IMU callback computes:

```text
p_dot      = v
euler_dot  = G^-1 (gyro - b_g)
v_dot      = gravity + R (accel - b_a)
b_g_dot    = 0   (noise represented through process covariance)
b_a_dot    = 0   (noise represented through process covariance)
```

It builds a 15x15 state Jacobian, a 15x12 noise Jacobian and propagates covariance. The assignment specifies IMU at 400 Hz and image pose at 20 Hz.

### Camera/tag measurement update

The tag detector provides a camera pose convention that must be transformed into the IMU/world convention. The fixed camera-to-IMU translation in the recovered source is `(0.05, 0.05, 0)`, with rotation `diag(1,-1,-1)`. The assignment states this relative camera/IMU transform was provided as project infrastructure.

The update measurement is the first six states (position + Euler orientation). Angular residuals are wrapped with `atan2(sin(theta), cos(theta))` before applying the Kalman gain.

### Recovered covariance values

The source initializes:

- state covariance `P0 = 0.5 I`,
- measurement covariance `R = 0.01 I`,
- process-noise covariance with the first 6 diagonal entries scaled to `0.1` and the last 6 to `0.04`.

These are preserved implementation values, not claimed optimal tuning.

### Post-recovery code-review observation

The source constructs:

```cpp
MatrixXd Ft = MatrixXd::Identity(15,15) + dt*At;
Ft += dt * At;
```

so the covariance state transition used is effectively `I + 2 dt A` rather than the more usual first-order `I + dt A`. This is preserved unchanged and is one reason the result plots should not be interpreted as a quantitative accuracy validation.

## 4. 21-state augmented EKF attempt

### State

The model code keeps the original 15-state current state and the filter adds a 6-state keyframe pose:

```text
x_aug = [
  current position(3),
  current Euler(3),
  current velocity(3),
  gyro bias(3),
  accel bias(3),
  keyframe position(3),
  keyframe Euler(3)
]^T
```

The PnP measurement model observes current absolute position/orientation. The VO measurement model is intended to predict the relative position and orientation between the augmented keyframe pose and current pose.

### Asynchronous history

The filter stores `AugState` objects in a deque. Each state is typed as IMU, PnP, VO or keyframe. New measurements are inserted by timestamp, and the code attempts to repropagate subsequent states after an out-of-order update. This is the integration mechanism for different sensor rates and timestamps.

### Augmentation

`changeAugmentedState` uses two matrices to retain the 15-state current state and copy the current 6D pose into the augmented keyframe portion. This is the intended bridge between absolute state estimation and relative VO measurements.

The actual integration did not validate reliably; see [failure_analysis.md](failure_analysis.md).
