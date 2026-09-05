# Source recovery and provenance

This repository is an archival engineering case study assembled from the surviving files of a university aerial-robotics project. It is intentionally transparent about missing infrastructure and mixed code provenance.

## What was recovered

Eight unique C++ source files survived. No exact duplicate C++ files were present in the supplied recovery set.

| Repository path | Recovered filename | What it contains | Provenance / authorship boundary |
|---|---|---|---|
| `src/planar_pose_estimation/planar_pose_estimator.cpp` | `tag_detector_node_p2p1(1).cpp` | ArUco/ROS node, OpenCV reference PnP path, planar homography/DLT student-work path | The file itself labels `solvePnP` as **"version 1, as reference"** and the homography path as **"version 2, your work"**. The surviving material does not establish authorship of all surrounding detector/ROS infrastructure. |
| `src/stereo_visual_odometry/stereo_vo_estimator.cpp` | `estimator_p2p2(1).cpp` | Stereo VO processing, feature detection/tracking, geometric verification, PnP, keyframes, 3D point code | Mixed course skeleton and completed code. The assignment explicitly says `generate3dPoints` and `undistortedPts` were already provided. |
| `src/stereo_visual_odometry/stereo_vo_parameters.cpp` | `parameters_p2p2(1).cpp` | Loading topics, camera extrinsics, thresholds and calibration paths from YAML | Supporting skeleton/infrastructure; no independent authorship claim is made. |
| `src/stereo_visual_odometry/stereo_vo_node.cpp` | `stereo_vo_node_p2p2(1).cpp` | ROS image callback and odometry/path/point-cloud/relative-pose publication | Supporting/integration code. The usage string references a later phase and the node publishes a relative-pose message, suggesting this file may reflect a later workspace revision. |
| `src/imu_vision_ekf/imu_vision_ekf_node.cpp` | `ekf_node_p3p1(1).cpp` | 15-state EKF prediction/correction and ROS odometry publication | The assignment says this was an **individual project** inside a provided `ekf` skeleton. The repository therefore describes it as an individual implementation/completion, not a package written entirely from scratch. |
| `src/experimental_augmented_ekf/augmented_ekf_filter.cpp` | `ekf_filter_p3p2(1).cpp` | 21-state history, PnP/VO/IMU callbacks, updates, augmentation/repropagation | Experimental later-stage integration. Skeleton TODOs remain. The final report lists two team contributors; per-file authorship cannot be reconstructed. |
| `src/experimental_augmented_ekf/augmented_ekf_model.cpp` | `ekf_model_p3p2(1).cpp` | 15-state motion model plus PnP and relative-VO measurement models/Jacobians | Experimental later-stage integration; per-file authorship not asserted. |
| `src/experimental_augmented_ekf/augmented_ekf_node.cpp` | `ekf_node_p3p2_aug(1).cpp` | ROS entry point for the augmented filter | Thin integration entry point; per-file authorship not asserted. |

Each recovered source file has only a short archival header added at the top. Filenames were normalized for readability; the implementation logic was not silently corrected.

## Source material used but intentionally not included

The recovery set also contained:

- the stereo visual odometry assignment,
- the IMU/vision EKF assignment,
- the stereo VO report,
- the 15-state EKF report,
- the later augmented-EKF integration report.

The original PDFs are **not included** in this repository. Technical requirements and useful result images were extracted from them, while assignment pages and personally identifying student numbers were excluded.

## What is missing

The surviving files are not a complete ROS workspace. Missing components include, at minimum:

- C++ headers such as `estimator.h`, `parameters.h`, `stereo_vo.h`, `ekf_filter.h` and `ekf_model.h`,
- `CMakeLists.txt`, `package.xml` and the original catkin workspace structure,
- launch files and RViz configurations,
- YAML parameter files and camera calibration files,
- marker-board configuration,
- the custom `stereo_vo::relative_pose` message definition,
- rosbag datasets and the laboratory bag,
- any additional course-provided helper code not embedded in the recovered `.cpp` files.

The node source also references relative-pose fields (`rel_key_time`, `latest_rel_Q`, `latest_rel_P`) that are not assigned in the recovered estimator `.cpp`. This may indicate missing header/implementation logic or that the surviving stereo files came from different workspace revisions. It is one reason this repository is not presented as reproducible.

## Why no replacement build files were generated

Creating guessed headers, launch files, YAML, message definitions or build metadata would make the project look more complete than the evidence supports and could accidentally change the intended interfaces. This repository therefore preserves the source as evidence and documents the missing dependencies instead of fabricating a runnable package.

## Duplicate handling

No byte-identical duplicate C++ files were found in the supplied recovery set. Each unique source file is included once under a role-based filename.

## Reports and visual evidence

Only embedded result images were extracted from the reports. The repository does not include assignment-instruction screenshots or full report pages. The extracted images used here contain no student-number header text.
