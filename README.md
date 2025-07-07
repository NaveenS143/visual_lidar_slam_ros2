# visual_lidar_slam_ros2

🧭 Project Summary

This project implements a Visual-LiDAR SLAM system that fuses information from stereo camera images and 3D LiDAR point clouds to perform Simultaneous Localization and Mapping (SLAM). The system is designed to estimate the 6-DoF pose of a moving vehicle while constructing a map of its surrounding environment, using data from the KITTI Odometry Dataset.

The project aims to mimic a real-world autonomous perception pipeline by integrating computer vision techniques (visual odometry) with geometric point cloud registration (LiDAR odometry), followed by sensor fusion and back-end optimization. The final output is a globally consistent trajectory and a sparse 3D map.

🎯 Project Goals

    ✅ Implement Visual Odometry (VO)
    Extract and track visual features from stereo images to estimate relative motion.

    ✅ Implement LiDAR Odometry (LO)
    Align consecutive LiDAR scans using ICP or NDT for pose estimation.

    ✅ Perform Visual-LiDAR Sensor Fusion
    Combine visual and LiDAR odometry to improve robustness and accuracy.

    ✅ Build a Consistent 3D Map
    Aggregate LiDAR scans into a global point cloud or voxel map.

    ✅ Apply Back-End Optimization
    Use pose graph optimization (e.g., GTSAM) and loop closure detection to reduce accumulated drift.

    ✅ Evaluate on KITTI Odometry Benchmark
    Compare estimated trajectories against KITTI ground truth for quantitative evaluation.

    ✅ Design for Modularity and Extensibility
    Build the system in separate components for VO, LO, mapping, and optimization to allow future improvements (e.g., adding IMU or semantic SLAM).

🗓️ Week 1: Setup + Sensor Integration

✅ Tasks:

    Simulate LiDAR + RGB camera in Gazebo (Velodyne + RealSense)

    ROS 2 topics: /camera/image_raw, /velodyne_points

    Visualize both in RViz2

    Package setup: ros2_ws/src/visual_lidar_slam

✅ Deliverables:

    GitHub repo init

    RViz screenshot of fused data

    LinkedIn post: “Day 1: Building Visual-LiDAR SLAM System with ROS 2”

🗓️ Week 2: SLAM Core Implementation

✅ Tasks:

    Implement feature extraction from images (OpenCV ORB or GFTT)

    Extract keypoints from LiDAR (downsampling + clustering)

    Temporal association of frames (IMU/GPS optional)

    Apply ICP or NDT to align point clouds + estimate motion

✅ Deliverables:

    GitHub commit: SLAM pipeline node

    Video demo of odometry + map update

    LinkedIn post: “Fusing LiDAR & Camera for Real-Time SLAM”

🗓️ Week 3: Optimization + Deployment

✅ Tasks:

    Loop closure with g2o or Ceres (optional but impactful)

    Pose graph optimization

    Launch files for real-time testing

    Test on actual bag files or robot platform

✅ Deliverables:

    Final code with README + launch instructions

    RViz demo of full SLAM map

    LinkedIn post: “I Built a Visual-LiDAR SLAM System in ROS 2”
