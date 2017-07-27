# Visual_SLAM
Based on the collected IMU and odometry data from the real ground robot, implemented UKF to estimate a rough pose and trajectory first, then focused on feature detection and matching, using visual approaches such as linear / nonlinear Triangulation, PnP and Bundle Adjustment to update and optimize the robot pose, trajectory, and feature point cloud as well.

Strcuture from Motion
---------------------
The package implements feature matching and visual optimization algorithms such as linear and nonliear triangulation, PnP and bundle adjustment,
to verify the fesibility and accuracy of the visual slam algorithm when the feature detection result is good. The package plays an important role for the following *Visual Slam* package.

Visual SLAM 3D
--------------
The package implements visual slam using the monocular camera, and built a 3D feature point-cloud map as well as showing the walking robot trajectory.
