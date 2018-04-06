# Visual SLAM
Based on the collected IMU and odometry data from the real ground robot, implemented UKF to estimate a rough pose and trajectory first, then focused on feature detection and matching, using visual approaches such as linear / nonlinear Triangulation, PnP and Bundle Adjustment to update and optimize the robot pose, trajectory, and feature point cloud as well.


* **_Trajectory Generation_**
The package mainly implements the trajectory planning and generation algorithm for the real Quadrotor given the map environment, start and goal positions, obstacles and flight state constraints information.

* **_Camera Calibration_**
The package mainly contains algorithms for the camera calibration either for a hand-handled camera, and later will apply that to a monocular camera equipped on a real Quadrotor, using a April Tags space.


* **_Strcuture from Motion_**
The package implements feature matching and visual optimization algorithms such as linear and nonliear triangulation, PnP and bundle adjustment,
to verify the fesibility and accuracy of the visual slam algorithm when the feature detection result is good. The package plays an important role for the following *Visual Slam* package.


* **_Visual SLAM 3D_**
The package implements visual slam using the monocular camera, and built a 3D feature point-cloud map as well as showing the walking robot trajectory.
