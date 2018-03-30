# Visual SLAM using Monocular Camera (Particle Filter Based)

Usage
-----
The package achieves Visual Slam and 3D map building using only monocular camera, the base strcuture refers to two packages: 
1. *ML_in_Robotics* package provides the basic structure of feature detection, matching and tracking, UKF algorithm and 2D SLAM.
2. *Structure from Motion* package supports the well-work visual optimization methods including linear / nonlinear Triangulation, PnP and Bundle Adjustment to update robot and point cloud states. 

Package clarification
----------------------
The source folder 'feature_initial' contains codes to achieve robot motion model and feature detection, macthing and tracking, corresponding to three demo codes.
1. 'demo_prediction.py' is aimed to implement prediction step / motion model of robot. It will generate four data files after running it.
2. 'demo_orb_feature_match.py' is aimed to implement feature detection and matching. Besides, the file also contains 'RANSAC', 'Color Contrast' and some help functions.
3. 'demo_tracking.py' is aimed to implement feature tracking via KLT algorithm. When running this file, you are supposed to see a video in the screen showing the tracking process. 

The folder 'code_visual_update' contains code files to achieve visual update, such as linear / nonlinear triangulation, linear / nonlinear PnP and bindle adjustment.

Package Execution
-----------------
Please execute the main file, which is 'demo_visual_update.py'. Each time a figure pops up to indicate a completed step. Close it to proceed. 

Results
-------
This folder contains final results of estimated trajectory and 3D point cloud. And you can also take a look at the report which includes detailed algorithm explanations.
