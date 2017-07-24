The package achieves visual slam using only monocular camera, the base strcuture refers to the package 'ML_in_Robotics' and 'Structure from Motion'. 

# Src folder clarification
The folder 'feature_initial' contains codes to achieve robot motion model and feature detection, macthing and tracking, corresponding to three demo codes.
1. 'demo_prediction.py' is aimed to implement prediction step / motion model of robot. It will generate four data files after running it.
2. 'demo_orb_feature_match.py' is aimed to implement feature detection and matching. Besides, the file also contains 'RANSAC', 'Color Contrast' and some help functions.
3. 'demo_tracking.py' is aimed to implement feature tracking via KLT algorithm. When running this file, you are supposed to see a video in the screen showing the tracking process. 

The folder 'code_visual_update' contains code files to achieve visual update, such as linear / nonlinear triangulation, linear / nonlinear PnP and bindle adjustment.

# Package Execution
Please execute the main file, which is 'demo_visual_update.py'. Each time a figure pops up to indicate a completed step. Close it to proceed. 

# Results
This folder contains final results of estimated trajectory and 3D point cloud. And you can also take a look at the report which includes detailed algorithm explanations.
