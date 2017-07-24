In the zip file, there contains three folders.

A. Folder 'code_feature_initial'
This folder contains code files to achieve robot motion model and feature detection, macthing and tracking, corresponding to three demo codes.
1. 'demo_prediction.py' is aimed to implement prediction step / motion model of robot. It will generate four data files after running it.
2. 'demo_orb_feature_match.py' is aimed to implement feature detection and matching. Besides, the file also contains 'RANSAC', 'Color Contrast' and some help functions.
3. 'demo_tracking.py' is aimed to implement feature tracking via KLT algorithm. When running this file, you are supposed to see a video in the screen showing the tracking process. 

The 'data' folder contains all data we will use in this project.

B. Folder 'code_visual_update'
This folder contains code files to achieve visual update, such as linear / nonlinear triangulation, linear / nonlinear PnP and bindle adjustment.
Just need to run the main file, which is 'demo_visual_update.py'. Each time a figure pops up to indicate a completed step. Close it to proceed. 

C. Folder 'Results'
This folder contains final results of estimated trajectory and 3D point cloud. The ground truth is the data set 3 in Project 3 SLAM.

Thanks for your time!
