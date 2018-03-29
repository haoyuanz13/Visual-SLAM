# Structure From Motion (3D Structure Reconstruction)
The package mainly implemented the **_Epipolar Geometry_** algorithm, including the fundamental and essential matrices estimation in order to find the relative pose configuration, linear and nonlinear Triangulation, PnP.      

Finally, using Bundle Adjustment and completed the **_Structure From Motion_** pipeline such that can reconstruct a 3D environment.

Introduction
------------
The _Structure From Motion(SFM)_ is popular used in most Visual SLAM and AR algorithm. The basic idea is, given a series of camera frames and using corresponding features (e.g. SIFT, ORB, BRIEF), we can reconstruct a real 3D environment as well as estimate the pose/trajectory of those cameras.       

Typically, SFM is a nonlinear optimization problem, trying to minimize the **_reprojection error_** from 3D point cloud and corresponding 2D features. In addition, its vision pipeline is also be popular used in the update/measurement step of the modern Vision Odometry/Vision Inertial Odometry algorithm.

http://ieeexplore.ieee.org/document/6619046/

Data
----
All data for 3D reconstruction are stored in the folder **_data_**, including:
* _Images_: a series of images captured by the camera and can guarantee each of them have some common regions.
* _matching txt file_: contains correspondeces information for each image frame.

Also, feel free to detect your own features if those are not enough or optimal to you.


Algorithms
----------
The pipeline is straightforward, below shows the pipeline.
<div align=center>
  <img width="560" height="420" src="./sfm_pipeline.png", alt="sfm pipeline"/>
</div>   

This package is mainly designed to check the feasibility and accracy of visual SLAM algorithm once the feature detection is good. Add the package to your catkin workspace as well as its dependencies, execute the 'demo.m' in src folder to check the accuracy of the visual SLAM.

Package clarification
------------------------
The src folder includes two parts code set, one part completes feature matching, cleaning and updating for optimization problem, the other part implements visual optimization problems such as linear and nonlinear triangulaion, PnP and bundle adjustment. For nonlinear problem, in addition, you can find all Jacobian matrix implementation to improve the speed and accuracy of algorithm.

Results
-------
Two result folders, one shows the accuracy and function of feafure detection and matching, as well as fundamental matrix estimation; the other shows the 3D point cloud map construction as well as robot trajectory with camera pose. 

Report
------
For more algorithm explanation, please feel free to take a look at the report.
