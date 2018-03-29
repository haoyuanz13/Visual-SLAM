# Structure From Motion (3D Structure Reconstruction)

Usage
-----
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
