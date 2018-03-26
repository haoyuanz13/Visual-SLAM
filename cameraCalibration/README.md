# Pose Estimation and Camera Calibration
In this package, we mainly implement the **Camera Calibration** algorithm for both hand-handle camera and the monocular camera equipped on a real Quadrotor.

Hand-handled Camera Calibration
-------------------------------
We aim to estimate the intrinsic (K) and extrinsic (R, t) matrices for a hand-handled camera using a planer world space(e.g. checkerboard) and multiple views, which is also called as _camera calibration_. 

* _Data Collection_
For the data collection, simply capture multiple images of a planar checkerboard pattern (known 3D points). More specifically, the pipeline shows below:
  * a
