# Pose Estimation and Camera Calibration
In this package, we mainly implement the **Camera Calibration** algorithm for both hand-handle camera and the monocular camera equipped on a real Quadrotor.

Hand-handled Camera Calibration
-------------------------------
We aim to estimate the intrinsic (K) and extrinsic (R, t) matrices for a hand-handled camera using a planer world space(e.g. checkerboard) and multiple views, which is also called as _camera calibration_. 

* _**Data Collection**_           
For the data collection, simply capture multiple images of a planar checkerboard pattern (known 3D points). More specifically, the pipeline shows below:
  *  _Get a checkerboard_         
  You can use a physical checkerboard if you can find one. Alternatively, print out the predeﬁned checkerboard pattern (use open checkerboardPattern.pdf to show the pattern in MATLAB). Note that the checkerboard pattern is made of odd number of squares in one side and even number of squares on the other side to disambiguate the orientation of the pattern.       
  * _Measure Size_    
  Measure the size of your own checkerboard square in mm (just a recommended unit) such that you can obtain the 3D coordinate for every corner in the checkerboard.    
  * _Images Capture_    
  Capture multiple imgaes of the checkerboard using your target camera, 10-20 images are recommended for the more robust calibration. In addtion, below tips might be helpful.     
    1. Disable auto-focus and **DO NOT** change the zoom settings while capturing.     
    2. **DO NOT** modify your images after capturing, e.g., cropping, resizing, or rotating images.    
    3. Capture the pattern from diﬀerent angles with a suﬃciently large baseline and try to cover most parts of the images with the pattern. 
    4. **DO NOT** use a camera lens with large distortion such as a GoPro camera.                    

  Good-qualified data can do a lot of favor for the calibration, and Matlab also provides awesome instructions in this part, which you can check out for reference [here](http://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html#bt19jdq-1).

* _**Algorithms**_      
Typically, both intrinsic matrix _K_ and extrinsic matrix _[R, t]_ are estimated linearly first, and then refined by nonlinear optimization minimizing the geometric reprojection error. Overall, we estimate those parameters:
  * _Intrinsic matrix K_: fx, fy (focal length); s (slant factor); Px, Py (principle points).
  * _Extrinsic matrix [R, t]_: R (rotation matrix); t (translation vector).
  * _Distortion parameters k_: k1; k2.
  
   we linearly compute camera parameters, K, R, and t assuming no lens distortion and then, estimate lens parameters, k1 and k2, sequentially.         
   
   Given initial values from the linear estimator, then we refine those estimations via minimizing the geometric reprojection error. We use the projection relationship between known the 3D points on the checkerboard pattern and the corresponding 2D points in each image. The 2D points are detected by a corner detector in _InitCalibration.m_ script. The script also provides the association between the 2D and 3D points. Note that we assume the 3D points are located at z = 0, a planer world.


* _**Execution**_       
All source codes with captured images are stored in the folder, **src_camCalib**, feel free to execute the main file **demo.m** such that you are supposed to see the iterative estimation process running and final calibration results.
