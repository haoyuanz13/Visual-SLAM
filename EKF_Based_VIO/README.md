# EKF based Visual-Inertial-Odometry
This package implements the Visual Inertial Odometry to estimate the states of a Quadrotor, including its global position [x, y, z], pose [roll, pitch, yaw] and linear velocity with respected to the world [vx, vy, vz]. 


Experiment Environment
----------------------
All practical experiments and data collections are completed in an indoor environment (GRASP Lab), moreover, in order to detect good features, we use _April Tags Map_ on the floor to help the estimation.       

Below shows the map, since we know the size of each Tag and the arranged position, such that we can obtain the 3D position of each corner.
<div align=center>
  <img width="500" height="600" src="./docs/AprilTagsMap.png", alt="April Tags map">
</div>

In addition, for each AprilTag, we define its four corner order as below, which can also help to pre-process data for the vision-based update step.
<div align=center>
  <img width="200" height="200" src="./docs/AprilTag.png", alt="April Tag">
</div>


Data
----
All data are collected via a Quadrotor onboard sensor, including time, body frame linear acceleration and velocity, captured images. The format of the sensor data struct is as follows:
  1. Time stamp (**t**) in seconds.
  2. The ID of every AprilTag that is observed in the image (**id**).
  3. All detected corners and Tags' center positions in the image frame (**p0**, **p1**, **p2**, **p3**, **p4**).
  4. Rectified image (**img**).
  5. IMU data with Euler angles (**rpy**), body frame angular velocity (**omg**) and linear acceelration (**acc**).

The data file also contains the **Vicon** data taken at 100Hz, which serve as the ground truth measurements, its format is:
<div align=center>
  <img width="200" height="20" src="./docs/viconData.png", alt="vicon">
</div>

Note that for some packets no tags or valid features are observed, such that we cannot implement the vision-based estimation for update. However, as the IMU runs at a faster rate than the camera, the EKF will allow us to have somewhat accurate state estimates in these situations.


Algorithms
----------
The algorithm mainly refers to the _Extended Kalman Filter (EKF)_, use observed data (e.g. body frame acceleration or angular velocity from the onboard IMU) to complete the state prediction, and then use detected visual features (e.g. April Tags) to update. 
