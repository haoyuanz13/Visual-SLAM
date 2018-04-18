function [pos, q, R_world2cam, t_cam2world] = estimate_pose(sensor, input)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k

%% check whether there exists detected tag
tag_num = size(sensor.id, 2);
if tag_num <= 0
  pos = [];
  q = [];
  R_world2cam = [];
  t_cam2world = [];

%% detected valid tags
else 
  detected_id = sensor.id' + 1;
  K = input{1}.K; 
  
  %% obtain detected tags 3D coordinates 
  tags_3D_X = input{1}.tag3D_X;
  tags_3D_Y = input{1}.tag3D_Y;

  p0_3d = [tags_3D_X(detected_id(:), 1) tags_3D_Y(detected_id(:), 1)]; 
  p1_3d = [tags_3D_X(detected_id(:), 2) tags_3D_Y(detected_id(:), 2)]; 
  p2_3d = [tags_3D_X(detected_id(:), 3) tags_3D_Y(detected_id(:), 3)]; 
  p3_3d = [tags_3D_X(detected_id(:), 4) tags_3D_Y(detected_id(:), 4)]; 
  p4_3d = [tags_3D_X(detected_id(:), 5) tags_3D_Y(detected_id(:), 5)];

  X_3d = [p0_3d(:, 1); p1_3d(:, 1); p2_3d(:, 1); p3_3d(:, 1); p4_3d(:, 1)];
  Y_3d = [p0_3d(:, 2); p1_3d(:, 2); p2_3d(:, 2); p3_3d(:, 2); p4_3d(:, 2)];

  %% obtain corresponding coordinates in 2D image
  u_im = [sensor.p0(1, :)'; sensor.p1(1, :)'; sensor.p2(1, :)'; sensor.p3(1, :)'; sensor.p4(1, :)'];
  v_im = [sensor.p0(2, :)'; sensor.p1(2, :)'; sensor.p2(2, :)'; sensor.p3(2, :)'; sensor.p4(2, :)'];
 
  % calculate homography: transform from 3d world to camera
  H_world2cam = EstimateHomography(u_im, v_im, X_3d, Y_3d);
  
  % pose estimation
  [R_world2cam, t_cam2world] = EstimateRt_linear(H_world2cam, K, input{1}.linearEstimator);
  

  R_world2cam_opt = R_world2cam;
  t_cam2world_opt = t_cam2world;

  %% build homography that transforms from world to imu
  H_cam2world = [R_world2cam_opt' -(R_world2cam_opt)' * t_cam2world_opt; 
                 0, 0, 0, 1];

  H_imu2cam = input{1}.H_i2c;
  H_imu2world = H_cam2world * H_imu2cam;  % pre product

  %% obtain pose of robot based on the homography
  R_imu2world = H_imu2world(1 : 3, 1 : 3);
  q = rotm2quat(R_imu2world)';  % transform into quaternion format 4x1

  pos = H_imu2world(1 : 3, 4);  % estimated translation 3x1
end

end
