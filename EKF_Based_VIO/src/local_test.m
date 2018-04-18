%% demo code to test the accuracy of designed pose estimation
%% initial environment and read sensor
close all;
clear all;
addpath('.\');
addpath('.\utils');
addpath('.\data');

%% create variables for the input argument
% camera intrinsic matrix K
K = [311.0520, 0, 201.8724;
     0, 311.3885, 113.6210;
     0, 0, 1];

% Rotation from imu to camera (roll: pi -> yaw: pi/4)
yaw = 0.248 * pi;  % pi / 4

R_roll = [1, 0, 0; 0, -1, 0; 0, 0, -1];
R_yaw = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
 
R_imu2cam = (R_roll * R_yaw)';
 
% Translation from camera to imu measured in camera
T_cam2imu = [-0.04; 0.0065; -0.03]; 

% homography transforms from imu frame to camera
H_imu2cam = [R_imu2cam, T_cam2imu; 0, 0, 0, 1];

% April Tag Environment
num_X = 12; num_Y = 9;
num_tags = num_X * num_Y;

% 3D position of all tags in the planer world frame (Z = 0)
tag_size = 0.152;
interval_short = 0.152; interval_long = 0.178;

% the return are two matrices with size [num_tags, 5], each row has structure [p0, p1, p2, p3, p4]
[tags_3D_corner_X, tags_3D_corner_Y] = obtain3DCorners(num_X, num_Y, tag_size, interval_short, interval_long);


%% Assign variables to the input
args.K = K;  % intrinsic matrix
args.R_i2c = R_imu2cam;  % rotation mapping from imu to camera
args.t_c2i = T_cam2imu;  % translation mapping from camera to imu
args.H_i2c = H_imu2cam;  % homography mapping from imu to camera
args.numTags = num_tags;  % number of tags
args.numTags_X = num_X;  % number of tags respected to the X axis
args.numTags_Y = num_Y;  % number of tags respected to the Y axis
args.tag3D_X = tags_3D_corner_X;  % 3D positions in X axis of all tags
args.tag3D_Y = tags_3D_corner_Y;  % 3D positions in Y axis of all tags
args.linearEstimator = false;  % approach index to estimate pose linearly
args.useRANSAC = false;  % represent whether using RANSAC


%% read in data
dataset = load('studentdata9.mat');
vicon = dataset.vicon;
time = dataset.time;
sensor_set = dataset.data;

%% estimate pose for each time stamp 
num_sensor = length(sensor_set);
pos_error_sum = 0; rpy_error_sum = 0;

x_est = zeros(num_sensor, 1); y_est = zeros(num_sensor, 1); z_est = zeros(num_sensor, 1);
x_gt = zeros(num_sensor, 1); y_gt = zeros(num_sensor, 1); z_gt = zeros(num_sensor, 1);

roll_est = zeros(num_sensor, 1); pitch_est = zeros(num_sensor, 1); yaw_est = zeros(num_sensor, 1);
roll_gt = zeros(num_sensor, 1); pitch_gt = zeros(num_sensor, 1); yaw_gt = zeros(num_sensor, 1);

for i = 1 : num_sensor
  sensor_cur = sensor_set(i);  % extract the sensor data at current timestamp
  
  % match time stamp
  [~, ind] = min(abs(time - sensor_cur.t));
  
  vic.pose = vicon(1 : 6, ind);
  vic.vel = vicon(7 : end, ind);
  vic.t = time(ind);
  
  % [pos_cur, quat_cur] = estimate_pose(sensor_cur, args);  % estimate pose at the timestamp
  [X, Z] = ekf1(sensor_cur, vic, args);
  
  pos_cur = X(1 : 3);
  quat_cur = X(4 : 7);

  % convert quaternion into Euler angle
  ypr_cur = quat2eul(quat_cur');
  rpy_cur = flip(ypr_cur)';

  pos_vicon = vic.pose(1 : 3);
  rpy_vicon = vic.pose(4 : 6);

  % compute error
  pos_error = sqrt(sum((pos_cur - pos_vicon) .^ 2));
  pos_error_sum = pos_error_sum + pos_error;
  
  rpy_error = sqrt(sum((rpy_cur - rpy_vicon) .^ 2));
  rpy_error_sum = rpy_error_sum + rpy_error;

  % display numerical information
  fprintf('[+] Time Stamp: %f || Position Error: %f meter(s) || Euler Angles Error: %f rad(s). \n',sensor_cur.t, pos_error, rpy_error);

  % store estimated and vicon data
  x_est(i) = pos_cur(1); y_est(i) = pos_cur(2); z_est(i) = pos_cur(3);
  x_gt(i) = pos_vicon(1); y_gt(i) = pos_vicon(2); z_gt(i) = pos_vicon(3);
  
  roll_est(i) = rpy_cur(1); pitch_est(i) = rpy_cur(2); yaw_est(i) = rpy_cur(3);
  roll_gt(i) = rpy_vicon(1); pitch_gt(i) = rpy_vicon(2); yaw_gt(i) = rpy_vicon(3);

end

pos_error_avg = pos_error_sum / num_sensor;
rpy_error_avg = rpy_error_sum / num_sensor;

fprintf('[*] Completed Pose estimation! \n');
fprintf('[+] Average Position Error: %f meter(s) || Average Euler Angles Error: %f. \n', pos_error_avg, rpy_error_avg);


%% plot figures
t_axis = 1 : 1 : num_sensor;

% position [x, y, z] comparison
figure
% x
ax1 = subplot(3, 1, 1);
plot(ax1, t_axis, x_gt, 'r');

hold on
plot(ax1, t_axis, x_est, 'b');

legend('Vicon', 'Estimated Result');
title(ax1, 'Position (x dimension)')
ylabel(ax1, 'x')

% y
ax2 = subplot(3, 1, 2);
plot(ax2, t_axis, y_gt, 'r');

hold on
plot(ax2, t_axis, y_est, 'b');

legend('Vicon', 'Estimated Result');
title(ax2, 'Position (y dimension)')
ylabel(ax2, 'y')

% z
ax3 = subplot(3, 1, 3);
plot(ax3, t_axis, z_gt, 'r');

hold on
plot(ax3, t_axis, z_est, 'b');

legend('Vicon', 'Estimated Result');
title(ax3, 'Position (z dimension)')
ylabel(ax3, 'z')


% euler angle comparison
figure
% roll
ax1 = subplot(3, 1, 1);
plot(ax1, t_axis, roll_gt, 'r');

hold on
plot(ax1, t_axis, roll_est, 'b');

legend('Vicon', 'Estimated Result');
title(ax1, 'Euler Angle Roll')
ylabel(ax1, 'roll')

% pitch
ax2 = subplot(3, 1, 2);
plot(ax2, t_axis, pitch_gt, 'r');

hold on
plot(ax2, t_axis, pitch_est, 'b');

legend('Vicon', 'Estimated Result');
title(ax2, 'Euler Angle Pitch')
ylabel(ax2, 'pitch')

% yaw
ax3 = subplot(3, 1, 3);
plot(ax3, t_axis, yaw_gt, 'r');

hold on
plot(ax3, t_axis, yaw_est, 'b');

legend('Vicon', 'Estimated Result');
title(ax3, 'Euler Angle Yaw')
ylabel(ax3, 'yaw')
