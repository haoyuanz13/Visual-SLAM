% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) ekf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) ekf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function

addpath('.\utils');

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


ekf1_handle = @(sensor, vic) ekf1(sensor, vic, args);
ekf2_handle = @(sensor) ekf2(sensor, args);
