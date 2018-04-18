function [X, Z] = ekf2(sensor, varargin)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
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
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

%% persistent variables
persistent t_pre;  % the previous time stamp
persistent mu_pre;  % the previous state [x; y; z; roll; pitch; yaw] (6x1)
persistent sigma_pre;  % the previous covariance matrix  (6x6)
persistent roll_pre;  % the previous roll angle 
persistent pitch_pre;  % the previous pitch angle 
persistent yaw_pre;  % the previous yaw angle 
persistent velocity_pre;  % the previous linear velocity
persistent gravity;  % the gravity vector

%% no valid vicon and sensor input
if isempty(sensor.id) && isempty(sensor.omg)
  X = [];
  Z = [];
end

%% vision-based pose estimation (April Tags based)
[pos, quat, R_w2c, t_c2w] = estimate_pose(sensor, varargin);

%% vision-based velocity estimation
[vel] = estimate_vel(sensor, R_w2c, t_c2w, varargin);


%% initialize the states
if isempty(t_pre)
  t_pre = sensor.t;
  sigma_pre = zeros(9);
  yaw_pre = 0; pitch_pre = 0; roll_pre = 0;
  velocity_pre = vel;
  
  gravity = [0; 0; -9.81];
  Z = zeros(7, 1);
  
  % no valid pos or quat estimation
  if isempty(pos) || isempty(quat)
    pos = [0; 0; 0];
    X = [pos; 0; 0; 0; 1; 0; 0; 0];
  
  else
    [yaw_pre, pitch_pre, roll_pre] = quat2angle(quat);  % transform quaternion to euler angle 
    X = [pos; velocity_pre; quat];
  end
  
  mu_pre = [pos; roll_pre; pitch_pre; yaw_pre; velocity_pre];
  
  return;


%% the iterative processing
else
  t_cur = sensor.t;  % assign current time stamp
   
  % Motion model
  if sensor.is_ready && ~isempty(sensor.omg) && ~isempty(sensor.acc)
    Qt = eye(9);  % covariance matrix of motion model noise
    nv = [-0.01; -0.01; -0.01];  % vicon linear velocity noise
    ng = [0.0; 0.0; 0.0];  % gyroscope angular velocity noise
    na = [-0.015; 0.2; 0.3];  % linear acceleration noise
    n = [nv; ng; na];  % mean value of motion model noise
    
    gyro_bias = [0; 0; 0];  % gyro bias
    
    dt = t_cur - t_pre;
    
    % obtain vicon linear velocity
    vm = velocity_pre;
    
    % obtain gyroscope angular velocity
    omega_m = sensor.omg;

    % obtain linear acceleration
    acc_m = sensor.acc;
     
    % calculate G matrix
    G_q = [cos(pitch_pre), 0, -cos(roll_pre) * sin(pitch_pre);
           0, 1, sin(roll_pre); 
	         sin(pitch_pre), 0, cos(roll_pre) * cos(pitch_pre)];
    
    % assumption (9x1)
    Rot = [cos(yaw_pre) * cos(pitch_pre) - sin(roll_pre) * sin(yaw_pre) * sin(pitch_pre), ...
           cos(pitch_pre) * sin(yaw_pre) + cos(yaw_pre) * sin(roll_pre) * sin(pitch_pre), ...
           -cos(roll_pre) * sin(pitch_pre); ...
      
           -cos(roll_pre) * sin(yaw_pre), cos(roll_pre) * cos(yaw_pre), sin(roll_pre);...
       
           cos(yaw_pre) * sin(pitch_pre) + cos(pitch_pre) * sin(roll_pre) * sin(yaw_pre),...
           sin(yaw_pre) * sin(pitch_pre) - cos(yaw_pre) * cos(pitch_pre) * sin(roll_pre),...
           cos(roll_pre) * cos(pitch_pre)];


    mu_dot = [vm - nv; G_q \ (omega_m - gyro_bias - ng); gravity + Rot * (acc_m - gyro_bias - na)];

    % linearization using Jacobian
    At = Jacobian_df_dmu_2(roll_pre, pitch_pre, yaw_pre, gyro_bias, omega_m, acc_m, ng, na);
    Ut = Jacobian_df_dn_2(roll_pre, pitch_pre, yaw_pre);
    
    % discretization    
    Ft = eye(9) + dt .* At;
    Vt = Ut;
    Qd = dt .* Qt;
    
    % prediction
    mu_prediction = mu_pre + dt .* mu_dot;
    sigma_prediction = Ft * sigma_pre * Ft' + Vt * Qd * Vt';
  end
  
  %% Measurement model (Camera based)
  % Use sensor data without valid feature detection
  if isempty(pos) || isempty(quat)
    mu_update = mu_prediction;
    sigma_update = sigma_prediction;
    Z = zeros(7, 1);
  
  % Vision-based update if exists valid detected features
  elseif ~isempty(sensor)
    if (sensor.is_ready) && ~isempty(sensor.id)
      Rt = 0.0006 .* eye(9);  % covariance matrix of measurement model noise
      vt = zeros(9, 1);  % mean value of measurement model noise      
      
      % obtain current euler angle
      [yaw_cur, pitch_cur, roll_cur] = quat2angle(quat); 
      
      % measurement model
      mu_cur = [pos; roll_cur; pitch_cur; yaw_cur; vel];
      Z = eye(9) * mu_cur + vt;

      % linearization
      Ct = Jacobian_dg_dmu_2(mu_cur, vt);
      Wt = Jacobian_dg_dv_2(mu_cur, vt);

      % Kalman Gain
      Kt = sigma_prediction * Ct' / (Ct * sigma_prediction * Ct' + Wt * Rt * Wt');

      % Update
      mu_update = mu_prediction + Kt * (Z - Ct * mu_prediction);
      sigma_update = sigma_prediction - Kt * Ct * sigma_prediction;
 
      quat_Z = angle2quat(Z(6), Z(5), Z(4));
      Z = [Z(1 : 3); quat_Z'];
    end
  
  end

  %% assign variables
  t_pre = t_cur;
  mu_pre = mu_update;
  sigma_pre = sigma_update;

  quat_X = angle2quat(mu_update(6), mu_update(5), mu_update(4));
  X = [mu_update(1 : 3); mu_update(7 : 9); quat_X'];

  velocity_pre = mu_update(7 : 9);   % vel;

end  % end for the outer if-else	

end

%%% helper functions %%%
%% Jacobian computation At = df/dmu
function [Jaco] = Jacobian_df_dmu_2(r, p, y, bias, wm, am, na, ng)
% assign variables
bias_x = bias(1); bias_y = bias(2); bias_z = bias(3);
wx = wm(1); wy = wm(2); wz = wm(3);
ng_x = ng(1); ng_y = ng(2); ng_z = ng(3);
lambda = am - bias - na;  % factors

%%% [
%  0, 0, 0, 0, 0, 0, _, 0, 0;
%  0, 0, 0, 0, 0, 0, 0, _, 0;
%  0, 0, 0, 0, 0, 0, 0, 0, _;
%  0, 0, 0, 0, _, 0, 0, 0, 0;
%  0, 0, 0, _, _, 0, 0, 0, 0;
%  0, 0, 0, _, _, 0, 0, 0, 0;
%  0, 0, 0, _, _, _, 0, 0, 0;
%  0, 0, 0, _, 0, _, 0, 0, 0;
%  0, 0, 0, _, _, _, 0, 0, 0
%%% ];

Jaco = zeros(9);

% df1/dvx
Jaco(1, 7) = 1;
% df2/dvy
Jaco(2, 8) = 1;
% df3/dvz
Jaco(3, 9) = 1;

% df4/dp
Jaco(4, 5) = wz * cos(p) - ng_z * cos(p) - bias_z * cos(p) + bias_x * sin(p) + ng_x * sin(p) - wx * sin(p);

% df5/dr
Jaco(5, 4) = (bias_z * cos(p) + ng_z * cos(p) - wz * cos(p) - bias_x * sin(p) - ng_x * sin(p) + wx * sin(p)) / (cos(r) ^ 2);
% df5/dp
Jaco(5, 5) = -(sin(r) * (bias_x * cos(p) + ng_x * cos(p) - wx * cos(p) + bias_z * sin(p) + ng_z * sin(p) - wz * sin(p))) / cos(r);

% df6/dr
Jaco(6, 4) = -(sin(r) * (bias_z * cos(p) + ng_z * cos(p) - wz * cos(p) - bias_x * sin(p) - ng_x * sin(p) + wx * sin(p))) / (cos(r) ^ 2);
% df6/dp
Jaco(6, 5) = (bias_x * cos(p) + ng_x * cos(p) - wx * cos(p) + bias_z * sin(p) + ng_z * sin(p) - wz * sin(p)) / cos(r);

% df7/dr
Jaco(7, 4) = -lambda(1) * cos(r) * sin(y) * sin(p) + lambda(2) * cos(r) * cos(y) * sin(p) + lambda(3) * sin(r) * sin(p);
% df7/dp
Jaco(7, 5) = -lambda(1) * (cos(y) * sin(p) + cos(p) * sin(r) * sin(y)) - lambda(2) * (sin(y) * sin(p) - cos(y) * cos(p) * sin(r)) - lambda(3) * cos(r) * cos(p);
% df7/dyaw
Jaco(7, 6) = -lambda(1) * (cos(p) * sin(y) + cos(y) * sin(r) * sin(p)) + lambda(2) * (cos(y) * cos(p) - sin(r) * sin(y) * sin(p));

% df8/dr
Jaco(8, 4) = lambda(1) * sin(r) * sin(y) - lambda(2) * cos(y) * sin(r) + lambda(3) * cos(r);
% df8/dyaw
Jaco(8, 6) = -lambda(1) * cos(r) * cos(y) - lambda(2) * cos(r) * sin(y);

% df9/dr
Jaco(9, 4) = lambda(1) * cos(r) * cos(p) * sin(y) - lambda(2) * cos(r) * cos(y) * cos(p) - lambda(3) * cos(p) * sin(r);
% df9/dp
Jaco(9, 5) = lambda(1) * (cos(y) * cos(p) - sin(r) * sin(y) * sin(p)) + lambda(2) * (cos(p) * sin(y) + cos(y) * sin(r) * sin(p)) - lambda(3) * cos(r) * sin(p);
% df9/dyaw
Jaco(9, 6) = -lambda(1) * (sin(y) * sin(p) - cos(y) * cos(p) * sin(r)) + lambda(2) * (cos(y) * sin(p) + cos(p) * sin(r) * sin(y)) ;

end


%% Jacobian computation Ut = df/dn
function [Jaco] = Jacobian_df_dn_2(r, p, y)
%%% [                        
%   _, 0, 0, 0, 0, 0, 0, 0, 0;
%   0, _, 0, 0, 0, 0, 0, 0, 0;
%   0, 0, _, 0, 0, 0, 0, 0, 0;

%   0, 0, 0, _, 0, _, 0, 0, 0;
%   0, 0, 0, _, _, _, 0, 0, 0;
%   0, 0, 0, _, 0, _, 0, 0, 0;

%   0, 0, 0, 0, 0, 0, _, _, _;
%   0, 0, 0, 0, 0, 0, _, _, _;
%   0, 0, 0, 0, 0, 0, _, _, _
%%% ];

Jaco = zeros(9);

% df1/dn1
Jaco(1, 1) = -1;

% df2/dn2
Jaco(2, 2) = -1;

% df3/dn3
Jaco(3, 3) = -1;

% df4/dn4
Jaco(4, 4) = -cos(p);
% df4/dn6
Jaco(4, 6) = -sin(p);

% df5/dn4
Jaco(5, 4) = -(sin(r) * sin(p)) / cos(r);
% df5/dn5
Jaco(5, 5) = -1;
% df5/dn6
Jaco(5, 6) = (cos(p) * sin(r)) / cos(r);

% df6/dn4
Jaco(6, 4) = sin(p) / cos(r);
% df6/dn6
Jaco(6, 6) = -cos(p) / cos(r);

% df7/dn7
Jaco(7, 7) = sin(r) * sin(y) * sin(p) - cos(y) * cos(p);
% df7/dn8
Jaco(7, 8) = -cos(p) * sin(y) - cos(y) * sin(r) * sin(p);
% df7/dn9
Jaco(7, 9) = cos(r) * sin(p);

% df8/dn7
Jaco(8, 7) = cos(r) * sin(y);
% df8/dn8
Jaco(8, 8) = -cos(r) * cos(y);
% df8/dn9
Jaco(8, 9) = -sin(r);

% df9/dn7
Jaco(9, 7) = -cos(y) * sin(p) - cos(p) * sin(r) * sin(y);
% df9/dn8
Jaco(9, 8) = cos(y) * cos(p) * sin(r) - sin(y) * sin(p);
% df9/dn9
Jaco(9, 9) = -cos(r) * cos(p);

end


%% Jacobian computation Ct = dg/dmu
function [Jaco] = Jacobian_dg_dmu_2(x, v)
Jaco = eye(9);
end

%% Jacobian computation Wt = dg/dv
function [Jaco] = Jacobian_dg_dv_2(x, v)
Jaco = eye(9);
end

