function [X, Z] = ekf1(sensor, vic, varargin)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
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
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
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

%% no valid vicon and sensor input
if isempty(vic) && isempty(sensor)
  X = [];
  Z = [];
end

%% vision-based pose estimation (April Tags based)
[pos, quat, ~, ~] = estimate_pose(sensor, varargin);

%% initialize the states
if isempty(t_pre)
  t_pre = vic.t;
  sigma_pre = zeros(6);
  yaw_pre = 0; pitch_pre = 0; roll_pre = 0;
  
  Z = zeros(7, 1);
  
  % no valid pos or quat estimation
  if isempty(pos) || isempty(quat)
    pos = [0; 0; 0];
    X = [pos; 1; 0; 0; 0];
  
  else
    [yaw_pre, pitch_pre, roll_pre] = quat2angle(quat);  % transform quaternion to euler angle 
    X = [pos; quat];
  end
  
  mu_pre = [pos; roll_pre; pitch_pre; yaw_pre];
  
  return;

%% the iterative processing
else
  t_cur = vic.t;  % assign current time stamp
  
  
  
  
  % Motion model (Vicon based)
  if ~isempty(vic)
    Qt = eye(6);  % covariance matrix of motion model noise
    nv = [-0.03; -0.03; -0.03];  % vicon linear velocity noise
    ng = [0.001; 0.001; 0.001];  % gyroscope angular velocity noise
    n = [nv; ng];  % mean value of motion model noise
    
    gyro_bias = [0; 0; 0];  % gyro bias
    dt = t_cur - t_pre;
    
    % obtain vicon linear velocity
    vm = [vic.vel(1); vic.vel(2); vic.vel(3)];
    
    % obtain gyroscope angular velocity
    omega_m = [vic.vel(4); vic.vel(5); vic.vel(6)];
    
    % calculate G matrix
    G_q = [cos(pitch_pre), 0, -cos(roll_pre) * sin(pitch_pre);
           0, 1, sin(roll_pre); 
	         sin(pitch_pre), 0, cos(roll_pre) * cos(pitch_pre)];
    
    % assumption (6x1)
    mu_dot = [vm - nv; G_q \ (omega_m - gyro_bias - ng)];

    % linearization using Jacobian
    At = Jacobian_df_dmu(roll_pre, pitch_pre, yaw_pre, omega_m, gyro_bias, ng);
    Ut = Jacobian_df_dn(roll_pre, pitch_pre, yaw_pre);
    
    % discretization
    Ft = eye(6) + dt .* At;
    Vt = Ut;
    Qd = dt .* Qt;
    
    % prediction
    mu_prediction = mu_pre + dt .* mu_dot;
    sigma_prediction = Ft * sigma_pre * Ft' + Vt * Qd * Vt';
  end   
  
  %% Measurement model (Camera based)
  % Use IMU data if there is no valid April tag detection
  if isempty(pos) || isempty(quat)
    mu_update = mu_prediction;
    sigma_update = sigma_prediction;
    Z = zeros(7, 1);
  
  % valid April tags detection
  elseif ~isempty(sensor)
    if (sensor.is_ready) && ~isempty(sensor.id)
      Rt = 0.000000001 .* eye(6);  % covariance matrix of measurement model noise
      vt = zeros(6, 1);  % mean value of measurement model noise      
      
      % obtain current euler angle
      [yaw_cur, pitch_cur, roll_cur] = quat2angle(quat);

      % measurement model
      mu_cur = [pos; roll_cur; pitch_cur; yaw_cur];
      Z = eye(6) * mu_cur + vt;

      % linearization
      Ct = Jacobian_dg_dmu(mu_cur, vt);
      Wt = Jacobian_dg_dv(mu_cur, vt);

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
  X = [mu_update(1 : 3); quat_X'];

end  % end for the outer if-else	

end  % end for the main function


%% Jacobian computation At = df/dmu
function [Jaco] = Jacobian_df_dmu(r, p, y, wm, bias, n)
% assign variables
bias_x = bias(1); bias_y = bias(2); bias_z = bias(3);
wx = wm(1); wy = wm(2); wz = wm(3);
ng_x = n(1); ng_y = n(2); ng_z = n(3);

Jaco = zeros(6);

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

end


%% Jacobian computation Ut = df/dn
function [Jaco] = Jacobian_df_dn(r, p, y)
Jaco = zeros(6);

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

end


%% Jacobian computation Ct = dg/dmu
function [Jaco] = Jacobian_dg_dmu(x, v)
Jaco = eye(6);
end

%% Jacobian computation Wt = dg/dv
function [Jaco] = Jacobian_dg_dv(x, v)
Jaco = eye(6);
end

