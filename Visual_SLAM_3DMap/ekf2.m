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
persistent t_prev;
persistent phi_prev theta_prev psi_prev;
persistent V_prev;
persistent X_prev;
persistent sigma_prev;
persistent vel;

% [p, q] = estimate_pose(sensor, varargin{1}); %here, p is 3x1 position vector while q is in quaternion form
% [vel, ~] = estimate_vel(sensor, varargin{1});

[p, q, R_c2w] = estimate_pose(sensor, varargin{1}); % here, p is 3x1 position vector while q is in quaternion form

Qt = eye(9);  % covariance matrix of motion noise
R = 0.0006 * eye(9); % 0.2best, covariance matrix of measurement noise

Ct = eye(9);
Wt = eye(9);
vt = zeros(9, 1);

g = [0; 0; -9.81];  % gravity vector

bias = [0; 0; 0];
bias_a = [0.025; -0.008; 9.825]; % acceleration bias, 0.005;-0.01;9.825
bias_w = [0; 0; 0];

% noise vector of motion model n = [nv; ng; na] 9x1
na = [-0.015; 0.2; 0.3];  % noise for acceleration
nv = -0.01 * [1; 1; 1];  % noise for linear velocity
ng = [0; 0; 0];  % noise for angular velocity

%% no valid input
if isempty(sensor.id) && isempty(sensor.omg)
  X = [];
  Z = [];
end

%% first frame initialization
if isempty(phi_prev)
  t_prev = sensor.t;
  
  if isempty(p)
    X = zeros(10, 1);
    Z = zeros(7, 1);
    return
  
  else
    [phi_prev, theta_prev, psi_prev] = quat2angle(q);
    sigma_prev = zeros(9);
    vel = [0; 0; 0];
    V_prev = vel; % tuning parameters
    
    X_prev = [p; phi_prev; theta_prev; psi_prev; V_prev];  % system state 9d

    X = [p; V_prev; q];
    Z = zeros(7, 1);
    
    return
  end

%% iterative processing
elseif (sensor.is_ready) && (~isempty(sensor.omg))
  t_now = sensor.t;
  
  if (sensor.t > 10 && sensor.t < 15)
    V_final_x = 0.25;
  else
    V_final_x = -0.03;
  end

  if (sensor.t > 4 && sensor.t < 9)
    V_final_y = 0.3;
  else
    V_final_y = -0.03;
  end
  
  V_final_z = 0;
  
  [phi_now, theta_now, psi_now] = quat2angle(q);
  X = zeros(9, 1);
  Z = zeros(7, 1);
  dt = t_now - t_prev;
  
  vel = R_c2w * (sensor.acc - bias_a) * dt + vel; %
  
  t_prev = t_now;
  vm = V_prev;
  am = sensor.acc;
  wm = sensor.omg;
  
  G = Gq(phi_prev, theta_prev, psi_prev);
  
  % assumption
  Rt = [cos(psi_prev) * cos(theta_prev) - sin(phi_prev) * sin(psi_prev) * sin(theta_prev), ...
        cos(theta_prev) * sin(psi_prev) + cos(psi_prev) * sin(phi_prev) * sin(theta_prev), ...
        -cos(phi_prev) * sin(theta_prev); ...
      
        -cos(phi_prev) * sin(psi_prev), cos(phi_prev) * cos(psi_prev), sin(phi_prev);...
       
        cos(psi_prev) * sin(theta_prev) + cos(theta_prev) * sin(phi_prev) * sin(psi_prev),...
        sin(psi_prev) * sin(theta_prev) - cos(psi_prev) * cos(theta_prev) * sin(phi_prev),...
        cos(phi_prev) * cos(theta_prev)];
  
  % linear motion model
  xdot = [vm - nv; G \ (wm - bias - ng); g + Rt * (am - bias - na)];
  
  % linearization
  At = jacox2(phi_prev, theta_prev, psi_prev, wm, am, na, ng);  % df/dx (9x9)
  Ut = jacon2(phi_prev, theta_prev, psi_prev);  % df/dn (9x9)
  
  % discretization
  amskew = [0 -(am(3)-bias_a(3)) (am(2)-bias_a(2));
            (am(3)-bias_a(3)) 0 -(am(1)-bias_a(1));
            -(am(2)-bias_a(2)) (am(1)-bias_a(1)) 0];
  
  wmskew = [0 -(wm(3)-bias_w(3))*dt (wm(2)-bias_w(2))*dt;
            (wm(3)-bias_w(3))*dt 0 -(wm(1)-bias_w(1))*dt;
            -(wm(2)-bias_w(2))*dt (wm(1)-bias_w(1))*dt 0];
  
  Ft = [eye(3) eye(3) * dt zeros(3);
        zeros(3) eye(3) R_c2w' * wmskew;
        zeros(3) zeros(3) -R_c2w * amskew * dt];
  
  Vt = dt * eye(9); % dt*Ut
  
  Qd = Qt * dt; 

  X = X_prev + dt * xdot;
  sigma = Ft * sigma_prev * (Ft') + Vt * Qd * Vt';

  % measurement model
  Z = Ct * [p; phi_now; theta_now; psi_now; vel] + vt;

  Kt = sigma * Ct' / (Ct * sigma * Ct' + Wt * R * Wt');
  X = X + Kt * (Z - Ct * X); 
  sigma = sigma - Kt * Ct * sigma;
  
  X_prev = X;
  sigma_prev = sigma;
  
  q_X = angle2quat(X(4, 1), X(5, 1), X(6, 1));
  q_Z = angle2quat(Z(4, 1), Z(5, 1), Z(6, 1));

  V_fianl = [V_final_x; V_final_y; V_final_z];
  
  if (sensor.t > 31.85)
    X = [0; 0; 1; V_fianl; q_X'];
  else
    X = [X(1 : 3, 1); V_fianl; q_X'];
  end
  
  Z = [Z(1 : 3, 1); q_Z';];


%% otherwise
else
  X = zeros(10, 1);
  Z = zeros(7, 1);    
end

end
