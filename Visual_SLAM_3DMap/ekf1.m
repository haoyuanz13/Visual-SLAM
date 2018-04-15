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

persistent t_prev;
persistent phi_prev theta_prev psi_prev;
persistent X_prev;
persistent sigma_prev;


% vision based pose estimation
[p, q, ~] = estimate_pose(sensor, varargin{1}); % here, p is 3x1 position vector while q is in quaternion form

% initialization
Qt = eye(6);  % 6x6 covariance matrix of the motion noise
nv = [-0.03; -0.03; -0.03];
ng = [0.001; 0.001; 0.001];
n = [nv; ng];  % mean value of the motion model noise

R = 0.23 * eye(6);  % 6x6 covariance matrix of the measurement noise
vt = zeros(6, 1);  % mean value of the measurement model noise

bias = [0; 0; 0];  % gyroscope bias


%% no valid input
if isempty(vic) && isempty(sensor)
  X = [];
  Z = [];
end

%% the initial frame
if isempty(phi_prev)
  t_prev = vic.t;
  
  % no valid pose estimation from the vision side
  if isempty(p) || isempty(q)
    X = zeros(7, 1);
    Z = zeros(7, 1);
    return
  
  else
    [phi_prev, theta_prev, psi_prev] = quat2angle(q);
    sigma_prev = zeros(6);
    X_prev = [p; phi_prev; theta_prev; psi_prev];
    X = [p; q];
    Z = zeros(7, 1);
    return
  end

%% the iterative process
else
  t_now = vic.t;

  [phi_now, theta_now, psi_now] = quat2angle(q);
  
  X = zeros(6, 1);
  Z = zeros(6, 1);
  
  % Motion model
  if ~isempty(vic)
    dt = t_now - t_prev;
    t_prev = t_now;
    
    % obtain vicon input
    vx = vic.vel(1);
    vy = vic.vel(2);
    vz = vic.vel(3);
    vm = [vx; vy; vz];  % linear velocity wrt the world frame
    
    wx = vic.vel(4);
    wy = vic.vel(5);
    wz = vic.vel(6);
    wm = [wx; wy; wz];  % angular velocity wrt the boday frame
    
    % G = Gq(phi_prev, theta_prev, psi_prev);

    G = [cos(theta_prev), 0, -cos(phi_prev) * sin(theta_prev); 0, 1, sin(phi_prev);
         sin(theta_prev), 0, cos(phi_prev) * cos(theta_prev)];
    
    % assumption
    xdot = [vm - nv; G \ (wm - bias - ng)];  % 6x1
    
    % linearization
    At = jacox(phi_prev, theta_prev, psi_prev, wm, ng, bias);
    Ut = jacon(phi_prev, theta_prev, psi_prev);
    
    % discretization
    Ft = eye(6) + dt * At;
    Vt = Ut;
    Qd = Qt * dt; 
    
    % make prediction
    X = X_prev + dt * xdot;
    sigma = Ft * sigma_prev * Ft' + Vt * Qd * Vt';  % update the covariance matrix
  end

  % measurement model
  if ~isempty(sensor)
    if ((sensor.is_ready) && ~isempty(sensor.id))
      Ct = eye(6);  % jacobian dg/dx
      Wt = eye(6);  % jacobian dg/dv      
      
      Z = Ct * [p; phi_now; theta_now; psi_now] + vt;  % measurement model

      Kt = sigma * Ct' / (Ct * sigma * Ct' + Wt * R * Wt');
      X = X + Kt * (Z - Ct * X); 
      sigma = sigma - Kt * Ct * sigma;

    end
  end
  
  X_prev = X;
  sigma_prev = sigma;
  
  Eul_X = angle2quat(X(4, 1), X(5, 1), X(6, 1));
  Eul_Z = angle2quat(Z(4, 1), Z(5, 1), Z(6, 1));
  
  pos_x = X(1, 1);

  X = [X(1 : 3, 1); Eul_X'];
  Z = [Z(1 : 3, 1); Eul_Z'];

end  % end for outer if

end  % end for function

