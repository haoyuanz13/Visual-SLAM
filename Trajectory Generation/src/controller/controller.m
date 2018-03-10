% function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% % CONTROLLER quadrotor controller
% % The current states are:
% %% Inputs:
% %
% % qd{qn}: state and desired state information for quadrotor #qn (qn
% %         will be = 1 since we are only flying a single robot)
% %
% %  qd{qn}.pos, qd{qn}.vel   position and velocity
% %  qd{qn}.euler = [roll;pitch;yaw]
% %  qd{qn}.omega     angular velocity in body frame
% % 
% %  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
% %  qd{qn}.yaw_des, qd{qn}.yawdot_des
% %
% % t: current time
% %    
% % qn: quadrotor number, should always be 1
% %    
% % params: various parameters
% %  params.I     moment of inertia
% %  params.grav  gravitational constant g (9.8...m/s^2)
% %  params.mass  mass of robot
% %
% %% Outputs:
% %
% % F: total thrust commanded (sum of forces from all rotors)
% % M: total torque commanded
% % trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% % drpy: time derivative of trpy
% %
% % Using these current and desired states, you have to compute the desired
% % controls u, and from there F and M
% %
% 
% % =================== Your code goes here ===================
% %% Compute the commanded acceleration
% pos_cur = qd{qn}.pos;
% vel_cur = qd{qn}.vel;
% 
% pos_des_T = qd{qn}.pos_des;
% vel_des_T = qd{qn}.vel_des;
% 
% err_pos = pos_cur - pos_des_T;
% err_vel = vel_cur - vel_des_T;
% 
% % kd_r = [4.4; 3; 10];
% % kp_r = [12; 4; 85];
% % kd_r = [3; 3; 28];
% kp_r = [10; 10; 110];
% kd_r = 0.7 .* kp_r;
% 
% % the commanded acceleration
% r_dotdot_des = qd{qn}.acc_des - kd_r .* err_vel - kp_r .* err_pos;
% 
% 
% %% Compute desired Euler Angles
% 
% % Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% % When you are flying in the lab, they *will* be used (because the platform
% % has a built-in attitude controller). Best to fill them in already
% % during simulation.
% 
% r_des1 = r_dotdot_des(1);
% r_des2 = r_dotdot_des(2);
% r_des3 = r_dotdot_des(3);
% 
% psi_des = qd{qn}.yaw_des;
% phi_des = (1 / params.grav) * (r_des1 * sin(psi_des) - r_des2 * cos(psi_des));
% theta_des = (1 / params.grav) * (r_des1 * cos(psi_des) - r_des2 * sin(psi_des));
% 
% %% Compute input u [u1; u2]
% u = zeros(4,1); % control input u, you should fill this in
% 
% % Thrust
% u1 = params.mass * (params.grav + r_des3);
% u(1) = u1;
% F    = u(1);       % This should be F = u(1) from the project handout
% 
% % Moment
% p_des = 0;
% q_des = 0;
% r_des = qd{qn}.yawdot_des;
% 
% err_euler = qd{qn}.euler - [phi_des; theta_des; psi_des];
% err_omega = qd{qn}.omega - [p_des; q_des; r_des];
% 
% % kd_euler = [0.05; 0.05; 0.05];
% % kp_euler = [3; 3.4; 15];
% 
% % kd_euler = [40; 40; 50];
% % kp_euler = [600; 600; 1000];
% % kd_euler = [40; 40; 50];
% 
% kp_euler = [3000; 3000; 3000];
% kd_euler = 0.7 .* kp_euler;
% 
% u2 = params.I * (- kp_euler .* err_euler - kd_euler .* err_omega);
% % u2 = -kp_euler .* err_euler - kd_euler .* err_omega;
% u(2:4) = u2;
% M    = u(2:4);     % note: params.I has the moment of inertia
% 
% % =================== Your code ends here ===================
% 
% % Output trpy and drpy as in hardware
% trpy = [F, phi_des, theta_des, psi_des];
% drpy = [0, 0,       0,         0];
% 
% end
% 
% %
% % ------------------------------------------------------------
% %    should you decide to write a geometric controller,
% %    the following functions should come in handy
% %
% 
% function m = eulzxy2rotmat(ang)
%     phi   = ang(1);
%     theta = ang(2);
%     psi   = ang(3);
%     
%     m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
%           cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
%          [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
%           sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
%          [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
% end
% 
% function eul = rotmat2eulzxy(R)
%     if R(3,2) < 1
%         if R(3,2) > -1
%             thetaX = asin(R(3,2));
%             thetaZ = atan2(-R(1,2), R(2,2));
%             thetaY = atan2(-R(3,1), R(3,3));
%         else % R(3,2) == -1
%             thetaX = -pi/2;
%             thetaZ = -atan2(R(1,3),R(1,1));
%             thetaY = 0;
%         end
%     else % R(3,2) == +1
%         thetaX = pi/2;
%         thetaZ = atan2(R(1,3),R(1,1));
%         thetaY = 0;
%     end
%     eul = [thetaX, thetaY, thetaZ];
% end
% 
% function w = veemap(R)
%     w = [-R(2,3), R(1,3), -R(1,2)];
% end


function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% ...
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

kp_xy = 14.5;
kd_xy = 6.2;
kp_z = 110;
kd_z = 28;

K_p = diag([kp_xy, kp_xy, kp_z]);
% K_d = diag([kd_xy, kd_xy, kd_z]);
K_d = 0.7 .* K_p;

kr_xy = 650;
ko_xy = 60;
kr_z = 1000;
ko_z = 50;

K_R = diag([kr_xy, kr_xy, kr_z]);
K_omega = diag([ko_xy, ko_xy, ko_z]);

r_ddot_des = qd{qn}.acc_des - K_d * (qd{qn}.vel - qd{qn}.vel_des) - K_p * (qd{qn}.pos - qd{qn}.pos_des);

F_des = params.mass * (r_ddot_des + [0; 0; params.grav]);

R = eulzxy2rotmat(qd{qn}.euler);
b3 = R * [0; 0; 1];

u1 = b3' * F_des;

b3_des = F_des / norm(F_des);

a_yaw = [cos(qd{qn}.yaw_des); sin(qd{qn}.yaw_des); 0];

b2_des = cross(b3_des, a_yaw) / norm(cross(b3_des, a_yaw));

R_des = [cross(b2_des, b3_des), b2_des, b3_des];

euler = rotmat2eulzxy(R_des);

phi_des   = euler(1);
theta_des = euler(2);
psi_des   = euler(3);

e_R = 0.5 * veemap(R_des' * R - R' * R_des)';
e_omega = qd{qn}.omega;

u2 = params.I * (- K_R * e_R - K_omega * e_omega);


% xddot_cmd = qd{qn}.acc_des(1) + kd_xy * (qd{qn}.vel_des(1) - qd{qn}.vel(1)) + kp_xy * (qd{qn}.pos_des(1) - qd{qn}.pos(1));
% yddot_cmd = qd{qn}.acc_des(2) + kd_xy * (qd{qn}.vel_des(2) - qd{qn}.vel(2)) + kp_xy * (qd{qn}.pos_des(2) - qd{qn}.pos(2));
% zddot_cmd = qd{qn}.acc_des(3) + kd_z * (qd{qn}.vel_des(3) - qd{qn}.vel(3)) + kp_z * (qd{qn}.pos_des(3) - qd{qn}.pos(3));
% 
% phi_des = (xddot_cmd * sin(qd{qn}.yaw_des) - yddot_cmd * cos(qd{qn}.yaw_des)) / params.grav;
% theta_des = (xddot_cmd * cos(qd{qn}.yaw_des) + yddot_cmd * sin(qd{qn}.yaw_des)) / params.grav;
% % Thrust
% u1 = params.grav * params.mass + params.grav * zddot_cmd;
% 
% % Moment
% u2 = params.I * [kr_xy * (phi_des - qd{qn}.euler(1)) + ko_xy * (0 - qd{qn}.omega(1));
%                  kr_xy * (theta_des - qd{qn}.euler(2)) + ko_xy * (0 - qd{qn}.omega(2));
%                  kr_z * (qd{qn}.yaw_des - qd{qn}.euler(3)) + ko_z * (qd{qn}.yawdot_des - qd{qn}.omega(3))];
%
%
%
u    = [u1; u2]; % control input u, you should fill this in
% u = zeros(4, 1);
                  
% Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
M    = u(2:4);     % note: params.I has the moment of inertia

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
  phi   = ang(1);
  theta = ang(2);
  psi   = ang(3);
  
  m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
        cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
       [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
        sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
       [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
  if R(3,2) < 1
    if R(3,2) > -1
      thetaX = asin(R(3,2));
      thetaZ = atan2(-R(1,2), R(2,2));
      thetaY = atan2(-R(3,1), R(3,3));
    else % R(3,2) == -1
      thetaX = -pi/2;
      thetaZ = -atan2(R(1,3),R(1,1));
      thetaY = 0;
    end
  else % R(3,2) == +1
    thetaX = pi/2;
    thetaZ = atan2(R(1,3),R(1,1));
    thetaY = 0;
  end
  eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
  w = [-R(2,3), R(1,3), -R(1,2)];
end

