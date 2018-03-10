function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

%% define presistent variables, use the "persistent" keyword to keep your trajectory around
% inbetween function calls
persistent map0;  % map object
persistent path0;  % waypoints matrix
persistent waypoints;  % store all valid waypoints
persistent num_wayp;   % number of segments
persistent seg_dist;  % distance for each segment
persistent seg_durations;  % duration for each segment
persistent seg_t_start;  % start time stamp for each segment
persistent seg_t_end;  % terminal time stamp for each segment
persistent seg_ignore_flag;  % the flag to tell whether the ith segment can be ignored due to its small distance between waypoints
persistent fused_waypoints;  % fused waypoints which connect all segment with same direction together
persistent validDist_waypoint;  % fused waypoints which connect all segment without any collsion


%% the case for trajectory generator
if nargin == 4
  map0 = map;
  path0 = path{1};  % N x 3 matrix to store waypoints information (xi, yi, zi)

  num_wayp = size(path0, 1);

  % initial 3d vector
  vector_ini = (path0(2, :) - path0(1, :)) / norm(path0(2, :) - path0(1, :));
  pre_vector = vector_ini;

  fused_waypoints = [];

  % the vector of the path, calculate from the first path besides start
  for i = 2 : num_wayp - 1
    now_vector = (path0(i + 1, :) - path0(i, :)) / norm(path0(i + 1, :) - path0(i, :));
    % the current vector is same as previous one, so fuse together
    if now_vector == pre_vector  % big problem
      continue;
    
    else
     fused_waypoints = [fused_waypoints; path0(i, :)];
     pre_vector = now_vector;
    end

  end
  % add two-side waypoints into the fused_waypoints set
  fused_waypoints = [path0(1, :); fused_waypoints; path0(end, :)];
  
  % update the number of fused waypoints
  num_wayp = size(fused_waypoints, 1);

  % count waypoints: remove waypoints that are too close to any of other
  validDist_waypoint = [];
  for i = 1 : num_wayp - 2
    if norm(fused_waypoints(i + 1, :) - fused_waypoints(i, :)) > 0.1
      validDist_waypoint = [validDist_waypoint; fused_waypoints(i, :)];
    else
      continue;
    end
  end
  validDist_waypoint = [validDist_waypoint; path0(end,:)];


  % check collision state of waypoints
  num_wayp = size(validDist_waypoint, 1);
  div = 10;

  wp_start = validDist_waypoint(1, :);
  waypoint_temp = [];
  for i = 1 : num_wayp - 1
    len_interval = norm(validDist_waypoint(i + 1, :) - wp_start);    
    vec_interval = (validDist_waypoint(i + 1, :) - wp_start) / len_interval;

    delta_len = len_interval / div;
    delta_vec = delta_len .* vec_interval;
    % creata a list of points along two waypoints to check the collsion state
    points = [wp_start + 1 .* delta_vec;    wp_start + 2 .* delta_vec;   wp_start + 3 .* delta_vec;
              wp_start + 4 .* delta_vec;    wp_start + 5 .* delta_vec;   wp_start + 6 .* delta_vec; 
              wp_start + 7 .* delta_vec;    wp_start + 8 .* delta_vec;   wp_start + 9 .* delta_vec;
              wp_start + 10 .* delta_vec];

    check = map0.collide(points);
    
    % all longside waypoints are safe such that we can fuse two waypoints together
    if isempty(check(check == 1))
      continue;
    
    % if there exits collision, append it into the final waypoints set
    else
      waypoint_temp = [waypoint_temp; validDist_waypoint(i, :)];
      wp_start = validDist_waypoint(i, :);
    end
  end
  % the final updated waypoints set
  waypoints = [path0(1, :); waypoint_temp; path0(end, :)];
  

  % obtain distance for each segment 
  num_wayp = size(waypoints, 1);
  seg_dist = [];
  seg_ignore_flag = [];
  for i = 1 : num_wayp - 1
    path_distance = norm(waypoints(i + 1, :) - waypoints(i, :));
    if path_distance < 1.0
      flag = 1;
    else
      flag = 0;
    end
    seg_dist = [seg_dist; path_distance];
    seg_ignore_flag = [seg_ignore_flag; flag];
  end

  % calculate segment duration and time stamps
  t_s = 0;
  seg_t_start = [];
  seg_t_end = [];
  seg_durations = [];

  for i = 1 : num_wayp - 1
    % try to average durations
    if seg_dist(i) < 1.
      seg_cur_t = seg_dist(i);
    elseif (seg_dist(i) >= 1) && (seg_dist(i) < 5)
      seg_cur_t = seg_dist(i) / 1.5;
    else
      seg_cur_t = seg_dist(i) / 3;
    end

    seg_durations = [seg_durations; seg_cur_t];
    seg_t_start = [seg_t_start; t_s];

    % update the start time stamp for each segment
    t_s = t_s + seg_cur_t;
    seg_t_end = [seg_t_end; t_s];
  end


%% the case that only two input arguments which is for test_trajectory
else
  desired_state = [];
  % Process trajectory when t < terminal time stamp
  if t <= seg_t_end(end)
    % determine the segment it locates in
    for i = 1 : num_wayp - 1
      if (seg_t_start(i) <= t) && (t < seg_t_end(i))
        ind_seg = i;
        break;
      end
    end

    % obtain the corresponding two-sides waypoints
    wp_s = waypoints(ind_seg, :);
    wp_e = waypoints(ind_seg + 1, :);

    % if the current segment is too short, use waypoints directly
    if seg_ignore_flag(ind_seg) == 1
      pos = wp_e';
      vel = [0; 0; 0];
      acc = [0; 0; 0];
    
    % generate polynomial trajectory and determine its state
    else
      t_0 = 0;  % the initial time stamp
      t_f = seg_durations(ind_seg); % the terminal time stamp
      t = t - seg_t_start(ind_seg);  % remove the offset in the current segment

      Coeff_0 = [1, t_0, t_0^2, t_0^3, t_0^4, t_0^5;
                 0, 1, 2 * t_0, 3 * t_0^2, 4 * t_0^3, 5 * t_0^4;
                 0, 0, 2, 6 * t_0, 12 * t_0^2, 20 * t_0^3];
             
      Coeff_f = [1, t_f, t_f^2, t_f^3, t_f^4, t_f^5;
                 0, 1, 2 * t_f, 3 * t_f^2, 4 * t_f^3, 5 * t_f^4;
                 0, 0, 2, 6 * t_f, 12 * t_f^2, 20 * t_f^3];

      Coeff = [Coeff_0; Coeff_f];

      %% Compute the Polynomial trajectory in angle domain

      % define the initial and terminal states in x domain
      b_x = [wp_s(1); 0; 0; wp_e(1); 0; 0];
      a_x = Coeff \ b_x;
      
      pos_x = a_x' * [1; t; t^2; t^3; t^4; t^5];
      vel_x = a_x' * [0; 1; 2 * t; 3 * t^2; 4 * t^3; 5 * t^4];
      acc_x = a_x' * [0; 0; 2; 6 * t; 12 * t^2; 20 * t^3];

      % define the initial and terminal states in y domain
      b_y = [wp_s(2); 0; 0; wp_e(2); 0; 0];
      a_y = Coeff \ b_y;
      
      pos_y = a_y' * [1; t; t^2; t^3; t^4; t^5];
      vel_y = a_y' * [0; 1; 2 * t; 3 * t^2; 4 * t^3; 5 * t^4];
      acc_y = a_y' * [0; 0; 2; 6 * t; 12 * t^2; 20 * t^3];

      % define the initial and terminal states in z domain
      b_z = [wp_s(3); 0; 0; wp_e(3); 0; 0];
      a_z = Coeff \ b_z;
      
      pos_z = a_z' * [1; t; t^2; t^3; t^4; t^5];
      vel_z = a_z' * [0; 1; 2 * t; 3 * t^2; 4 * t^3; 5 * t^4];
      acc_z = a_z' * [0; 0; 2; 6 * t; 12 * t^2; 20 * t^3];

     %% Assign return variables
      pos = [pos_x; pos_y; pos_z];
      vel = [vel_x; vel_y; vel_z];
      acc = [acc_x; acc_y; acc_z];
    end
  

  % The terminal static state
  else
    pos = [waypoints(end, 1); waypoints(end, 2); waypoints(end, 3)];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
  end

  desired_state.pos = pos(:);
  desired_state.vel = vel(:);
  desired_state.acc = acc(:);
  desired_state.yaw = 0;
  desired_state.yawdot = 0;
end

end
