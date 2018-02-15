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

persistent map0 path0;
persistent seg_waypoint;
persistent seg_dis;
persistent seg_num;
persistent t_start;
persistent t_end;
persistent path_time;
persistent seg_flag;
persistent waypoint;
persistent waypoint_num;
persistent path_dis;
persistent path_flag;
persistent waypoint_s;

%% Four intact variables case: trajectory_generator([], [], map, path)
if nargin == 4
  path0 = cell2mat(path);
  map0 = map;

  path_num = size(path0, 1);

  vector_ini = (path0(2, :) - path0(1, :)) / norm(path0(2, :) - path0(1, :));
  former_vector = vector_ini;

  seg_waypoint = [];
  waypoint_temp = [];
  now_vector = [];
  seg_count = 0;
  count = [];

  % the vector of the path, calculate from the first path besides start
  for i = 1 : path_num - 1
    now_vector = (path0(i + 1, :) - path0(i, :)) / norm(path0(i + 1, :) - path0(i, :));
    % the current vector is same as previous one, so fuse together
    if now_vector == former_vector  % big problem
      former_vector = now_vector;

      if seg_count > 9
        if former_vector == now_vector
          inf_num = size(seg_waypoint, 1);
          seg_waypoint(inf_num - seg_count + 2 : inf_num - 1,:) = [];
          seg_count = 0;
        else
          continue;
        end
     
     else
      seg_count = 0;
     end
    
    else
     seg_waypoint = [seg_waypoint; path0(i, :)];
     former_vector = now_vector;
     seg_count = seg_count + 1;
     
     if now_vector(3) ~= 0
      seg_count = 0;
     end
    end
    
    count = [count; seg_count];
  end


  % count segment distance
  seg_waypoint = [path0(1, :); seg_waypoint; path0(end, :)];
  seg_num = size(seg_waypoint, 1) - 1;
  seg_dis = [];
  seg_flag = [];
  for i = 1 : seg_num
    seg_distance = norm(seg_waypoint(i + 1, :) - seg_waypoint(i, :));
    
    if seg_distance < 1.0
      flag = 1;
    else
      flag = 0;
    end

    seg_dis = [seg_dis; seg_distance];
    seg_flag = [seg_flag; flag];
  end

  % count waypoints: remove waypoints that are too close to any of other
  waypoint = [];
  for i = 1 : seg_num - 1
    if norm(seg_waypoint(i + 1, :) - seg_waypoint(i, :)) > 0.1
      waypoint = [waypoint; seg_waypoint(i,:)];
    else
      continue;
    end
  end
  waypoint = [waypoint; path0(end,:)];

  % check collision state of waypoints
  num = size(waypoint, 1);
  div = 10;
  waypoint_s = waypoint(1, :);
  waypoint_final = [];
  check = [];
  for i = 1 : num - 1
    length = waypoint(i + 1,: ) - waypoint_s;
    delt = length / div;
    
    points = [];
    points = [waypoint_s(1, :) + delt; waypoint_s(1, :) + 2 * delt; waypoint_s(1, :) + 3 * delt; ...
              waypoint_s(1, :) + 4 * delt; waypoint_s(1, :) + 5 * delt; waypoint_s(1, :) + 6 * delt; ...
              waypoint_s(1, :) + 7 * delt; waypoint_s(1, :) + 8 * delt; waypoint_s(1, :) + 9 * delt; ...
              waypoint_s(1, :) + delt];

    check = collide(map0, points);
    % all waypoints are safe
    if check == zeros(10, 1)
      continue;
    else
      waypoint_temp = waypoint(i, :);
      waypoint_final = [waypoint_final; waypoint_temp];
      waypoint_s = waypoint(i, :);
    end
  end


  waypoint = [];
  waypoint = [path0(1, :); waypoint_final; path0(end, :)];
  waypoint_num = size(waypoint, 1) - 1;
  path_dis = [];
  path_flag = [];
  for i = 1 : waypoint_num
    path_distance = norm(waypoint(i + 1, :) - waypoint(i, :));
    if path_distance < 1.0
      flag = 1;
    else
      flag = 0;
    end
    path_dis = [path_dis; path_distance];
    path_flag = [path_flag; flag];
  end


%% Two input variables case for test: trajectory_generator(t, qn)
else
  t_s = 0;
  t_start = [];
  t_end = [];
  path_time = [];
  % assign segment times
  for i = 1 : waypoint_num
    if path_dis(i) < 1
      seg_t = path_dis(i) / 1;
    elseif 1 <= path_dis(i) && path_dis(i) <= 5
      seg_t = path_dis(i) / 2;
    else 
      seg_t = path_dis(i) / 2.5;
    end
    
    path_time = [path_time; seg_t];
    t_f = t_s + seg_t;
    t_end = [t_end; t_f];
    t_start = [t_start; t_s];
    t_s = t_f;
  end
  
  % trajectory processing if t < t_end
  if t <= t_end(end)    
    for i = 1 : waypoint_num
      % locate in the ith segment and segment distance is larger than 1
      if t_start(i) <= t && t < t_end(i) && path_flag(i) == 0
        ts = 0;
        t = t - t_start(i);
        tf = ts + path_time(i);
        T = [1  ts  ts ^ 2    ts ^ 3        ts ^ 4        ts ^ 5;
             0   1  2 * ts    3 * ts ^ 2    4 * ts ^ 3    5 * ts ^ 4;
             0   0       2    6 * ts        12 * ts ^ 2   20 * ts ^ 3;
             1  tf  tf ^ 2    tf ^ 3        tf ^ 4        tf ^ 5;
             0   1  2 * tf    3 * tf ^ 2    4 * tf ^ 3    5 * tf ^ 4;
             0   0       2    6 * tf        12 * tf ^ 2   20 * tf ^ 3];           
        
        conditions_x = [waypoint(i, 1) 0 0 waypoint(i + 1, 1) 0 0]';
        coeffs_x = T \ conditions_x;
        pos_x = coeffs_x' * [1;  t;  t ^ 2;      t ^ 3;       t ^ 4;       t ^ 5];
        vel_x = coeffs_x' * [0;  1;  2 * t;  3 * t ^ 2;   4 * t ^ 3;   5 * t ^ 4];
        acc_x = coeffs_x' * [0;  0;      2;      6 * t;  12 * t ^ 2;  20 * t ^ 3];

        conditions_y = [waypoint(i, 2) 0 0 waypoint(i + 1, 2) 0 0]';
        coeffs_y = T \ conditions_y;
        pos_y = coeffs_y' * [1;  t;  t ^ 2;      t ^ 3;       t ^ 4;       t ^ 5];
        vel_y = coeffs_y' * [0;  1;  2 * t;  3 * t ^ 2;   4 * t ^ 3;   5 * t ^ 4];
        acc_y = coeffs_y' * [0;  0;      2;      6 * t;  12 * t ^ 2;  20 * t ^ 3];
        
        conditions_z = [waypoint(i, 3) 0 0 waypoint(i + 1, 3) 0 0]';
        coeffs_z = T \ conditions_z;
        pos_z = coeffs_z' * [1;  t;  t ^ 2;      t ^ 3;       t ^ 4;       t ^ 5];
        vel_z = coeffs_z' * [0;  1;  2 * t;  3 * t ^ 2;   4 * t ^ 3;   5 * t ^ 4];
        acc_z = coeffs_z' * [0;  0;      2;      6 * t;  12 * t ^ 2;  20 * t ^ 3];

        pos = [pos_x; pos_y; pos_z];
        vel = [vel_x; vel_y; vel_z];
        acc = [acc_x; acc_y; acc_z];
      
      % locate in the ith segment and segment distance is samller than 1, s.t. no need to compute polynomial
      elseif t_start(i) <= t && t < t_end(i) && path_flag(i) == 1
        pos = waypoint(i + 1, :);
        vel = [0 0 0];
        acc = [0 0 0];
      
      end 
    end
  
  % terminal state
  else 
    pos = [waypoint(end, 1); waypoint(end, 2); waypoint(end, 3)];
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