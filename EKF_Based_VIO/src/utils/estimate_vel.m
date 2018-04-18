function [vel] = estimate_vel(sensor, R_world2cam, t_cam2world, input)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
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
%   input - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor

%% define some persistent variables for the iterative estimation
persistent is_the_first_frame;  % label whether the current frame is the first frame
persistent time_pre;  % the time stamp of previous keyframe
persistent dt_pre;  % the previous time interval
persistent dt_cur;  % the current time interval
persistent vel_pre;  % the previous estimated linear velocity (for filter usage)
persistent CornersTracker;  % the object to achieve KLT feature tracking
persistent num_corner;  % the number of detected corners in each frame
persistent minQualityVal;  % the min allowed corner quality
persistent img_pre;  % the previous captured image
persistent corner_pos_pre;  % all detected and valid corners position in the previous keyframe
persistent top_percent;  % select relative top qualified corners


%% define some variables 
if isempty(is_the_first_frame)
  is_the_first_frame = true;  % label flag

  num_corner = 100;  % initialize the target detected corner number
  minQualityVal = 0.6;  % initialize the min corner quality value
  top_percent = 0.7;

else
  is_the_first_frame = false;
end

% set factors for velocities
factor_vel = [0.4; 0.45; 0.35];

% set factor for time stamp
factor_t = 0.51;

%% obtain the current captured image, if empty, return empty;
img_cur = sensor.img;

if isempty(img_cur)
  vel = [];

%% detect a valid keyframe
else
  % suppose the static initial state
  vel_cur = [0; 0; 0]; vel = zeros(3, 1); 
  dt_cur = 0;

  % deal with the following frames using the optical flow
  if (~is_the_first_frame) && sensor.is_ready == 1
    % filter out the time stamp
    dt_cur = sensor.t - time_pre;
    if dt_pre == 0
      dt_filtered = dt_cur;
    else
      dt_filtered = factor_t * dt_pre + (1 - factor_t) * dt_cur;
    end
    
    %% check whether the current frame exists any detected tag
    tag_num = size(sensor.id, 2);

    %% detected valid tags and features such that implements KLT
    if ~isempty(corner_pos_pre) && tag_num > 0 
      % estimate the velocity: KLT tracker
      CornersTracker = vision.PointTracker('MaxBidirectionalError', 1);
      initialize(CornersTracker, corner_pos_pre, img_pre);
      [corner_pos_cur, corner_valid, corner_score] = step(CornersTracker, img_cur);

      num_pre_corner = size(corner_pos_pre, 1); 
      % back project previous corners into 3D rays
      corner_pos_pre_homo = [corner_pos_pre ones(num_pre_corner, 1)]';
      corner_xyZ_pre = input{1}.K \ corner_pos_pre_homo;
      % back project current tracked corners into 3D rays 
      corner_pos_cur_homo = [corner_pos_cur ones(num_pre_corner, 1)]';
      corner_xyZ_cur = input{1}.K \ corner_pos_cur_homo;  % 3xN               
    
      % compute p_dot
      x_dot = (corner_xyZ_cur(1, :) - corner_xyZ_pre(1, :)) / dt_filtered;
      y_dot = (corner_xyZ_cur(2, :) - corner_xyZ_pre(2, :)) / dt_filtered;
    
      % compute the depth Z
      % homography H to map corners in the AprilTags into camera frame
      r1 = R_world2cam(:, 1); r2 = R_world2cam(:, 2); r3 = R_world2cam(:, 3); 
      H_world2cam = [r1 r2 t_cam2world];
    
      temp = (input{1}.K * H_world2cam) \ corner_pos_cur_homo;
      corner_xyZ_cur(3, :) = 1 ./ temp(3, :);
    
      % filter out invalid and poor-qualified corners
      valid_ind = (corner_valid == 1);
    
      corner_xyZ_valid = corner_xyZ_cur(:, valid_ind);
      x_dot_valid = x_dot(valid_ind);
      y_dot_valid = y_dot(valid_ind);
      corner_score_valid = corner_score(valid_ind);
    
      % create a corner information table for sorting and search
      num_valid = length(x_dot_valid);
      corner_table = zeros(num_valid, 6);
    
      % [x, y, Z, x_dot, y_dot, score]
      corner_table(:, 1 : 3) = corner_xyZ_valid';
      corner_table(:, 4) = x_dot_valid'; 
      corner_table(:, 5) = y_dot_valid'; 
      corner_table(:, 6) = corner_score_valid; 
    
      % sort corners in descending order according to the score
      corner_table = sortrows(corner_table, -6);
    
      num_selection = floor(num_valid * top_percent);  % otherwise choose some well-qualified points

      % construct linear equation to solve Fx = p_dot
      p_dot = zeros(2 * num_selection, 1);  % solution vector: 2N x 1
      F_func = zeros(2 * num_selection, 6);  % coefficient matrix: 2N x 6
    
      % vectorization approach to build F and p_dot matrix 
      x_vec = corner_table(1 : num_selection, 1);
      y_vec = corner_table(1 : num_selection, 2);
      Z_vec = corner_table(1 : num_selection, 3);
      x_dot_vec = corner_table(1 : num_selection, 4);
      y_dot_vec = corner_table(1 : num_selection, 5);

      p_dot(1 : num_selection) = x_dot_vec;
      p_dot(num_selection + 1 : 2 * num_selection) = y_dot_vec;

      F_func(1 : num_selection, :) = [-1 ./ Z_vec, zeros(num_selection, 1), x_vec ./ Z_vec, x_vec .* y_vec, -(1 + x_vec .^ 2), y_vec]; 
      F_func(num_selection + 1 : 2 * num_selection, :) = [zeros(num_selection, 1), -1 ./ Z_vec, y_vec ./ Z_vec, 1 + y_vec .^ 2, -x_vec .* y_vec,  -x_vec];    

      vel_omg_vec = F_func \ p_dot;

      vel_cur = R_world2cam' * vel_omg_vec(1 : 3);  % linear velocity
      omg_cur = R_world2cam' * vel_omg_vec(4 : 6);  % angular velocity
      vel_cur = vel_cur - cross(R_world2cam' * input{1}.t_c2i, omg_cur);

      % filter the estimated velocities
      vel = factor_vel .* vel_pre + (1 - factor_vel) .* vel_cur;
    
    end
  
  end

  %% detect and refine features in the current keyframe
  corners = detectFASTFeatures(img_cur, 'MinQuality', minQualityVal);
  corners_filtered = corners.selectStrongest(num_corner);

  %% assgin persistent variables for the iterative usage
  corner_pos_pre = corners_filtered.Location;
  img_pre = img_cur;

  time_pre = sensor.t;
  dt_pre = dt_cur;

  vel_pre = vel_cur;

end

end
