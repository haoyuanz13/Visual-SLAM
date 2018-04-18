function [tags_3d_X, tags_3d_Y] = obtain3DCorners(num_x, num_y, tag_size, inter_s, inter_l)
% Create a search table containing all corners in 3D space
% tags_3d_X/Y: [num_tag x 5]
% tags_3d(i, :) = [p0_i, p1_i, p2_i, p3_i, p4_i]

num_total = num_x * num_y;

% if the map is empty
if num_total <= 0
  tags_3d_X = []; 
  tags_3d_Y = [];

% more common case
else
  tags_3d_X = zeros(num_total, 5); 
  tags_3d_Y = zeros(num_total, 5);

  % create grids
  [y, x] = meshgrid((1 : 1 : num_y), (1 : 1 : num_x));
  
  %% calculate p4 coordinate in Y axis 
  map_p4_y = (y - 1) .* tag_size;  % all previous tags length sum
  map_p4_y = map_p4_y + (y - 1) .* inter_s;  % all previous intervals sum 
  
  % special case between column 3, 4 and column 6, 7 with larger interval distance
  delta_inter = inter_l - inter_s;
  map_p4_y(:, 4 : 6) = map_p4_y(:, 4 : 6) + delta_inter;
  map_p4_y(:, 7 : 9) = map_p4_y(:, 7 : 9) + 2 * delta_inter;

  % assign to the return matrix (Y coordiante)
  tags_3d_Y(:, 5) = map_p4_y(:); 
  
  % calculate Y coordinates based on known p4_y
  tags_3d_Y(:, 1) = tags_3d_Y(:, 5) + tag_size / 2;  % p0_y 
  tags_3d_Y(:, 2) = tags_3d_Y(:, 5);                 % p1_y
  tags_3d_Y(:, 3) = tags_3d_Y(:, 5) + tag_size;      % p2_y
  tags_3d_Y(:, 4) = tags_3d_Y(:, 3);                 % p3_y
  
  %% calculate p4 coordinate in X axis  
  map_p4_x = (x - 1) .* tag_size;  % all previous tags length sum
  map_p4_x = map_p4_x + (x - 1) .* inter_s;  % all previous intervals sum 

  % assign to the return matrix (X coordiante)
  tags_3d_X(:, 5) = map_p4_x(:); 
  
  % calculate X coordinates based on known p4_x
  tags_3d_X(:, 1) = tags_3d_X(:, 5) + tag_size / 2;  % p0_x 
  tags_3d_X(:, 2) = tags_3d_X(:, 5) + tag_size;      % p1_x
  tags_3d_X(:, 3) = tags_3d_X(:, 2);                 % p2_x
  tags_3d_X(:, 4) = tags_3d_X(:, 5);                 % p3_x
  
end
end
