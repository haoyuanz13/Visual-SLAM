function [velocity] = RANSAC(error, k, cnew, c, Zinv, dt)
% error - the maximum allowed error for a point to be considered an inlier 
% k - the number of iteration
% cnew - new coodinates of the points tracked - 3 x N
% c - original coordinates of the points tracked - 3 x N
% Zinv - inverse of z - 1 x N
% dt - time difference between when cnew and c are tracked 

A = zeros(2 * size(cnew, 2), 6);  % the coefficient matrix can be an argument 

p_dot = (cnew - c) / dt;  % estimate optical flow - 3 x N

% construct coefficient matrix
for i = 1 : size(cnew, 2)
  A(2 * i - 1 : 2 * i, :) = [...
        -Zinv(i), 0, c(1, i ) * Zinv(i), c(1, i) * c(2, i), -1 - (c(1, i)) ^ 2, c(2, i);
        0, -Zinv(i), c(2, i) * Zinv(i), 1 + c(2, i) ^ 2, -c(1, i) * c(2, i,) -c(1, i)];
end

max_num_inliers = 0;
opt_inliers_ind = [];

p_dot(3, :) = []; % make shape become 2 x N

% RANSAC iteration
for i = 1 : k
  [p, pindex] = datasample(p_dot', 3);  % sample 3 points randomly to estimate velocity
    
  A0 = zeros(6, 6);  % coefficient matrix
  for j = 1 : 3
    A0(2 * j - 1 : 2 * j, :) = A(2 * pindex(j) - 1 : 2 * pindex(j), :);
  end
    
  p = p';
  v_est = A0 \ p(:);  % estimate velocity
  
  p_dot_est = A * v_est;  % recompute p_dot
  
  e = p_dot(:) - p_dot_est;  % calculate error 
  e = reshape(e, 2, size(cnew, 2));
  
  e = e(1, :) .^ 2 + e(2, :) .^ 2;  
  inliers_index = find(e < error);  % find inliers
  cur_num_inliers = size(inliers_index, 2);
  
  if cur_num_inliers >= max_num_inliers
    opt_inliers_ind = inliers_index;
    max_num_inliers = cur_num_inliers;
  end

end

%% use all inliers to re-estimate an optimal velocity
inlier = p_dot(:, opt_inliers_ind);
A_inlier = zeros(2 * size(ind, 2), 6);

for i = 1 : size(opt_inliers_ind, 2)
  A_inlier(2 * i - 1 : 2 * i, :) = A(2 * opt_inliers_ind(i) - 1 : 2 * opt_inliers_ind(i), :);
end

velocity = A_inlier \ inlier(:);
end
