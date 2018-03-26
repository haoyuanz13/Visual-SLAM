function [error, f] = GeoError(x, X, ks, K, Rs, ts)
% Measure a geometric error
%
%   x:  2D points. n x 2 x N matrix, where n is the number of corners in
%   a checkerboard and N is the number of calibration images
%       
%   X:  3D points. n x 2 matrix, where n is the number of corners in a
%   checkerboard and assumes the points are on the Z=0 plane
%
%   K: a camera calibration matrix. 3 x 3 matrix.
%
%   Rs: rotation matrices. 3 x 3 x N matrix, where N is the number of calibration images. 
%
%   ts: translation vectors. 3 x 1 x N matrix, where N is the number of calibration images. 
%
%   ks: radial distortion parameters. 2 x 1 matrix, where ks(1) = k_1 and
%   ks(2) = k_2.
%

%% Your code goes here
N = size(x, 3);
n = size(X, 1);

error = 0;
f = zeros(2, N, n);
px = K(1, 3);
py = K(2, 3);
kk = K;
kk(1, 2) = 0;

pos_phy = [X'; zeros(1, n); ones(1, n)];

for i = 1 : N
  u_img = x(:, 1, i);
  v_img = x(:, 2, i);
  R_cur = Rs(:, :, i);
  t_cur = ts(:, :, i);
  
  proj = [R_cur, t_cur] * pos_phy;
  a = proj(1, :) ./ proj(3, :);
  b = proj(2, :) ./ proj(3, :);
  r_s = a .^ 2 + b .^ 2;
  r_ss = r_s .^ 2;
  factor = 1 + ks(1) .* r_s + ks(2) .* r_ss;
  
  proj_undis = kk * [R_cur, t_cur] * pos_phy;
  u_proj_dis = (factor .* (proj_undis(1, :) ./ proj_undis(3, :) - px)) + px;
  v_proj_dis = (factor .* (proj_undis(2, :) ./ proj_undis(3, :) - py)) + py;
  
  f(1, i, :) = u_img' - u_proj_dis;
  f(2, i, :) = v_img' - v_proj_dis;
  
  error_x = sum(f(1, i, :) .^ 2);
  error_y = sum(f(2, i, :) .^ 2);
  
  error = error + error_x + error_y;    

end
end