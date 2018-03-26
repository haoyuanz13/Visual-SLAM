function [ks] = EstimateDistort_linear(x, X, K, Rs, ts)
% Linear parameter estimation of k
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
num_points = size(X, 1);
pos_phy = [X'; zeros(1, num_points); ones(1, num_points)];
N = size(x ,3);
kk = K;
kk(1, 2) = 0;
px = K(1, 3);
py = K(2, 3);

A = zeros(2 * num_points * N, 2);
B = zeros(2 * num_points * N, 1);

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
    
    pos_ideal = kk * proj;
    u_ideal = pos_ideal(1, :) ./ pos_ideal(3, :);
    v_ideal = pos_ideal(2, :) ./ pos_ideal(3, :);
    
    start = 2 * (i - 1) * num_points + 1;
    B(start : 2 : start + 2 * num_points - 2, :) = u_img - u_ideal';
    B(start + 1 : 2 : start + 2 * num_points - 1, :) = v_img - v_ideal';
    
    u_ideal = u_ideal - px;
    v_ideal = v_ideal - py;
    
    term_u1 = u_ideal .* r_s;
    term_u2 = u_ideal .* r_ss;
    term_u = [term_u1; term_u2]';
    
    term_v1 = v_ideal .* r_s;
    term_v2 = v_ideal .* r_ss;
    term_v = [term_v1; term_v2]';
    
    A(start : 2 : start + 2 * num_points - 2, :) = term_u;
    A(start + 1 : 2 : start + 2 * num_points - 1, :) = term_v;
end
ks = A \ B;
end