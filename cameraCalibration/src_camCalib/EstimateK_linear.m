function [K, Hs] = EstimateK_linear(x, X)
% Linear parameter estimation of K
%
%   x:  2D points. n x 2 x N matrix, where n is the number of corners in
%   a checkerboard and N is the number of calibration images
%       
%   X:  3D points. n x 2 matrix, where n is the number of corners in a
%   checkerboard and assumes the points are on the Z=0 plane
%
%   imgs: calibration images. N x 1 cell, where N is the number of calibration images
%
%   K: a camera calibration matrix. 3 x 3 matrix.
%
%   Hs: a homography from the world to images. 3 x 3 x N matrix, where N is 
%   the number of calibration images. You can use est_homography.m to
%   estimate homographies.
%

%% Your code goes here
N = size(x, 3);
A = zeros(2 * N, 6);
Hs = zeros(3, 3, N);

for i = 1 : 2 : 2 * N - 1
    ind = floor(i / 2) + 1;
    H = EstimateHomography(x(:, 1, ind), x(:, 2, ind), X(:, 1), X(:, 2));
    Hs(:, :, ind) = H;
    
    v11 = [H(1,1) * H(1,1); H(1,1) * H(2,1) + H(2,1) * H(1,1); H(2,1) * H(2,1); H(3,1) * H(1,1) + H(1,1) * H(3,1); H(3,1) * H(2,1) + H(2,1) * H(3,1); H(3,1) * H(3,1)];
    v22 = [H(1,2) * H(1,2); H(1,2) * H(2,2) + H(2,2) * H(1,2); H(2,2) * H(2,2); H(3,2) * H(1,2) + H(1,2) * H(3,2); H(3,2) * H(2,2) + H(2,2) * H(3,2); H(3,2) * H(3,2)];
    v12 = [H(1,1) * H(1,2); H(1,1) * H(2,2) + H(2,1) * H(1,2); H(2,1) * H(2,2); H(3,1) * H(1,2) + H(1,1) * H(3,2); H(3,1) * H(2,2) + H(2,1) * H(3,2); H(3,1) * H(3,2)];
    
    A(i, :) = v11' - v22';
    A(i + 1, :) = v12';
end

[u, d, v] = svd(A);
B = v(:, end);

py = (B(2) * B(4) - B(1) * B(5)) / (B(1) * B(3) - B(2) .^ 2);
c = B(6) - ((B(4) .^ 2 + py * (B(2) * B(4) - B(1) * B(5))) / B(1));

fy = sqrt((c * B(1)) / (B(1) * B(3) - B(2) .^ 2));
fx = sqrt(c / B(1));

s = -(B(2) * (fx .^ 2) * fy) / c;
px = (s * py / fy) - (B(4) * (fx .^ 2) / c);
    
K = [fx, s, px; 0, fy, py; 0, 0, 1];

end