function [ Rs, ts ] = EstimateRt_linear( Hs, K )
% Linear parameter estimation of R and t
%
%   K: a camera calibration matrix. 3 x 3 matrix.
%
%   Hs: a homography from the world to images. 3 x 3 x N matrix, where N is 
%   the number of calibration images. 
%
%   Rs: rotation matrices. 3 x 3 x N matrix, where N is the number of calibration images. 
%
%   ts: translation vectors. 3 x 1 x N matrix, where N is the number of calibration images. 
%

%% Your code goes here.
N = size(Hs, 3);
Rs = zeros(3, 3, N);
%%
% *BOLD TEXT*
ts = zeros(3, 1, N);

for i = 1 : N
  H = Hs(:, :, i);
  z = norm(K \ H(:, 1));
  
  r1 = (K \ H(:, 1)) ./ z;
  r2 = (K \ H(:, 2)) ./ z;
  r3 = cross(r1, r2);
  t = (K \ H(:, 3)) ./ z;
  
  Rs(:, :, i) = [r1, r2, r3];
  ts(:, :, i) = t;
end

end

