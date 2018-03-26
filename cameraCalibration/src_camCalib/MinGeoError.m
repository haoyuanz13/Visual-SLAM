function [ ks_opt, K_opt, Rs_opt, ts_opt ] = MinGeoError( x, X, ks_init, K_init, Rs_init, ts_init )
% Non-linear optimization to esimate K, R, t, ks
%
%   x:  2D points. n x 2 x N matrix, where n is the number of corners in
%   a checkerboard and N is the number of calibration images
%       
%   X:  3D points. n x 2 matrix, where n is the number of corners in a
%   checkerboard and assumes the points are on the Z=0 plane
%
%   K_init: a camera calibration matrix. 3 x 3 matrix.
%
%   Rs_init: rotation matrices. 3 x 3 x N matrix, where N is the number of calibration images. 
%
%   ts_init: translation vectors. 3 x 1 x N matrix, where N is the number of calibration images. 
%
%   ks_init: radial distortion parameters. 2 x 1 matrix, where ks_init(1) = k_1 and
%   ks_init(2) = k_2.
%
%   ks_opt/K_opt/Rs_opt/ts_opt: optimized parameters of ks, K, Rs, and ts,
%   respectively.


params0 = encodeParams(ks_init, K_init, Rs_init, ts_init);
opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, 'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'iter');

%% Your code goes here. You can use any functions defined below.
fun = @(par)GeoError_cb(par, x, X);
params_opt = lsqnonlin(fun, params0, [], [], opts);
[ks_opt, K_opt, Rs_opt, ts_opt] = decodeParams(params_opt, size(x, 3));

end

function [params_encoded] = encodeParams(ks, K, R, t)
q = R2q(R);
params_encoded = [...
  K(1, 1); ... % fx
  K(2, 2); ... % fy
  K(1, 2); ... % s
  K(1, 3); ... % px
  K(2, 3); ... % py
  q(:); ...    % R
  t(:); ...    % t
  ks(:) ...    % k1, k2
  ];
end

function [ks, K, Rs, ts] = decodeParams(params_encoded, I)
  %% K
  K = zeros(3, 3);
  K(1, 1) = params_encoded(1);
  K(2, 2) = params_encoded(2);
  K(1, 2) = params_encoded(3);
  K(1, 3) = params_encoded(4);
  K(2, 3) = params_encoded(5);
  K(3, 3) = 1;
  params_encoded = params_encoded(6:end);
  
  %% R
  q = reshape(params_encoded(1:4*I), [4, 1, I]);
  Rs = q2R(q);
  params_encoded = params_encoded(4*I+1:end);
  
  %% ts
  ts = reshape(params_encoded(1:3*I), [3, 1, I]);
  params_encoded = params_encoded(3*I+1:end);
  
  %% ks
  ks = params_encoded;
  params_encoded = params_encoded(3:end);
  
  assert(isempty(params_encoded));
end

function f = GeoError_cb(params, x, X)
  [ks, K, Rs, ts] = decodeParams(params, size(x, 3));
  [~, f] = GeoError(x, X, ks, K, Rs, ts);
end

