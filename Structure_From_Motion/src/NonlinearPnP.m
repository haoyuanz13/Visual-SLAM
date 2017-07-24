function [Cset, Rset] = NonlinearPnP(Cset, Rset, curImage, X, x, K, C_ori, R_ori)
%% Nonlinear PnP to update camera pose
N = length(X);
X_homo = [X, ones(N, 1)];
% construct parameters vector using quaternion
q_ori = rotm2quat(R_ori);
param_ori = [q_ori(1), q_ori(2), q_ori(3), q_ori(4), C_ori(1), C_ori(2), C_ori(3)];
% parameters for optimization function 
opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, ...
    'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');

e = @(params)err_reproject_nonlinearPnP(X_homo, x, K, params);
param_opt = lsqnonlin(e, param_ori, [], [], opts); 
% convert parameters into rotation and translation
quat_opt = param_opt(1 : 4);
% R = quat2rotm(quat_opt);
% C = (param_opt(5 : 7))';

Cset{curImage} = (param_opt(5 : 7))';
Rset{curImage} = quat2rotm(quat_opt);
end


function [error] = err_reproject_nonlinearPnP(X, x, K, param)
% input X should be a single 3D point (Nx4)
% input x should be 2d position which is (Nx2)
% input K should be intrinsic matrix which is (3x3)
% input parameters shoud be 1x7 vector contains quaternions and translation
quat = param(1 : 4);
R = quat2rotm(quat);
C = (param(5 : 7))';
P = K * R * [eye(3), -C];

reproject = P * X';
u = x(:, 1); v = x(:, 2);
error = [(u - (bsxfun(@rdivide, reproject(1, :), reproject(3, :)))'),...
    (v - (bsxfun(@rdivide, reproject(2, :), reproject(3, :)))')];
% error = abs(error);
end