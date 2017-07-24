function [Cset, Rset, Xset] = BundleAdjustmentWithAnalysisJacobian(Cset, Rset, num_camera, X, K, Mu, Mv, V, ind_estedPC)
%% construct parameters for optimization function
% the parameter should be a vector contains camera and 3D points information
% input X should be N x 3 size
% return Xset should be N x 3 size
% training parameter's size should be 
% (7 x number of cameras) + (3 x number of 3D points)
num_points = size(X, 1);
params_ori = zeros(1, 7 * num_camera + 3 * num_points);
for i = 1 : num_camera
    quat = rotm2quat(Rset{i});
    q1 = quat(1); q2 = quat(2); q3 = quat(3); q4 = quat(4);
    params_ori(7 * (i - 1) + 1 : 7 * (i - 1) + 4) = [q1, q2, q3, q4];
    params_ori(7 * (i - 1) + 5 : 7 * (i - 1) + 7) = Cset{i}';
end
% add 3D points information
ravel_X = reshape(X', 1, num_points * 3);
params_ori(7 * num_camera + 1: end) = ravel_X;

opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-25, 'MaxIter', 300, 'Jacobian', 'on', 'Display', 'iter');
e = @(params)error_reprojectionBAwithAnaJaco(params, num_camera, num_points, K, Mu, Mv, V, ind_estedPC);
params_opt = lsqnonlin(e, params_ori, [], [], opts); 
%% roll parameters into rotation, translation form
for i = 1 : num_camera
    quat_cur = params_opt(7 * (i - 1) + 1 : 7 * (i - 1) + 4);
    Rset{i} = quat2rotm(quat_cur);
    Cset{i} = (params_opt(7 * (i - 1) + 5 : 7 * (i - 1) + 7))';
end
%% update X set
Xset = (reshape(params_opt(7 * num_camera + 1 : end), 3, num_points))';
end