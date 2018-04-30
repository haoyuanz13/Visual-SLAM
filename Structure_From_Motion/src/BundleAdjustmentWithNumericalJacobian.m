function [Cset, Rset, Xset] = BundleAdjustmentWithNumericalJacobian(Cset, Rset, num_camera, X, K, haveEstPC, Mu, Mv, V, ind_estedPC)
%% construct parameters for optimization function
% the parameter should be a vector contains camera and 3D points information
% input X should be N x 3 size
% return Xset should be N x 3 size
% training parameter's size should be 
% (7 x number of cameras) + (4 x number of 3D points)

num_points = size(X, 1);
params0 = zeros(1, 7 * num_camera + 3 * num_points);

for i = 1 : num_camera
  quat = rotm2quat(Rset{i});
  q1 = quat(1); q2 = quat(2); q3 = quat(3); q4 = quat(4);
  params0(7 * (i - 1) + 1 : 7 * (i - 1) + 4) = [q1, q2, q3, q4];
  params0(7 * (i - 1) + 5 : 7 * (i - 1) + 7) = Cset{i}';
end

% add 3D points information
ravel_X = reshape(X', 1, num_points * 3);
params0(7 * num_camera + 1: end) = ravel_X;

% optimization process
f=@(params)err_reproject_BA(K, Mu, Mv, V, num_camera, num_points, ind_estedPC, params);
[Jstr, ~, ~] = createJacobianBA(num_camera, V, haveEstPC);

% 'trust-region-dogleg', 'trust-region', or 'levenberg-marquardt'.
% opts = optimset('Display', 'iter', 'JacobPattern', Jstr, 'Algorithm', ...
%     'trust-region-reflective', 'PrecondBandWidth', 1);

opts = optimoptions(@fsolve, 'Display', 'iter', 'JacobPattern', Jstr, 'Algorithm', ...
            'trust-region-reflective', 'PrecondBandWidth', 1 , 'MaxIter', 60);

params_opt = fsolve(f, params0, opts);

%% roll parameters into rotation, translation form
for i = 1 : num_camera
  quat_cur = params_opt(7 * (i - 1) + 1 : 7 * (i - 1) + 4);
  R_cur = quat2rotm(quat_cur);
  C_cur = params_opt(7 * (i - 1) + 5 : 7 * (i - 1) + 7);
    
  Rset{i} = R_cur; Cset{i} = C_cur';
end

%% update X set
Xset = (reshape(params_opt(7 * num_camera + 1 : end), 3, num_points))';

end

%% function to compute reprojection error
% error vector form should be consistent with Jacobian form
function [error] = err_reproject_BA(K, Mu, Mv, V, num_camera, num_points, ind_estedPC, params)
error = [];
offset_ind = 7 * num_camera;

for ind_3D = 1 : num_points
  X_cur = (params(offset_ind + 1 + 3 * (ind_3D - 1) : offset_ind + 3 * ind_3D))';
  
  for ind_camera = 1 : num_camera
    % this camera doesn't detect the 3D point
    if V(ind_estedPC(ind_3D), ind_camera) == 0
      continue;
    end
    % camera extrinsic matrix
    quat_cur = params(7 * (ind_camera - 1) + 1 : 7 * (ind_camera - 1) + 4);
    R_cur = quat2rotm(quat_cur);
    C_cur = (params(7 * (ind_camera - 1) + 5 : 7 * (ind_camera - 1) + 7))';
    P_cur = K * R_cur * [eye(3), -C_cur];

    % compute reprojection error 
    u_cur = Mu(ind_estedPC(ind_3D), ind_camera); 
    v_cur = Mv(ind_estedPC(ind_3D), ind_camera);

    reproject = P_cur * [X_cur; 1];
    u_repro = bsxfun(@rdivide, reproject(1), reproject(3));
    v_repro = bsxfun(@rdivide, reproject(2), reproject(3));
    error = [error, u_cur - u_repro, v_cur - v_repro];
  end  % end inner for

end  % end outer for

end
