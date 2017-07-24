function [Cset, Rset] =  NonlinearPnPWithJacobian(Cset, Rset, K, Mu, Mv, PointCloud, haveEstPC, V)
num_camera = size(Cset, 2);
params0 = zeros(1, 7 * num_camera);
for i = 1 : num_camera
    quat = rotm2quat(Rset{i});
    q1 = quat(1); q2 = quat(2); q3 = quat(3); q4 = quat(4);
    params0(7 * (i - 1) + 1 : 7 * (i - 1) + 4) = [q1, q2, q3, q4];
    params0(7 * (i - 1) + 5 : 7 * (i - 1) + 7) = Cset{i}';
end

%% optimization process
ind_estedPC = find(haveEstPC == 1);
num_points = length(ind_estedPC);
f = @(params)err_reproject_NonPnP(params, num_camera, num_points, K, PointCloud, Mu, Mv, V, ind_estedPC);
Jstr = createJacobianNonPnP((1 : num_camera), V, ind_estedPC);

opts = optimoptions(@fsolve, 'Display', 'off', 'JacobPattern', Jstr, 'Algorithm', ...
            'trust-region-reflective', 'PrecondBandWidth', 1 , 'MaxIter', 1000000);
params_opt = fsolve(f, params0, opts);

%% roll parameters into rotation, translation form
for i = 1 : num_camera
    quat_cur = params_opt(7 * (i - 1) + 1 : 7 * (i - 1) + 4);
    R_cur = quat2rotm(quat_cur);
    C_cur = params_opt(7 * (i - 1) + 5 : 7 * (i - 1) + 7);
    
    Rset{i} = R_cur; Cset{i} = C_cur';
end
end

%% function to compute reprojection error
function [error] = err_reproject_NonPnP(params, num_camera, num_points, K, PointCloud, Mu, Mv, V, ind_estedPC)
error = [];
for ind_3D = 1 : num_points
    X_cur = (PointCloud(ind_estedPC(ind_3D), :))';
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
    end
end
end