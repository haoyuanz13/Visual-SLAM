function [error, J] = error_reprojectionBAwithAnaJaco(params, num_camera, num_points, K, Mu, Mv, V, ind_estedPC)
error = [];
offset_ind = 7 * num_camera;
% construct several cells to store camera parameters and 3D points
X_set = cell(num_points, 1);
q_set = cell(num_camera, 1); R_set = cell(num_camera, 1); C_set = cell(num_camera, 1);
%% compute reprojection error
num_obs = 0;  % count the number of observations
for ind_3D = 1 : num_points
    X_cur = (params(offset_ind + 1 + 3 * (ind_3D - 1) : offset_ind + 3 * ind_3D))';
    X_set{ind_3D} = X_cur;
    for ind_camera = 1 : num_camera
        % this camera doesn't detect the 3D point
        if V(ind_estedPC(ind_3D), ind_camera) == 0
            continue;
        end
        num_obs = num_obs + 1; % count the observation number
        % camera extrinsic matrix
        quat_cur = params(7 * (ind_camera - 1) + 1 : 7 * (ind_camera - 1) + 4);
        R_cur = quat2rotm(quat_cur);
        C_cur = (params(7 * (ind_camera - 1) + 5 : 7 * (ind_camera - 1) + 7))';
        P_cur = K * R_cur * [eye(3), -C_cur];
        % store camera parameters
        q_set{ind_camera} = quat_cur; R_set{ind_camera} = R_cur; C_set{ind_camera} = C_cur; 
        % compute reprojection error 
        u_cur = Mu(ind_estedPC(ind_3D), ind_camera); 
        v_cur = Mv(ind_estedPC(ind_3D), ind_camera);
        
        reproject = P_cur * [X_cur; 1];
        u_repro = bsxfun(@rdivide, reproject(1), reproject(3));
        v_repro = bsxfun(@rdivide, reproject(2), reproject(3));
        error = [error, u_cur - u_repro, v_cur - v_repro];
    end
end
%% calculate some constant variables 
d_R2q = cell(num_camera, 1);
U_scale = cell(num_camera, 1); V_scale = cell(num_camera, 1); W_scale = cell(num_camera, 1);
d_U2X = cell(num_camera, 1); d_V2X = cell(num_camera, 1); d_W2X = cell(num_camera, 1);
d_U2C = cell(num_camera, 1); d_V2C = cell(num_camera, 1); d_W2C = cell(num_camera, 1);
fu = K(1, 1); fv = K(2, 2); pu = K(1, 3); pv = K(2, 3);

for i = 1 : num_camera
    q_cur = q_set{i}; R_cur = R_set{i};
    q1 = q_cur(1); q2 = q_cur(2); q3 = q_cur(3); q4 = q_cur(4);
    d_R2q{i} = [0, 0, -4 * q3, -4 * q4; -2 * q4, 2 * q3, 2 * q2, -2 * q1; 2 * q3, 2 * q4, 2 * q1, 2 * q2; 
                2 * q4, 2 * q3, 2 * q2, 2 * q1; 0, -4 * q2, 0, -4 * q4; -2 * q2, -2 * q1, 2 * q4, 2 * q3;
                -2 * q3, 2 * q4, -2 * q1, 2 * q2; 2 * q2, 2 * q1, 2 * q4, 2 * q3; 0, -4 * q2, -4 * q3, 0];
    
    U_scale{i} = [fu * R_cur(1, 1) + pu * R_cur(3, 1), fu * R_cur(1, 2) + pu * R_cur(3, 2), fu * R_cur(1, 3) + pu * R_cur(3, 3)];
    V_scale{i} = [fv * R_cur(2, 1) + pv * R_cur(3, 1), fv * R_cur(2, 2) + pv * R_cur(3, 2), fv * R_cur(2, 3) + pv * R_cur(3, 3)];
    W_scale{i} = [R_cur(3, 1), R_cur(3, 2), R_cur(3, 3)];
    
    d_U2X{i} = U_scale{i}; d_V2X{i} = V_scale{i}; d_W2X{i} = W_scale{i};
    d_U2C{i} = -U_scale{i}; d_V2C{i} = -V_scale{i}; d_W2C{i} = -W_scale{i};
end
%% create Jacobian
% calculate row and column number
num_col = 7 * num_camera + num_points * 3;
num_row = 2 * num_obs;
% create a sparse matrix 
J = sparse(num_row, num_col); 
% create indices arraies
encode = find((V(ind_estedPC, 1 : num_camera) == 1)');
% construct camera_indices
camera_indices = mod(encode, num_camera);
camera_indices(camera_indices == 0) = num_camera;
camera_indices = camera_indices';
% construct point_indices
point_indices = (ceil(encode / num_camera))';
% fill up Jacobian matrix
for i = 1 : num_obs
    indcam = camera_indices(i); indpc = point_indices(i);
    C_cur = C_set{indcam}; X_cur = X_set{indpc};
    diff = X_cur - C_cur; 
    U_cur = U_scale{indcam} * diff; V_cur = V_scale{indcam} * diff; W_cur = W_scale{indcam} * diff;
    
    %% fill up f/q
    indr = repmat(2 * (i - 1) + 1, 1, 4);
    indc = 7 * (indcam - 1) + 1 : 7 * (indcam - 1) + 4;
    valu = (([fu * diff', zeros(1, 3), pu * diff'] / W_cur) - (U_cur * [zeros(1, 6), diff'] / (W_cur .^ 2))) * d_R2q{indcam};
    valv = (([zeros(1, 3), fv * diff', pv * diff'] / W_cur) - (V_cur * [zeros(1, 6), diff'] / (W_cur .^ 2))) * d_R2q{indcam};
    
    J = J + sparse(indr, indc, valu, num_row, num_col);
    J = J + sparse(indr + 1, indc, valv, num_row, num_col);
%     J(2 * (i - 1) + 1, 7 * (indcam - 1) + 1 : 7 * (indcam - 1) + 4) = ...
%         (([fu * diff', zeros(1, 3), pu * diff'] / W_cur) - (U_cur * [zeros(1, 6), diff'] / (W_cur .^ 2))) * d_R2q{indcam};
%     J(2 * i, 7 * (indcam - 1) + 1 : 7 * (indcam - 1) + 4) = ...
%         (([zeros(1, 3), fv * diff', pv * diff'] / W_cur) - (V_cur * [zeros(1, 6), diff'] / (W_cur .^ 2))) * d_R2q{indcam};
    
    %% fill up f/C
    indr = repmat(2 * (i - 1) + 1, 1, 3);
    indc = 7 * (indcam - 1) + 5 : 7 * (indcam - 1) + 7;
    valu = d_U2C{indcam} / W_cur - U_cur * d_W2C{indcam} / (W_cur .^ 2);
    valv = d_V2C{indcam} / W_cur - V_cur * d_W2C{indcam} / (W_cur .^ 2);
    
    J = J + sparse(indr, indc, valu, num_row, num_col);
    J = J + sparse(indr + 1, indc, valv, num_row, num_col);
%     J(2 * (i - 1) + 1, 7 * (indcam - 1) + 5 : 7 * (indcam - 1) + 7) = ...
%         d_U2C{indcam} / W_cur - U_cur * d_W2C{indcam} / (W_cur .^ 2);
%     J(2 * i, 7 * (indcam - 1) + 5 : 7 * (indcam - 1) + 7) = ...
%         d_V2C{indcam} / W_cur - V_cur * d_W2C{indcam} / (W_cur .^ 2);
    %% fill up f/X
    indr = repmat(2 * (i - 1) + 1, 1, 3);
    indc = offset_ind + 1 + 3 * (indpc - 1) : offset_ind + 3 * indpc;
    valu = d_U2X{indcam} / W_cur - U_cur * d_W2X{indcam} / (W_cur .^ 2);
    valv = d_V2X{indcam} / W_cur - V_cur * d_W2X{indcam} / (W_cur .^ 2);
    
    J = J + sparse(indr, indc, valu, num_row, num_col);
    J = J + sparse(indr + 1, indc, valv, num_row, num_col);
    
%     J(2 * (i - 1) + 1, offset_ind + 1 + 3 * (indpc - 1) : offset_ind + 3 * indpc) = ...
%         d_U2X{indcam} / W_cur - U_cur * d_W2X{indcam} / (W_cur .^ 2);
%     J(2 * i, offset_ind + 1 + 3 * (indpc - 1) : offset_ind + 3 * indpc) = ...
%         d_V2X{indcam} / W_cur - V_cur * d_W2X{indcam} / (W_cur .^ 2);
end
J = -J;
end