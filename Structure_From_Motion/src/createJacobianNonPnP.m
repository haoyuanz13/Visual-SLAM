function [J] = createJacobianNonPnP(ind_cam, V, ind_estedPC)
%% create a sparse matrix J
num_camera = length(ind_cam);
% calculate the total number of estimated 3D points and observations
num_obs = sum(sum(V(ind_estedPC, ind_cam)));
% calculate row and column number
num_col = 7 * num_camera;
num_row = 2 * num_obs;
% create a sparse matrix 
ind = 1 : num_obs;
encode = find((V(ind_estedPC, ind_cam) == 1)');
vv = ones(1, num_obs);
%% create a sparse matrix J
% construct camera_indices
camera_indices = mod(encode, num_camera);
camera_indices(camera_indices == 0) = num_camera;
camera_indices = camera_indices';

J = sparse(ind * 2 - 1, 7 * (camera_indices - 1) + 1, vv, num_row, num_col);
J = J + sparse(ind * 2, 7 * (camera_indices - 1) + 1, vv, num_row, num_col);

for i = 2 : 7
    J = J + sparse(ind * 2 - 1, 7 * (camera_indices - 1) + i, vv, num_row, num_col);
    J = J + sparse(ind * 2, 7 * (camera_indices - 1) + i, vv, num_row, num_col);
end
end