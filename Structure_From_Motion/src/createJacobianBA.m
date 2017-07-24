function [J, camera_indices, point_indices] = createJacobianBA(num_camera, V, haveEstPC)
%% create a sparse matrix J
ind_estedPC = find(haveEstPC == 1);
% calculate the total number of estimated 3D points and observations
num_PC = length(ind_estedPC);
num_obs = sum(sum(V(ind_estedPC, 1 : num_camera)));
% calculate row and column number
num_col = 7 * num_camera + num_PC * 3;
num_row = 2 * num_obs;
% create a sparse matrix 
ind = 1 : num_obs;
encode = find((V(ind_estedPC, 1 : num_camera) == 1)');

% construct camera_indices
camera_indices = mod(encode, num_camera);
camera_indices(camera_indices == 0) = num_camera;
camera_indices = camera_indices';

vv = ones(1, num_obs);
J = sparse(ind * 2 - 1, 7 * (camera_indices - 1) + 1, vv, num_row, num_col);
J = J + sparse(ind * 2, 7 * (camera_indices - 1) + 1, vv, num_row, num_col);

for i = 2 : 7
    J = J + sparse(ind * 2 - 1, 7 * (camera_indices - 1) + i, vv, num_row, num_col);
    J = J + sparse(ind * 2, 7 * (camera_indices - 1) + i, vv, num_row, num_col);
end

% construct point_indices
point_indices = (ceil(encode / num_camera))';
% fill up the 3D point side
for i = 1 : 3
    J = J + sparse(ind * 2 - 1, 7 * num_camera + 3 * (point_indices - 1) + i, vv, num_row, num_col);
    J = J + sparse(ind * 2, 7 * num_camera + 3 * (point_indices - 1) + i, vv, num_row, num_col);
end
end