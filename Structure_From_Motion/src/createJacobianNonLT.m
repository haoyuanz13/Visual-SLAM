function [J] = createJacobianNonLT(ind_cam, V, ind_PC)
%% create a sparse matrix J
num_camera = length(ind_cam);
% calculate the total number of estimated 3D points and observations
num_PC = length(ind_PC);
num_obs = sum(sum(V(ind_PC, ind_cam)));
% calculate row and column number
num_col = num_PC * 3;
num_row = 2 * num_obs;
% create a sparse matrix 
ind = 1 : num_obs;
encode = find((V(ind_PC, ind_cam) == 1)');
vv = ones(1, num_obs);

point_indices = (ceil(encode / num_camera))';
J = sparse(ind * 2 - 1, 3 * (point_indices - 1) + 1, vv, num_row, num_col);
J = J + sparse(ind * 2, 3 * (point_indices - 1) + 1, vv, num_row, num_col);

% fill up the 3D point side
for i = 2 : 3
    J = J + sparse(ind * 2 - 1, 3 * (point_indices - 1) + i, vv, num_row, num_col);
    J = J + sparse(ind * 2, 3 * (point_indices - 1) + i, vv, num_row, num_col);
end
end