function [ind_isvalid, ind_invalid] = removeBehind(X)
% input X should be N x 3 matrix
ind_valid = ones(size(X, 1), 1);
% remove point so far away
ind_far = (abs(X(:, 1)) > 100 | abs(X(:, 2)) > 100  | abs(X(:, 3)) > 100);
ind_valid(ind_far) = 0;
%remove points behind camera
ind_behind = (X(:, 3) < 0);
ind_valid(ind_behind) = 0;
ind_isvalid = find(ind_valid == 1);
ind_invalid = find(ind_valid == 0);
end