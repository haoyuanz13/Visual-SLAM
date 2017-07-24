function [PointCloud, haveEstPC, V, X_update] = cleanAndUpdate(idcam, X, PointCloud, haveEstPC, V, ind_corrs)
% remove features locate behind camera
[ind_valid, ind_invalid] = removeBehind(X);
X_update = X(ind_valid, :);
%% update matrixs
PointCloud(ind_corrs(ind_valid), :) = X_update;
haveEstPC(ind_corrs(ind_valid)) = 1;
haveEstPC(ind_corrs(ind_invalid)) = 0;
% set correspondences label for camera frames
V(ind_corrs(ind_valid), idcam) = 1;
V(ind_corrs(ind_invalid), idcam) = 0;
end