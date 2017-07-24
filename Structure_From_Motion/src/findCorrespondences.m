function [x1, x2, idx_corrs] = findCorrespondences(Mu, Mv, V, ind_ori, ind_target)
%% find correspondences for two certain cameras
idx_ori = find(V(:, ind_ori) == 1);  % index of features in the original image
idx_target = find(V(:, ind_target) == 1); % index of features in the target image
idx_corrs = intersect(idx_ori, idx_target);  % find common index in two images which represents correspondences

x1 = []; x2 = [];
if ~isempty(idx_corrs)
    x1 = [Mu(idx_corrs, ind_ori), Mv(idx_corrs, ind_ori)];
    x2 = [Mu(idx_corrs, ind_target), Mv(idx_corrs, ind_target)];
end
end