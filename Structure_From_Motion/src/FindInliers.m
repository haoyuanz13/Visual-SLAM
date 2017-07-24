function [Mu, Mv, V, RGB] = FindInliers(folder)
%% find inlier correspondences for each two image pairs
% return new data structure Mu, Mv, V and RGB in matrix form
% parse the raw data 
[Mu, Mv, V, RGB] = ParseData(folder);
num_images = 6;
for ind_ori = 1 : num_images - 1
    for ind_target = ind_ori + 1 : num_images
        % find correspondences for two camera frames
        [x1, x2, idx_corrs] = findCorrespondences(Mu, Mv, V, ind_ori, ind_target);
        % if correspondences less than 8 points, skip current image frames
        if size(x1, 1) < 8
            continue;
        end
        % use fundamental matrix to achieve RANSAC
        [~, ~, inlier] = GetInliersRANSAC(x1, x2); % inlier is a mask of where the inlies are. 
        V(idx_corrs(~inlier), ind_ori) = 0; % set all non-inliers to 0 and only for the original image frame
        fprintf('images %d and %d: Correspondences Inliers Percentage = %f \n', ind_ori, ind_target, 100 * sum(inlier) / length(inlier));
    end
end
end

