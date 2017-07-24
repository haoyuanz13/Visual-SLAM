function [inlier_x1, inlier_x2, ind_inlier] = GetInliersRANSAC(x1, x2)
max_vote = 0;  % store max vote number 
iteration = 5000; % iteration time
N = length(x1);
threshold = 2;
ind_inlier = boolean(ones(N, 1));
% convert 2d position into homogeneous form
homo_x1 = [x1, ones(N, 1)];
homo_x2 = [x2, ones(N, 1)];

% RANSAC iteration 
for k = 1 : iteration
    % randomly select 8 points without repetitions
    ind = randperm(N, 8); 
    % use 8 points to estimate fundamental matrix
    F_cur = EstimateFundamentalMatrix(x1(ind, :), x2(ind, :));
    % compute reprojection error
    lines  = (F_cur * homo_x1')';
    % normalize the error into pixel unit
    error = abs(sum(homo_x2 .* lines, 2)) ./ sqrt(lines(:, 1).^ 2 + lines(:, 2) .^ 2);
    % remove outliers
    vote = sum(error < threshold);
    % store current optimal fundamental matrix
    if vote > max_vote
        max_vote = vote;
        ind_inlier = (error < threshold);
        % jump out the RANSAC loop if almost most correspondences vote for
        % current fundamental matrix
        if vote > 0.99 * N
            break;
        end
    end
end
inlier_x1 = x1(ind_inlier, :);
inlier_x2 = x2(ind_inlier, :);
end