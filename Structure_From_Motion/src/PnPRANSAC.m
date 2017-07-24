function [C, R] = PnPRANSAC(X, x, K)
%% Impplement linear PnP to estimate the optimal pose of camera
% input X is N x 3
% input x is N x 2
%%
max_vote = 0;  % store max vote number 
iteration = 5000; % iteration time
N = length(x);
threshold = 5;
ind_inlier = boolean(ones(N, 1));
% construct homogeneous form
x_homo = [x, ones(N, 1)];
X_homo = [X, ones(N, 1)];
% RANSAC iteration 
for k = 1 : iteration
    % randomly select 6 points without repetitions
    ind = randperm(N, 6); 
    % use 8 points to estimate fundamental matrix
    [C_cur, R_cur] = LinearPnP(X_homo(ind, :), x_homo(ind, :), K);
    % compute reprojection error
    P = K * R_cur * [eye(3), -C_cur];
    error = err_reproject_linearPnP(P, X_homo, x_homo);
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
fprintf('-----> The inliers percentage is %d. \n', max_vote * 100 / N);
inlier_x = x_homo(ind_inlier, :);
inlier_X = X_homo(ind_inlier, :);
% use all inliers to estimate camera pose
[C, R] = LinearPnP(inlier_X, inlier_x, K);
end

function [error] = err_reproject_linearPnP(P, X, x)
% input X should be a single 3D point (N x 4)
% input x should be 2d position which is (N x 2)
% input P should be transform matrix which is (3x4)
% reprojection error in first image frame
reproject = P * X';
error = [(x(:, 1) - (bsxfun(@rdivide, reproject(1, :), reproject(3, :)))'),...
    (x(:, 2) - (bsxfun(@rdivide, reproject(2, :), reproject(3, :)))')];
error = sum(abs(error), 2);
end