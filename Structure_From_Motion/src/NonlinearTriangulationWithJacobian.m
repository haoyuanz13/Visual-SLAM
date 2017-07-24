function [X_new] = NonlinearTriangulationWithJacobian(K, C1, R1, C2, R2, x1, x2, X0, ind_cam, V, ind_PC)
num_points = length(X0);
% create transformation matrix P
P1 = K * R1 * [eye(3), -C1];
P2 = K * R2 * [eye(3), -C2];
% encode X as training parameters
ravel_X = reshape(X0', 1, num_points * 3);
params0 = ravel_X;

% optimization process
f=@(params)err_reproject_NonLT(params, P1, P2, x1, x2);
Jstr = createJacobianNonLT(ind_cam, V, ind_PC);

opts = optimoptions(@fsolve, 'Display', 'off', 'JacobPattern', Jstr, 'Algorithm', ...
            'trust-region-reflective', 'PrecondBandWidth', 1 , 'MaxIter', 1000000);
params_opt = fsolve(f, params0, opts);
%% update X set
X_new = (reshape(params_opt, 3, num_points))';
end

%% function to compute reprojection error
function [error] = err_reproject_NonLT(params, P1, P2, x1, x2)
% input x should be 2d position which is (N x 2)
% input P should be transform matrix which is (3x4)
N = length(params) / 3;
X = (reshape(params, 3, N))';
X = [X, ones(N, 1)];

reproject1 = P1 * X';
u1 = x1(:, 1); v1 = x1(:, 2);
error1 = [(u1 - (bsxfun(@rdivide, reproject1(1, :), reproject1(3, :)))'),...
    (v1 - (bsxfun(@rdivide, reproject1(2, :), reproject1(3, :)))')];

% reprojection error in second image frame
reproject2 = P2 * X';
u2 = x2(:, 1); v2 = x2(:, 2);
error2 = [(u2 - (bsxfun(@rdivide, reproject2(1, :), reproject2(3, :)))'), ...
    (v2 - (bsxfun(@rdivide, reproject2(2, :), reproject2(3, :)))')];

error = reshape(abs([error1, error2]'), 1, 4 * N);
end