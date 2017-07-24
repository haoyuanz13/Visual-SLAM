function [X_new] = NonlinearTriangulation(K, C1, R1, C2, R2, x1, x2, X0)
N = length(X0);
X0_homo = [X0, ones(N, 1)];
X_new = zeros(size(X0));
% create transformation matrix P
P1 = K * R1 * [eye(3), -C1];
P2 = K * R2 * [eye(3), -C2];
% parameters for optimization function 
opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, ...
    'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');

% iterate all 3D points one by one
for i = 1 : N
    curX = X0_homo(i, :);
    e = @(x)error_reprojection(x, P1, P2, x1(i, :), x2(i, :));
    Xx = lsqnonlin(e, curX', [], [], opts); 
    X_new(i, :) = Xx(1 : 3) / Xx(4);
end
end

%% compute reprojection error 
function [error] = error_reprojection(X, P1, P2, x1, x2)
% input X should be a single 3D point (N x 4)
% input x should be 2d position which is (N x 2)
% input P should be transform matrix which is (3x4)
% reprojection error in first image frame
reproject1 = P1 * X;
u1 = x1(:, 1); v1 = x1(:, 2);
error1 = [(u1 - (bsxfun(@rdivide, reproject1(1, :), reproject1(3, :)))'),...
    (v1 - (bsxfun(@rdivide, reproject1(2, :), reproject1(3, :)))')];

% reprojection error in second image frame
reproject2 = P2 * X;
u2 = x2(:, 1); v2 = x2(:, 2);
error2 = [(u2 - (bsxfun(@rdivide, reproject2(1, :), reproject2(3, :)))'), ...
    (v2 - (bsxfun(@rdivide, reproject2(2, :), reproject2(3, :)))')];

error = [abs(error1), abs(error2)];
end