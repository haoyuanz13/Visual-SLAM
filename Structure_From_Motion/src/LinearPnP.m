function [C, R] = LinearPnP(X, x, K)
%% linear PnP estimate the rotation and translation of camera given 2D and 3D information
% theoretically, linear PnP needs at least 6 correspondences
% input x is N x 3
% input X is N x 4
%% transform from image frame to matrices plane
N = size(X, 1);
x_project = (K \ x')';
%% construct P matrix to solve linear problem
A = [];
for i = 1 : N
    cur_u = [X(i, 1), X(i, 2), X(i, 3), 1, 0, 0, 0, 0, -x_project(i, 1) * X(i, 1), -x_project(i, 1) * X(i, 2), -x_project(i, 1) * X(i, 3), -x_project(i, 1)];
    cur_v = [0, 0, 0, 0, X(i, 1), X(i, 2), X(i, 3), 1, -x_project(i, 2) * X(i, 1), -x_project(i, 2) * X(i, 2), -x_project(i, 2) * X(i, 3), -x_project(i, 2)];
    A = [A; cur_u; cur_v];
end
[~, ~, v] = svd(A);
P = v(:, end);
R = [P(1), P(2), P(3); P(5), P(6), P(7); P(9), P(10), P(11)];
t = [P(4); P(8); P(12)];
%% clean up
[u, d, vv] = svd(R);
gama = d(1, 1);
R = u * vv';
t = t ./ gama;

if det(R) < 0
    R = -R;
    t = -t;
end
C = -R' * t;
end