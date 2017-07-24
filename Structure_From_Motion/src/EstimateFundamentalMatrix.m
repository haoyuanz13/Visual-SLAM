function [F] = EstimateFundamentalMatrix(x1, x2)
[N, ~] = size(x1);
A = [x1(:, 1) .* x2(:, 1), x1(:, 2) .* x2(:, 1), x2(:, 1), ...
     x1(:, 1) .* x2(:, 2), x1(:, 2) .* x2(:, 2), x2(:, 2), x1(:, 1), x1(:, 2), ones(N, 1)];
    
[~, ~, V] = svd(A);
f = V(:, end);

F_r3 = [f(1 : 3)'; f(4 : 6)'; f(7 : 9)'];
[u, d, v] = svd(F_r3);
% svd clean up
d(3 ,3) = 0;
F = u * d * v';    
end