function [X] = LinearTriangulation(K1, K2, C1, R1, C2, R2, x1, x2)
[rc, nc] = size(C1);
if isequal(rc, 1) && isequal(nc, 3)
    C1 = C1';
    C2 = C2';
end

P1 = K1 * [R1, -R1 * C1];
P2 = K2 * [R2, -R2 * C2];

[N, ~] = size(x1);
X = zeros(N, 3);
for i = 1 : N
    x1_cur = x1(i, :);
    x2_cur = x2(i, :);
    
    x1_skew = [0, -1, x1_cur(2); 1, 0, -x1_cur(1); -x1_cur(2), x1_cur(1), 0];
    x2_skew = [0, -1, x2_cur(2); 1, 0, -x2_cur(1); -x2_cur(2), x2_cur(1), 0];
    
    A = [x1_skew * P1; x2_skew * P2];
    [~, ~, v] = svd(A);
    
    X_cur = v(:, end) / v(end, end);
    X(i, :) = X_cur(1 : 3) / X_cur(4);
end
end