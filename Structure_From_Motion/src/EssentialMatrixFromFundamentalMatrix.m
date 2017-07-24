function [E] = EssentialMatrixFromFundamentalMatrix(F, K1, K2)
E = K2' * F * K1;
[u, d, v] = svd(E);
d(1, 1) = 1;
d(2, 2) = 1;
d(3, 3) = 0;

E = u * d * v';
end