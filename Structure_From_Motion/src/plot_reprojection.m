function [] = plot_reprojection(Img, R, C, K, X, x)
%% input clarification
% Img is the image
% R and C is the rotation and translation of current camera frame
% K is the intrinsic matrix
% X is the 3D points which should be Nx4
% x is the 2D observations which should be Nx2
%% reprojection plot
x = [x, ones(size(x, 1), 1)];
X = [X, ones(size(X, 1), 1)];

P = K * R * [eye(3), -C];
reproj = bsxfun(@rdivide, P(1 : 2, :) * X', P(3, :) * X');

u = x(:, 1); v = x(:, 2);
error = [(u - reproj(1, :)'), (v - reproj(2, :)')];
error = sqrt(sum(error .^ 2, 2));

error_accum = sum(error);
error_aver = error_accum / (size(x, 1));

[r, ~, ~] = size(Img);
imshow(Img); hold on;
plot(reproj(1, :), reproj(2, :), 'rx');
plot(x(:, 1), x(:, 2), 'go');
legend('reprojection', 'image features');
txt1 = ['The total number of features is: ', num2str(size(X, 1))];
txt2 = ['The total reprojection error is: ', num2str(error_accum)];
txt3 = ['The average reprojection error is: ', num2str(error_aver)];

text(0, r - 100, txt1, 'Color', 'cyan', 'FontSize', 12.5);
text(0, r - 60, txt2, 'Color', 'cyan', 'FontSize', 12.5);
text(0, r - 20, txt3, 'Color', 'cyan', 'FontSize', 12.5);
end