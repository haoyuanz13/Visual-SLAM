%% visualize R, t
figure(1); clf;
% 3D points
plot3(X(:, 1), X(:, 2), zeros(size(X, 1), 1), 'kx');
% cameras
for i=1:numel(imgs)
    hold on;
    camLoc = Rs_opt(:, :, i)'*(-ts_opt(:, :, i));
    plot3(camLoc(1), camLoc(2), camLoc(3), 'rs');
    quiver3(camLoc(1), camLoc(2), camLoc(3), Rs_opt(3, 1, i), Rs_opt(3, 2, i), Rs_opt(3, 3, i), 100);
    hold off;
end
% others
grid on; axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca,'XAxisLocation', 'top', 'YAxisLocation', 'left', 'YDir', 'reverse', 'ZDir', 'reverse');
title('camera positions and directions with 3D points')
saveas(1, 'Rt.png');


%% run baseline
[x_baseline, boardSize, imagesUse] = detectCheckerboardPoints(cell2mat(reshape(imgs, [1 1 1 numel(imgs)])));
X_baseline = generateCheckerboardPoints(boardSize, squareSize);
cameraParams = estimateCameraParameters(x_baseline, X_baseline, 'EstimateSkew', true);

%% fin: check outputs
acc = 1e-1;
assert(all(all(abs(cameraParams.IntrinsicMatrix' - K_opt)<acc)));
assert(all(all(abs(cameraParams.TranslationVectors' - reshape(ts_opt, [3, size(ts_opt, 3)]))<acc)));
assert(all(all(all(abs(permute(cameraParams.RotationMatrices, [2 1 3]) - Rs_opt)<acc*1e-3))));
assert(all(all(abs(cameraParams.RadialDistortion(:) - ks_opt(:))<acc)));
assert(exist('Rt.png', 'file')>0);