function [ x, X, imgs ] = InitCalibration( caliImgDir, squareSize )
% This function loads calibration images and generates world/image points.
% 
%   x:  2D points. n x 2 x N matrix, where n is the number of corners in
%   a checkerboard and N is the number of calibration images
%       
%   X:  3D points. n x 2 matrix, where n is the number of corners in a
%   checkerboard and assumes the points are on the Z=0 plane
%
%   imgs: calibration images. N x 1 cell, where N is the number of calibration images
%

%% load images
cbImgFns = dir(sprintf('%s/*.jpg', caliImgDir));
imgs = arrayfun(@(x) imread(sprintf('%s/%s', caliImgDir, x.name)), cbImgFns, 'UniformOutput', false);

%% detect corners
[x, boardSize, imagesUse] = detectCheckerboardPoints(cell2mat(reshape(imgs, [1 1 1 numel(imgs)])));
imgs = imgs(imagesUse);

% show valid images
figure(11);
montage(cell2mat(reshape(imgs, [1 1 1 numel(imgs)])));
title('Calibration images');

% show 2d points
figure(12); 
imagesc(imgs{1});
hold on;
plot(x(:, 1, 1), x(:, 2, 1), 'rx');
hold off;
title('detected corners in the first image');

%% generate world points
X = generateCheckerboardPoints(boardSize, squareSize);

end

