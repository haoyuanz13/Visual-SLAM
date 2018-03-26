%% settings
caliImgDir = 'checkerboard';  % the folder of your calibration images in a jpg format. e.g. './pics'
squareSize = 30;  % in mm

%% initialize calibration process
[x, X, imgs] = InitCalibration(caliImgDir, squareSize);

%% Estimate an intrinsic parameter
[K, Hs] = EstimateK_linear(x, X);

%% Estimate an extrinsic parameter
[Rs, ts] = EstimateRt_linear(Hs, K);
fprintf('* Reprojection error per measurement: %f pixel(s)\n', GeoError(x, X, [0 0], K, Rs, ts)/(size(x, 1)*size(x, 3)));

%% Estimate a distortion parameter
ks = EstimateDistort_linear(x, X, K, Rs, ts);
fprintf('* Reprojection error per measurement: %f pixel(s)\n', GeoError(x, X, ks, K, Rs, ts)/(size(x, 1)*size(x, 3)));

%% Minimize reprojection errors
[ ks_opt, K_opt, Rs_opt, ts_opt ] = MinGeoError( x, X, ks, K, Rs, ts );
fprintf('* Reprojection error per measurement: %f pixel(s)\n', GeoError(x, X, ks_opt, K_opt, Rs_opt, ts_opt)/(size(x, 1)*size(x, 3)));

%% Evaluate parameters
Evaluate;

