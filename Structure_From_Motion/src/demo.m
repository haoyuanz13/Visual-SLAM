%% The main code contains the whole pipeline of structure from motion.
%% parse data, find inliers and create data structure
fprintf('Parsing raw data and Creating data strcture......... \n');
folder = 'data\*.txt';
[Mu, Mv, VisM, RGB] = FindInliers(folder);

% store all valid point cloud
PointCloud = zeros(size(VisM, 1), 3);  

% show whether current 3D point has been reconstructed
haveEstPC = zeros(size(VisM, 1), 1); 

% Visibility matrix to store the relationship between point cloud and camera frame
V = zeros(size(VisM, 1), 6); 
Cset = cell(6, 1); Rset = cell(6, 1);
fprintf('Data parsing is Done! \n');

%% Use the first two frames to initialize 3D point cloud
fprintf('\n ---------------------------------------- NEXT STEP --------------------------------------------- \n');
fprintf('Initializing the 3D Point Cloud......... \n');
K = [568.996140852, 0, 643.21055941;
   0, 568.988362396, 477.982801038;
   0, 0, 1];

% take the first camera as the global origin
Cset{1} = [0; 0; 0]; Rset{1} = eye(3);
[x1, x2, ind_VisM] = findCorrespondences(Mu, Mv, VisM, 1, 2);

% estimate the pose of second camera as well as initial 3D point cloud
[C, R, X0] = Initial3D(x1, x2, K);
Cset{2} = C; Rset{2} = R;

% clean up and update 
[PointCloud, haveEstPC, V, X0] = cleanAndUpdate([1, 2], X0, PointCloud, haveEstPC, V, ind_VisM);

% nonlinear triangulation to update initial 3D position
ind_PC = find(haveEstPC == 1);
x1 = [Mu(ind_PC, 1), Mv(ind_PC, 1)];
x2 = [Mu(ind_PC, 2), Mv(ind_PC, 2)];

% X0_update = NonlinearTriangulation(K, Cset{1}, Rset{1}, Cset{2}, Rset{2}, x1, x2, X0);
X0_update = NonlinearTriangulationWithJacobian(K, Cset{1}, Rset{1}, Cset{2}, Rset{2}, ... 
                          x1, x2, X0, [1, 2], V, ind_PC);

% clean up and update
[PointCloud, haveEstPC, V, X0_update] = cleanAndUpdate([1, 2], X0_update, PointCloud, haveEstPC, V, ind_PC);
fprintf('The construction of initial 3D Point Cloud is Done! \n');

%% ************************* Add new camera frames *************************
fprintf('---------------------------------------- NEXT STEP --------------------------------------------- \n');
fprintf('***************************** Add new camera frames ****************************** \n');
for curImage = 3 : 6
  fprintf('\n ----------------- Estimating the camera frame #%d ......... ---------------- \n', curImage);
  
  % find correspondence which has been estimated 3D position in previous image frames
  ind_estPC = find(haveEstPC == 1);
  ind_cur2D = find(VisM(:, curImage) == 1);
  ind_corrs = intersect(ind_estPC, ind_cur2D);
  
  % if the number of correspondences is smaller than 6, skip
  if length(ind_corrs) < 6
    continue;
  end
  
  % use those 3D information to implement PnP such that estimating the current camera pose (linear and nonlinear PnP)
  fprintf('>> Finding 3D to 2D correspondences in the previous Point Cloud. \n');
  fprintf('>>>> Working on the linear PnP..... \n');
  X_corrs = PointCloud(ind_corrs, :);
  x_corrs = [Mu(ind_corrs, curImage), Mv(ind_corrs, curImage)];
  
  % linear PnP to estimate initial pose of current camera frame
  [C_ori, R_ori] = PnPRANSAC(X_corrs, x_corrs, K);
  
  % update store information
  Cset{curImage} = C_ori; Rset{curImage} = R_ori; V(ind_corrs, curImage) = 1;
  
  % nonlinear PnP to update current camera pose
  fprintf('>>>> Working on the nonlinear PnP.... \n');
  
  % two types of nonlinear PnP with or without Jacobian
  [Cset, Rset] = NonlinearPnP(Cset, Rset, curImage, X_corrs, x_corrs, K, C_ori, R_ori);
  
  % nonlinear PnP with Jacobian will estimate all previous camera frames
  % [Cset, Rset] = NonlinearPnPWithJacobian(Cset, Rset, K, Mu, Mv, PointCloud, haveEstPC, V);
  fprintf('>>>> PnP is Done! \n');

  % Expand 3D point cloud, estimate new correspondences
  % the ideal method is traversing all previous image frames to get new correspondences
  fprintf('\n>> Adding new 3D features to Point Cloud.');
  for preImage = 1 : curImage - 1
    fprintf('\n>>>> Estimating for frame %d and frame %d. \n', preImage, curImage);
    [~, ~, ind_VisM] = findCorrespondences(Mu, Mv, VisM, preImage, curImage);
    
    % if two image frames have no correspondences, skip it.
    if isempty(ind_VisM)
      fprintf('-----> No correspondences found in frame %d and frame %d. \n', preImage, curImage);
      continue
    end
    fprintf('-----> The number of correspondences between frame %d and frame %d is %d. \n', preImage, curImage, length(ind_VisM));
    
    % find new 3D points need to be estimated
    ind_notestedPC = find(haveEstPC == 0);
    ind_newCorrs = intersect(ind_VisM, ind_notestedPC);
    fprintf('-----> The number of valid new feature is %d. \n', length(ind_newCorrs));
    
    x_preI = [Mu(ind_newCorrs, preImage), Mv(ind_newCorrs, preImage)];
    x_curI = [Mu(ind_newCorrs, curImage), Mv(ind_newCorrs, curImage)];
    
    % linear triangulation 
    X_LinearT = LinearTriangulation(K, K, Cset{preImage}, Rset{preImage}, Cset{curImage}, ...
        Rset{curImage}, x_preI, x_curI);
    
    % clean up and update
    [PointCloud, haveEstPC, V, X_LinearT] = cleanAndUpdate([preImage, curImage], ...
                        X_LinearT, PointCloud, haveEstPC, V, ind_newCorrs);
    
    % remove outliers
    ind_newvalid = intersect(ind_newCorrs, find(haveEstPC == 1));
    x_preI = [Mu(ind_newvalid, preImage), Mv(ind_newvalid, preImage)];
    x_curI = [Mu(ind_newvalid, curImage), Mv(ind_newvalid, curImage)];
    
    % nonlinear triangulation to update (two types with or without Jacobian matrix)
    % X_LinearT_new = NonlinearTriangulation(K, Cset{preImage}, Rset{preImage}, Cset{curImage}, ...
    %        Rset{curImage}, x_preI, x_curI, X_LinearT);
    
    X_LinearT_new = NonlinearTriangulationWithJacobian(K, Cset{preImage}, Rset{preImage}, Cset{curImage}, ...
        Rset{curImage}, x_preI, x_curI, X_LinearT, [preImage, curImage], V, ind_newvalid);
    
    % clean up and update
    [PointCloud, haveEstPC, V, X_LinearT_new] = cleanAndUpdate([preImage, curImage], ...
                        X_LinearT_new, PointCloud, haveEstPC, V, ind_newvalid);
    
    fprintf('>>>> The estimation of New 3D features given by frame %d and %d is Done! \n', preImage, curImage);
  end

  % Bundle Adjustment
  fprintf('\n>> Bundle Adjustment Process to update %d cameras and Point Cloud...... \n', curImage);
  
  % find all estimated 3D points and store indices
  ind_estedPC = find(haveEstPC == 1);  
  X_forBA = PointCloud(ind_estedPC, :); 
  
  % bundle adjustment with analysis and numerical Jacobian
  % [Cset, Rset, X_BA] = BundleAdjustmentWithAnalysisJacobian(Cset, Rset, curImage, X_forBA, K, Mu, Mv, V, ind_estedPC);
  [Cset, Rset, X_BA] = BundleAdjustmentWithNumericalJacobian(Cset, Rset, curImage, X_forBA, K, haveEstPC, Mu, Mv, V, ind_estedPC);
  
  % clean and update
  [ind_valid_BA, ind_invalid_BA] = removeBehind(X_BA);
  X_BA = X_BA(ind_valid_BA, :);
  
  % update point cloud
  PointCloud(ind_estedPC(ind_valid_BA), :) = X_BA;
  
  % clean up invalid 3D points
  haveEstPC(ind_estedPC(ind_invalid_BA)) = 0;
  V(ind_estedPC(ind_invalid_BA), 1 : curImage) = 0;
  fprintf('>> Bundle Adjustment Done! \n');
end

%% visualization 3D point cloud
ind_estedPC = find(haveEstPC == 1);

% binary
PlotCamerasAndPoints(Cset, Rset, PointCloud(ind_estedPC, :), 25);

% color map
figure
PC3Dshow(PointCloud(ind_estedPC, :), Cset, Rset, RGB(ind_estedPC, :))

%% visualization reprojection error
num_images = length(Cset);
for i = 1 : num_images
  if isempty(Cset{i})
    break;
  end
  
  im = imread(['data/image000000', num2str(i), '.bmp']);
  ind_feature_im = find(V(:, i) == 1);
  ind_common = intersect(ind_estedPC, ind_feature_im);
  X = PointCloud(ind_common, :);
  x = [Mu(ind_common, i), Mv(ind_common, i)];
  C = Cset{i}; R = Rset{i}; 
  figure;
  plot_reprojection(im, R, C, K, X, x);
end
