function [Rs, ts] = EstimateRt_linear(H, K, option)
% linear parameter estimation of Rs(rotation) and ts(translation)
% Input: 
% - H: homography, mapping from frame A to B 
% - K: camera intrinsic matrix
% - option: 1 or 2 to represent which method will be selected
% Output:
% - Rs: rotation matrix (3x3), transforming from A to B
% - ts: translation vector (3x1), translation from B to A measured in B


% left multiply inversed K to update H
H_p = K \ H;
hp_1 = H_p(:, 1); hp_2 = H_p(:, 2); hp_3 = H_p(:, 3); 

%% Method 1
if option
  z = norm(hp_1);

  r1 = hp_1 ./ z;
  r2 = hp_2 ./ z;
  r3 = cross(r1, r2);

  % rotation
  Rs = [r1, r2, r3];
  % translation
  ts = hp_3 ./ z;

%% Method 2: use svd solver
else
  concat = [hp_1 hp_2 cross(hp_1, hp_2)];
  [U S V] = svd(concat);  % concat = USV'
  Rs = U * [1, 0, 0; 0, 1, 0; 0, 0, det(U * V')] * V';
  ts = hp_3 ./ norm(hp_1);
end

end
