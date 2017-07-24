function [Mu, Mv, V, RGB] = ParseData(folder)
%% parse data and encode them into proper data structure
% the folder stores correspondences matching data
srcFiles = dir(folder);  
nfiles = length(srcFiles);
% Mu and Mv store u and v coordinates respectfully
Mu = []; Mv = []; 
% V is the visibility matrix
V = []; RGB = [];
for i = 1 : nfiles
    filename = strcat('data/matching', int2str(i), '.txt');
    % store matching data into matrix form
    [mu, mv, v, rgb] = createMatrix(filename, i, 6);
    Mu = [Mu; mu];
    Mv = [Mv; mv];
    V = [V; v];
    RGB = [RGB; rgb];
end
end

%% encode data
function [Mu, Mv, v, rgb] = createMatrix(filename, image_idx, nImages)
fid = fopen(filename);
fscanf(fid, '%s', 1);
n = fscanf(fid, '%d', 1);  % n is the total number of observation in current camera frame
% Mu and Mv are used to store 2d position information (number of observations x number of camera frames)
Mu = zeros(n, nImages); Mv = zeros(n, nImages);
% v is a logical matrix which shows whether current 3D point can be
% detected by the certain camera (same size with Mu)
v = zeros(n, nImages);
% rgb is used to store rgb information (number of observations x 3)
rgb = uint8(zeros(n, 3));
for i = 1 : n
    m = fscanf(fid, '%d', 1);  % total number of correspondecens
    rgb(i, :) = fscanf(fid, '%d', 3);   % read in RGB data
    Mu(i, image_idx) = fscanf(fid, '%f', 1);  % read in u coordinate in original frame
    Mv(i, image_idx) = fscanf(fid, '%f', 1);  % read in v coordinate in original frame
    v(i, image_idx) = 1;
    for k = 1 : m - 1
        j = fscanf(fid, '%d', 1);
        Mu(i, j) = fscanf(fid, '%f', 1);
        Mv(i, j) = fscanf(fid, '%f', 1);
        v(i, j) = 1;   % represent image i and j have correspondences
    end
end
fclose(fid);
end