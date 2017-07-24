function [C, R, X0] = Initial3D(x1, x2, K)
%% use fundamental, essential and linear triangulation to implement point cloud initialization
correspondence_points_1 = x1; % Should be p x 2, where p is number of points
correspondence_points_2 = x2; % Should be p x 2, where p is number of points
%% Do the computation
[C, R, X0 ] = GetTransformation(correspondence_points_1, correspondence_points_2, K, K );
end

function [ C, R, X0 ] = GetTransformation( x1, x2, K1, K2 )
%GETTRANSFORMATION Computes the transformation between the two cameraman.
%   Inputs:
%   x1: (8x2) points in Image 1
%   x2: (8x2) corresponding points in Image 2
%   K1: (3x2) intrinsic calibration for Camera 1
%   K2: (3x2) intrinsic calibration for Camera 2
%   Outputs:
%   C: (3x1) transformation from Camera 1 to Camera 2
%   R: (3x3) rotation from Camera 1 to Camera 2
%   X0: (nx3) the correspondence points in 3D

% Estimate F and E.
F = EstimateFundamentalMatrix(x1, x2);
E = EssentialMatrixFromFundamentalMatrix(F, K1, K2);   

% Extract C and R.
[Cs, Rs] = ExtractCameraPose(E);

% Triangulate Points for each C and R.
for i = 1 : 4
    Xset{i} = LinearTriangulation(K1, K2, zeros(3,1), eye(3), Cs{i}, Rs{i}, x1, x2);
end
% Disambiguate the actual pose.
[C, R, X0] = DisambiguateCameraPose(Cs, Rs, Xset);
end