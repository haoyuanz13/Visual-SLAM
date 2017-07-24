function [] = PlotCamerasAndPoints( Cset, Rset, X, scale )
% Visualizes the camera 
% Inputs: 
%   C{i}: C for ith camera
%   R{i}:  R for ith camera
%   X: point cloud in the 1st camera's frame.
%   scale: scaling factor to approximate camera distances

% Colors that each camera will be rendered as. For more than 6 cameras,
% modify this.
Colors = ['k', 'b', 'g' , 'm', 'c', 'r'];

figure;
hold on; grid on

for i = 1:size(Cset,2)
    % Show the ith camera
    plotCamera('Location',Cset{i}*scale,'Orientation',Rset{i},'Opacity',0, 'Color', Colors(i));
end

% Show the correspondence points.
scatter3(X(:,1)*scale, X(:,2)*scale, X(:,3)*scale, [Colors(2), '*']);  
% Set the limits so that your pointcloud can be seen.
% xlim([-15,15*scale]);
% ylim([-15,15*scale]);
% zlim([-15,15*scale]);

view(360,0);
axis square;
axis equal;

end

