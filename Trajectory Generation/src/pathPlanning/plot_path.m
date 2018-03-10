% function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
% % plot start and goal position
% total_step = size(path, 1);
% scatter3(path(1, 1), path(1, 2), path(1, 3), 30, 'ob');
% hold on
% scatter3(path(total_step, 1), path(total_step, 2), path(total_step, 3), 30, '*b');
% hold on
% % plot path
% plot3(path(:, 1), path(:, 2), path(:, 3), 'r');
% hold on;
% 
% % blocks
% num_block = size(map.blocks, 1);
% for i = 1 : num_block
%   block_i = map.blocks(i, :);
% 
%   obtain min and max x, y, z
%   min_x = block_i(1); max_x = block_i(4);
%   min_y = block_i(2); max_y = block_i(5);
%   min_z = block_i(3); max_z = block_i(6);
%   
%   center_x = (min_x + max_x) / 2; size_x = max_x - min_x;
%   center_y = (min_y + max_y) / 2; size_y = max_y - min_y;
%   center_z = (min_z + max_z) / 2; size_z = max_z - min_z;
%   
%   rgb = block_i(7 : 9);
%   color8 = [1 1 1 1 1 1 1 1];
%   color8 = [rgb rgb rgb rgb rgb rgb rgb rgb];
%   plotcube([center_x, center_y, center_z], [size_x, size_y, size_z], [0, 0, 0], color8, 0.5, 1);
% end
% 
% end
% 
% % plot cube
% function handle = plotcube(coordinates, size, orientation, color8, transparency, LineOnOff)
% 
%   
%      c7---------c8
%     /|          /|
%    c3---------c4 |
%    | |         | |
%    | |         | |
%    | c5--------c6
%    | /         |/
%    c1---------c2
%   
% 
% 
%   compute the 8 corners
%   c1 = get3Dpoints(coordinates,[-size(1)/2;-size(2)/2;-size(3)/2],orientation);
%   c2 = get3Dpoints(coordinates,[+size(1)/2;-size(2)/2;-size(3)/2],orientation);
%   c3 = get3Dpoints(coordinates,[-size(1)/2;-size(2)/2;+size(3)/2],orientation);
%   c4 = get3Dpoints(coordinates,[+size(1)/2;-size(2)/2;+size(3)/2],orientation);
% 
%   c5 = get3Dpoints(coordinates,[-size(1)/2;+size(2)/2;-size(3)/2],orientation);
%   c6 = get3Dpoints(coordinates,[+size(1)/2;+size(2)/2;-size(3)/2],orientation);
%   c7 = get3Dpoints(coordinates,[-size(1)/2;+size(2)/2;+size(3)/2],orientation);
%   c8 = get3Dpoints(coordinates,[+size(1)/2;+size(2)/2;+size(3)/2],orientation);
% 
%   plot the 6 sides
%   hold on;
%   handle(1) = fill3([c1(1) c2(1) c4(1) c3(1) c3(1) c3(1) c3(1) c3(1)],[c1(2) c2(2) c4(2) c3(2) c3(2) c3(2) c3(2) c3(2)],[c1(3) c2(3) c4(3) c3(3) c3(3) c3(3) c3(3) c3(3)],[color8(1) color8(2) color8(4) color8(3) color8(3) 0 1 color8(3)]);
%   handle(2) = fill3([c2(1) c4(1) c8(1) c6(1) c6(1) c6(1) c6(1) c6(1)],[c2(2) c4(2) c8(2) c6(2) c6(2) c6(2) c6(2) c6(2)],[c2(3) c4(3) c8(3) c6(3) c6(3) c6(3) c6(3) c6(3)],[color8(2) color8(4) color8(8) color8(6) color8(6) 0 1 color8(6)]);
%   handle(3) = fill3([c5(1) c6(1) c8(1) c7(1) c7(1) c7(1) c7(1) c7(1)],[c5(2) c6(2) c8(2) c7(2) c7(2) c7(2) c7(2) c7(2)],[c5(3) c6(3) c8(3) c7(3) c7(3) c7(3) c7(3) c7(3)],[color8(5) color8(6) color8(8) color8(7) color8(7) 0 1 color8(7)]);
%   handle(4) = fill3([c1(1) c3(1) c7(1) c5(1) c5(1) c5(1) c5(1) c5(1)],[c1(2) c3(2) c7(2) c5(2) c5(2) c5(2) c5(2) c5(2)],[c1(3) c3(3) c7(3) c5(3) c5(3) c5(3) c5(3) c5(3)],[color8(1) color8(3) color8(7) color8(5) color8(5) 0 1 color8(5)]);
%   handle(5) = fill3([c1(1) c2(1) c6(1) c5(1) c5(1) c5(1) c5(1) c5(1)],[c1(2) c2(2) c6(2) c5(2) c5(2) c5(2) c5(2) c5(2)],[c1(3) c2(3) c6(3) c5(3) c5(3) c5(3) c5(3) c5(3)],[color8(1) color8(2) color8(6) color8(5) color8(5) 0 1 color8(5)]);
%   handle(6) = fill3([c3(1) c4(1) c8(1) c7(1) c7(1) c7(1) c7(1) c7(1)],[c3(2) c4(2) c8(2) c7(2) c7(2) c7(2) c7(2) c7(2)],[c3(3) c4(3) c8(3) c7(3) c7(3) c7(3) c7(3) c7(3)],[color8(3) color8(4) color8(8) color8(7) color8(7) 0 1 color8(7)]);
% 
%   set transparency
%   alpha(handle, transparency)
%   set LineStyle
%   if LineOnOff == 0
%     for cnt1 = 1:1:6
%       set(handle(cnt1),'LineStyle','none');
%     end
%   end
% end
% 
% 
% function erg_coordinates = get3Dpoints(vec,coordinates,orientation)
% 
%   rot_mat = [...
%               cos(orientation(1))*cos(orientation(2)),                  -cos(orientation(1))*sin(orientation(2)),                    sin(orientation(1));...
%               cos(orientation(3))*sin(orientation(2))+sin(orientation(3))*sin(orientation(1))*cos(orientation(2)), cos(orientation(3))*cos(orientation(2))-sin(orientation(3))*sin(orientation(1))*sin(orientation(2)), -sin(orientation(3))*cos(orientation(1));...
%               sin(orientation(3))*sin(orientation(2))-cos(orientation(3))*sin(orientation(1))*cos(orientation(2)), sin(orientation(3))*cos(orientation(2))+cos(orientation(3))*sin(orientation(1))*sin(orientation(2)),  cos(orientation(3))*cos(orientation(1))...
%           ];
% 
%   erg_coordinates = (vec'+rot_mat*coordinates)';
% end
function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

num_block = size(map.blocks, 1);

if num_block == 0
  return;
else
  X = [];
  Y = [];
  Z = [];
  for i = 1 : num_block
    X = [X; linspace(map.blocks(i, 1), map.blocks(i, 4), 10)];
    Y = [Y; linspace(map.blocks(i, 2), map.blocks(i, 5), 10)];
    Z = [Z; linspace(map.blocks(i, 3), map.blocks(i, 6), 100)];
  end
end
for i = 1 : num_block
  [x, y] = meshgrid(X(i,:), Y(i,:));
  for j = 1 : 100
    z = Z(i, j) * ones(10, 10);
    mesh(x, y, z);
    hold on;
    grid on;
  end
end
hold on;
plot3(path(:, 1), path(:, 2), path(:, 3), 'r');

end

