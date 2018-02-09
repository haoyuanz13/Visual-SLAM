function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
  astar = false;
end

%% initialization
path = [];
num_expanded = 0;

%% check whether the start and goal is valid
if map.collide(start) == 1 || map.collide(goal) == 1
  return;
end

%% convert vs and vg from physical metric space into indicies space and get its order
vs = map.xyzToSub(start);
vg = map.xyzToSub(goal);

% calculate the total number of nodes in the map using map.occgrid
map_shape = size(map.occgrid);
n_x = map_shape[2]; n_y = map_shape[1]; n_z = map_shape[3];
total_num_nodes = n_x * n_y * n_z;

% calculate the order of the start and the terminal node (x:1st domain; y:2nd domain; z:3rd domain)
order_vs = ijk2Order(vs, n_x, n_y);
order_vg = ijk2Order(vg, n_x, n_y);

%% calculate the Euclidean distance from valid node to the goal
xx = 1 : 1 : n_x;
yy = 1 : 1 : n_y;
zz = 1 : 1 : n_z;

[X, Y, Z] = meshgrid(xx, yy, zz);  %% X is column index; Y is row index
% heuristic array
h = zeros(total_num_nodes, 1);
for i = 1 : n_z:
  % obtain ijk matrix (N x 3)
  ijk_cur(:, :, 1) = xx(:, :, 1);
  ijk_cur(:, :, 2) = yy(:, :, 1);
  ijk_cur(:, :, 3) = zz(:, :, i);
  ijk_cur_unroll = reshape(ijk_cur, [n_x * n_y, 3]);

  % obtain its corresponding orders
  order_cur_unroll = ijk2Order(ijk_cur_unroll, n_x, n_y);

  % convert into physical xyz
  xyz_cur_unroll = map.subToXYZ(ijk_cur_unroll);

  % compute Euclidean distance
  h_dist = sqrt(sum((xyz_cur_unroll - goal) .^ 2, 2));

  % assign h value
  h(order_cur_unroll) = h_dist;
end


%% Dij Loop to obtain optimal path
Q_vis = zeros(total_num_nodes, 1);  
Q_vis(:) = -1;  % Q array to show whether the node i has been visited (-1: not vis; other value: store dis from vs to i)

g_dist_vs2v = inf(total_num_nodes, 1); % q array to store the min distance from vs to v (physical space)
p_preNode = (1 : 1 : total_num_nodes)'; % p array to store the previous node of current node

% initialize for the start node
g_dist_vs2v[order_vs] = 0;


% loop when vg hasn't been travsersed yet
while Q_vis(order_vg) == 0:
  % obtain the terminal node of shortest path wwithout being visited so far
  [g_u, order_u] = min(g_dist_vs2v, [], 'omitnan');
  
  % label u as visited to store its distance from vs to u
  Q_vis(order_u) = g_u;
  % set g[order_u] = nan, as travsersed
  g_dist_vs2v[order_u] = NaN;

  % obtain u's all valid neighbors
  ijk_u = order2Ijk(order_u, nx, ny);









end


%% obtain order index based on the ijk indices (N x 3)
function [order_list] = ijk2Order(ijk, nx, ny)
  order_list = ijk(:, 1) + (ijk(:, 2) - 1) * nx + (ijk(:, 3) - 1) * nx * ny;
end


%% obtain ijk index from order (revserse function of ijk2Order)
function [ijk] = order2Ijk(order_cur, nx, ny)
  kk = ceil(order_cur / (nx * ny));
  
  temp_n = order_cur - nx * ny * (kk - 1);
  
  jj = ceil(temp_n / nx);
  ii = temp_n - (jj - 1) * nx;

  ijk = [ii, jj, kk];
end
