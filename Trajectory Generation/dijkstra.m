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

% calculate the order of the start and the terminal node (x:1st domain; y:2nd domain; z:3rd domain)
order_vs = map.xyzToInd(start);
order_vg = map.xyzToInd(goal);

% calculate the total number of nodes in the map using map.occgrid
map_shape = size(map.occgrid);
n_i = map_shape(1); n_j = map_shape(2); n_k = map_shape(3);
total_num_nodes = n_i * n_j * n_k;

%% calculate the Euclidean distance from valid node to the goal
ii = 1 : 1 : n_i;
jj = 1 : 1 : n_j;
kk = 1 : 1 : n_k;

[J, I, K] = meshgrid(jj, ii, kk);  %% X is row index; Y is column index
%% initialization: Build a large search table with elements:{order || ijk(1:3) || xyz(4:6) || h(7) || g(8) || p(9) || vis_state(10) || collide_state(11)}
searchTable = zeros(total_num_nodes, 11);

searchTable(:, 10) = -1;  % initialize vis_state column (-1 as un-traversed)
searchTable(:, 9) = (1: 1: total_num_nodes)'; % initialize previous_node column
searchTable(:, 8) = inf; % initialize g_dist column with inf

% set ijk, xyz, h and collide state
for i = 1 : n_k
  % obtain ijk matrix (N x 3)
  ijk_cur(:, :, 1) = I(:, :, 1);
  ijk_cur(:, :, 2) = J(:, :, 1);
  ijk_cur(:, :, 3) = K(:, :, i);
  ijk_cur_unroll = reshape(ijk_cur, [n_i * n_j, 3]);

  % obtain its corresponding orders and store in the searchTable
  order_cur_unroll = ijk2Order(ijk_cur_unroll, n_i, n_j);
  searchTable(order_cur_unroll, 1:3) = ijk_cur_unroll;
  
  % convert into physical xyz and store in the searchTable
  xyz_cur_unroll = map.subToXYZ(ijk_cur_unroll);
  searchTable(order_cur_unroll, 4:6) = xyz_cur_unroll;
  
  % compute Euclidean distance and store in the searchTable
  h_dist = sqrt(sum((xyz_cur_unroll - goal) .^ 2, 2));
  searchTable(order_cur_unroll, 7) = h_dist;
  
  % check collide state and store in the searchTable
  collide_list = map.collide(xyz_cur_unroll);
  searchTable(order_cur_unroll, 11) = collide_list;
  
end

% set g(vs) = 0
searchTable(order_vs, 8) = 0;

%% Dijkstra Loop to obtain optimal path
fprintf('---------> Start Searching .... \n');

% loop when vg is valid and hasn't been travsersed yet
while searchTable(order_vg, 11) == 0 && searchTable(order_vg, 10) == -1
  % obtain the terminal node of shortest path without being visited so far
  if astar
    [f_u, order_u] = min(searchTable(:, 8) + searchTable(:, 7), [], 'omitnan');
    g_u = searchTable(order_u, 8);
  else
    [g_u, order_u] = min(searchTable(:, 8), [], 'omitnan');
  end
  
  if g_u == inf
    break;
  end
  
  fprintf('The current node order: %d || The distance from start position: %4.6f. ', order_u, g_u);
  
  % label u as visited to store its distance from vs to u
  searchTable(order_u, 10) = g_u;
  % set g[order_u] = nan, as travsersed
  searchTable(order_u, 8) = NaN;

  % check whether the current min node is the goal node
  if searchTable(order_vg, 10) ~= -1
    fprintf('\n*** Reach the goal position ***\n');
    break;
  end

  % obtain u's all valid neighbors (not visited, not within block and not outside )
  ijk_u = searchTable(order_u, 1 : 3);
  [order_u_neighbors, ijk_u_neighbors] = findNeighbors(order_u, ijk_u, n_i, n_j);
  
  % exclude outside-boundary nodes
  valid_i = (ijk_u_neighbors(:, 1) >= 1) & (ijk_u_neighbors(:, 1) <= n_i);
  valid_j = (ijk_u_neighbors(:, 2) >= 1) & (ijk_u_neighbors(:, 2) <= n_j);
  valid_k = (ijk_u_neighbors(:, 3) >= 1) & (ijk_u_neighbors(:, 3) <= n_k);
  inbound_order_u_nei = order_u_neighbors(valid_i & valid_j & valid_k);  
  
  % exclude outside-boundary or visited or collided nodes
  unvis_bool = (searchTable(inbound_order_u_nei, 10) == -1);
  uncollide_bool = (searchTable(inbound_order_u_nei, 11) == 0);
  valid_order_u_nei = inbound_order_u_nei(unvis_bool & uncollide_bool);
  
  fprintf('The current center node has %d valid neighbors. \n', size(valid_order_u_nei, 1));
  if isempty(valid_order_u_nei)
      continue;
  end 
  
  % obtain valid neighbors' ijk to compute distance
  valid_ijk_u_nei = searchTable(valid_order_u_nei, 1 : 3);
  
  % compute distance between u and its valid neighbors
  dist_u_nei = sum(abs(valid_ijk_u_nei - ijk_u) .* map.res_xyz, 2);
  d_vs_u_nei = g_u + dist_u_nei;

  % check distance
  pre_vs_u_nei = searchTable(valid_order_u_nei, 8);
  update_ind = (d_vs_u_nei < pre_vs_u_nei);  % update only when current distance is smaller than previous
  update_order = valid_order_u_nei(update_ind);
  
  % update if necessary
  if ~isempty(update_order)
    % update previous node
    searchTable(update_order, 9) = order_u;

    % update min distance
    searchTable(update_order, 8) = d_vs_u_nei(update_ind);
  end

  % update expand numbers
  num_expanded = num_expanded + 1;

end
fprintf('\n---------> Finish Searching ! \n');

%% record path
fprintf('---------> Recording the Path .... \n');
path = [goal; path];
cur_order = order_vg;
while true
  % obtain the previous order
  pre_order = searchTable(cur_order, 9);

  % if reach the start node, break
  if pre_order == order_vs
    path = [start; path];
    break;
  end

  % obtain its xyz
  pre_xyz = searchTable(pre_order, 4 : 6);

  % update path
  path = [pre_xyz; path];
  
  % update current order
  cur_order = pre_order;
end

if astar
  save './path_res/path_astar.mat' path;
else
  save './path_res/path_dijkstra.mat' path;
end

%% plot path and optimal distance
plot_path(map, path);

opt_dis = searchTable(order_vg, 10);
fprintf('The found optimal path has distance: %4.6f m.\n', opt_dis);
fprintf('Searching finished.\n');
end


%% Obtain neighbors, consider 6 directions up-down-right-left-forward-back
function [neighbors_order, neighbors_ijk] = findNeighbors(order_u, ijk_u, ni, nj)
  % 6 neighbors
  neighbors_ijk(1, :) = ijk_u + [1, 0, 0];   % forward i-axis
  neighbors_order(1, :) = order_u + 1;
  
  neighbors_ijk(2, :) = ijk_u + [-1, 0, 0];  % backward i-axis
  neighbors_order(2, :) = order_u - 1;
  
  neighbors_ijk(3, :) = ijk_u + [0, 1, 0];   % right j-axis
  neighbors_order(3, :) = order_u + ni;
  
  neighbors_ijk(4, :) = ijk_u + [0, -1, 0];  % left j-axis
  neighbors_order(4, :) = order_u - ni;
  
  neighbors_ijk(5, :) = ijk_u + [0, 0, 1];   % up k-axis
  neighbors_order(5, :) = order_u + ni * nj;
  
  neighbors_ijk(6, :) = ijk_u + [0, 0, -1];  % down k-axis
  neighbors_order(6, :) = order_u - ni * nj;
end


%% obtain order index based on the ijk indices (N x 3)
function [order_list] = ijk2Order(ijk, nx, ny)
  order_list = ijk(:, 1) + (ijk(:, 2) - 1) * nx + (ijk(:, 3) - 1) * nx * ny;
end

%% obtain ijk index from order (revserse function of ijk2Order)
function [ijk] = order2Ijk(order_cur, nx, ny)
  kk = ceil(order_cur ./ (nx * ny));
  
  temp_n = order_cur - nx * ny * (kk - 1);
  
  jj = ceil(temp_n ./ nx);
  ii = temp_n - (jj - 1) * nx;

  ijk = [ii, jj, kk];
end

%% own xyz to ijk
function [ijk] = xyz2ijk(map, xyz)
  ijk(:, 1) = floor( (xyz(:, 1) - map.bound_xyz(1)) / map.res_xyz(1) ) + 1;
  ijk(:, 2) = floor( (xyz(:, 2) - map.bound_xyz(2)) / map.res_xyz(2) ) + 1;
  ijk(:, 3) = floor( (xyz(:, 3) - map.bound_xyz(3)) / map.res_xyz(3) ) + 1;
end

%% own ijk to xyz
function [xyz] = ijk2xyz(map, ijk)
  xyz(:, 1) = (ijk(:, 1) - 1) .* map.res_xyz(1) + map.bound_xyz(1);
  xyz(:, 2) = (ijk(:, 2) - 1) .* map.res_xyz(2) + map.bound_xyz(2);
  xyz(:, 3) = (ijk(:, 3) - 1) .* map.res_xyz(3) + map.bound_xyz(3);
end

