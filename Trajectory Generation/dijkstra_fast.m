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

path = [];
num_expanded = 0;

% initialization start and goal points
if map.collide(goal) == 1 || map.collide(start) == 1
  return;
end

% check empty map
if isempty(map.occgrid)
  return;
end

% ijk of the goal
ijk_goal = map.xyzToSub(goal);

% calculate the total number of nodes in the map using map.occgrid
% i <-> y; j <-> x; k <-> z
n_i = ceil((map.bound_xyz(5) - map.bound_xyz(2)) / map.res_xyz(2)); 
n_j = ceil((map.bound_xyz(4) - map.bound_xyz(1)) / map.res_xyz(1)); 
n_k = ceil((map.bound_xyz(6) - map.bound_xyz(3)) / map.res_xyz(3));
total_num_nodes = n_i * n_j * n_k;

% calculate the order of the start and the terminal node (x:1st domain; y:2nd domain; z:3rd domain)
order_start = map.xyzToInd(start);
order_goal = map.xyzToInd(goal);

%% calculate the Euclidean distance from valid node to the goal
ii = 1 : 1 : n_i;
jj = 1 : 1 : n_j;
kk = 1 : 1 : n_k;

[J, I, K] = meshgrid(jj, ii, kk);  %% X is row index; Y is column index
%% initialization: Build a large search table with elements:{||ijk(1:3) || collide_state/vis(4)}
searchTable = zeros(total_num_nodes, 4);

% set ijk, xyz, h and collide state
for i = 1 : n_k
  % obtain ijk matrix (N x 3)
  ijk_cur(:, :, 1) = I(:, :, 1);
  ijk_cur(:, :, 2) = J(:, :, 1);
  ijk_cur(:, :, 3) = K(:, :, i);
  ijk_cur_unroll = reshape(ijk_cur, [n_i * n_j, 3]);

  % obtain its corresponding orders and store in the searchTable
  order_cur_unroll = ijk_cur_unroll(:, 1) + (ijk_cur_unroll(:, 2) - 1) * n_i + (ijk_cur_unroll(:, 3) - 1) * n_i * n_j;
  searchTable(order_cur_unroll, 1:3) = ijk_cur_unroll;
  
  % convert into physical xyz and store in the searchTable
  xyz_cur_unroll = map.subToXYZ(ijk_cur_unroll);
  
  % check collide state and store in the searchTable
  collide_list = map.collide(xyz_cur_unroll);
  searchTable(order_cur_unroll, 4) = collide_list;
end
flag = searchTable(:, 4);
order_map = searchTable(:, 1:3);

% initialization distance, Q, S
Q = [];
g = inf(total_num_nodes, 1);  % physical distance from vs to v
h = zeros(total_num_nodes, 1); 
p = zeros(total_num_nodes, 1); 
g(order_start) = 0;

%initialization center node, path and num_expanded
center = order_start;
flag(order_start) = 1;

% enter the loop to find path
while flag(order_goal) == 0
  % defind neighbor no. (total six neighbors with unit distance)
  n1 = center - 1;
  n2 = center + 1;
  n3 = center - n_i;
  n4 = center + n_i;
  n5 = center - n_i * n_j;
  n6 = center + n_i * n_j;

  % convert center node from order index into ijk index form
  A = order_map(center, :);

  % boundary check
  if A(1) == n_i
    n2 = 0;
  end
  
  if A(1) == 1
    n1 = 0;
  end
 
  if A(2) == n_j
    n4 = 0;
  end

  if A(2) == 1
    n3 = 0;
  end

  if A(3) == n_k
    n6 = 0;
  end

  if A(3) == 1
    n5 = 0;
  end
  
  % 6 neighbors order of center center node
  neighbor = [n1 n2 n3 n4 n5 n6];

  for i = 1 : 6
    cur_nei_order = neighbor(i);

    % reach boundary
    if cur_nei_order == 0
      continue;
    end
    
    cur_nei_ijk = order_map(cur_nei_order, :);
    if flag(cur_nei_order) == 1
      continue;
    end
    
    % check neighbors in xy domian
    if i <= 2
      delta_res = map.res_xyz(2);
    elseif i <= 4
      delta_res = map.res_xyz(1);
    else
      delta_res = map.res_xyz(3);
    end

    if g(cur_nei_order) > g(center) + delta_res 
      g(cur_nei_order) = g(center) + delta_res;
      p(cur_nei_order) = center;
          
      % using astar or not   
      if astar == false
        Q = [Q; g(cur_nei_order), cur_nei_order];
        
      else
        h(cur_nei_order) = sqrt(((cur_nei_ijk(1) - ijk_goal(1)) * map.res_xyz(2)) ^ 2 + ...
          ((cur_nei_ijk(2) - ijk_goal(2)) * map.res_xyz(1)) ^ 2 + ((cur_nei_ijk(3) - ijk_goal(3)) * map.res_xyz(3)) ^ 2);
%         h(cur_nei_order) = sqrt(((cur_nei_ijk(1) - goal(1)) * map.res_xyz(2)) ^ 2 + ...
%           ((cur_nei_ijk(2) - goal(2)) * map.res_xyz(1)) ^ 2 + ((cur_nei_ijk(3) - goal(3)) * map.res_xyz(3)) ^ 2);

        Q = [Q; g(cur_nei_order) + h(cur_nei_order), cur_nei_order];
      end
    
    end

  end  % end for

  % obtain min neighbors so for
  [~, number] = min(Q(:, 1));
  center = Q(number, 2);

  % delete center node in neighbors set
  Q(number, :) = [];

  % label new center node
  flag(center) = 1;

  % increase the traverse step number
  num_expanded = num_expanded + 1;

end

%% plot path
cur_order = order_goal;
path = [goal; path];
while order_start ~= cur_order
  pre_order = p(cur_order);
  path = [map.indToXYZ(pre_order); path];
  cur_order = pre_order;
end    
path = [start; path];

plot_path(map, path);

end
