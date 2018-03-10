function map = load_map(filename, res_xy, res_z, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, res_xy, res_z).  Creates an occupancy grid map
%  which represents obstacles and .
map = OccMap.LoadMap(filename, res_xy, res_z, margin);
end
