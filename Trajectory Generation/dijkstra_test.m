function tests = dijkstra_test
tests = functiontests(localfunctions);
end

function assertPathValid(testcase, map, path, start, stop, xlim, ylim, zlim)
valid = xlim(1) <= path(:, 1) & path(:, 1) <= xlim(2);
valid = ylim(1) <= path(:, 2) & path(:, 2) <= ylim(2) & valid;
valid = zlim(1) <= path(:, 3) & path(:, 3) <= zlim(2) & valid;
verifyTrue(testcase, all(valid));

c = collide(map, path);
verifyFalse(testcase, any(c));

verifyEqual(testcase, path(1, :), start);
verifyEqual(testcase, path(end, :), stop);
end

function testMap1(testcase)
% map = load_map('map1.txt', 0.1, 2.0, 0.3);
% start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0];

map = load_map('map0.txt', 0.1, 2.0, 0.0);
start = [2.0  4.0 3.0];
stop  = [17.0  5.0 4.0];
path = dijkstra(map, start, stop);
assertPathValid(testcase, map, path, start, stop, [0 10], [-5 20], [0 6]);
end

