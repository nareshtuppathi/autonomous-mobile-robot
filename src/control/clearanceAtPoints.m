function [clearanceM, inMap] = clearanceAtPoints(env, xy)
%CLEARANCEATPOINTS Exact distance from world points to the nearest occupied cell.
%   [clearanceM, inMap] = clearanceAtPoints(env, xy) with xy an N-by-2 list of world
%   positions [x y] in metres. clearanceM is N-by-1 (Inf if the map has no obstacles);
%   inMap is N-by-1 and false for points outside the map.
    rc = worldToGridCoords(xy, env.gridSize, env.cellSizeM);
    inMap = rc(:, 1) >= 0.5 & rc(:, 1) <= env.gridSize(1) + 0.5 & ...
            rc(:, 2) >= 0.5 & rc(:, 2) <= env.gridSize(2) + 0.5;
    n = size(rc, 1);
    clearanceM = inf(n, 1);
    if isempty(env.obstacleCenters)
        return;
    end
    chunk = 2000;
    for k = 1:chunk:n
        idx = k:min(k + chunk - 1, n);
        dr = max(abs(rc(idx, 1) - env.obstacleCenters(:, 1).') - 0.5, 0);
        dc = max(abs(rc(idx, 2) - env.obstacleCenters(:, 2).') - 0.5, 0);
        clearanceM(idx) = min(hypot(dr, dc), [], 2) * env.cellSizeM;
    end
end
