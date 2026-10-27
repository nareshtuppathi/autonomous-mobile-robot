function [dist, clearanceM] = pathClearanceProfile(rawGrid, pathCells, cfg, stepM)
%PATHCLEARANCEPROFILE Clearance to the nearest real obstacle along a polyline path.
%   [dist, clearanceM] = pathClearanceProfile(rawGrid, pathCells, cfg, stepM)
%   samples the path every stepM metres (default cellSizeM/10) and returns the
%   distance travelled along the path (m) and the exact distance (m) from each
%   sample to the nearest obstacle cell square.

    if nargin < 4
        stepM = cfg.cellSizeM / 10;
    end
    centers = findOccupiedCenters(rawGrid);
    stepCells = stepM / cfg.cellSizeM;

    pts = pathCells(1, :);
    dist = 0;
    travelled = 0;
    for k = 1:size(pathCells, 1) - 1
        a = pathCells(k, :);  b = pathCells(k + 1, :);
        m = max(1, ceil(norm(b - a) / stepCells));
        s = (1:m).' / m;
        pts = [pts; a + s .* (b - a)]; %#ok<AGROW>
        dist = [dist; travelled + s * norm(b - a) * cfg.cellSizeM]; %#ok<AGROW>
        travelled = travelled + norm(b - a) * cfg.cellSizeM;
    end

    dr = max(abs(pts(:, 1) - centers(:, 1).') - 0.5, 0);
    dc = max(abs(pts(:, 2) - centers(:, 2).') - 0.5, 0);
    clearanceM = min(hypot(dr, dc), [], 2) * cfg.cellSizeM;
end
