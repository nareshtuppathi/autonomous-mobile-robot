function report = validatePathClearance(rawGrid, inflatedGrid, pathCells, startCell, goalCell, cfg)
%VALIDATEPATHCLEARANCE Verify a polyline path against inflated and real obstacles.
%   report = validatePathClearance(rawGrid, inflatedGrid, pathCells, startCell, goalCell, cfg)
%   works for any piecewise-linear path in grid coordinates (a cell-by-cell A*
%   path or a smoothed path) and checks the whole segments, not only waypoints.
%
%   Report fields (logical unless noted)
%     numWaypoints         - number of waypoints (double)
%     startMatches         - first waypoint equals start
%     goalMatches          - last waypoint equals goal
%     waypointsInBounds    - every waypoint lies inside the map
%     waypointsFree        - every waypoint lies in a free inflated cell
%     segmentsCellFree     - every segment crosses only free inflated cells (exact test)
%     numBlockedSegments   - segments that fail the previous test (double)
%     denseSamplesFree     - independent check: points sampled every 0.05 cell along
%                            every segment all fall in free inflated cells
%     minClearanceM        - exact minimum distance from the path to any real obstacle
%                            cell, in metres (double; Inf if there are no obstacles)
%     meetsRobotRadius     - minClearanceM >= robot radius (no collision)
%     meetsTotalClearance  - minClearanceM >= robot radius + safety margin
%     lengthM              - path length in metres (double)
%     isValid              - all of the checks above are true

    arguments
        rawGrid logical
        inflatedGrid logical
        pathCells (:, 2) double
        startCell (1, 2) double
        goalCell (1, 2) double
        cfg (1, 1) struct
    end

    tol = 1e-9;
    [numRows, numCols] = size(rawGrid);
    n = size(pathCells, 1);
    report.numWaypoints = n;

    if n == 0
        report.startMatches = false;  report.goalMatches = false;
        report.waypointsInBounds = false;  report.waypointsFree = false;
        report.segmentsCellFree = false;  report.numBlockedSegments = 0;
        report.denseSamplesFree = false;  report.minClearanceM = NaN;
        report.meetsRobotRadius = false;  report.meetsTotalClearance = false;
        report.lengthM = NaN;  report.isValid = false;
        return;
    end

    report.startMatches = isequal(pathCells(1, :), startCell);
    report.goalMatches  = isequal(pathCells(end, :), goalCell);

    inBounds = pathCells(:, 1) >= 0.5 & pathCells(:, 1) <= numRows + 0.5 & ...
               pathCells(:, 2) >= 0.5 & pathCells(:, 2) <= numCols + 0.5;
    report.waypointsInBounds = all(inBounds);

    report.waypointsFree = false;
    if report.waypointsInBounds
        wp = round(pathCells);
        wp(:, 1) = min(max(wp(:, 1), 1), numRows);
        wp(:, 2) = min(max(wp(:, 2), 1), numCols);
        report.waypointsFree = ~any(inflatedGrid(sub2ind([numRows numCols], wp(:, 1), wp(:, 2))));
    end

    obstacleCenters = findOccupiedCenters(rawGrid);
    blocked = 0;
    minClear = inf;
    denseOk = report.waypointsInBounds;
    for k = 1:n - 1
        a = pathCells(k, :);
        b = pathCells(k + 1, :);
        if ~isSegmentCellFree(inflatedGrid, a, b)
            blocked = blocked + 1;
        end
        minClear = min(minClear, segmentClearanceCells(obstacleCenters, a, b));
        if denseOk
            m = max(2, ceil(norm(b - a) / 0.05) + 1);
            s = linspace(0, 1, m).';
            pts = a + s .* (b - a);
            cells = round(pts);
            if any(cells(:, 1) < 1 | cells(:, 1) > numRows | cells(:, 2) < 1 | cells(:, 2) > numCols) || ...
                    any(inflatedGrid(sub2ind([numRows numCols], cells(:, 1), cells(:, 2))))
                denseOk = false;
            end
        end
    end
    if n == 1
        minClear = min(segmentToSquaresDistance(pathCells(1, :), pathCells(1, :), obstacleCenters, 0.5));
    end

    report.numBlockedSegments = blocked;
    report.segmentsCellFree   = blocked == 0;
    report.denseSamplesFree   = denseOk;
    report.minClearanceM      = minClear * cfg.cellSizeM;
    report.meetsRobotRadius   = report.minClearanceM >= cfg.robotRadiusM - tol;
    report.meetsTotalClearance = report.minClearanceM >= cfg.inflationRadiusM - tol;
    report.lengthM            = computePathLength(pathCells) * cfg.cellSizeM;

    report.isValid = report.startMatches && report.goalMatches && ...
        report.waypointsInBounds && report.waypointsFree && report.segmentsCellFree && ...
        report.denseSamplesFree && report.meetsRobotRadius && report.meetsTotalClearance;
end
