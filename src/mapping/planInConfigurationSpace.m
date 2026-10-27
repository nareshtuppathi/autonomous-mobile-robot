function plan = planInConfigurationSpace(grid, startCell, goalCell, cfg)
%PLANINCONFIGURATIONSPACE Inflate obstacles, check endpoints, run A* (Step 1 code).
%   plan = planInConfigurationSpace(grid, startCell, goalCell, cfg)
%     1. wraps grid in a binaryOccupancyMap (cfg.resolution cells per metre)
%     2. inflates obstacles by robot radius + safety margin
%     3. errors clearly if start or goal is not valid in the inflated map
%     4. runs astarPlanner (the Step 1 implementation) on the inflated grid
%
%   Output fields
%     cfg, rawGrid, inflatedGrid, map, inflatedMap, startCell, goalCell
%     result      - struct returned by astarPlanner (found, path, expandedNodes, ...)
%     pathCells   - N-by-2 [row col] cells ([] rows if no path)
%     pathWorld   - N-by-2 [x y] in metres
%     pathLengthM - path length in metres (NaN if no path)

    arguments
        grid logical
        startCell (1, 2) double
        goalCell (1, 2) double
        cfg (1, 1) struct
    end

    plan.cfg         = cfg;
    plan.rawGrid     = grid;
    plan.startCell   = startCell;
    plan.goalCell    = goalCell;
    plan.map         = buildBinaryOccupancyMap(grid, cfg);
    plan.inflatedMap = inflateObstacles(plan.map, cfg);
    plan.inflatedGrid = logical(getOccupancy(plan.inflatedMap));

    assertEndpointsFree(grid, plan.inflatedGrid, startCell, goalCell, cfg);

    plan.result      = astarPlanner(plan.inflatedGrid, startCell, goalCell);
    plan.pathCells   = plan.result.path;
    plan.pathWorld   = gridCoordsToWorld(plan.pathCells, size(grid), cfg.cellSizeM);
    plan.pathLengthM = plan.result.pathLength * cfg.cellSizeM;
end
