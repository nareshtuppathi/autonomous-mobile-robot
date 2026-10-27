function sc = buildStep3Scenario()
%BUILDSTEP3SCENARIO The nominal Step 3 (Scenario A) inputs, built exactly as run_step3_trajectory_tracking.m does.
%   Reuses the Step 1 map and A*, the Step 2 inflation and smoothing, and the Step 3 reference.
%   Output fields: mapCfg, grid, inflatedGrid, startCell, goalCell, pathWorld, startWorld, goalWorld, tcfg, ref,
%   env, startPose ([x y theta], heading along the first path segment).
    sc.mapCfg = robotConfig();
    [sc.grid, sc.startCell, sc.goalCell] = createOccupancyMap('default');
    gridSize = size(sc.grid);
    cspace = planInConfigurationSpace(sc.grid, sc.startCell, sc.goalCell, sc.mapCfg);
    smooth = smoothPath(cspace.pathCells, cspace.inflatedGrid, sc.grid, sc.mapCfg);
    sc.smoothCells = smooth.pathCells;
    sc.inflatedGrid = cspace.inflatedGrid;
    sc.pathWorld  = gridCoordsToWorld(smooth.pathCells, gridSize, sc.mapCfg.cellSizeM);
    sc.startWorld = gridCoordsToWorld(sc.startCell, gridSize, sc.mapCfg.cellSizeM);
    sc.goalWorld  = gridCoordsToWorld(sc.goalCell, gridSize, sc.mapCfg.cellSizeM);
    sc.tcfg = trackingConfig();
    sc.ref  = generateReferenceTrajectory(sc.pathWorld, sc.tcfg);
    sc.env  = makeClearanceEnvironment(sc.grid, sc.mapCfg);
    sc.startPose = [sc.startWorld, sc.ref.theta(1)];
end
