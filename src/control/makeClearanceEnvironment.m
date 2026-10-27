function env = makeClearanceEnvironment(rawGrid, mapCfg)
%MAKECLEARANCEENVIRONMENT Obstacle data for checking real-world trajectories.
%   env = makeClearanceEnvironment(rawGrid, mapCfg) with mapCfg = robotConfig().
%   The check uses the ORIGINAL occupied grid (not the inflated one):
%     collision          - clearance < robot radius (the footprint touches an obstacle)
%     clearance violation - clearance < robot radius + safety margin (the Step 2 requirement)
%   Clearance is the exact Euclidean distance from the robot centre to the nearest
%   occupied cell, with each occupied cell treated as a full square.
    env.obstacleCenters   = findOccupiedCenters(rawGrid);
    env.gridSize          = size(rawGrid);
    env.cellSizeM         = mapCfg.cellSizeM;
    env.robotRadiusM      = mapCfg.robotRadiusM;
    env.requiredClearanceM = mapCfg.inflationRadiusM;
end
