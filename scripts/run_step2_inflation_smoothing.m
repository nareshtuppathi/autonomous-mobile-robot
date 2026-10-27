%% run_step2_inflation_smoothing.m
% Step 2 - physical map resolution, robot footprint clearance, path smoothing.
%
% Builds a binaryOccupancyMap from the Step 1 grid, inflates obstacles by the
% robot radius plus a safety margin, plans with the Step 1 A* planner in the
% inflated map, smooths the path while preserving clearance, validates every
% segment, and compares against Navigation Toolbox plannerAStarGrid.
% Everything is deterministic (fixed map, no random numbers).

clear; clc; close all;

scriptDir   = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
addpath(genpath(fullfile(projectRoot, 'src')));

figDir  = fullfile(projectRoot, 'results', 'figures');
csvFile = fullfile(projectRoot, 'results', 'step2_comparison.csv');
if ~exist(figDir, 'dir'), mkdir(figDir); end

checks = struct('name', {}, 'pass', {});

%% 1. Physical configuration and binaryOccupancyMap
cfg = robotConfig();   % 0.25 m/cell, robot radius 0.20 m, safety margin 0.05 m
[grid, startCell, goalCell] = createOccupancyMap('default');
gridSize = size(grid);

fprintf('=== Step 2: configuration ===\n');
fprintf('Cell size        : %.3f m/cell  (resolution = %g cells/m)\n', cfg.cellSizeM, cfg.resolution);
fprintf('Map              : %d x %d cells = %.2f m x %.2f m\n', gridSize(1), gridSize(2), ...
    gridSize(2) * cfg.cellSizeM, gridSize(1) * cfg.cellSizeM);
fprintf('Robot radius     : %.3f m (diameter %.3f m)\n', cfg.robotRadiusM, 2 * cfg.robotRadiusM);
fprintf('Safety margin    : %.3f m\n', cfg.safetyMarginM);
fprintf('Inflation        : %.3f m requested -> %d cell(s) = %.3f m applied\n', ...
    cfg.inflationRadiusM, cfg.inflationCells, cfg.effectiveInflationM);

map = buildBinaryOccupancyMap(grid, cfg);
fprintf('binaryOccupancyMap: GridSize [%d %d], Resolution %g, X [%g %g] m, Y [%g %g] m\n', ...
    map.GridSize, map.Resolution, map.XWorldLimits, map.YWorldLimits);
checks = addCheck(checks, 'binaryOccupancyMap resolution = 1 / cellSize', abs(map.Resolution - 1 / cfg.cellSizeM) < 1e-12);
checks = addCheck(checks, 'binaryOccupancyMap contents equal the Step 1 grid', isequal(logical(getOccupancy(map)), grid));
checks = addCheck(checks, 'map extent equals grid size times cell size', ...
    abs(diff(map.XWorldLimits) - gridSize(2) * cfg.cellSizeM) < 1e-9 && abs(diff(map.YWorldLimits) - gridSize(1) * cfg.cellSizeM) < 1e-9);
testCells = [1 1; 20 5; 20 55; 40 60];
checks = addCheck(checks, 'gridCoordsToWorld matches grid2world', ...
    max(abs(gridCoordsToWorld(testCells, gridSize, cfg.cellSizeM) - grid2world(map, testCells)), [], 'all') < 1e-12);

%% 2. Inflation
inflatedMap  = inflateObstacles(map, cfg);
inflatedGrid = logical(getOccupancy(inflatedMap));
fprintf('\n=== Obstacle inflation ===\n');
fprintf('Obstacle cells   : %d real -> %d inflated (%.1f%% -> %.1f%% of the map)\n', ...
    nnz(grid), nnz(inflatedGrid), 100 * nnz(grid) / numel(grid), 100 * nnz(inflatedGrid) / numel(grid));
fprintf('Start [%d %d] free in inflated map: %d, goal [%d %d]: %d\n', startCell, ~inflatedGrid(startCell(1), startCell(2)), ...
    goalCell, ~inflatedGrid(goalCell(1), goalCell(2)));
checks = addCheck(checks, 'Inflated map contains every real obstacle', all(inflatedGrid(grid)));
checks = addCheck(checks, 'Inflation added obstacle cells', nnz(inflatedGrid) > nnz(grid));
checks = addCheck(checks, 'Original map is unchanged by inflation', isequal(logical(getOccupancy(map)), grid));
plotInflationFigure(grid, inflatedGrid, cfg, startCell, goalCell, fullfile(figDir, 'step2_01_inflation.png'));

%% 3. A* (Step 1 planner) in the configuration-space map
cspace = planInConfigurationSpace(grid, startCell, goalCell, cfg);
fprintf('\n=== A* on the inflated map ===\n');
fprintf('Path found      : %d\n', cspace.result.found);
fprintf('Path length     : %.4f m (%.4f cells)\n', cspace.pathLengthM, cspace.result.pathLength);
fprintf('Waypoints       : %d\n', size(cspace.pathCells, 1));
fprintf('Expanded nodes  : %d of %d free inflated cells\n', cspace.result.expandedNodes, nnz(~inflatedGrid));
checks = addCheck(checks, 'A* finds a path in the inflated map', cspace.result.found);

%% 4. Smoothing
smooth = smoothPath(cspace.pathCells, inflatedGrid, grid, cfg);
fprintf('\n=== Path smoothing (clearance-preserving line-of-sight) ===\n');
fprintf('Waypoints       : %d -> %d\n', size(cspace.pathCells, 1), size(smooth.pathCells, 1));
fprintf('Length          : %.4f m -> %.4f m\n', cspace.pathLengthM, smooth.lengthCells * cfg.cellSizeM);
fprintf('Segment checks  : %d candidate shortcuts tested\n', smooth.numSegmentChecks);

%% 5. Validation of the grid path, the smoothed path and the Step 1 path
vGrid   = validatePathClearance(grid, inflatedGrid, cspace.pathCells, startCell, goalCell, cfg);
vSmooth = validatePathClearance(grid, inflatedGrid, smooth.pathCells, startCell, goalCell, cfg);
step1Result = astarPlanner(grid, startCell, goalCell);   % Step 1: no inflation
vStep1  = validatePathClearance(grid, inflatedGrid, step1Result.path, startCell, goalCell, cfg);

fprintf('\n=== Validation ===\n');
printValidation('A* grid path (inflated map)', vGrid);
printValidation('Smoothed path', vSmooth);
printValidation('Step 1 path (plain map, no inflation)', vStep1);

checks = addCheck(checks, 'Grid path passes all clearance checks', vGrid.isValid);
checks = addCheck(checks, 'Smoothed path passes all clearance checks', vSmooth.isValid);
checks = addCheck(checks, 'Smoothed waypoints are a subset of the grid path', ...
    isequal(smooth.pathCells, cspace.pathCells(smooth.keptIndices, :)));
checks = addCheck(checks, 'Smoothed path starts at start and ends at goal', vSmooth.startMatches && vSmooth.goalMatches);
checks = addCheck(checks, 'Smoothed path is not longer than the grid path', vSmooth.lengthM <= vGrid.lengthM + 1e-9);
checks = addCheck(checks, 'Smoothed path has fewer waypoints', vSmooth.numWaypoints < vGrid.numWaypoints);
checks = addCheck(checks, 'Smoothed path keeps at least the required clearance', vSmooth.minClearanceM >= cfg.inflationRadiusM - 1e-9);
checks = addCheck(checks, 'Step 1 path violates the robot footprint (shows why inflation is needed)', ~vStep1.meetsRobotRadius);

%% 6. Independent comparison with Navigation Toolbox plannerAStarGrid
tb          = runToolboxAStar(inflatedMap, startCell, goalCell);
oursNoCut   = astarPlanner(inflatedGrid, startCell, goalCell);
oursCut     = astarPlanner(inflatedGrid, startCell, goalCell, 'AllowCornerCutting', true);
vTb         = validatePathClearance(grid, inflatedGrid, tb.pathCells, startCell, goalCell, cfg);
cutsTb      = countCornerCuts(inflatedGrid, tb.pathCells);
cutsOursCut = countCornerCuts(inflatedGrid, oursCut.path);
cutsOurs    = countCornerCuts(inflatedGrid, oursNoCut.path);

tbTie = plannerAStarGrid(inflatedMap, 'TieBreaker', 'on');
[tbTiePath, tbTieInfo] = plan(tbTie, startCell, goalCell);

fprintf('\n=== plannerAStarGrid comparison (same inflated map) ===\n');
fprintf('%-38s %10s %12s %10s %10s\n', 'Planner', 'Length (m)', 'Expanded', 'Waypoints', 'Corner cuts');
fprintf('%-38s %10.4f %12d %10d %10d\n', 'Ours (Step 1 rules, no corner cutting)', ...
    oursNoCut.pathLength * cfg.cellSizeM, oursNoCut.expandedNodes, size(oursNoCut.path, 1), cutsOurs);
fprintf('%-38s %10.4f %12d %10d %10d\n', 'Ours with AllowCornerCutting = true', ...
    oursCut.pathLength * cfg.cellSizeM, oursCut.expandedNodes, size(oursCut.path, 1), cutsOursCut);
fprintf('%-38s %10.4f %12d %10d %10d\n', 'plannerAStarGrid (default settings)', ...
    tb.pathCostM, tb.expandedNodes, size(tb.pathCells, 1), cutsTb);
fprintf('%-38s %10.4f %12d %10d %10d\n', 'plannerAStarGrid (TieBreaker on)', ...
    tbTieInfo.PathCost, tbTieInfo.NumNodesExplored, size(tbTiePath, 1), countCornerCuts(inflatedGrid, tbTiePath));
fprintf('Toolbox path vs our corner-cutting path: cost difference %.2e m, identical cells: %d\n', ...
    abs(tb.pathCostM - oursCut.pathLength * cfg.cellSizeM), isequal(tb.pathCells, oursCut.path));
fprintf('Toolbox path under our checks: cell-free segments %d (blocked %d), min clearance %.3f m\n', ...
    vTb.segmentsCellFree, vTb.numBlockedSegments, vTb.minClearanceM);
checks = addCheck(checks, 'Toolbox cost equals our A* cost when corner cutting is allowed', ...
    abs(tb.pathCostM - oursCut.pathLength * cfg.cellSizeM) < 1e-9);
checks = addCheck(checks, 'Toolbox path is no longer than our no-corner-cutting path', ...
    tb.pathCostM <= oursNoCut.pathLength * cfg.cellSizeM + 1e-9);
checks = addCheck(checks, 'Toolbox PathCost (m) matches its own path length x cell size', ...
    abs(tb.pathCostM - tb.pathLengthCells * cfg.cellSizeM) < 1e-6);

%% 7. Comparison table and figures
names = {'Step 1 path (plain map, no inflation)'; 'A* grid path (inflated map)'; 'Smoothed path (inflated map)'; 'plannerAStarGrid (toolbox)'};
vAll  = [vStep1; vGrid; vSmooth; vTb];
T = table(names, [vAll.numWaypoints].', [vAll.lengthM].', [vAll.minClearanceM].', ...
    [vAll.meetsRobotRadius].', [vAll.meetsTotalClearance].', [vAll.segmentsCellFree].', [vAll.isValid].', ...
    'VariableNames', {'Path', 'Waypoints', 'LengthM', 'MinClearanceM', 'MeetsRobotRadius', 'MeetsTotalClearance', 'SegmentsInflatedFree', 'AllChecksPass'});
writetable(T, csvFile);
fprintf('\n=== Comparison table (saved to results/step2_comparison.csv) ===\n');
disp(T);

blue = [0.10 0.35 0.80];  red = [0.85 0.15 0.15];  gray = [0.45 0.45 0.45];
orange = [0.95 0.55 0.05]; purple = [0.50 0.20 0.70];
mk = @(name, cells, color, ls, lw, marker) struct('name', name, 'cells', cells, 'color', color, 'lineStyle', ls, 'lineWidth', lw, 'marker', marker);

pStep1  = mk('Step 1 path, plain map (collides with footprint)', step1Result.path, gray, '--', 1.5, 'none');
pGrid   = mk(sprintf('A* grid path, inflated map (%d waypoints, %.2f m)', vGrid.numWaypoints, vGrid.lengthM), cspace.pathCells, blue, '-', 1.5, '.');
pSmooth = mk(sprintf('Smoothed path (%d waypoints, %.2f m)', vSmooth.numWaypoints, vSmooth.lengthM), smooth.pathCells, red, '-', 2.8, 'o');
plotWorldPaths(grid, inflatedGrid, cfg, startCell, goalCell, [pStep1 pGrid pSmooth], ...
    'Grid path vs smoothed path in the inflated map', fullfile(figDir, 'step2_02_grid_vs_smoothed.png'));
plotClearanceProfile(grid, cfg, [pStep1 pGrid pSmooth], fullfile(figDir, 'step2_03_clearance_profile.png'));

pOurs = mk(sprintf('Ours, no corner cutting (%.2f m)', oursNoCut.pathLength * cfg.cellSizeM), oursNoCut.path, blue, '-', 2.5, 'none');
pTb   = mk(sprintf('plannerAStarGrid default (%.2f m)', tb.pathCostM), tb.pathCells, orange, '--', 2.5, 'none');
pCut  = mk(sprintf('Ours, corner cutting allowed (%.2f m)', oursCut.pathLength * cfg.cellSizeM), oursCut.path, purple, ':', 2.5, 'none');
plotWorldPaths(grid, inflatedGrid, cfg, startCell, goalCell, [pOurs pTb pCut], ...
    'Our A* vs Navigation Toolbox plannerAStarGrid', fullfile(figDir, 'step2_04_toolbox_comparison.png'));

%% 8. Failure cases
fprintf('\n=== Failure cases ===\n');

errId = '';
try
    planInConfigurationSpace(grid, [20 14], goalCell, cfg);   % adjacent to wall 1
catch ME
    errId = ME.identifier;
    fprintf('Start next to a wall -> %s\n  %s\n', ME.identifier, ME.message);
end
checks = addCheck(checks, 'Start inside inflated region raises amrStep2:endpointInInflatedRegion', strcmp(errId, 'amrStep2:endpointInInflatedRegion'));

errId = '';
try
    planInConfigurationSpace(grid, startCell, [10 15], cfg);   % inside wall 1
catch ME
    errId = ME.identifier;
    fprintf('Goal inside obstacle -> %s\n  %s\n', ME.identifier, ME.message);
end
checks = addCheck(checks, 'Goal inside obstacle raises amrStep2:endpointInObstacle', strcmp(errId, 'amrStep2:endpointInObstacle'));

errId = '';
try
    planInConfigurationSpace(grid, startCell, [99 99], cfg);
catch ME
    errId = ME.identifier;
    fprintf('Goal outside map -> %s\n  %s\n', ME.identifier, ME.message);
end
checks = addCheck(checks, 'Goal outside map raises amrStep2:endpointOutOfBounds', strcmp(errId, 'amrStep2:endpointOutOfBounds'));

bigCfg = robotConfig('RobotRadiusM', 0.45, 'SafetyMarginM', 0.05);   % 0.5 m = 2 cells
bigPlan = planInConfigurationSpace(grid, startCell, goalCell, bigCfg);
fprintf('Robot radius 0.45 m + 0.05 m margin (%d-cell inflation): found = %d, message: %s\n', ...
    bigCfg.inflationCells, bigPlan.result.found, bigPlan.result.message);
checks = addCheck(checks, 'Oversized robot: planner reports no path without error', ~bigPlan.result.found && isempty(bigPlan.pathCells));

errId = '';
try
    robotConfig('RobotRadiusM', -0.1);
catch ME
    errId = ME.identifier;
    fprintf('Negative radius -> %s\n', ME.identifier);
end
checks = addCheck(checks, 'Negative robot radius is rejected', ~isempty(errId));

%% 9. Summary
fprintf('\n=== Check summary ===\n');
for k = 1:numel(checks)
    fprintf('[%s] %s\n', passText(checks(k).pass), checks(k).name);
end
fprintf('\n%d of %d checks passed.\n', nnz([checks.pass]), numel(checks));
if ~all([checks.pass])
    error('run_step2:checksFailed', 'One or more checks failed.');
end

%% Local helpers
function checks = addCheck(checks, name, pass)
    checks(end + 1) = struct('name', name, 'pass', logical(pass));
end

function s = passText(pass)
    if pass, s = 'PASS'; else, s = 'FAIL'; end
end

function printValidation(label, v)
    fprintf('%s\n', label);
    fprintf('  waypoints %d | length %.4f m | min clearance %.4f m\n', v.numWaypoints, v.lengthM, v.minClearanceM);
    fprintf('  start/goal match %d/%d | waypoints in map %d, in free cells %d\n', v.startMatches, v.goalMatches, v.waypointsInBounds, v.waypointsFree);
    fprintf('  all segments in free inflated cells %d (blocked: %d) | dense samples free %d\n', v.segmentsCellFree, v.numBlockedSegments, v.denseSamplesFree);
    fprintf('  clearance >= robot radius %d | >= radius + margin %d | ALL PASS %d\n', v.meetsRobotRadius, v.meetsTotalClearance, v.isValid);
end

function n = countCornerCuts(grid, path)
    n = 0;
    steps = diff(path, 1, 1);
    for k = find(abs(steps(:, 1)) == 1 & abs(steps(:, 2)) == 1).'
        a = path(k, :) + [steps(k, 1), 0];
        b = path(k, :) + [0, steps(k, 2)];
        if grid(a(1), a(2)) || grid(b(1), b(2))
            n = n + 1;
        end
    end
end
