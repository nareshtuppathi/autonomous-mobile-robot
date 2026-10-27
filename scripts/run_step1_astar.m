%% run_step1_astar.m
% Step 1 - A* path planning on a 2D occupancy grid.
%
% Runs the default scenario (path exists) and the no-path scenario, verifies
% the results, and saves figures to results/figures. All results are
% deterministic: the maps are fixed and no random numbers are used.

clear; clc; close all;

scriptDir   = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
addpath(fullfile(projectRoot, 'src'));

figDir  = fullfile(projectRoot, 'results', 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end

checks = struct('name', {}, 'pass', {});

%% 1. Default scenario: build the map
[grid, start, goal] = createOccupancyMap('default');
fprintf('=== Scenario 1: default map ===\n');
fprintf('Map size : %d rows x %d cols, %d obstacle cells (%.1f%%)\n', ...
    size(grid, 1), size(grid, 2), nnz(grid), 100 * nnz(grid) / numel(grid));
fprintf('Start    : [row %d, col %d]\n', start);
fprintf('Goal     : [row %d, col %d]\n', goal);
plotPlanningResult(grid, start, goal, [], 'Occupancy grid, start and goal', ...
    fullfile(figDir, '01_occupancy_map.png'));

%% 2. Run A*
result = astarPlanner(grid, start, goal);
fprintf('\n--- A* result ---\n');
fprintf('Path found      : %d\n', result.found);
fprintf('Path length     : %.4f cell units (%d cells, %d steps)\n', ...
    result.pathLength, size(result.path, 1), size(result.path, 1) - 1);
fprintf('Expanded nodes  : %d of %d free cells\n', ...
    result.expandedNodes, nnz(~grid));
fprintf('Runtime         : %.4f s\n', result.runtimeSec);
checks = addCheck(checks, 'A* finds a path on the default map', result.found);

%% 3. Verify the path independently of the planner
report = validatePath(grid, result.path, start, goal);
fprintf('\n--- Path validation ---\n');
fprintf('Starts at start cell        : %d\n', report.startMatches);
fprintf('Ends at goal cell           : %d\n', report.goalMatches);
fprintf('All %d cells inside the map : %d\n', report.numCells, report.allInBounds);
fprintf('All cells obstacle-free     : %d\n', report.allFree);
fprintf('Consecutive cells adjacent  : %d  (%d diagonal steps)\n', ...
    report.allAdjacent, report.numDiagonalSteps);
fprintf('No obstacle corner cutting  : %d\n', report.noCornerCutting);
fprintf('No repeated cells           : %d\n', report.noRepeatedCells);
checks = addCheck(checks, 'Path is inside the map', report.allInBounds);
checks = addCheck(checks, 'Path avoids all obstacles', report.allFree);
checks = addCheck(checks, 'Path cells are connected (adjacent)', report.allAdjacent);
checks = addCheck(checks, 'Path starts at start and ends at goal', report.startMatches && report.goalMatches);
checks = addCheck(checks, 'No diagonal cuts across obstacle corners', report.noCornerCutting);
checks = addCheck(checks, 'Path has no repeated cells', report.noRepeatedCells);

%% 4. Cross-checks: optimality and reproducibility
dijkstra = astarPlanner(grid, start, goal, 'Heuristic', 'zero');
fprintf('\n--- Cross-checks ---\n');
fprintf('Dijkstra (h = 0) length : %.4f, expanded %d nodes\n', ...
    dijkstra.pathLength, dijkstra.expandedNodes);
fprintf('A* (octile) length      : %.4f, expanded %d nodes\n', ...
    result.pathLength, result.expandedNodes);
checks = addCheck(checks, 'A* cost equals Dijkstra cost (optimal)', ...
    abs(result.pathLength - dijkstra.pathLength) < 1e-9);
checks = addCheck(checks, 'A* expands fewer nodes than Dijkstra', ...
    result.expandedNodes < dijkstra.expandedNodes);

repeat = astarPlanner(grid, start, goal);
checks = addCheck(checks, 'Repeated run gives an identical path', isequal(repeat.path, result.path));

plotPlanningResult(grid, start, goal, result, ...
    sprintf('A* path: length %.2f, %d expanded nodes', result.pathLength, result.expandedNodes), ...
    fullfile(figDir, '02_astar_path.png'));

%% 5. No-path scenario
[gridNP, startNP, goalNP] = createOccupancyMap('noPath');
fprintf('\n=== Scenario 2: no-path map (goal sealed inside a box) ===\n');
threw = false;
try
    resultNP = astarPlanner(gridNP, startNP, goalNP);
catch ME
    threw = true;
    fprintf('Unexpected error: %s\n', ME.message);
end
checks = addCheck(checks, 'No-path case does not throw an error', ~threw);
if ~threw
    fprintf('Path found      : %d\n', resultNP.found);
    fprintf('Message         : %s\n', resultNP.message);
    fprintf('Expanded nodes  : %d (entire reachable region explored)\n', resultNP.expandedNodes);
    fprintf('Path cells      : %d, path length: %g\n', size(resultNP.path, 1), resultNP.pathLength);
    checks = addCheck(checks, 'No-path case reports found = false', ~resultNP.found);
    checks = addCheck(checks, 'No-path case returns an empty path', isempty(resultNP.path));
    checks = addCheck(checks, 'No-path case: validatePath rejects empty path', ~validatePath(gridNP, resultNP.path, startNP, goalNP).isValid);
    plotPlanningResult(gridNP, startNP, goalNP, resultNP, ...
        sprintf('No path exists: search exhausted after %d expanded nodes', resultNP.expandedNodes), ...
        fullfile(figDir, '03_no_path.png'));
end

%% 6. Invalid input is rejected with a clear error
fprintf('\n=== Input validation ===\n');
badStart = [10, 15];   % inside wall 1
errId = '';
try
    astarPlanner(grid, badStart, goal);
catch ME
    errId = ME.identifier;
    fprintf('Start inside obstacle -> %s\n', ME.message);
end
checks = addCheck(checks, 'Obstacle start cell raises astarPlanner:occupiedEndpoint', ...
    strcmp(errId, 'astarPlanner:occupiedEndpoint'));

%% 7. Summary
fprintf('\n=== Check summary ===\n');
for k = 1:numel(checks)
    fprintf('[%s] %s\n', passText(checks(k).pass), checks(k).name);
end
allPassed = all([checks.pass]);
fprintf('\n%d of %d checks passed.\n', nnz([checks.pass]), numel(checks));
if ~allPassed
    error('run_step1_astar:checksFailed', 'One or more checks failed.');
end

%% Local helpers
function checks = addCheck(checks, name, pass)
    checks(end + 1) = struct('name', name, 'pass', logical(pass));
end

function s = passText(pass)
    if pass, s = 'PASS'; else, s = 'FAIL'; end
end
