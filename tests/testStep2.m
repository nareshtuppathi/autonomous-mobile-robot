classdef testStep2 < matlab.unittest.TestCase
%TESTSTEP2 Unit tests for Step 2: resolution, inflation, endpoints, smoothing, validation.
%   Run with: runtests('tests/testStep2.m')
%   Coordinates in the geometry tests are [row col] grid coordinates.

    methods (TestClassSetup)
        function addSourcePath(testCase)
            srcDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(srcDir, 'IncludeSubfolders', true));
        end
    end

    methods (Test)
        %% Configuration and map resolution
        function configDefaults(testCase)
            cfg = robotConfig();
            testCase.verifyEqual(cfg.cellSizeM, 0.25);
            testCase.verifyEqual(cfg.resolution, 4, 'AbsTol', 1e-12);
            testCase.verifyEqual(cfg.inflationRadiusM, 0.25, 'AbsTol', 1e-12);
            testCase.verifyEqual(cfg.inflationCells, 1);
        end

        function configRoundsUpToWholeCells(testCase)
            cfg = robotConfig('CellSizeM', 0.1, 'RobotRadiusM', 0.25, 'SafetyMarginM', 0.1);
            testCase.verifyEqual(cfg.inflationCells, 4);                  % 3.5 cells -> 4
            testCase.verifyEqual(cfg.effectiveInflationM, 0.4, 'AbsTol', 1e-12);
        end

        function configIgnoresFloatingPointNoise(testCase)
            cfg = robotConfig('CellSizeM', 0.1, 'RobotRadiusM', 0.2, 'SafetyMarginM', 0.1);
            testCase.verifyEqual(cfg.inflationCells, 3);                  % not 4
        end

        function configRejectsBadInput(testCase)
            testCase.verifyError(@() robotConfig('RobotRadiusM', -0.1), 'MATLAB:validators:mustBePositive');
            testCase.verifyError(@() robotConfig('CellSizeM', 0), 'MATLAB:validators:mustBePositive');
            testCase.verifyError(@() robotConfig('SafetyMarginM', -0.01), 'MATLAB:validators:mustBeNonnegative');
        end

        function mapResolutionAndWorldLimits(testCase)
            cfg = robotConfig('CellSizeM', 0.5);
            grid = false(4, 6);
            grid(2, 3) = true;
            map = buildBinaryOccupancyMap(grid, cfg);
            testCase.verifyEqual(map.Resolution, 2, 'AbsTol', 1e-12);         % cells per metre
            testCase.verifyEqual(map.XWorldLimits, [0 3], 'AbsTol', 1e-12);
            testCase.verifyEqual(map.YWorldLimits, [0 2], 'AbsTol', 1e-12);
            testCase.verifyEqual(logical(getOccupancy(map)), grid);
        end

        function coordinateConversionsAgreeWithToolbox(testCase)
            cfg = robotConfig('CellSizeM', 0.5);
            grid = false(4, 6);
            map = buildBinaryOccupancyMap(grid, cfg);
            [r, c] = ndgrid(1:4, 1:6);
            cells = [r(:), c(:)];
            xy = gridCoordsToWorld(cells, size(grid), cfg.cellSizeM);
            testCase.verifyEqual(xy, grid2world(map, cells), 'AbsTol', 1e-12);
            testCase.verifyEqual(worldToGridCoords(xy, size(grid), cfg.cellSizeM), cells, 'AbsTol', 1e-12);
        end

        %% Inflation
        function singleCellInflatesTo3x3(testCase)
            cfg = robotConfig();                       % 1-cell inflation
            grid = false(9, 9);  grid(5, 5) = true;
            inflated = logical(getOccupancy(inflateObstacles(buildBinaryOccupancyMap(grid, cfg), cfg)));
            expected = false(9, 9);  expected(4:6, 4:6) = true;
            testCase.verifyEqual(inflated, expected);
        end

        function twoCellInflationCoversDisk(testCase)
            cfg = robotConfig('RobotRadiusM', 0.45, 'SafetyMarginM', 0.05);   % 2 cells
            testCase.assertEqual(cfg.inflationCells, 2);
            grid = false(11, 11);  grid(6, 6) = true;
            inflated = logical(getOccupancy(inflateObstacles(buildBinaryOccupancyMap(grid, cfg), cfg)));
            [r, c] = ndgrid(1:11, 1:11);
            dr = r - 6;  dc = c - 6;
            testCase.verifyTrue(all(inflated(dr .^ 2 + dc .^ 2 <= 4)));           % full disk covered
            testCase.verifyFalse(any(inflated(max(abs(dr), abs(dc)) > 2)));      % nothing further out
        end

        function inflationUsesExactWholeCellsDespiteRounding(testCase)
            cfg = robotConfig('CellSizeM', 0.1, 'RobotRadiusM', 0.2, 'SafetyMarginM', 0.1);   % 3 cells
            grid = false(15, 15);  grid(8, 8) = true;
            inflated = logical(getOccupancy(inflateObstacles(buildBinaryOccupancyMap(grid, cfg), cfg)));
            [r, c] = find(inflated);
            testCase.verifyEqual([min(r) max(r) min(c) max(c)], [5 11 5 11]);
        end

        function inflationKeepsInputAndGrowsMonotonically(testCase)
            [grid, ~, ~] = createOccupancyMap('default');
            cfgSmall = robotConfig('RobotRadiusM', 0.20, 'SafetyMarginM', 0.05);
            cfgLarge = robotConfig('RobotRadiusM', 0.45, 'SafetyMarginM', 0.05);
            map = buildBinaryOccupancyMap(grid, cfgSmall);
            small = logical(getOccupancy(inflateObstacles(map, cfgSmall)));
            large = logical(getOccupancy(inflateObstacles(map, cfgLarge)));
            testCase.verifyEqual(logical(getOccupancy(map)), grid);            % input untouched
            testCase.verifyTrue(all(small(grid)));                             % contains real obstacles
            testCase.verifyTrue(all(large(small)));                            % bigger robot, bigger region
            testCase.verifyGreaterThan(nnz(large), nnz(small));
        end

        %% Endpoints
        function validEndpointsPass(testCase)
            [grid, s, g] = createOccupancyMap('default');
            cfg = robotConfig();
            plan = planInConfigurationSpace(grid, s, g, cfg);
            testCase.verifyTrue(plan.result.found);
        end

        function endpointInsideInflatedRegionErrors(testCase)
            [grid, ~, g] = createOccupancyMap('default');
            testCase.verifyError(@() planInConfigurationSpace(grid, [20 14], g, robotConfig()), ...
                'amrStep2:endpointInInflatedRegion');
            testCase.verifyError(@() planInConfigurationSpace(grid, [20 5], [20 16], robotConfig()), ...
                'amrStep2:endpointInInflatedRegion');
        end

        function endpointErrorDoesNotMoveThePoint(testCase)
            [grid, ~, g] = createOccupancyMap('default');
            try
                planInConfigurationSpace(grid, [20 14], g, robotConfig());
                testCase.verifyFail('Expected an error');
            catch ME
                testCase.verifyTrue(contains(ME.message, 'Start cell [20 14]'));
            end
        end

        function endpointInObstacleOrOutsideErrors(testCase)
            [grid, s, g] = createOccupancyMap('default');
            testCase.verifyError(@() planInConfigurationSpace(grid, s, [10 15], robotConfig()), 'amrStep2:endpointInObstacle');
            testCase.verifyError(@() planInConfigurationSpace(grid, s, [99 99], robotConfig()), 'amrStep2:endpointOutOfBounds');
            testCase.verifyError(@() planInConfigurationSpace(grid, [0 5], g, robotConfig()), 'amrStep2:endpointOutOfBounds');
        end

        %% Planning in the configuration space
        function planUsesInflatedMapAndReportsMetres(testCase)
            [grid, s, g] = createOccupancyMap('default');
            cfg = robotConfig();
            plan = planInConfigurationSpace(grid, s, g, cfg);
            idx = sub2ind(size(grid), plan.pathCells(:, 1), plan.pathCells(:, 2));
            testCase.verifyFalse(any(plan.inflatedGrid(idx)));
            testCase.verifyEqual(plan.pathLengthM, plan.result.pathLength * cfg.cellSizeM, 'AbsTol', 1e-12);
            testCase.verifyEqual(plan.pathWorld(1, :), gridCoordsToWorld(s, size(grid), cfg.cellSizeM), 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(plan.pathLengthM, astarPlanner(grid, s, g).pathLength * cfg.cellSizeM);
        end

        function oversizedRobotGivesCleanNoPath(testCase)
            [grid, s, g] = createOccupancyMap('default');
            cfg = robotConfig('RobotRadiusM', 0.45, 'SafetyMarginM', 0.05);
            plan = planInConfigurationSpace(grid, s, g, cfg);
            testCase.verifyFalse(plan.result.found);
            testCase.verifyEmpty(plan.pathCells);
            testCase.verifyTrue(isnan(plan.pathLengthM));
        end

        %% Segment geometry
        function segmentSquareIntersection(testCase)
            c = [0 0];
            testCase.verifyTrue(segmentIntersectsSquares([-2 0], [2 0], c, 0.5));      % through the middle
            testCase.verifyFalse(segmentIntersectsSquares([1 -2], [1 2], c, 0.5));     % parallel, outside
            testCase.verifyTrue(segmentIntersectsSquares([0.5 -2], [0.5 2], c, 0.5));  % grazes an edge
            testCase.verifyTrue(segmentIntersectsSquares([0 0], [0 0], c, 0.5));       % degenerate, inside
            testCase.verifyTrue(segmentIntersectsSquares([-2 0.2], [2 0.2], c, 0.5));  % vertical-direction slab case
            testCase.verifyFalse(segmentIntersectsSquares([-2 0.8], [2 0.8], c, 0.5));
        end

        function segmentSquareDistanceKnownValues(testCase)
            c = [0 0];
            testCase.verifyEqual(segmentToSquaresDistance([2 -5], [2 5], c, 0.5), 1.5, 'AbsTol', 1e-12);
            testCase.verifyEqual(segmentToSquaresDistance([2 2], [3 3], c, 0.5), hypot(1.5, 1.5), 'AbsTol', 1e-12);
            testCase.verifyEqual(segmentToSquaresDistance([-2 0], [2 0], c, 0.5), 0);
        end

        function segmentSquareDistanceMatchesBruteForce(testCase)
            rng(0);
            centers = [3 3; 5 8; 7 2; 4 6; 8 8];
            for k = 1:20
                a = 10 * rand(1, 2);  b = 10 * rand(1, 2);
                exact = segmentToSquaresDistance(a, b, centers, 0.5);
                t = linspace(0, 1, 20001).';
                pts = a + t .* (b - a);
                for q = 1:size(centers, 1)
                    dr = max(abs(pts(:, 1) - centers(q, 1)) - 0.5, 0);
                    dc = max(abs(pts(:, 2) - centers(q, 2)) - 0.5, 0);
                    testCase.verifyEqual(exact(q), min(hypot(dr, dc)), 'AbsTol', 1e-3);
                end
            end
        end

        function cellFreeSegmentTests(testCase)
            grid = false(7, 7);  grid(4, 4) = true;
            testCase.verifyFalse(isSegmentCellFree(grid, [4 1], [4 7]));        % through the obstacle
            testCase.verifyTrue(isSegmentCellFree(grid, [2 1], [2 7]));         % clear
            testCase.verifyFalse(isSegmentCellFree(grid, [3.5 1], [3.5 7]));    % runs along its edge
            testCase.verifyFalse(isSegmentCellFree(grid, [2 3], [5 6]));        % touches one corner only
            testCase.verifyFalse(isSegmentCellFree(grid, [0 3], [2 3]));        % leaves the map
        end

        %% Smoothing
        function openMapCollapsesToStraightLine(testCase)
            cfg = robotConfig('CellSizeM', 1, 'RobotRadiusM', 0.3, 'SafetyMarginM', 0.1);
            grid = false(10, 10);
            path = [(1:10).', (1:10).'];
            sm = smoothPath(path, grid, grid, cfg);
            testCase.verifyEqual(sm.pathCells, [1 1; 10 10]);
            testCase.verifyEqual(sm.lengthCells, 9 * sqrt(2), 'AbsTol', 1e-12);
        end

        function smoothingKeepsNecessaryCorner(testCase)
            cfg = robotConfig('CellSizeM', 1, 'RobotRadiusM', 0.3, 'SafetyMarginM', 0.1);
            grid = true(7, 7);  grid(:, 1) = false;  grid(7, :) = false;          % L-shaped corridor
            path = [(1:7).', ones(7, 1); 7 * ones(6, 1), (2:7).'];
            sm = smoothPath(path, grid, grid, cfg);
            testCase.verifyEqual(sm.pathCells, [1 1; 7 1; 7 7]);
            testCase.verifyEqual(sm.lengthCells, 12, 'AbsTol', 1e-12);
        end

        function smoothingRejectsShortcutThatLacksClearance(testCase)
            cfg = robotConfig('CellSizeM', 1, 'RobotRadiusM', 0.3, 'SafetyMarginM', 0.2);   % needs 0.5 cell
            grid = false(9, 9);  grid(5, 5) = true;
            looseInflated = false(9, 9);        % deliberately too permissive: cell test alone would accept
            path = [5 1; 3 3; 3 7; 5 9];
            sm = smoothPath(path, looseInflated, grid, cfg);
            testCase.verifyGreaterThan(size(sm.pathCells, 1), 2);                    % direct line hits the obstacle
            centers = findOccupiedCenters(grid);
            for k = 1:size(sm.pathCells, 1) - 1
                testCase.verifyGreaterThanOrEqual( ...
                    segmentClearanceCells(centers, sm.pathCells(k, :), sm.pathCells(k + 1, :)), 0.5 - 1e-9);
            end
        end

        function smoothingTinyPathsAreReturnedUnchanged(testCase)
            cfg = robotConfig();
            grid = false(5, 5);
            testCase.verifyEqual(smoothPath([2 2], grid, grid, cfg).pathCells, [2 2]);
            testCase.verifyEqual(smoothPath([2 2; 2 3], grid, grid, cfg).pathCells, [2 2; 2 3]);
            testCase.verifyEmpty(smoothPath(zeros(0, 2), grid, grid, cfg).pathCells);
        end

        function smoothedDefaultPathIsValidAndShorter(testCase)
            [grid, s, g] = createOccupancyMap('default');
            cfg = robotConfig();
            plan = planInConfigurationSpace(grid, s, g, cfg);
            sm = smoothPath(plan.pathCells, plan.inflatedGrid, grid, cfg);
            v = validatePathClearance(grid, plan.inflatedGrid, sm.pathCells, s, g, cfg);
            testCase.verifyTrue(v.isValid);
            testCase.verifyLessThan(size(sm.pathCells, 1), size(plan.pathCells, 1));
            testCase.verifyLessThanOrEqual(v.lengthM, plan.pathLengthM + 1e-9);
            testCase.verifyEqual(sm.pathCells, plan.pathCells(sm.keptIndices, :));
        end

        %% Path validation
        function validatorAcceptsGoodPathAndRejectsCollisions(testCase)
            cfg = robotConfig('CellSizeM', 1, 'RobotRadiusM', 0.3, 'SafetyMarginM', 0.2);
            grid = false(9, 9);  grid(5, 5) = true;
            inflated = false(9, 9);  inflated(4:6, 4:6) = true;
            good = validatePathClearance(grid, inflated, [3 1; 3 9], [3 1], [3 9], cfg);
            testCase.verifyTrue(good.isValid);
            testCase.verifyEqual(good.minClearanceM, 1.5, 'AbsTol', 1e-12);
            bad = validatePathClearance(grid, inflated, [5 1; 5 9], [5 1], [5 9], cfg);
            testCase.verifyFalse(bad.segmentsCellFree);
            testCase.verifyEqual(bad.numBlockedSegments, 1);
            testCase.verifyFalse(bad.denseSamplesFree);
            testCase.verifyFalse(bad.meetsRobotRadius);
            testCase.verifyFalse(bad.isValid);
        end

        function validatorChecksAlongSegmentsNotOnlyWaypoints(testCase)
            cfg = robotConfig('CellSizeM', 1, 'RobotRadiusM', 0.3, 'SafetyMarginM', 0.2);
            grid = false(9, 9);  grid(5, 5) = true;
            inflated = false(9, 9);  inflated(4:6, 4:6) = true;
            % Both waypoints are free, but the segment between them crosses the obstacle
            r = validatePathClearance(grid, inflated, [5 1; 5 9], [5 1], [5 9], cfg);
            testCase.verifyTrue(r.waypointsFree);
            testCase.verifyFalse(r.segmentsCellFree);
        end

        function validatorDetectsInsufficientMargin(testCase)
            cfg = robotConfig('CellSizeM', 1, 'RobotRadiusM', 0.4, 'SafetyMarginM', 0.2);   % needs 0.6
            grid = false(9, 9);  grid(5, 5) = true;
            loose = false(9, 9);                      % cell test passes everywhere
            r = validatePathClearance(grid, loose, [4 1; 4 9], [4 1], [4 9], cfg);
            testCase.verifyEqual(r.minClearanceM, 0.5, 'AbsTol', 1e-12);
            testCase.verifyTrue(r.meetsRobotRadius);              % 0.5 >= 0.4: no collision
            testCase.verifyFalse(r.meetsTotalClearance);          % 0.5 < 0.6: margin violated
            testCase.verifyFalse(r.isValid);
        end

        function validatorFlagsBadEndpointsBoundsAndEmpty(testCase)
            cfg = robotConfig('CellSizeM', 1, 'RobotRadiusM', 0.3, 'SafetyMarginM', 0.2);
            grid = false(9, 9);
            r = validatePathClearance(grid, grid, [3 1; 3 9], [1 1], [3 9], cfg);
            testCase.verifyFalse(r.startMatches);
            testCase.verifyFalse(r.isValid);
            r = validatePathClearance(grid, grid, [3 1; 3 12], [3 1], [3 12], cfg);
            testCase.verifyFalse(r.waypointsInBounds);
            testCase.verifyFalse(r.isValid);
            r = validatePathClearance(grid, grid, zeros(0, 2), [3 1], [3 9], cfg);
            testCase.verifyFalse(r.isValid);
        end

        %% Toolbox comparison and Step 1 regression
        function toolboxPlannerMatchesOurCornerCuttingAStar(testCase)
            [grid, s, g] = createOccupancyMap('default');
            cfg = robotConfig();
            plan = planInConfigurationSpace(grid, s, g, cfg);
            tb = runToolboxAStar(plan.inflatedMap, s, g);
            ours = astarPlanner(plan.inflatedGrid, s, g, 'AllowCornerCutting', true);
            testCase.verifyTrue(tb.found);
            testCase.verifyEqual(tb.pathCostM, ours.pathLength * cfg.cellSizeM, 'AbsTol', 1e-9);
            testCase.verifyEqual(tb.pathCostM, tb.pathLengthCells * cfg.cellSizeM, 'AbsTol', 1e-9);
            testCase.verifyLessThan(tb.pathCostM, plan.pathLengthM);   % corner cutting is cheaper
        end

        function cornerCuttingOptionOffByDefault(testCase)
            grid = false(2, 2);  grid(1, 2) = true;
            testCase.verifyEqual(astarPlanner(grid, [1 1], [2 2]).path, [1 1; 2 1; 2 2]);
            r = astarPlanner(grid, [1 1], [2 2], 'AllowCornerCutting', true);
            testCase.verifyEqual(r.path, [1 1; 2 2]);
            testCase.verifyEqual(r.pathLength, sqrt(2), 'AbsTol', 1e-12);
        end

        function step1ResultsAreUnchanged(testCase)
            [grid, s, g] = createOccupancyMap('default');
            r = astarPlanner(grid, s, g);
            testCase.verifyEqual(r.pathLength, 73.2548, 'AbsTol', 1e-4);
            testCase.verifyEqual(r.expandedNodes, 807);
            testCase.verifyEqual(size(r.path, 1), 61);
        end
    end
end
