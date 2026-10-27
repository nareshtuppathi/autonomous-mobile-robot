classdef testAStarPlanner < matlab.unittest.TestCase
%TESTASTARPLANNER Unit tests for the A* planner and its helpers.
%   Run with: runtests('tests/testAStarPlanner.m')

    methods (TestClassSetup)
        function addSourcePath(testCase)
            srcDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(srcDir));
        end
    end

    methods (Test)
        function straightCorridor(testCase)
            grid = false(1, 5);
            r = astarPlanner(grid, [1 1], [1 5]);
            testCase.verifyTrue(r.found);
            testCase.verifyEqual(r.pathLength, 4, 'AbsTol', 1e-12);
            testCase.verifyEqual(r.path, [1 1; 1 2; 1 3; 1 4; 1 5]);
        end

        function openGridUsesDiagonals(testCase)
            r = astarPlanner(false(5, 5), [1 1], [5 5]);
            testCase.verifyEqual(r.pathLength, 4 * sqrt(2), 'AbsTol', 1e-12);
            testCase.verifyEqual(size(r.path, 1), 5);
        end

        function startEqualsGoal(testCase)
            r = astarPlanner(false(3, 3), [2 2], [2 2]);
            testCase.verifyTrue(r.found);
            testCase.verifyEqual(r.pathLength, 0);
            testCase.verifyEqual(r.path, [2 2]);
        end

        function noCornerCutting(testCase)
            grid = false(2, 2);
            grid(1, 2) = true;            % blocks the diagonal (1,1) -> (2,2)
            r = astarPlanner(grid, [1 1], [2 2]);
            testCase.verifyTrue(r.found);
            testCase.verifyEqual(r.path, [1 1; 2 1; 2 2]);
            testCase.verifyEqual(r.pathLength, 2, 'AbsTol', 1e-12);
        end

        function blockedGoalReturnsNoPath(testCase)
            grid = false(3, 3);
            grid(2, :) = true;            % full wall across the map
            r = astarPlanner(grid, [1 1], [3 3]);
            testCase.verifyFalse(r.found);
            testCase.verifyEmpty(r.path);
            testCase.verifyTrue(isnan(r.pathLength));
            testCase.verifyEqual(r.expandedNodes, 3);   % the whole top row
        end

        function defaultMapPathIsValidAndOptimal(testCase)
            [grid, start, goal] = createOccupancyMap('default');
            r = astarPlanner(grid, start, goal);
            d = astarPlanner(grid, start, goal, 'Heuristic', 'zero');
            testCase.verifyTrue(r.found);
            testCase.verifyTrue(validatePath(grid, r.path, start, goal).isValid);
            testCase.verifyEqual(r.pathLength, d.pathLength, 'AbsTol', 1e-9);
            testCase.verifyLessThan(r.expandedNodes, d.expandedNodes);
        end

        function noPathMapReturnsNotFound(testCase)
            [grid, start, goal] = createOccupancyMap('noPath');
            r = astarPlanner(grid, start, goal);
            testCase.verifyFalse(r.found);
            testCase.verifyEmpty(r.path);
        end

        function occupiedStartThrows(testCase)
            [grid, ~, goal] = createOccupancyMap('default');
            testCase.verifyError(@() astarPlanner(grid, [10 15], goal), ...
                'astarPlanner:occupiedEndpoint');
        end

        function outOfBoundsGoalThrows(testCase)
            [grid, start, ~] = createOccupancyMap('default');
            testCase.verifyError(@() astarPlanner(grid, start, [99 99]), ...
                'astarPlanner:outOfBounds');
        end

        function validatePathCatchesBadPaths(testCase)
            grid = false(5, 5);
            grid(3, 3) = true;
            start = [1 1];  goal = [5 5];
            jump     = [1 1; 3 3; 5 5];                 % non-adjacent steps
            throughObstacle = [1 1; 2 2; 3 3; 4 4; 5 5]; % crosses the obstacle
            outside  = [1 1; 1 0];                      % leaves the map
            testCase.verifyFalse(validatePath(grid, jump, start, goal).allAdjacent);
            testCase.verifyFalse(validatePath(grid, throughObstacle, start, goal).allFree);
            testCase.verifyFalse(validatePath(grid, outside, start, goal).allInBounds);
        end

        function validatePathCatchesCornerCutting(testCase)
            grid = false(2, 2);
            grid(1, 2) = true;
            report = validatePath(grid, [1 1; 2 2], [1 1], [2 2]);
            testCase.verifyTrue(report.allAdjacent);
            testCase.verifyFalse(report.noCornerCutting);
            testCase.verifyFalse(report.isValid);
        end
    end
end
