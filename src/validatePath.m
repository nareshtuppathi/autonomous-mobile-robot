function report = validatePath(grid, path, start, goal)
%VALIDATEPATH Independently verify a planned path against the occupancy grid.
%   report = validatePath(grid, path, start, goal) checks a path without
%   reusing any planner internals. Report fields (logical unless noted):
%     hasCells         - path contains at least one cell
%     startMatches     - first cell equals start
%     goalMatches      - last cell equals goal
%     allInBounds      - every cell lies inside the map
%     allFree          - no cell is an obstacle
%     allAdjacent      - every consecutive pair of cells are 8-neighbours
%     noCornerCutting  - every diagonal step has both side cells free
%     noRepeatedCells  - no cell is visited twice
%     numCells         - number of cells in the path (double)
%     numDiagonalSteps - number of diagonal steps (double)
%     isValid          - all of the checks above are true

    [numRows, numCols] = size(grid);
    report.numCells = size(path, 1);
    report.hasCells = report.numCells > 0;

    if ~report.hasCells
        report.startMatches = false;
        report.goalMatches = false;
        report.allInBounds = false;
        report.allFree = false;
        report.allAdjacent = false;
        report.noCornerCutting = false;
        report.noRepeatedCells = false;
        report.numDiagonalSteps = 0;
        report.isValid = false;
        return;
    end

    report.startMatches = isequal(path(1, :), start);
    report.goalMatches  = isequal(path(end, :), goal);

    inBounds = path(:, 1) >= 1 & path(:, 1) <= numRows & ...
               path(:, 2) >= 1 & path(:, 2) <= numCols;
    report.allInBounds = all(inBounds);

    if report.allInBounds
        report.allFree = ~any(grid(sub2ind([numRows, numCols], path(:, 1), path(:, 2))));
    else
        report.allFree = false;   % cannot index cells outside the map
    end

    steps = diff(path, 1, 1);
    report.allAdjacent = all(max(abs(steps), [], 2) == 1);

    isDiagonal = abs(steps(:, 1)) == 1 & abs(steps(:, 2)) == 1;
    report.numDiagonalSteps = nnz(isDiagonal);
    report.noCornerCutting = true;
    if report.allInBounds && report.allAdjacent
        for k = find(isDiagonal).'
            sideA = path(k, :) + [steps(k, 1), 0];
            sideB = path(k, :) + [0, steps(k, 2)];
            if grid(sideA(1), sideA(2)) || grid(sideB(1), sideB(2))
                report.noCornerCutting = false;
                break;
            end
        end
    end

    report.noRepeatedCells = size(unique(path, 'rows'), 1) == size(path, 1);

    report.isValid = report.hasCells && report.startMatches && report.goalMatches && ...
        report.allInBounds && report.allFree && report.allAdjacent && ...
        report.noCornerCutting && report.noRepeatedCells;
end
