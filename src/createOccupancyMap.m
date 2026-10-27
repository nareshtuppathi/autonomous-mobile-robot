function [grid, start, goal] = createOccupancyMap(scenario)
%CREATEOCCUPANCYMAP Build a fixed, reproducible 2D occupancy grid.
%   [grid, start, goal] = createOccupancyMap() returns the default scenario.
%   [grid, start, goal] = createOccupancyMap('noPath') returns the same map
%   with the goal sealed inside a closed box, so no path can exist.
%
%   grid  - logical matrix [rows x cols]; true = occupied, false = free
%   start - [row col] of the start cell
%   goal  - [row col] of the goal cell
%
%   The map is fully deterministic: no random numbers are used, so every
%   run produces exactly the same map.

    if nargin < 1
        scenario = 'default';
    end
    scenario = validatestring(scenario, {'default', 'noPath'});

    numRows = 40;
    numCols = 60;
    grid = false(numRows, numCols);

    % Outer boundary walls
    grid([1 numRows], :) = true;
    grid(:, [1 numCols]) = true;

    % Three vertical walls, each with a gap the robot must find
    grid(1:30, 15) = true;          % wall 1: gap at rows 31-39 (bottom)
    grid(11:numRows, 30) = true;    % wall 2: gap at rows 2-10 (top)
    grid(:, 45) = true;             % wall 3: gap at rows 18-23 (middle)
    grid(18:23, 45) = false;

    % Rectangular obstacles that force additional detours
    grid(8:12, 20:25) = true;
    grid(25:32, 35:40) = true;
    grid(5:9, 50:55) = true;
    grid(28:34, 5:10) = true;

    start = [20, 5];
    goal  = [20, 55];

    if strcmp(scenario, 'noPath')
        % Seal the goal inside a closed rectangular box (4-connected ring).
        grid(17:23, 52:58) = true;
        grid(18:22, 53:57) = false;
    end

    assert(~grid(start(1), start(2)), 'Start cell must be free.');
    assert(~grid(goal(1),  goal(2)),  'Goal cell must be free.');
end
