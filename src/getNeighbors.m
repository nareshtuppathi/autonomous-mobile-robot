function [nbrIdx, nbrCost] = getNeighbors(grid, idx, allowCornerCutting)
%GETNEIGHBORS Free 8-connected neighbours of a cell, with step costs.
%   [nbrIdx, nbrCost] = getNeighbors(grid, idx) returns the linear indices
%   of all reachable neighbours of cell idx and the cost of moving to each
%   (1 for orthogonal, sqrt(2) for diagonal).
%
%   A diagonal move is rejected if either of the two orthogonally adjacent
%   cells is an obstacle, so the robot never cuts across an obstacle corner.
%   Pass allowCornerCutting = true to permit such diagonals (default false).
%   The option exists so the planner can be compared with toolbox planners
%   that allow corner cutting.

    if nargin < 3
        allowCornerCutting = false;
    end

    [numRows, numCols] = size(grid);
    [r, c] = ind2sub([numRows, numCols], idx);

    moves = [-1  0; 1  0; 0 -1; 0  1; -1 -1; -1  1; 1 -1; 1  1];
    nbrIdx  = zeros(8, 1);
    nbrCost = zeros(8, 1);
    count = 0;

    for k = 1:size(moves, 1)
        nr = r + moves(k, 1);
        nc = c + moves(k, 2);

        if nr < 1 || nr > numRows || nc < 1 || nc > numCols
            continue;                       % outside the map
        end
        if grid(nr, nc)
            continue;                       % occupied
        end

        isDiagonal = moves(k, 1) ~= 0 && moves(k, 2) ~= 0;
        if isDiagonal && ~allowCornerCutting && ...
                (grid(r + moves(k, 1), c) || grid(r, c + moves(k, 2)))
            continue;                       % would cut an obstacle corner
        end

        count = count + 1;
        nbrIdx(count)  = sub2ind([numRows, numCols], nr, nc);
        nbrCost(count) = hypot(moves(k, 1), moves(k, 2));
    end

    nbrIdx  = nbrIdx(1:count);
    nbrCost = nbrCost(1:count);
end
