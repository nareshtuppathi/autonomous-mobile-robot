function result = astarPlanner(grid, start, goal, options)
%ASTARPLANNER A* shortest-path search on a 2D occupancy grid.
%   result = astarPlanner(grid, start, goal) searches an 8-connected grid.
%   Orthogonal moves cost 1, diagonal moves cost sqrt(2), and diagonal moves
%   that would cut an obstacle corner are not allowed (unless
%   'AllowCornerCutting' is true).
%
%   Inputs
%     grid  - logical matrix, true = occupied
%     start - [row col] start cell (must be free)
%     goal  - [row col] goal cell (must be free)
%
%   Name-value options
%     'Heuristic'          - 'octile' (default, A*) or 'zero' (turns the search
%                            into Dijkstra; used to cross-check optimality)
%     'AllowCornerCutting' - false (default). If true, diagonal moves may pass
%                            between two diagonally touching obstacle cells.
%
%   Output struct fields
%     found         - true if a path exists
%     path          - N-by-2 [row col] cells from start to goal ([] if none)
%     pathLength    - Euclidean path length in cell units (NaN if none)
%     expandedNodes - number of nodes removed from the open list and expanded
%                     (the goal counts when it is reached)
%     expandedCells - expandedNodes-by-2 [row col] cells, in expansion order
%     runtimeSec    - wall-clock search time
%     message       - short human-readable status
%
%   Open list: a vector of linear indices scanned for the lowest f = g + h
%   on every expansion (ties go to the lower h). That is O(n) per pop, which
%   is simple and fast enough for maps of this size; a binary heap is the
%   natural upgrade for large maps.

    arguments
        grid logical
        start (1, 2) double
        goal (1, 2) double
        options.Heuristic (1, :) char {mustBeMember(options.Heuristic, {'octile', 'zero'})} = 'octile'
        options.AllowCornerCutting (1, 1) logical = false
    end

    checkEndpoints(grid, start, goal);

    tStart = tic;
    [numRows, numCols] = size(grid);
    startIdx = sub2ind([numRows, numCols], start(1), start(2));
    goalIdx  = sub2ind([numRows, numCols], goal(1),  goal(2));

    if strcmp(options.Heuristic, 'octile')
        heuristic = @(cells) octileHeuristic(cells, goal);
    else
        heuristic = @(cells) zeros(size(cells, 1), 1);
    end

    gScore  = inf(numRows, numCols);   % best known cost from start
    parent  = zeros(numRows, numCols); % predecessor of each cell
    closed  = false(numRows, numCols); % already expanded
    inOpen  = false(numRows, numCols);

    gScore(startIdx) = 0;
    openList = startIdx;
    inOpen(startIdx) = true;

    expandedIdx = zeros(numel(grid), 1);
    numExpanded = 0;
    found = false;

    while ~isempty(openList)
        [current, openList] = popBestNode(openList, gScore, heuristic);
        inOpen(current) = false;
        closed(current) = true;
        numExpanded = numExpanded + 1;
        expandedIdx(numExpanded) = current;

        if current == goalIdx
            found = true;
            break;
        end

        [nbrIdx, nbrCost] = getNeighbors(grid, current, options.AllowCornerCutting);
        for k = 1:numel(nbrIdx)
            nbr = nbrIdx(k);
            if closed(nbr)
                continue;
            end
            tentativeG = gScore(current) + nbrCost(k);
            if tentativeG < gScore(nbr)
                gScore(nbr) = tentativeG;
                parent(nbr) = current;
                if ~inOpen(nbr)
                    openList(end + 1, 1) = nbr; %#ok<AGROW>
                    inOpen(nbr) = true;
                end
            end
        end
    end

    expandedIdx = expandedIdx(1:numExpanded);
    [er, ec] = ind2sub([numRows, numCols], expandedIdx);

    result.found         = found;
    result.expandedNodes = numExpanded;
    result.expandedCells = [er, ec];
    if found
        result.path       = reconstructPath(parent, goalIdx);
        result.pathLength = computePathLength(result.path);
        result.message    = 'Path found.';
    else
        result.path       = zeros(0, 2);
        result.pathLength = NaN;
        result.message    = 'No path exists between start and goal.';
    end
    result.runtimeSec = toc(tStart);
end

function [bestIdx, openList] = popBestNode(openList, gScore, heuristic)
%POPBESTNODE Remove and return the open node with the lowest f = g + h.
    [r, c] = ind2sub(size(gScore), openList);
    h = heuristic([r, c]);
    f = gScore(openList) + h;
    candidates = find(f <= min(f) + 1e-9);   % tolerance for floating-point ties
    [~, k] = min(h(candidates));             % prefer the node closest to the goal
    pos = candidates(k);
    bestIdx = openList(pos);
    openList(pos) = [];
    openList = reshape(openList, [], 1);   % keep a column even when emptied
end

function checkEndpoints(grid, start, goal)
%CHECKENDPOINTS Throw a clear error if start or goal is unusable.
    [numRows, numCols] = size(grid);
    names = {'start', 'goal'};
    points = {start, goal};
    for k = 1:2
        p = points{k};
        if any(p ~= round(p)) || p(1) < 1 || p(1) > numRows || p(2) < 1 || p(2) > numCols
            error('astarPlanner:outOfBounds', ...
                  'The %s cell [%g %g] is outside the %dx%d map.', ...
                  names{k}, p(1), p(2), numRows, numCols);
        end
        if grid(p(1), p(2))
            error('astarPlanner:occupiedEndpoint', ...
                  'The %s cell [%d %d] is inside an obstacle.', ...
                  names{k}, p(1), p(2));
        end
    end
end
