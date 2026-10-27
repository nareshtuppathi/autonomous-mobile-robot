function out = runToolboxAStar(inflatedMap, startCell, goalCell)
%RUNTOOLBOXASTAR Plan with Navigation Toolbox plannerAStarGrid for comparison.
%   out = runToolboxAStar(inflatedMap, startCell, goalCell) runs plannerAStarGrid
%   with its default settings (8-connected, Euclidean costs, tie-breaker off)
%   on the inflated binaryOccupancyMap.
%     out.found, out.pathCells ([row col]), out.pathLengthCells,
%     out.pathCostM (toolbox PathCost, in metres), out.expandedNodes (toolbox
%     NumNodesExplored), out.runtimeSec

    planner = plannerAStarGrid(inflatedMap);
    tStart = tic;
    [p, info] = plan(planner, startCell, goalCell);
    out.runtimeSec = toc(tStart);
    out.found = ~isempty(p);
    out.pathCells = p;
    out.pathLengthCells = computePathLength(p);
    out.pathCostM = info.PathCost;
    out.expandedNodes = info.NumNodesExplored;
end
