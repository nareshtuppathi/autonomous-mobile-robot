function xy = gridCoordsToWorld(rc, gridSize, cellSizeM)
%GRIDCOORDSTOWORLD Convert grid coordinates [row col] to world [x y] in metres.
%   rc may be fractional: integer values are cell centres, and a cell [r c]
%   spans rc = [r +/- 0.5, c +/- 0.5]. gridSize = [numRows numCols].
%   The world origin is the bottom-left corner of the map.
%   Matches grid2world of a binaryOccupancyMap built by buildBinaryOccupancyMap.
    xy = [(rc(:, 2) - 0.5) * cellSizeM, (gridSize(1) - rc(:, 1) + 0.5) * cellSizeM];
end
