function rc = worldToGridCoords(xy, gridSize, cellSizeM)
%WORLDTOGRIDCOORDS Convert world [x y] in metres to continuous grid [row col].
%   Inverse of gridCoordsToWorld. Round the result to get the containing cell.
    rc = [gridSize(1) + 0.5 - xy(:, 2) / cellSizeM, xy(:, 1) / cellSizeM + 0.5];
end
