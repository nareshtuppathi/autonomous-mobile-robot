function map = buildBinaryOccupancyMap(grid, cfg)
%BUILDBINARYOCCUPANCYMAP Convert a logical occupancy grid to a binaryOccupancyMap.
%   map = buildBinaryOccupancyMap(grid, cfg) wraps the Step 1 grid (true =
%   occupied) in a Navigation Toolbox binaryOccupancyMap. The toolbox takes
%   its resolution in cells per metre, which robotConfig computes as
%   1 / cellSizeM.
%
%   Layout: grid row 1 is the top of the map (largest y). The map origin is
%   the bottom-left corner, so world x grows with column and world y shrinks
%   with row. Cell [r c] has its centre at
%       x = (c - 0.5) * cellSizeM,   y = (numRows - r + 0.5) * cellSizeM
%   which is what gridCoordsToWorld implements.

    arguments
        grid logical
        cfg (1, 1) struct
    end
    map = binaryOccupancyMap(grid, cfg.resolution);
end
