function inflatedMap = inflateObstacles(map, cfg)
%INFLATEOBSTACLES Inflate obstacles by robot radius plus safety margin.
%   inflatedMap = inflateObstacles(map, cfg) returns a copy of map in which
%   every obstacle is grown by cfg.inflationCells cells (the robot footprint
%   plus margin, rounded up to whole cells). The input map is not modified.
%
%   Planning then treats the robot as a single point: a cell is free in the
%   inflated map only if a disk of the inflation radius centred there would
%   not touch an obstacle cell (to within the cell quantisation).
%
%   Implementation note: binaryOccupancyMap/inflate rounds its radius up to
%   whole cells, so floating-point noise (for example 0.3/0.1) could add a
%   spurious extra cell. The radius passed to inflate is therefore the middle
%   of the last cell, (inflationCells - 0.5) / resolution, which rounds up to
%   exactly inflationCells regardless of rounding error.
%   For one cell the kernel is a 3x3 square; for larger radii it is a
%   disk-like shape (the square with its corners trimmed).

    arguments
        map (1, 1) binaryOccupancyMap
        cfg (1, 1) struct
    end
    inflatedMap = copy(map);
    inflate(inflatedMap, (cfg.inflationCells - 0.5) / cfg.resolution);
end
