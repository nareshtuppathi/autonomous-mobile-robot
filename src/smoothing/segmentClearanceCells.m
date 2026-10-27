function clearance = segmentClearanceCells(obstacleCenters, a, b)
%SEGMENTCLEARANCECELLS Smallest distance from segment a-b to any obstacle cell.
%   clearance = segmentClearanceCells(obstacleCenters, a, b) returns the exact
%   Euclidean distance, in cells, from the segment to the nearest occupied
%   cell square (obstacleCenters from findOccupiedCenters on the REAL grid, not
%   the inflated one). Multiply by the cell size for metres. Inf if no obstacles.
    if isempty(obstacleCenters)
        clearance = inf;
        return;
    end
    clearance = min(segmentToSquaresDistance(a, b, obstacleCenters, 0.5));
end
