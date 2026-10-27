function centers = findOccupiedCenters(grid)
%FINDOCCUPIEDCENTERS Centres [row col] of all occupied cells, as a K-by-2 matrix.
    [r, c] = find(grid);
    centers = [r, c];
end
