function free = isSegmentCellFree(inflatedGrid, a, b)
%ISSEGMENTCELLFREE True if segment a-b only crosses free cells of the inflated grid.
%   free = isSegmentCellFree(inflatedGrid, a, b) with a, b in continuous grid
%   coordinates [row col] (cell [r c] spans r +/- 0.5, c +/- 0.5).
%
%   Every occupied cell near the segment is tested for intersection with the
%   segment (exactly, not by sampling). Cells are closed squares, so a segment
%   that only grazes an obstacle corner counts as blocked. The segment must
%   also stay inside the map.

    [numRows, numCols] = size(inflatedGrid);
    inMap = @(p) p(1) >= 0.5 && p(1) <= numRows + 0.5 && p(2) >= 0.5 && p(2) <= numCols + 0.5;
    if ~inMap(a) || ~inMap(b)
        free = false;
        return;
    end

    r1 = max(1, floor(min(a(1), b(1))) - 1);
    r2 = min(numRows, ceil(max(a(1), b(1))) + 1);
    c1 = max(1, floor(min(a(2), b(2))) - 1);
    c2 = min(numCols, ceil(max(a(2), b(2))) + 1);
    [r, c] = find(inflatedGrid(r1:r2, c1:c2));
    centers = [r + r1 - 1, c + c1 - 1];
    free = ~any(segmentIntersectsSquares(a, b, centers, 0.5));
end
