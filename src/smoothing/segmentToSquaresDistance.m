function dist = segmentToSquaresDistance(a, b, centers, halfSize)
%SEGMENTTOSQUARESDISTANCE Exact distance from segment a-b to axis-aligned squares.
%   dist = segmentToSquaresDistance(a, b, centers, halfSize) returns a K-by-1
%   vector of Euclidean distances (in the units of a and b) from the segment to
%   each square, which is 0 where the segment touches or enters the square.
%
%   For a segment that misses a convex square, the closest pair of points
%   always involves either a segment end point or a square corner, so only
%   those candidates need to be examined.

    numSquares = size(centers, 1);
    dist = inf(numSquares, 1);
    if numSquares == 0
        return;
    end

    dist = min(pointToSquaresDistance(a, centers, halfSize), ...
               pointToSquaresDistance(b, centers, halfSize));
    for sr = [-1 1]
        for sc = [-1 1]
            corners = centers + halfSize * [sr, sc];
            dist = min(dist, pointsToSegmentDistance(corners, a, b));
        end
    end
    dist(segmentIntersectsSquares(a, b, centers, halfSize, 0)) = 0;
end

function d = pointToSquaresDistance(p, centers, halfSize)
    dr = max(abs(p(1) - centers(:, 1)) - halfSize, 0);
    dc = max(abs(p(2) - centers(:, 2)) - halfSize, 0);
    d = hypot(dr, dc);
end

function d = pointsToSegmentDistance(pts, a, b)
    ab = b - a;
    len2 = sum(ab .^ 2);
    if len2 == 0
        t = zeros(size(pts, 1), 1);
    else
        t = min(max(((pts(:, 1) - a(1)) * ab(1) + (pts(:, 2) - a(2)) * ab(2)) / len2, 0), 1);
    end
    closest = a + t .* ab;
    d = hypot(pts(:, 1) - closest(:, 1), pts(:, 2) - closest(:, 2));
end
