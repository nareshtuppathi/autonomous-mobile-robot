function hit = segmentIntersectsSquares(a, b, centers, halfSize, tol)
%SEGMENTINTERSECTSSQUARES Exact test: does segment a-b touch each axis-aligned square?
%   hit = segmentIntersectsSquares(a, b, centers, halfSize, tol)
%     a, b      - segment end points [row col] in grid coordinates
%     centers   - K-by-2 square centres [row col]
%     halfSize  - half the square edge length (0.5 for grid cells)
%     tol       - optional tolerance that grows each square (default 1e-9);
%                 squares are closed, so touching an edge or corner counts as a hit
%   Returns a K-by-1 logical. Uses the slab method (Liang-Barsky clipping).

    if nargin < 5
        tol = 1e-9;
    end
    numSquares = size(centers, 1);
    d = b - a;
    tEnter = zeros(numSquares, 1);
    tExit  = ones(numSquares, 1);

    for ax = 1:2
        lo = centers(:, ax) - halfSize - tol - a(ax);
        hi = centers(:, ax) + halfSize + tol - a(ax);
        if abs(d(ax)) < 1e-12
            % Segment is parallel to this slab: it hits only if it lies inside it
            outside = lo > 0 | hi < 0;
            tExit(outside) = -1;
        else
            t1 = lo / d(ax);
            t2 = hi / d(ax);
            tEnter = max(tEnter, min(t1, t2));
            tExit  = min(tExit,  max(t1, t2));
        end
    end
    hit = tEnter <= tExit;
end
