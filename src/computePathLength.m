function len = computePathLength(path)
%COMPUTEPATHLENGTH Euclidean length of a path, in grid-cell units.
%   len = computePathLength(path) where path is an N-by-2 matrix of
%   [row col] cells. Orthogonal steps count 1, diagonal steps count sqrt(2).
%   A path with fewer than two cells has length 0.

    if size(path, 1) < 2
        len = 0;
        return;
    end
    len = sum(hypot(diff(path(:, 1)), diff(path(:, 2))));
end
