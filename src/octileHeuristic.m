function h = octileHeuristic(cells, goal)
%OCTILEHEURISTIC Octile-distance estimate to the goal on an 8-connected grid.
%   h = octileHeuristic(cells, goal) where cells is N-by-2 [row col] and
%   goal is [row col]. With orthogonal cost 1 and diagonal cost sqrt(2),
%   this is the exact cost on an empty grid, so it never overestimates
%   (admissible) and is also consistent.

    dr = abs(cells(:, 1) - goal(1));
    dc = abs(cells(:, 2) - goal(2));
    h = (dr + dc) + (sqrt(2) - 2) * min(dr, dc);
end
