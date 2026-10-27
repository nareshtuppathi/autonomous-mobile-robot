function path = reconstructPath(parent, goalIdx)
%RECONSTRUCTPATH Walk parent pointers back from the goal to the start.
%   path = reconstructPath(parent, goalIdx) returns an N-by-2 matrix of
%   [row col] cells ordered from start to goal. parent is a matrix the size
%   of the grid holding each cell's predecessor (0 for the start cell).

    idxList = goalIdx;
    while parent(idxList(end)) ~= 0
        idxList(end + 1) = parent(idxList(end)); %#ok<AGROW>
    end
    idxList = flip(idxList);
    [r, c] = ind2sub(size(parent), idxList);
    path = [r(:), c(:)];
end
