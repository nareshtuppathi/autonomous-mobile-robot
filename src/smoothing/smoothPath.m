function smooth = smoothPath(pathCells, inflatedGrid, rawGrid, cfg)
%SMOOTHPATH Greedy line-of-sight shortcutting that preserves obstacle clearance.
%   smooth = smoothPath(pathCells, inflatedGrid, rawGrid, cfg)
%
%   Starting at the first waypoint, jump to the farthest later waypoint that
%   can be reached by a straight segment, then repeat from there. A shortcut
%   is accepted only if BOTH tests pass:
%     1. the segment crosses only free cells of the inflated grid (exact test)
%     2. the segment stays at least robotRadius + safetyMargin from every real
%        obstacle cell (exact distance, so nothing is missed between samples)
%   Waypoints are a subset of the input path, so the first and last points are
%   unchanged. Consecutive input cells are always kept if no shortcut passes.
%
%   Output fields
%     pathCells        - M-by-2 [row col] waypoints (M <= N)
%     keptIndices      - indices into the input path
%     numSegmentChecks - number of candidate segments tested
%     lengthCells      - Euclidean length in cells

    arguments
        pathCells (:, 2) double
        inflatedGrid logical
        rawGrid logical
        cfg (1, 1) struct
    end

    numPts = size(pathCells, 1);
    if numPts <= 2
        % Nothing to shortcut
        smooth.keptIndices      = 1:numPts;
        smooth.pathCells        = pathCells;
        smooth.numSegmentChecks = 0;
        smooth.lengthCells      = computePathLength(pathCells);
        return;
    end
    obstacleCenters = findOccupiedCenters(rawGrid);
    minClearanceCells = cfg.inflationRadiusM / cfg.cellSizeM - 1e-9;

    kept = 1;
    numChecks = 0;
    anchor = 1;
    while anchor < numPts
        next = anchor + 1;   % fall back to the original next waypoint
        for far = numPts:-1:anchor + 2
            numChecks = numChecks + 1;
            a = pathCells(anchor, :);
            b = pathCells(far, :);
            if isSegmentCellFree(inflatedGrid, a, b) && ...
                    segmentClearanceCells(obstacleCenters, a, b) >= minClearanceCells
                next = far;
                break;
            end
        end
        kept(end + 1) = next; %#ok<AGROW>
        anchor = next;
    end

    smooth.keptIndices      = kept;
    smooth.pathCells        = pathCells(kept, :);
    smooth.numSegmentChecks = numChecks;
    smooth.lengthCells      = computePathLength(smooth.pathCells);
end
