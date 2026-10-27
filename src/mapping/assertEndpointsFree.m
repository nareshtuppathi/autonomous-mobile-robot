function assertEndpointsFree(rawGrid, inflatedGrid, startCell, goalCell, cfg)
%ASSERTENDPOINTSFREE Error clearly if start or goal is unusable after inflation.
%   The start and goal are never moved silently. If either lies outside the
%   map, inside a real obstacle, or inside the inflated (configuration-space)
%   obstacle region, this function throws an error that names the cell, its
%   world position and the inflation used.
%
%   Error identifiers
%     amrStep2:endpointOutOfBounds
%     amrStep2:endpointInObstacle
%     amrStep2:endpointInInflatedRegion

    [numRows, numCols] = size(rawGrid);
    names  = {'Start', 'Goal'};
    points = {startCell, goalCell};

    for k = 1:2
        p = points{k};
        if any(p ~= round(p)) || p(1) < 1 || p(1) > numRows || p(2) < 1 || p(2) > numCols
            error('amrStep2:endpointOutOfBounds', ...
                '%s cell [%g %g] is outside the %dx%d map.', ...
                names{k}, p(1), p(2), numRows, numCols);
        end
        xy = gridCoordsToWorld(p, [numRows numCols], cfg.cellSizeM);
        if rawGrid(p(1), p(2))
            error('amrStep2:endpointInObstacle', ...
                '%s cell [%d %d] (x = %.3f m, y = %.3f m) is inside an obstacle.', ...
                names{k}, p(1), p(2), xy(1), xy(2));
        end
        if inflatedGrid(p(1), p(2))
            error('amrStep2:endpointInInflatedRegion', ...
                ['%s cell [%d %d] (x = %.3f m, y = %.3f m) is too close to an obstacle ' ...
                 'for the robot: it lies inside the inflated region (radius %.3f m + margin %.3f m, ' ...
                 'rounded up to %.3f m). Move it, or reduce the radius or margin.'], ...
                names{k}, p(1), p(2), xy(1), xy(2), ...
                cfg.robotRadiusM, cfg.safetyMarginM, cfg.effectiveInflationM);
        end
    end
end
