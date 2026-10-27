function plotInflationFigure(rawGrid, inflatedGrid, cfg, startCell, goalCell, savePath)
%PLOTINFLATIONFIGURE Show the real obstacles, the inflated region, start and goal.
    fig = figure('Color', 'w', 'Visible', 'off');
    try theme(fig, 'light'); catch, end   % keep exported figures light even if MATLAB runs in dark mode
    fig.Position(3:4) = [1000 700];
    ax = axes(fig);
    [ax, handles, labels] = drawInflatedMapWorld(ax, rawGrid, inflatedGrid, cfg);
    gs = size(rawGrid);
    s = gridCoordsToWorld(startCell, gs, cfg.cellSizeM);
    g = gridCoordsToWorld(goalCell, gs, cfg.cellSizeM);
    hS = plot(ax, s(1), s(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.10 0.65 0.25], 'MarkerEdgeColor', 'k');
    hG = plot(ax, g(1), g(2), 'p', 'MarkerSize', 16, 'MarkerFaceColor', [0.95 0.75 0.05], 'MarkerEdgeColor', 'k');
    legend(ax, [handles, hS, hG], [labels, {'Start', 'Goal'}], 'Location', 'southoutside', 'Orientation', 'horizontal');
    title(ax, sprintf('Configuration space: %.2f m/cell, robot radius %.2f m + margin %.2f m = %d-cell inflation (%.2f m)', ...
        cfg.cellSizeM, cfg.robotRadiusM, cfg.safetyMarginM, cfg.inflationCells, cfg.effectiveInflationM));
    exportgraphics(fig, savePath, 'Resolution', 200);
    close(fig);
end
