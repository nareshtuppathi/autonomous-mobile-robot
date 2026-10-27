function plotWorldPaths(rawGrid, inflatedGrid, cfg, startCell, goalCell, paths, titleText, savePath)
%PLOTWORLDPATHS Overlay several paths on the inflated map, in metres.
%   paths is a struct array with fields
%     name      - legend text
%     cells     - N-by-2 [row col] waypoints
%     color     - 1-by-3 RGB
%     lineStyle - e.g. '-' or '--'
%     lineWidth - number
%     marker    - marker for waypoints ('none' for no markers)
    fig = figure('Color', 'w', 'Visible', 'off');
    try theme(fig, 'light'); catch, end   % keep exported figures light even if MATLAB runs in dark mode
    fig.Position(3:4) = [1000 700];
    ax = axes(fig);
    [ax, handles, labels] = drawInflatedMapWorld(ax, rawGrid, inflatedGrid, cfg);
    gs = size(rawGrid);

    for k = 1:numel(paths)
        xy = gridCoordsToWorld(paths(k).cells, gs, cfg.cellSizeM);
        h = plot(ax, xy(:, 1), xy(:, 2), 'LineStyle', paths(k).lineStyle, 'Color', paths(k).color, ...
            'LineWidth', paths(k).lineWidth, 'Marker', paths(k).marker, 'MarkerSize', 6, ...
            'MarkerFaceColor', paths(k).color);
        handles(end + 1) = h; %#ok<AGROW>
        labels{end + 1} = paths(k).name; %#ok<AGROW>
    end

    s = gridCoordsToWorld(startCell, gs, cfg.cellSizeM);
    g = gridCoordsToWorld(goalCell, gs, cfg.cellSizeM);
    hS = plot(ax, s(1), s(2), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.10 0.65 0.25], 'MarkerEdgeColor', 'k');
    hG = plot(ax, g(1), g(2), 'p', 'MarkerSize', 16, 'MarkerFaceColor', [0.95 0.75 0.05], 'MarkerEdgeColor', 'k');
    legend(ax, [handles, hS, hG], [labels, {'Start', 'Goal'}], 'Location', 'southoutside', 'NumColumns', 3);
    title(ax, titleText, 'Interpreter', 'none');
    exportgraphics(fig, savePath, 'Resolution', 200);
    close(fig);
end
