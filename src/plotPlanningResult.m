function fig = plotPlanningResult(grid, start, goal, result, titleText, savePath)
%PLOTPLANNINGRESULT Draw the occupancy grid, explored cells and planned path.
%   fig = plotPlanningResult(grid, start, goal, result, titleText, savePath)
%   result may be [] to draw just the map. If savePath is given, the figure
%   is exported as a PNG (200 dpi) and closed.

    if nargin < 5, titleText = ''; end
    if nargin < 6, savePath = ''; end

    [numRows, numCols] = size(grid);

    colFree     = [1.00 1.00 1.00];
    colObstacle = [0.20 0.20 0.22];
    colExplored = [0.74 0.86 0.96];

    % Build an RGB image: free = white, explored = light blue, obstacle = dark
    img = ones(numRows, numCols, 3);
    for ch = 1:3
        layer = colFree(ch) * ones(numRows, numCols);
        if ~isempty(result) && ~isempty(result.expandedCells)
            exploredIdx = sub2ind([numRows, numCols], ...
                result.expandedCells(:, 1), result.expandedCells(:, 2));
            layer(exploredIdx) = colExplored(ch);
        end
        layer(grid) = colObstacle(ch);
        img(:, :, ch) = layer;
    end

    if isempty(savePath)
        fig = figure('Color', 'w');
    else
        fig = figure('Color', 'w', 'Visible', 'off');
    end
    try theme(fig, 'light'); catch, end   % keep exported figures light even if MATLAB runs in dark mode
    fig.Position(3:4) = [900 640];
    ax = axes(fig);
    image(ax, img);
    hold(ax, 'on');
    axis(ax, 'image');

    % Light cell grid lines
    set(ax, 'XTick', 0.5:10:numCols + 0.5, 'YTick', 0.5:10:numRows + 0.5, ...
        'XTickLabel', 0:10:numCols, 'YTickLabel', 0:10:numRows, ...
        'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.15, 'Layer', 'top');
    xlabel(ax, 'Column (x)');
    ylabel(ax, 'Row (y)');

    % Legend proxies, plotted off-screen so they carry the correct style
    hObs = patch(ax, NaN, NaN, colObstacle, 'EdgeColor', 'none');
    hExp = patch(ax, NaN, NaN, colExplored, 'EdgeColor', [0.5 0.6 0.7]);
    entries = [hObs, hExp];
    labels  = {'Obstacle', 'Explored cells'};

    if ~isempty(result) && result.found
        hPath = plot(ax, result.path(:, 2), result.path(:, 1), '-', ...
            'Color', [0.85 0.15 0.15], 'LineWidth', 2.5);
        entries(end + 1) = hPath;
        labels{end + 1} = 'Planned path';
    end
    hStart = plot(ax, start(2), start(1), 'o', 'MarkerSize', 11, ...
        'MarkerFaceColor', [0.10 0.65 0.25], 'MarkerEdgeColor', 'k');
    hGoal = plot(ax, goal(2), goal(1), 'p', 'MarkerSize', 16, ...
        'MarkerFaceColor', [0.95 0.75 0.05], 'MarkerEdgeColor', 'k');
    entries = [entries, hStart, hGoal];
    labels  = [labels, {'Start', 'Goal'}];

    legend(ax, entries, labels, 'Location', 'southoutside', 'Orientation', 'horizontal');
    title(ax, titleText, 'Interpreter', 'none');

    if ~isempty(savePath)
        exportgraphics(fig, savePath, 'Resolution', 200);
        close(fig);
    end
end
