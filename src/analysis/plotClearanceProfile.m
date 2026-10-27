function plotClearanceProfile(rawGrid, cfg, paths, savePath)
%PLOTCLEARANCEPROFILE Distance to the nearest obstacle along each path.
%   paths is a struct array with fields name, cells, color, lineStyle (see plotWorldPaths).
%   Horizontal lines mark the robot radius (collision limit) and the robot
%   radius plus safety margin (required clearance).
    fig = figure('Color', 'w', 'Visible', 'off');
    try theme(fig, 'light'); catch, end   % keep exported figures light even if MATLAB runs in dark mode
    fig.Position(3:4) = [1000 520];
    ax = axes(fig);
    hold(ax, 'on');  grid(ax, 'on');
    handles = gobjects(0);  labels = {};
    for k = 1:numel(paths)
        [d, c] = pathClearanceProfile(rawGrid, paths(k).cells, cfg);
        handles(end + 1) = plot(ax, d, c, 'LineStyle', paths(k).lineStyle, 'Color', paths(k).color, 'LineWidth', 1.8); %#ok<AGROW>
        labels{end + 1} = sprintf('%s (min %.3f m)', paths(k).name, min(c)); %#ok<AGROW>
    end
    hR = yline(ax, cfg.robotRadiusM, '--r', 'LineWidth', 1.5);
    hT = yline(ax, cfg.inflationRadiusM, ':r', 'LineWidth', 1.5);
    ylim(ax, [0, max(ylim(ax))]);
    xlabel(ax, 'Distance travelled along path (m)');
    ylabel(ax, 'Clearance to nearest obstacle (m)');
    title(ax, 'Clearance along each path');
    legend(ax, [handles, hR, hT], [labels, {'Robot radius (collision limit)', 'Radius + safety margin (required)'}], ...
        'Location', 'southoutside', 'NumColumns', 2);
    exportgraphics(fig, savePath, 'Resolution', 200);
    close(fig);
end
