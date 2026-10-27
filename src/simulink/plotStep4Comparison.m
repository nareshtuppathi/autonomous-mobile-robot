function plotStep4Comparison(sc, simM, simS, cmp, figDir)
%PLOTSTEP4COMPARISON Figures comparing the MATLAB and Simulink runs of Scenario A.
%   Writes step4_01_trajectory_comparison.png, step4_02_tracking_signals.png and
%   step4_03_max_differences.png to figDir.
    gs = size(sc.grid);
    cs = sc.mapCfg.cellSizeM;
    toCells = @(x, y) worldToGridCoords([x(:), y(:)], gs, cs);
    blue = [0.10 0.35 0.80];  red = [0.85 0.15 0.15];  gray = [0.45 0.45 0.45];
    mk = @(name, cells, color, ls, lw) struct('name', name, 'cells', cells, 'color', color, 'lineStyle', ls, 'lineWidth', lw, 'marker', 'none');
    paths = [mk('Reference path', toCells(sc.ref.x, sc.ref.y), gray, '--', 1.5), ...
             mk('MATLAB simulation', toCells(simM.log.x, simM.log.y), blue, '-', 3.0), ...
             mk('Simulink simulation', toCells(simS.log.x, simS.log.y), red, '-', 1.3)];
    plotWorldPaths(sc.grid, sc.inflatedGrid, sc.mapCfg, sc.startCell, sc.goalCell, paths, ...
        'Scenario A: MATLAB and Simulink trajectories (they overlap)', fullfile(figDir, 'step4_01_trajectory_comparison.png'));

    fig = figure('Color', 'w', 'Visible', 'off');
    try theme(fig, 'light'); catch, end   % keep exported figures light even if MATLAB runs in dark mode
    fig.Position(3:4) = [1100 800];
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    panels = {'crossTrack', 'Cross-track error (m)', 1; 'headingError', 'Heading error (deg)', 180 / pi; ...
              'v', 'Linear velocity (m/s)', 1; 'omega', 'Angular velocity (rad/s)', 1};
    for k = 1:size(panels, 1)
        ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
        f = panels{k, 1};  sc_ = panels{k, 3};
        plot(ax, simM.log.t, sc_ * simM.log.(f), 'Color', blue, 'LineWidth', 3.0);
        plot(ax, simS.log.t, sc_ * simS.log.(f), 'Color', red, 'LineWidth', 1.2);
        xlabel(ax, 'Time (s)');  ylabel(ax, panels{k, 2});
        if k == 1, legend(ax, {'MATLAB', 'Simulink'}, 'Location', 'best'); end
    end
    title(tl, 'Scenario A: MATLAB and Simulink signals');
    exportgraphics(fig, fullfile(figDir, 'step4_02_tracking_signals.png'), 'Resolution', 200);
    close(fig);

    fig = figure('Color', 'w', 'Visible', 'off');
    try theme(fig, 'light'); catch, end   % keep exported figures light even if MATLAB runs in dark mode
    fig.Position(3:4) = [1000 560];
    ax = axes(fig);  hold(ax, 'on');
    floorValue = 1e-17;                                   % exact zeros are drawn at this floor
    d = max(cmp.table.MaxAbsDiff, floorValue);
    bar(ax, categorical(cmp.table.Field, cmp.table.Field), d, 'FaceColor', blue);
    set(ax, 'YScale', 'log');
    hTol = plot(ax, [0.5, height(cmp.table) + 0.5], [step4Tolerances().lengthM, step4Tolerances().lengthM], '--r', 'LineWidth', 1.5);
    ylim(ax, [floorValue / 10, 1e-6]);
    ylabel(ax, 'Max |MATLAB - Simulink| over all samples');
    title(ax, sprintf('Largest sample-wise difference per signal (bars at the %.0e floor are exact zeros)', floorValue));
    legend(ax, hTol, {'Tolerance (1e-9 for lengths, angles, velocities)'}, 'Location', 'northeast');
    grid(ax, 'on');
    exportgraphics(fig, fullfile(figDir, 'step4_03_max_differences.png'), 'Resolution', 200);
    close(fig);
end
