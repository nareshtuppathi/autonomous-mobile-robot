function plotHeadingErrorInvestigation(sim, tcfg, res, titleText, savePath)
%PLOTHEADINGERRORINVESTIGATION Three-panel figure explaining the jumps in the heading error.
%   Top: logged heading error, the counterfactual with rate-limited progress, and the events.
%   Middle: progress rate (d sProj / dt) against the robot speed, on a log axis.
%   Bottom: reference heading at the projected point and robot heading.
    L = sim.log;
    t = L.t(:);
    dt = tcfg.sim.timeStep;
    fig = figure('Color', 'w', 'Visible', 'off');
    try theme(fig, 'light'); catch, end   % keep exported figures light even if MATLAB runs in dark mode
    fig.Position(3:4) = [1000 820];
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ev = res.events.Step;

    ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
    h1 = plot(ax, t, rad2deg(L.headingError), 'Color', [0.10 0.35 0.80], 'LineWidth', 1.6);
    h2 = plot(ax, t, rad2deg(res.counterfactualError), '--', 'Color', [0.95 0.55 0.05], 'LineWidth', 1.4);
    h3 = plot(ax, t(ev + 1), rad2deg(L.headingError(ev + 1)), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
    ylabel(ax, 'Heading error (deg)');
    title(ax, titleText, 'Interpreter', 'none');
    legend(ax, [h1 h2 h3], {'Logged heading error', 'Same, with progress limited to top speed', 'Abrupt step (projection jump)'}, 'Location', 'best');

    ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
    rate = diff(L.sProj(:)) / dt;
    semilogy(ax, t(1:end - 1), max(rate, 1e-3), 'Color', [0.10 0.35 0.80], 'LineWidth', 1.2);
    semilogy(ax, t, max(abs(L.v(:)), 1e-3), 'Color', [0.20 0.65 0.30], 'LineWidth', 1.6);
    plot(ax, t(ev), max(rate(ev), 1e-3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
    set(ax, 'YScale', 'log');
    ylabel(ax, 'Rate (m/s)');
    legend(ax, {'Progress rate d(sProj)/dt', 'Robot speed v', 'Events'}, 'Location', 'best');

    ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
    plot(ax, t, rad2deg(L.thetaRef), 'Color', [0.55 0.20 0.70], 'LineWidth', 1.4);
    plot(ax, t, rad2deg(L.theta), 'Color', [0.85 0.15 0.15], 'LineWidth', 1.4);
    xlabel(ax, 'Time (s)');  ylabel(ax, 'Heading (deg)');
    legend(ax, {'Reference heading at projected point', 'Robot heading'}, 'Location', 'best');

    exportgraphics(fig, savePath, 'Resolution', 200);
    close(fig);
end
