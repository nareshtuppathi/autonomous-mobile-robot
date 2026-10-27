function plotTrackingResults(rawGrid, inflatedGrid, mapCfg, startCell, goalCell, ref, sim, tcfg, savePrefix, fullSet)
%PLOTTRACKINGRESULTS Save the Step 3 figures for one simulation run.
%   plotTrackingResults(rawGrid, inflatedGrid, mapCfg, startCell, goalCell, ref, sim, tcfg, savePrefix, fullSet)
%   savePrefix is a path prefix such as <figDir>/step3_. With fullSet = true it writes
%     <prefix>01_trajectory.png, 02_cross_track_error.png, 03_heading_error.png,
%     04_velocities.png, 05_clearance.png
%   With fullSet = false it writes only <prefix>trajectory.png and <prefix>heading_error.png.
    L = sim.log;
    gs = size(rawGrid);
    blue = [0.10 0.35 0.80];  red = [0.85 0.15 0.15];  gray = [0.45 0.45 0.45];

    % Trajectory over the map (world metres)
    refCells = worldToGridCoords([ref.x, ref.y], gs, mapCfg.cellSizeM);
    actCells = worldToGridCoords([L.x, L.y], gs, mapCfg.cellSizeM);
    mk = @(name, cells, color, ls, lw) struct('name', name, 'cells', cells, 'color', color, 'lineStyle', ls, 'lineWidth', lw, 'marker', 'none');
    paths = [mk('Reference path (Step 2 smoothed path)', refCells, blue, '--', 2.0), ...
             mk(sprintf('Simulated robot trajectory (%s, %.1f s)', strrep(sim.terminationReason, '_', ' '), L.t(end)), actCells, red, '-', 1.8)];
    if fullSet
        trajFile = [savePrefix '01_trajectory.png'];
    else
        trajFile = [savePrefix 'trajectory.png'];
    end
    if fullSet
        ttl = 'Pure Pursuit tracking of the smoothed path (kinematic simulation)';
    else
        ttl = sprintf('Recovery from an initial heading error of %.0f deg', rad2deg(wrapAngle(sim.startPose(3) - ref.theta(1))));
    end
    plotWorldPaths(rawGrid, inflatedGrid, mapCfg, startCell, goalCell, paths, ttl, trajFile);

    % Heading error
    if fullSet
        hdgFile = [savePrefix '03_heading_error.png'];
    else
        hdgFile = [savePrefix 'heading_error.png'];
    end
    [fig, ax] = newFigure(1000, 450);
    plot(ax, L.t, rad2deg(L.headingError), 'Color', red, 'LineWidth', 1.6);
    yline(ax, 0, ':', 'Color', gray);
    labelAxes(ax, 'Time (s)', 'Heading error (deg)', 'Heading error: wrap(reference heading - robot heading), in (-180, 180] deg');
    exportgraphics(fig, hdgFile, 'Resolution', 200);  close(fig);
    if ~fullSet
        return;
    end

    % Cross-track error
    [fig, ax] = newFigure(1000, 450);
    plot(ax, L.t, L.crossTrack, 'Color', red, 'LineWidth', 1.6);
    yline(ax, 0, ':', 'Color', gray);
    labelAxes(ax, 'Time (s)', 'Cross-track error (m)', 'Cross-track error (signed, positive = left of the path)');
    exportgraphics(fig, [savePrefix '02_cross_track_error.png'], 'Resolution', 200);  close(fig);

    % Velocities
    [fig, ~] = newFigure(1000, 700);
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact');
    ax1 = nexttile(tl);  hold(ax1, 'on');
    plot(ax1, L.t, L.vRef, '--', 'Color', blue, 'LineWidth', 1.4);
    plot(ax1, L.t, L.v, '-', 'Color', red, 'LineWidth', 1.6);
    yline(ax1, tcfg.robot.maxLinearSpeed, ':k');
    labelAxes(ax1, '', 'Linear velocity (m/s)', 'Linear velocity');
    legend(ax1, {'Reference speed at current progress', 'Applied', 'Limit'}, 'Location', 'best');
    ax2 = nexttile(tl);  hold(ax2, 'on');
    plot(ax2, L.t, L.omegaRef, '--', 'Color', blue, 'LineWidth', 1.4);
    plot(ax2, L.t, L.omega, '-', 'Color', red, 'LineWidth', 1.6);
    yline(ax2, tcfg.robot.maxAngularSpeed, ':k');  yline(ax2, -tcfg.robot.maxAngularSpeed, ':k');
    labelAxes(ax2, 'Time (s)', 'Angular velocity (rad/s)', 'Angular velocity');
    legend(ax2, {'Reference yaw rate at current progress', 'Applied', 'Limit'}, 'Location', 'best');
    exportgraphics(fig, [savePrefix '04_velocities.png'], 'Resolution', 200);  close(fig);

    % Clearance
    [fig, ax] = newFigure(1000, 450);  hold(ax, 'on');
    hC = plot(ax, L.t, L.clearanceM, 'Color', red, 'LineWidth', 1.6);
    hR = yline(ax, mapCfg.robotRadiusM, '--k', 'LineWidth', 1.3);
    hT = yline(ax, mapCfg.inflationRadiusM, ':k', 'LineWidth', 1.5);
    ylim(ax, [0, max(L.clearanceM) * 1.05]);
    labelAxes(ax, 'Time (s)', 'Clearance to nearest obstacle (m)', 'Clearance of the simulated trajectory to the original occupied map');
    legend(ax, [hC, hR, hT], {sprintf('Robot centre (min %.3f m)', min(L.clearanceM)), 'Robot radius (collision limit)', 'Radius + safety margin (required)'}, 'Location', 'best');
    exportgraphics(fig, [savePrefix '05_clearance.png'], 'Resolution', 200);  close(fig);
end

function [fig, ax] = newFigure(w, h)
    fig = figure('Color', 'w', 'Visible', 'off');
    try theme(fig, 'light'); catch, end
    fig.Position(3:4) = [w h];
    ax = axes(fig);
    hold(ax, 'on');
end

function labelAxes(ax, xl, yl, ttl)
    grid(ax, 'on');  box(ax, 'on');
    if ~isempty(xl), xlabel(ax, xl); end
    ylabel(ax, yl);  title(ax, ttl);
end
