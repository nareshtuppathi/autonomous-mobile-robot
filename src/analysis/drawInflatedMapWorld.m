function [ax, legendHandles, legendLabels] = drawInflatedMapWorld(ax, rawGrid, inflatedGrid, cfg)
%DRAWINFLATEDMAPWORLD Draw real and inflated obstacles on axes scaled in metres.
%   Free space is white, the inflated margin is light orange and real obstacle
%   cells are dark grey. The origin is the bottom-left corner of the map.
    [numRows, numCols] = size(rawGrid);
    width  = numCols * cfg.cellSizeM;
    height = numRows * cfg.cellSizeM;

    colObstacle = [0.20 0.20 0.22];
    colInflated = [0.99 0.82 0.65];
    img = ones(numRows, numCols, 3);
    for ch = 1:3
        layer = ones(numRows, numCols);
        layer(inflatedGrid) = colInflated(ch);
        layer(rawGrid) = colObstacle(ch);
        img(:, :, ch) = layer;
    end

    half = cfg.cellSizeM / 2;
    image(ax, [half, width - half], [height - half, half], img);
    set(ax, 'YDir', 'normal');
    hold(ax, 'on');
    axis(ax, 'image');
    xlim(ax, [0 width]);  ylim(ax, [0 height]);
    xlabel(ax, 'x (m)');  ylabel(ax, 'y (m)');
    box(ax, 'on');

    hObs = patch(ax, NaN, NaN, colObstacle, 'EdgeColor', 'none');
    hInf = patch(ax, NaN, NaN, colInflated, 'EdgeColor', [0.8 0.6 0.4]);
    legendHandles = [hObs, hInf];
    legendLabels  = {'Obstacle', 'Inflated margin'};
end
