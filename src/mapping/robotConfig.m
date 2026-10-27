function cfg = robotConfig(options)
%ROBOTCONFIG Physical parameters for Step 2: map resolution and robot footprint.
%   cfg = robotConfig() returns the defaults. Override with name-value pairs:
%     'CellSizeM'     - edge length of one grid cell in metres per cell
%                       (default 0.25 m/cell, so the 40x60 grid is a 10 m x 15 m arena)
%     'RobotRadiusM'  - radius of the circular robot footprint in metres (default 0.20)
%     'SafetyMarginM' - extra clearance added to the footprint in metres (default 0.05)
%
%   Output fields
%     cellSizeM          - metres per cell (the input)
%     resolution         - cells per metre = 1/cellSizeM. This is the value that
%                          binaryOccupancyMap expects as its Resolution argument.
%     robotRadiusM       - footprint radius (m)
%     safetyMarginM      - safety margin (m)
%     inflationRadiusM   - robotRadiusM + safetyMarginM, the requested inflation (m)
%     inflationCells     - whole cells actually inflated: ceil(inflationRadiusM/cellSizeM).
%                          binaryOccupancyMap/inflate rounds up to whole cells.
%     effectiveInflationM- inflationCells * cellSizeM (>= inflationRadiusM)
%
%   Why 0.25 m/cell: the Step 1 map has 4-cell-wide passages. With 0.25 m cells
%   those are 1.0 m wide, comfortably wider than a 0.40 m diameter robot plus margin.

    arguments
        options.CellSizeM     (1, 1) double {mustBePositive, mustBeFinite} = 0.25
        options.RobotRadiusM  (1, 1) double {mustBePositive, mustBeFinite} = 0.20
        options.SafetyMarginM (1, 1) double {mustBeNonnegative, mustBeFinite} = 0.05
    end

    cfg.cellSizeM           = options.CellSizeM;
    cfg.resolution          = 1 / options.CellSizeM;
    cfg.robotRadiusM        = options.RobotRadiusM;
    cfg.safetyMarginM       = options.SafetyMarginM;
    cfg.inflationRadiusM    = options.RobotRadiusM + options.SafetyMarginM;
    % round() guards against floating-point noise such as 0.25/0.25 = 1.0000000000000002
    cfg.inflationCells      = ceil(round(cfg.inflationRadiusM / cfg.cellSizeM, 9));
    cfg.effectiveInflationM = cfg.inflationCells * cfg.cellSizeM;
end
