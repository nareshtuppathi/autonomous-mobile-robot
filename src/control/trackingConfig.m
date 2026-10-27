function cfg = trackingConfig(options)
%TRACKINGCONFIG Parameters for Step 3: reference trajectory, Pure Pursuit and simulation.
%   cfg = trackingConfig() returns the defaults; override with name-value pairs.
%   Units: metres, seconds, radians. Every parameter is documented below.
%
%   Robot limits (cfg.robot) - a differential-drive-compatible unicycle
%     MaxLinearSpeedMps      0.60  top forward speed (no reversing is commanded)
%     MaxAngularSpeedRadps   1.50  top yaw rate
%     MaxLinearAccelMps2     0.50  forward acceleration and deceleration limit
%     MaxAngularAccelRadps2  3.00  yaw acceleration limit
%     TrackWidthM            0.34  wheel separation (smaller than the 0.40 m footprint)
%     MaxWheelSpeedMps       0.80  per-wheel speed limit; wheel speeds are v -/+ omega*TrackWidth/2
%
%   Reference trajectory (cfg.reference)
%     MaxLateralAccelMps2    0.40  speed is limited to sqrt(this/|curvature|) in turns
%     MinTurnSpeedMps        0.15  speed floor inside turns, so the robot never crawls to a stop
%     SampleSpacingM         0.05  arc-length spacing of reference samples
%     CornerBlendM           0.80  heading of the reference is blended over this length at each
%                                  corner (limited to half of each adjacent segment)
%
%   Pure Pursuit controller (cfg.controller)
%     LookaheadMinM          0.30  lookahead = LookaheadMinM + LookaheadGainS * speed,
%     LookaheadGainS         0.80  clipped to [LookaheadMinM, LookaheadMaxM]
%     LookaheadMaxM          1.20
%     RotateThresholdRad     80 deg  if the lookahead point is further off the heading than
%                                  this, the robot turns on the spot instead of driving
%     RotateGainPerS         2.00  yaw rate per radian of heading error while turning on the spot
%     MinSpeedMps            0.05  creep speed that guarantees the goal tolerance is reached
%     GoalToleranceM         0.10  goal reached when within this distance of the final point
%     SearchAheadM           1.50  path-projection search window ahead of current progress
%
%   Simulation (cfg.sim)
%     TimeStepS              0.02  control and integration period
%     TimeoutFactor          2.0   timeout = TimeoutFactor * reference duration + TimeoutPadS
%     TimeoutPadS            10
%     MaxCrossTrackM         0.50  larger cross-track error is declared divergence
%     StallWindowS           5     failure if progress along the path over this window is
%     StallProgressM         0.05  below this distance

    arguments
        options.MaxLinearSpeedMps     (1, 1) double {mustBePositive, mustBeFinite} = 0.60
        options.MaxAngularSpeedRadps  (1, 1) double {mustBePositive, mustBeFinite} = 1.50
        options.MaxLinearAccelMps2    (1, 1) double {mustBePositive, mustBeFinite} = 0.50
        options.MaxAngularAccelRadps2 (1, 1) double {mustBePositive, mustBeFinite} = 3.00
        options.TrackWidthM           (1, 1) double {mustBePositive, mustBeFinite} = 0.34
        options.MaxWheelSpeedMps      (1, 1) double {mustBePositive, mustBeFinite} = 0.80
        options.MaxLateralAccelMps2   (1, 1) double {mustBePositive, mustBeFinite} = 0.40
        options.MinTurnSpeedMps       (1, 1) double {mustBePositive, mustBeFinite} = 0.15
        options.SampleSpacingM        (1, 1) double {mustBePositive, mustBeFinite} = 0.05
        options.CornerBlendM          (1, 1) double {mustBePositive, mustBeFinite} = 0.80
        options.LookaheadMinM         (1, 1) double {mustBePositive, mustBeFinite} = 0.30
        options.LookaheadGainS        (1, 1) double {mustBeNonnegative, mustBeFinite} = 0.80
        options.LookaheadMaxM         (1, 1) double {mustBePositive, mustBeFinite} = 1.20
        options.RotateThresholdRad    (1, 1) double {mustBePositive, mustBeFinite} = deg2rad(80)
        options.RotateGainPerS        (1, 1) double {mustBePositive, mustBeFinite} = 2.00
        options.MinSpeedMps           (1, 1) double {mustBeNonnegative, mustBeFinite} = 0.05
        options.GoalToleranceM        (1, 1) double {mustBePositive, mustBeFinite} = 0.10
        options.SearchAheadM          (1, 1) double {mustBePositive, mustBeFinite} = 1.50
        options.TimeStepS             (1, 1) double {mustBePositive, mustBeFinite} = 0.02
        options.TimeoutFactor         (1, 1) double {mustBePositive, mustBeFinite} = 2.0
        options.TimeoutPadS           (1, 1) double {mustBeNonnegative, mustBeFinite} = 10
        options.MaxCrossTrackM        (1, 1) double {mustBePositive, mustBeFinite} = 0.50
        options.StallWindowS          (1, 1) double {mustBePositive, mustBeFinite} = 5
        options.StallProgressM        (1, 1) double {mustBeNonnegative, mustBeFinite} = 0.05
    end

    cfg.robot.maxLinearSpeed  = options.MaxLinearSpeedMps;
    cfg.robot.maxAngularSpeed = options.MaxAngularSpeedRadps;
    cfg.robot.maxLinearAccel  = options.MaxLinearAccelMps2;
    cfg.robot.maxAngularAccel = options.MaxAngularAccelRadps2;
    cfg.robot.trackWidth      = options.TrackWidthM;
    cfg.robot.maxWheelSpeed   = options.MaxWheelSpeedMps;

    cfg.reference.maxLateralAccel = options.MaxLateralAccelMps2;
    cfg.reference.minTurnSpeed    = options.MinTurnSpeedMps;
    cfg.reference.sampleSpacing   = options.SampleSpacingM;
    cfg.reference.cornerBlend     = options.CornerBlendM;

    cfg.controller.lookaheadMin    = options.LookaheadMinM;
    cfg.controller.lookaheadGain   = options.LookaheadGainS;
    cfg.controller.lookaheadMax    = options.LookaheadMaxM;
    cfg.controller.rotateThreshold = options.RotateThresholdRad;
    cfg.controller.rotateGain      = options.RotateGainPerS;
    cfg.controller.minSpeed        = options.MinSpeedMps;
    cfg.controller.goalTolerance   = options.GoalToleranceM;
    cfg.controller.searchAhead     = options.SearchAheadM;

    cfg.sim.timeStep      = options.TimeStepS;
    cfg.sim.timeoutFactor = options.TimeoutFactor;
    cfg.sim.timeoutPad    = options.TimeoutPadS;
    cfg.sim.maxCrossTrack = options.MaxCrossTrackM;
    cfg.sim.stallWindow   = options.StallWindowS;
    cfg.sim.stallProgress = options.StallProgressM;

    % Cross-parameter consistency
    problems = {};
    if cfg.controller.lookaheadMin > cfg.controller.lookaheadMax
        problems{end + 1} = 'LookaheadMinM must not exceed LookaheadMaxM.';
    end
    if cfg.controller.minSpeed > cfg.robot.maxLinearSpeed
        problems{end + 1} = 'MinSpeedMps must not exceed MaxLinearSpeedMps.';
    end
    if cfg.reference.minTurnSpeed > cfg.robot.maxLinearSpeed
        problems{end + 1} = 'MinTurnSpeedMps must not exceed MaxLinearSpeedMps.';
    end
    if cfg.robot.maxWheelSpeed < cfg.robot.maxLinearSpeed
        problems{end + 1} = 'MaxWheelSpeedMps must be at least MaxLinearSpeedMps.';
    end
    if cfg.controller.rotateThreshold >= pi
        problems{end + 1} = 'RotateThresholdRad must be below pi.';
    end
    if ~isempty(problems)
        error('amrStep3:invalidConfig', 'Invalid tracking configuration: %s', strjoin(problems, ' '));
    end
end
