function m = computeTrackingMetrics(sim, ref, env, goalXY, cfg)
%COMPUTETRACKINGMETRICS Performance metrics of a simulated tracking run.
%   m = computeTrackingMetrics(sim, ref, env, goalXY, cfg)
%     goalXY - [x y] world position of the goal (m)
%
%   Definitions
%     goalReached          the simulation ended with reason goal_reached
%     durationS            time of the last logged sample (s)
%     distanceTravelledM   sum of the straight-line distances between consecutive logged
%                          poses (m); chord vs arc error is below 1e-4 m at the 0.02 s step
%     finalPositionErrorM  distance from the last logged position to goalXY (m)
%     maxCrossTrackM       max |cross-track error|; cross-track error is the signed
%                          perpendicular distance from the robot centre to the reference
%                          polyline (nearest point, positive = left of the path direction)
%     rmsCrossTrackM       sqrt(mean(cross-track error^2)) over all logged samples
%     maxHeadingErrorRad   max |wrapAngle(reference heading at the nearest path point -
%                          robot heading)|; the error is wrapped to (-pi, pi]
%     maxHeadingErrorAfterAlignRad  max heading error from the first sample at which it is
%                          within 30 deg (excludes a large initial heading error)
%     timeToAlignS         time of that first sample (s)
%     maxLinearSpeedMps, maxAngularSpeedRadps   max |v|, max |omega| actually applied
%     finalSpeedMps        |v| at the last logged sample (0 means the robot came to rest)
%     maxLinearAccelMps2, maxAngularAccelRadps2   max |change of applied v or omega between
%                          consecutive samples| / time step, over ALL samples including the last
%     maxWheelSpeedMps     max of |v -/+ omega*trackWidth/2|
%     minClearanceM        minimum, over all logged poses, of the exact distance from the
%                          robot centre to the nearest occupied cell of the ORIGINAL map
%     collision            minClearanceM < robot radius
%     clearanceViolation   minClearanceM < robot radius + safety margin (Step 2 requirement)
%     numViolatingSamples  number of logged poses below the required clearance
%     allFinite            no NaN/Inf in the logged pose, velocity and error data and no NaN in the
%                          clearance data (clearance is Inf only if the map has no obstacles)
%     overallPass          goalReached and no collision and no clearance violation and allFinite
    L = sim.log;
    xy = [L.x, L.y];
    [clr, inMap] = clearanceAtPoints(env, xy);

    m.terminationReason  = sim.terminationReason;
    m.goalReached        = sim.goalReached;
    m.durationS          = L.t(end);
    m.referenceDurationS = ref.duration;
    m.distanceTravelledM = sum(hypot(diff(L.x), diff(L.y)));
    m.referenceLengthM   = ref.length;
    m.finalPositionErrorM = hypot(L.x(end) - goalXY(1), L.y(end) - goalXY(2));
    m.maxCrossTrackM     = max(abs(L.crossTrack));
    m.rmsCrossTrackM     = sqrt(mean(L.crossTrack .^ 2));
    m.maxHeadingErrorRad = max(abs(L.headingError));
    alignIdx = find(abs(L.headingError) <= deg2rad(30), 1, 'first');
    if isempty(alignIdx)
        m.maxHeadingErrorAfterAlignRad = NaN;
        m.timeToAlignS = NaN;
    else
        m.maxHeadingErrorAfterAlignRad = max(abs(L.headingError(alignIdx:end)));
        m.timeToAlignS = L.t(alignIdx);
    end
    dtStep = cfg.sim.timeStep;
    m.finalSpeedMps = abs(L.v(end));
    m.maxLinearAccelMps2 = max(abs(diff([0; L.v]))) / dtStep;
    m.maxAngularAccelRadps2 = max(abs(diff([0; L.omega]))) / dtStep;
    m.maxLinearSpeedMps  = max(abs(L.v));
    m.maxAngularSpeedRadps = max(abs(L.omega));
    m.maxWheelSpeedMps   = max(abs(L.v) + abs(L.omega) * cfg.robot.trackWidth / 2);
    m.minClearanceM      = min(clr);
    m.collision          = m.minClearanceM < env.robotRadiusM || any(~inMap);
    m.clearanceViolation = m.minClearanceM < env.requiredClearanceM - 1e-9 || any(~inMap);
    m.numViolatingSamples = nnz(clr < env.requiredClearanceM - 1e-9);
    fields = {'t', 'x', 'y', 'theta', 'v', 'omega', 'crossTrack', 'headingError'};
    m.allFinite = true;
    for k = 1:numel(fields)
        m.allFinite = m.allFinite && all(isfinite(L.(fields{k})));
    end
    % Clearance is Inf when the map has no obstacles, which is valid; only NaN is an error
    m.allFinite = m.allFinite && ~any(isnan(L.clearanceM));
    m.overallPass = m.goalReached && ~m.collision && ~m.clearanceViolation && m.allFinite;
end
