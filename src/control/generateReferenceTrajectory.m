function ref = generateReferenceTrajectory(pathXY, cfg)
%GENERATEREFERENCETRAJECTORY Time-parameterised reference along a polyline path.
%   ref = generateReferenceTrajectory(pathXY, cfg) where pathXY is an N-by-2 list of
%   world waypoints [x y] in metres (for example the Step 2 smoothed path converted
%   with gridCoordsToWorld) and cfg = trackingConfig().
%
%   Method
%   1. Geometry: the path stays exactly on the polyline. Linear interpolation is
%      used on purpose: every segment of the Step 2 path was collision-checked, and
%      a spline would leave that validated corridor and need re-validation.
%      Samples are placed every cfg.reference.sampleSpacing metres and at every
%      waypoint, so the reference passes exactly through the waypoints.
%   2. Heading: each segment has the heading of its tangent. At each corner the
%      heading is blended linearly over a window centred on the corner (width
%      cfg.reference.cornerBlend, at most half of each adjacent segment), so heading
%      and curvature stay finite. Inside a window the heading is not exactly the
%      polyline tangent.
%   3. Curvature: kappa = d(heading)/d(arc length), constant inside each blend window.
%   4. Speed: the limit at each point is the smallest of the top speed, the speed at
%      which lateral acceleration v^2*|kappa| reaches its limit, the speed at which
%      the yaw rate v*|kappa| reaches its limit, and the speed at which a wheel
%      reaches its limit. A floor (MinTurnSpeedMps) keeps tight turns moving. A
%      forward-backward pass then enforces the longitudinal acceleration limit,
%      with zero speed at the start and the end.
%   5. Time: constant acceleration between samples gives dt = 2*ds/(v1 + v2).
%
%   This is a kinematic speed profile. It respects the stated limits but is not a
%   dynamic model, and nothing here shows that a real robot could execute it.
%
%   Output fields (column vectors over samples)
%     s, t              arc length (m) and time (s)
%     x, y              world position (m)
%     theta             heading wrapped to (-pi, pi] (rad)
%     thetaUnwrapped    continuous heading (rad), used for interpolation
%     kappa             curvature (1/m)
%     v, omega          reference linear (m/s) and angular (rad/s) speed (omega = v*kappa)
%     accelSeg          longitudinal acceleration on each interval (m/s^2), N-1 values
%     waypointsXY       the waypoints used (consecutive duplicates removed)
%     length, duration  total path length (m) and duration (s)
%     maxLateralAccel   max of v^2*|kappa| (m/s^2); can exceed the limit where the
%                       speed floor applies
%   Errors: amrStep3:invalidPath (NaN/Inf) and amrStep3:degeneratePath (fewer than two
%   distinct points).

    arguments
        pathXY (:, 2) double
        cfg (1, 1) struct
    end

    if any(~isfinite(pathXY), 'all')
        error('amrStep3:invalidPath', 'The path contains NaN or Inf values.');
    end
    segLen0 = hypot(diff(pathXY(:, 1)), diff(pathXY(:, 2)));
    keep = [true; segLen0 > 1e-9];                 % drop zero-length (duplicate) points
    P = pathXY(keep, :);
    if size(P, 1) < 2
        error('amrStep3:degeneratePath', 'The path needs at least two distinct points.');
    end

    segLen = hypot(diff(P(:, 1)), diff(P(:, 2)));
    wpS = [0; cumsum(segLen)];
    totalLen = wpS(end);
    segHead = unwrap(atan2(diff(P(:, 2)), diff(P(:, 1))));   % continuous heading per segment

    % Heading knots: constant on segments, linear ramp across each corner window
    knotS = 0;  knotTh = segHead(1);
    for i = 2:numel(segHead)
        dth = segHead(i) - segHead(i - 1);
        if abs(dth) < 1e-9
            continue;
        end
        half = min([cfg.reference.cornerBlend / 2, segLen(i - 1) / 2, segLen(i) / 2]);
        knotS(end + 1)  = wpS(i) - half;  knotTh(end + 1) = segHead(i - 1); %#ok<AGROW>
        knotS(end + 1)  = wpS(i) + half;  knotTh(end + 1) = segHead(i);     %#ok<AGROW>
    end
    knotS(end + 1) = totalLen;  knotTh(end + 1) = segHead(end);
    knotS = cummax(knotS);
    [knotS, iu] = unique(knotS);
    knotTh = knotTh(iu);

    % Sample positions
    ds = cfg.reference.sampleSpacing;
    s = unique([0:ds:totalLen, wpS.', totalLen, knotS]).';
    s = s([true; diff(s) > 1e-6]);
    if numel(s) < 3
        s = unique([s; totalLen / 2]);
    end
    x = interp1(wpS, P(:, 1), s, 'linear');
    y = interp1(wpS, P(:, 2), s, 'linear');
    thU = interp1(knotS, knotTh, s, 'linear');

    dsSeg = diff(s);
    kappaSeg = diff(thU) ./ dsSeg;
    kappa = [kappaSeg(1); (kappaSeg(1:end - 1) + kappaSeg(2:end)) / 2; kappaSeg(end)];

    % Speed limit on each interval, then at each sample (the slower neighbouring interval)
    absK = max(abs(kappaSeg), eps);
    vLimSeg = min([cfg.robot.maxLinearSpeed * ones(size(absK)), ...
                   sqrt(cfg.reference.maxLateralAccel ./ absK), ...
                   cfg.robot.maxAngularSpeed ./ absK, ...
                   cfg.robot.maxWheelSpeed ./ (1 + absK * cfg.robot.trackWidth / 2)], [], 2);
    vLimSeg = min(max(vLimSeg, cfg.reference.minTurnSpeed), cfg.robot.maxLinearSpeed);
    vLim = [vLimSeg(1); min(vLimSeg(1:end - 1), vLimSeg(2:end)); vLimSeg(end)];

    % Acceleration-limited profile: forward pass, then backward pass
    aMax = cfg.robot.maxLinearAccel;
    n = numel(s);
    v = vLim;
    v(1) = 0;
    for i = 2:n
        v(i) = min(v(i), sqrt(v(i - 1)^2 + 2 * aMax * dsSeg(i - 1)));
    end
    v(n) = 0;
    for i = n - 1:-1:1
        v(i) = min(v(i), sqrt(v(i + 1)^2 + 2 * aMax * dsSeg(i)));
    end

    dt = 2 * dsSeg ./ (v(1:end - 1) + v(2:end));
    t = [0; cumsum(dt)];

    ref.s = s;  ref.t = t;
    ref.x = x;  ref.y = y;
    ref.theta = wrapAngle(thU);
    ref.thetaUnwrapped = thU;
    ref.kappa = kappa;
    ref.v = v;
    ref.omega = v .* kappa;
    ref.accelSeg = (v(2:end).^2 - v(1:end - 1).^2) ./ (2 * dsSeg);
    ref.waypointsXY = P;
    ref.length = totalLen;
    ref.duration = t(end);
    ref.maxLateralAccel = max(v .^ 2 .* abs(kappa));
end
