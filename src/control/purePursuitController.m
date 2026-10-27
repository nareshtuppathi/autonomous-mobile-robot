function [cmd, info] = purePursuitController(pose, ref, sPrev, vNow, cfg)
%PUREPURSUITCONTROLLER One Pure Pursuit control step for a unicycle robot.
%   [cmd, info] = purePursuitController(pose, ref, sPrev, vNow, cfg)
%     pose  - [x y theta] in metres and radians (world frame)
%     ref   - reference trajectory from generateReferenceTrajectory
%     sPrev - arc-length progress along the reference at the previous step (m);
%             progress never moves backwards
%     vNow  - current linear speed (m/s), used to set the lookahead distance
%     cfg   - trackingConfig()
%   cmd = [v omega] - commanded linear (m/s) and angular (rad/s) velocity.
%
%   Algorithm
%   1. Project the robot onto the reference polyline (search window starting at the
%      previous progress) -> progress sProj and signed cross-track error
%      (positive = robot is to the left of the path direction).
%   2. Lookahead distance Ld = clip(LookaheadMin + LookaheadGain*|vNow|, LookaheadMin,
%      LookaheadMax). The lookahead point is Ld ahead along the path from sProj,
%      clamped to the final point.
%   3. alpha = wrap(bearing to lookahead point - heading), l = distance to that point.
%      Pure Pursuit curvature: kappa = 2*sin(alpha)/l (the circle through the robot
%      that is tangent to its heading and passes through the lookahead point).
%   4. Speed: reference speed at sProj, raised to at least MinSpeedMps, scaled by
%      cos(alpha) so the robot slows when pointing away from the path.
%      omega = v*kappa.
%   5. If |alpha| exceeds RotateThresholdRad the robot turns on the spot
%      (v = 0, omega proportional to alpha).
%   6. limitVelocityCommand enforces the linear, angular and wheel-speed limits.
%   Degenerate case: if the lookahead point coincides with the robot (l ~ 0), kappa = 0.
%
%   info fields: sProj, crossTrack (m, signed), headingError (rad, wrapped
%   reference heading minus robot heading), xRef, yRef, thetaRef, vRef, omegaRef, tRef,
%   lookahead (Ld, m), xLook, yLook, alpha, kappa, distToGoal (m), rotating (logical).
%   Error: amrStep3:invalidPose if pose is not a finite 1-by-3 vector.

    if ~isnumeric(pose) || numel(pose) ~= 3 || any(~isfinite(pose))
        error('amrStep3:invalidPose', 'pose must be a finite [x y theta] vector.');
    end
    pose = pose(:).';
    n = numel(ref.s);
    sEnd = ref.s(end);

    % 1. Projection onto the reference polyline
    i0 = find(ref.s <= sPrev, 1, 'last');
    if isempty(i0), i0 = 1; end
    i0 = min(i0, n - 1);
    i1 = find(ref.s >= sPrev + cfg.controller.searchAhead, 1, 'first');
    if isempty(i1), i1 = n; end
    i1 = max(i1, i0 + 1);
    idx = (i0:i1 - 1).';
    A  = [ref.x(idx), ref.y(idx)];
    AB = [ref.x(idx + 1), ref.y(idx + 1)] - A;
    AP = [pose(1) - A(:, 1), pose(2) - A(:, 2)];
    len2 = sum(AB .^ 2, 2);
    tt = min(max(sum(AP .* AB, 2) ./ len2, 0), 1);
    Q = A + tt .* AB;
    dist = hypot(pose(1) - Q(:, 1), pose(2) - Q(:, 2));
    [~, j] = min(dist);
    sProj = max(ref.s(idx(j)) + tt(j) * (ref.s(idx(j) + 1) - ref.s(idx(j))), sPrev);
    cross = AB(j, 1) * AP(j, 2) - AB(j, 2) * AP(j, 1);
    crossTrack = dist(j);
    if cross < 0
        crossTrack = -crossTrack;
    end

    thetaRef = wrapAngle(interp1(ref.s, ref.thetaUnwrapped, sProj));
    info.sProj = sProj;
    info.crossTrack = crossTrack;
    info.headingError = wrapAngle(thetaRef - pose(3));
    info.xRef = interp1(ref.s, ref.x, sProj);
    info.yRef = interp1(ref.s, ref.y, sProj);
    info.thetaRef = thetaRef;
    info.vRef = interp1(ref.s, ref.v, sProj);
    info.omegaRef = interp1(ref.s, ref.omega, sProj);
    info.tRef = interp1(ref.s, ref.t, sProj);
    info.distToGoal = hypot(pose(1) - ref.x(end), pose(2) - ref.y(end));

    % 2. Lookahead point
    c = cfg.controller;
    Ld = min(max(c.lookaheadMin + c.lookaheadGain * abs(vNow), c.lookaheadMin), c.lookaheadMax);
    sLook = min(sProj + Ld, sEnd);
    xL = interp1(ref.s, ref.x, sLook);
    yL = interp1(ref.s, ref.y, sLook);
    info.lookahead = Ld;  info.xLook = xL;  info.yLook = yL;

    % 3. Heading error to the lookahead point and Pure Pursuit curvature
    dx = xL - pose(1);  dy = yL - pose(2);
    l = hypot(dx, dy);
    if l < 1e-9
        alpha = 0;  kappa = 0;
    else
        alpha = wrapAngle(atan2(dy, dx) - pose(3));
        kappa = 2 * sin(alpha) / l;
    end
    info.alpha = alpha;  info.kappa = kappa;

    % 4./5. Speed and angular velocity
    info.rotating = abs(alpha) > c.rotateThreshold;
    if info.rotating
        v = 0;
        w = c.rotateGain * alpha;
    else
        v = max(info.vRef, c.minSpeed) * max(cos(alpha), 0);
        w = v * kappa;
    end

    % 6. Limits
    [v, w] = limitVelocityCommand(v, w, cfg.robot);
    cmd = [v, w];
end
