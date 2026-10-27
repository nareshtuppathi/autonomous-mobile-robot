function res = analyzeHeadingErrorJumps(sim, ref, tcfg)
%ANALYZEHEADINGERRORJUMPS Find abrupt steps in the logged heading error and attribute their cause.
%   res = analyzeHeadingErrorJumps(sim, ref, tcfg) works on any tracking log (MATLAB or Simulink).
%   Heading error is e = wrap(thetaRef(sProj) - theta): the heading of the reference at the
%   projection of the robot onto the path, minus the robot heading. A step in e therefore has
%   four possible origins, tested separately for every event:
%     1. projection jump   the projected progress sProj moved much further than the robot did
%                          (nearest-point projection flipping between two path segments)
%     2. reference heading the reference heading itself changes abruptly along the path
%                          (checked for the whole reference, not only at the events)
%     3. angle wrapping    the wrapped error jumps across the +/-pi branch cut (a step of more than pi,
%                          although the true angle changed only slightly)
%     4. controller/robot  the robot heading changes abruptly, or the controller switches into
%                          turn-on-the-spot mode at that step
%   An event is a step with |delta e| above the largest change smooth motion can produce:
%       threshold = 1.5 * (maxAngularSpeed + max|reference curvature| * maxLinearSpeed) * dt
%   A projection jump is flagged when the progress step exceeds 3 * maxLinearSpeed * dt (three
%   steps of travel at top speed). For these, the flip predicted by geometry for a robot at
%   cross-track distance d inside a corner of turn angle phi, 2*d*tan(phi/2), is reported.
%   A counterfactual with progress rate-limited to the top speed (it cannot jump) shows how much
%   of the heading-error step is due to the projection.
%   res fields: events (table), thresholdRad, summary (struct), counterfactualError.

    L = sim.log;
    dt = tcfg.sim.timeStep;
    e = L.headingError(:);
    n = numel(e);
    kappaMax = max(abs(ref.kappa));
    thresholdRad = 1.5 * (tcfg.robot.maxAngularSpeed + kappaMax * tcfg.robot.maxLinearSpeed) * dt;
    jumpProgress = 3 * tcfg.robot.maxLinearSpeed * dt;

    dE = diff(e);
    ev = find(abs(dE) > thresholdRad);
    nEv = numel(ev);

    dsProj = diff(L.sProj(:));
    dThetaRef = wrapAngle(diff(L.thetaRef(:)));
    dTheta = wrapAngle(diff(L.theta(:)));
    wrapGap = abs(dE - wrapAngle(dE));          % nonzero only if the error jumps across the branch cut
    rotating = L.rotating(:) > 0;

    wp = ref.waypointsXY;
    cornerS = [0; cumsum(hypot(diff(wp(:, 1)), diff(wp(:, 2))))];
    seg = diff(wp);
    psi = atan2(seg(:, 2), seg(:, 1));
    turn = abs(wrapAngle(diff(psi)));

    k = ev(:);
    isProj = dsProj(k) > jumpProgress;
    isWrap = wrapGap(k) > 1e-9;
    isRobot = abs(dTheta(k)) > thresholdRad;
    isMode = rotating(k) ~= rotating(k + 1);
    predicted = nan(nEv, 1);
    cornerId = nan(nEv, 1);
    for q = 1:nEv
        if size(wp, 1) > 2
            [~, ic] = min(abs(cornerS(2:end - 1) - 0.5 * (L.sProj(k(q)) + L.sProj(k(q) + 1))));
            cornerId(q) = ic + 1;
            predicted(q) = 2 * abs(L.crossTrack(k(q))) * tan(turn(ic) / 2);
        end
    end
    cause = repmat({'unexplained'}, nEv, 1);
    cause(isMode) = {'controller_mode_switch'};
    cause(isRobot) = {'robot_motion'};
    cause(isWrap) = {'angle_wrap'};
    cause(isProj) = {'projection_jump'};

    res.events = table(k, L.t(k), dE(k), dThetaRef(k), dTheta(k), dsProj(k), ...
        repmat(tcfg.robot.maxLinearSpeed * dt, nEv, 1), predicted, cornerId, cause, ...
        'VariableNames', {'Step', 'TimeS', 'DeltaHeadingError', 'DeltaRefHeading', 'DeltaRobotHeading', ...
        'DeltaProgressM', 'MaxTravelPerStepM', 'PredictedFlipM', 'Corner', 'Cause'});
    res.thresholdRad = thresholdRad;

    % Counterfactual: progress cannot advance faster than the top speed
    sAlt = L.sProj(:);
    for q = 2:n
        sAlt(q) = min(L.sProj(q), sAlt(q - 1) + tcfg.robot.maxLinearSpeed * dt);
    end
    res.counterfactualError = wrapAngle(interp1(ref.s, ref.thetaUnwrapped, sAlt) - L.theta(:));

    s.numSteps = n;
    s.numEvents = nEv;
    s.numProjectionJumps = nnz(strcmp(cause, 'projection_jump'));
    s.numWrap = nnz(isWrap);
    s.numRobotMotion = nnz(isRobot);
    s.numModeSwitch = nnz(isMode);
    s.numUnexplained = nnz(strcmp(cause, 'unexplained'));
    s.maxAbsErrorRad = max(abs(e));
    s.maxStepRad = max(abs(dE));
    s.maxStepCounterfactualRad = max(abs(diff(res.counterfactualError)));
    s.maxRefHeadingStepPerSampleRad = max(abs(diff(ref.thetaUnwrapped)));
    s.maxRefCurvature = kappaMax;
    s.maxRobotHeadingStepRad = max(abs(dTheta));
    s.maxProgressStepM = max(dsProj);
    s.maxTravelPerStepM = max(abs(L.v)) * dt;
    s.maxDesiredOmegaStepRadps = max(abs(diff(L.omegaDes(:))));
    s.maxAppliedOmegaStepRadps = max(abs(diff(L.omega(:))));
    s.wrapNeverActive = ~any(wrapGap > 1e-9) && s.maxAbsErrorRad < pi;
    res.summary = s;
end
