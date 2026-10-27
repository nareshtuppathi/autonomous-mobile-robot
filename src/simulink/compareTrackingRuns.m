function cmp = compareTrackingRuns(simA, simB, tol, ref, tcfg)
%COMPARETRACKINGRUNS Sample-by-sample comparison of two tracking simulations.
%   cmp = compareTrackingRuns(simA, simB, tol, ref, tcfg) compares the logs of two results shaped
%   like simulateUnicycleTracking (here MATLAB = A and Simulink = B) using the tolerances in tol
%   (step4Tolerances). Angles are compared after wrapping, so a difference across +/-pi is not
%   reported as 2*pi. NaN must appear in the same samples in both logs; equal infinities count
%   as equal. ref and tcfg are used to find the step at which braking starts.
%   cmp fields: sameSteps, sameReason, sameBrakingOnset, brakingOnsetStepMatlab,
%   brakingOnsetStepSimulink, sameRotating, table (Field, MaxAbsDiff, Tolerance, Pass),
%   maxAbsDiffOverall, allWithinTolerance.
    fields = {'t', 'x', 'y', 'theta', 'v', 'omega', 'vDes', 'omegaDes', 'sProj', 'xRef', 'yRef', ...
              'thetaRef', 'vRef', 'omegaRef', 'tRef', 'crossTrack', 'headingError', 'clearanceM', ...
              'lookahead', 'xLook', 'yLook', 'alpha', 'rotating', 'distToGoal'};
    angleFields = {'theta', 'thetaRef', 'headingError', 'alpha'};
    velocityFields = {'v', 'omega', 'vDes', 'omegaDes', 'vRef', 'omegaRef'};

    cmp.sameSteps  = simA.numSteps == simB.numSteps;
    cmp.sameReason = strcmp(simA.terminationReason, simB.terminationReason);
    n = min(simA.numSteps, simB.numSteps);

    maxDiff = zeros(numel(fields), 1);
    limit = zeros(numel(fields), 1);
    for k = 1:numel(fields)
        a = simA.log.(fields{k})(1:n);
        b = simB.log.(fields{k})(1:n);
        if ismember(fields{k}, angleFields)
            d = abs(wrapAngle(a - b));
            limit(k) = tol.angleRad;
        else
            d = abs(a - b);
            if ismember(fields{k}, velocityFields)
                limit(k) = tol.velocity;
            elseif strcmp(fields{k}, 'rotating')
                limit(k) = 0;
            else
                limit(k) = tol.lengthM;
            end
        end
        bothNaN = isnan(a) & isnan(b);
        equalValues = (a == b);                       % covers equal infinities
        d(equalValues | bothNaN) = 0;
        d(isnan(a) ~= isnan(b)) = Inf;                % NaN in only one log
        maxDiff(k) = max(d);
    end
    cmp.table = table(fields(:), maxDiff, limit, maxDiff <= limit, ...
        'VariableNames', {'Field', 'MaxAbsDiff', 'Tolerance', 'Pass'});
    cmp.maxAbsDiffOverall = max(maxDiff);

    cmp.brakingOnsetStepMatlab   = brakingOnsetStep(simA, ref, tcfg);
    cmp.brakingOnsetStepSimulink = brakingOnsetStep(simB, ref, tcfg);
    cmp.sameBrakingOnset = isequal(cmp.brakingOnsetStepMatlab, cmp.brakingOnsetStepSimulink);
    cmp.sameRotating = isequal(simA.log.rotating(1:n) > 0, simB.log.rotating(1:n) > 0);
    cmp.allWithinTolerance = cmp.sameSteps && cmp.sameReason && cmp.sameRotating && ...
        cmp.sameBrakingOnset && all(cmp.table.Pass);
end

function idx = brakingOnsetStep(sim, ref, tcfg)
%BRAKINGONSETSTEP First step at which the goal-stop logic starts braking, from the logged signals.
%   Braking starts at the first sample that is within the goal tolerance of the final point and
%   within 2*tolerance of the end of the path (the condition used by both implementations).
%   Returns [] if the robot never entered the goal tolerance.
    tolGoal = tcfg.controller.goalTolerance;
    inside = sim.log.distToGoal <= tolGoal & sim.log.sProj >= ref.s(end) - 2 * tolGoal;
    idx = find(inside, 1, 'first');
end
