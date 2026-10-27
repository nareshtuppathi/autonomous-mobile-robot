function sim = slLogToSim(logMatrix, tout, startPose, tMax)
%SLLOGTOSIM Convert the Simulink log matrix (one row per step, columns per slLogLayout) to a sim struct.
%   The result has the fields of simulateUnicycleTracking: log, terminationReason, goalReached,
%   numSteps, timeoutS, startPose.
    names = slLogLayout();
    L = struct();
    L.t = tout(:);
    for k = 1:numel(names)
        L.(names{k}) = logMatrix(:, k);
    end
    code = L.reasonCode(end);
    reasons = slReasonNames();
    if code >= 1 && code <= numel(reasons)
        reason = reasons{code};
    else
        reason = 'none';
    end
    sim.log = L;
    sim.terminationReason = reason;
    sim.goalReached = strcmp(reason, 'goal_reached');
    sim.numSteps = numel(L.t);
    sim.timeoutS = tMax;
    sim.startPose = startPose;
end
