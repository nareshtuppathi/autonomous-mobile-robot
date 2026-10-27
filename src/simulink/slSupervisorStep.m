function [braking, reasonCode, stop] = slSupervisorStep(clearanceM, inMap, poseFinite, cmdFinite, haveInfo, infoVec, velPrev, ref, cfg, env)
%SLSUPERVISORSTEP Goal-stop and failure logic of the Step 4 Simulink model.
%   Block body of Supervisor_GoalStop. It reproduces the decision chain of
%   simulateUnicycleTracking in the same order:
%     non-finite pose -> numerical_failure; outside map -> left_map; clearance below the
%     robot radius -> collision; non-finite command -> numerical_failure; then (only with
%     controller data) standstill after braking -> goal_reached / missed_goal, timeout,
%     entering the goal tolerance -> braking, divergence, stall.
%   reasonCode: 0 running, 1 goal_reached, 2 missed_goal, 3 timeout, 4 stalled, 5 diverged,
%   6 collision, 7 left_map, 8 numerical_failure (see slReasonNames).
%   braking stays true once the robot has entered the goal tolerance. stop = 1 on termination.
%   State is kept in persistent variables, which Simulink resets at the start of each run.
%   The progress history for stall detection is a ring buffer of 4096 steps.
    persistent k brakingState hist
    histLen = 4096;
    if isempty(k)
        k = 0;
        brakingState = false;
        hist = NaN(histLen, 1);
    end
    k = k + 1;

    dt = cfg.sim.timeStep;
    t = (k - 1) * dt;
    tMax = cfg.sim.timeoutFactor * ref.duration + cfg.sim.timeoutPad;
    stallSteps = max(1, round(cfg.sim.stallWindow / dt));
    c = cfg.controller;
    sProj = infoVec(1);  crossTrack = infoVec(2);  distToGoal = infoVec(15);
    vApplied = velPrev(1);  wApplied = velPrev(2);

    reason = 0;
    if poseFinite < 0.5
        reason = 8;
    elseif inMap < 0.5
        reason = 7;
    elseif clearanceM < env.robotRadiusM
        reason = 6;
    end
    if haveInfo > 0.5 && cmdFinite < 0.5 && reason == 0
        reason = 8;
    end

    if haveInfo > 0.5
        hist(mod(k - 1, histLen) + 1) = sProj;
        if reason ~= 0
            % already decided above
        elseif brakingState && abs(vApplied) < 1e-9 && abs(wApplied) < 1e-9
            if distToGoal <= c.goalTolerance
                reason = 1;
            else
                reason = 2;
            end
        elseif t >= tMax
            reason = 3;
        elseif ~brakingState && distToGoal <= c.goalTolerance && sProj >= ref.s(end) - 2 * c.goalTolerance
            brakingState = true;
        elseif ~brakingState && abs(crossTrack) > cfg.sim.maxCrossTrack
            reason = 5;
        elseif ~brakingState && k > stallSteps && sProj - hist(mod(k - 1 - stallSteps, histLen) + 1) < cfg.sim.stallProgress
            reason = 4;
        end
    end

    braking = double(brakingState);
    reasonCode = reason;
    stop = double(reason ~= 0);
end
