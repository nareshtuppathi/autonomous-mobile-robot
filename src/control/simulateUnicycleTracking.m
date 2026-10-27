function sim = simulateUnicycleTracking(ref, startPose, cfg, env)
%SIMULATEUNICYCLETRACKING Closed-loop kinematic simulation of Pure Pursuit tracking.
%   sim = simulateUnicycleTracking(ref, startPose, cfg, env)
%     ref       - reference trajectory (generateReferenceTrajectory)
%     startPose - [x y theta] in metres/radians
%     cfg       - trackingConfig()
%     env       - makeClearanceEnvironment(rawGrid, robotConfig())
%
%   Model: unicycle, x_dot = v*cos(theta), y_dot = v*sin(theta), theta_dot = omega,
%   integrated exactly for piecewise-constant commands every cfg.sim.timeStep.
%   Actuators: the commanded (v, omega) pass through acceleration limits
%   (cfg.robot.maxLinearAccel / maxAngularAccel) and the wheel-speed limit.
%   There is no wheel slip, motor dynamics, delay or sensor noise: this is a
%   kinematic simulation only.
%
%   Termination reasons (sim.terminationReason)
%     goal_reached       entered GoalToleranceM of the final point (near the end of the path),
%                        braked to a standstill within the acceleration limit, and stopped
%                        within GoalToleranceM of the final point
%     missed_goal        entered the goal tolerance but came to rest outside it
%     timeout            time exceeded TimeoutFactor*duration + TimeoutPadS
%     stalled            progress along the path over StallWindowS below StallProgressM
%     diverged           |cross-track error| above MaxCrossTrackM
%     collision          clearance to a real obstacle below the robot radius
%     left_map           robot centre outside the map
%     numerical_failure  non-finite pose or command
%
%   sim.log is a struct of column vectors (one row per control step, including the
%   final state): t, x, y, theta, v, omega (applied over the step that starts at that
%   sample; on the final sample the last applied values, so the log never contains an
%   instant stop), vDes, omegaDes (controller),
%   sProj, xRef, yRef, thetaRef, vRef, omegaRef, tRef, crossTrack, headingError,
%   clearanceM, lookahead, xLook, yLook, alpha, rotating, distToGoal.

    arguments
        ref (1, 1) struct
        startPose (1, 3) double
        cfg (1, 1) struct
        env (1, 1) struct
    end

    dt = cfg.sim.timeStep;
    tMax = cfg.sim.timeoutFactor * ref.duration + cfg.sim.timeoutPad;
    maxSteps = ceil(tMax / dt) + 2;
    names = {'t', 'x', 'y', 'theta', 'v', 'omega', 'vDes', 'omegaDes', 'sProj', 'xRef', 'yRef', ...
             'thetaRef', 'vRef', 'omegaRef', 'tRef', 'crossTrack', 'headingError', 'clearanceM', ...
             'lookahead', 'xLook', 'yLook', 'alpha', 'rotating', 'distToGoal'};
    L = struct();
    for k = 1:numel(names)
        L.(names{k}) = nan(maxSteps, 1);
    end

    pose = [startPose(1:2), wrapAngle(startPose(3))];
    vApplied = 0;  wApplied = 0;
    sProg = 0;
    reason = '';
    braking = false;   % true once the robot has entered the goal tolerance
    stallSteps = max(1, round(cfg.sim.stallWindow / dt));
    k = 0;
    c = cfg.controller;

    while true
        k = k + 1;
        t = (k - 1) * dt;
        L.t(k) = t;  L.x(k) = pose(1);  L.y(k) = pose(2);  L.theta(k) = pose(3);

        if any(~isfinite(pose))
            reason = 'numerical_failure';
        else
            [clr, inMap] = clearanceAtPoints(env, pose(1:2));
            L.clearanceM(k) = clr;
            if ~inMap
                reason = 'left_map';
            elseif clr < env.robotRadiusM
                reason = 'collision';
            end
        end

        haveInfo = false;
        if all(isfinite(pose))
            % Evaluated even after a collision or leaving the map, so the final sample is logged
            [cmd, info] = purePursuitController(pose, ref, sProg, vApplied, cfg);
            haveInfo = true;
            if any(~isfinite(cmd)) && isempty(reason)
                reason = 'numerical_failure';
            end
        end

        if haveInfo
            sProg = info.sProj;
            L.vDes(k) = cmd(1);  L.omegaDes(k) = cmd(2);
            L.sProj(k) = info.sProj;  L.xRef(k) = info.xRef;  L.yRef(k) = info.yRef;
            L.thetaRef(k) = info.thetaRef;  L.vRef(k) = info.vRef;  L.omegaRef(k) = info.omegaRef;
            L.tRef(k) = info.tRef;  L.crossTrack(k) = info.crossTrack;  L.headingError(k) = info.headingError;
            L.lookahead(k) = info.lookahead;  L.xLook(k) = info.xLook;  L.yLook(k) = info.yLook;
            L.alpha(k) = info.alpha;  L.rotating(k) = info.rotating;  L.distToGoal(k) = info.distToGoal;

            if ~isempty(reason)
                % collision, left_map or numerical failure already decided above
            elseif braking && abs(vApplied) < 1e-9 && abs(wApplied) < 1e-9
                % Standstill reached after entering the goal tolerance
                if info.distToGoal <= c.goalTolerance
                    reason = 'goal_reached';
                else
                    reason = 'missed_goal';
                end
            elseif t >= tMax
                reason = 'timeout';
            elseif ~braking && info.distToGoal <= c.goalTolerance && info.sProj >= ref.s(end) - 2 * c.goalTolerance
                braking = true;   % inside the goal tolerance: stop, within the acceleration limit
            elseif ~braking && abs(info.crossTrack) > cfg.sim.maxCrossTrack
                reason = 'diverged';
            elseif ~braking && k > stallSteps && info.sProj - L.sProj(k - stallSteps) < cfg.sim.stallProgress
                reason = 'stalled';
            end
        end

        if ~isempty(reason)
            % Final sample: log the last applied velocities (after a failure an emergency
            % stop is assumed to happen after this sample)
            L.v(k) = vApplied;  L.omega(k) = wApplied;
            break;
        end

        if braking
            cmd = [0, 0];   % stop, decelerating through the actuator limits below
        end

        % Actuator model: acceleration limits, then wheel-speed limit
        dvMax = cfg.robot.maxLinearAccel * dt;
        dwMax = cfg.robot.maxAngularAccel * dt;
        vApplied = vApplied + min(max(cmd(1) - vApplied, -dvMax), dvMax);
        wApplied = wApplied + min(max(cmd(2) - wApplied, -dwMax), dwMax);
        [vApplied, wApplied] = limitVelocityCommand(vApplied, wApplied, cfg.robot);
        L.v(k) = vApplied;  L.omega(k) = wApplied;

        pose = stepUnicycle(pose, vApplied, wApplied, dt);
    end

    for q = 1:numel(names)
        L.(names{q}) = L.(names{q})(1:k);
    end
    sim.log = L;
    sim.terminationReason = reason;
    sim.goalReached = strcmp(reason, 'goal_reached');
    sim.numSteps = k;
    sim.timeoutS = tMax;
    sim.startPose = startPose;
end
