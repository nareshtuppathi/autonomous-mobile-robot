classdef testStep3 < matlab.unittest.TestCase
%TESTSTEP3 Unit tests for Step 3: reference trajectory, Pure Pursuit and kinematic simulation.
%   Run with: runtests('tests/testStep3.m')
%   All paths are in world coordinates [x y] in metres, inside a 15 m x 15 m empty map
%   unless a test builds its own obstacles. Every scenario is deterministic.

    methods (TestClassSetup)
        function addSourcePath(testCase)
            srcDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(srcDir, 'IncludeSubfolders', true));
        end
    end

    methods (Static, Access = private)
        function env = freeEnv()
            env = makeClearanceEnvironment(false(60, 60), robotConfig());
        end

        function [sim, m, ref] = track(pathXY, pose, cfg, env, refCfg)
            if nargin < 5, refCfg = cfg; end
            ref = generateReferenceTrajectory(pathXY, refCfg);
            sim = simulateUnicycleTracking(ref, pose, cfg, env);
            m = computeTrackingMetrics(sim, ref, env, pathXY(end, :), cfg);
        end
    end

    methods (Test)
        %% Reference trajectory
        function referenceOnStraightLineIsATrapezoid(testCase)
            cfg = trackingConfig();
            ref = generateReferenceTrajectory([2 2; 9 2], cfg);
            report = checkReferenceLimits(ref, cfg);
            testCase.verifyTrue(report.passesAll);
            testCase.verifyEqual(ref.length, 7, 'AbsTol', 1e-12);
            testCase.verifyEqual([ref.v(1), ref.v(end)], [0 0]);
            testCase.verifyEqual(max(ref.v), cfg.robot.maxLinearSpeed, 'AbsTol', 1e-9);
            % Accelerate to 0.6 m/s (0.36 m), cruise, brake (0.36 m): duration = 6.28/0.6 + 2*0.6/0.5
            testCase.verifyEqual(ref.duration, (7 - 0.72) / 0.6 + 2.4, 'AbsTol', 0.1);
            testCase.verifyEqual(ref.theta, zeros(size(ref.theta)), 'AbsTol', 1e-12);
            testCase.verifyEqual(ref.omega, zeros(size(ref.omega)), 'AbsTol', 1e-12);
        end

        function referenceSlowsDownAtASharpCorner(testCase)
            cfg = trackingConfig();
            path = [2 2; 8 2; 8 8];
            ref = generateReferenceTrajectory(path, cfg);
            report = checkReferenceLimits(ref, cfg);
            testCase.verifyTrue(report.passesAll);
            [~, iCorner] = min(abs(ref.s - 6));                 % arc length of the corner
            testCase.verifyLessThan(ref.v(iCorner), 0.8 * cfg.robot.maxLinearSpeed);
            testCase.verifyGreaterThanOrEqual(ref.v(iCorner), cfg.reference.minTurnSpeed - 1e-9);
            testCase.verifyEqual(ref.theta(end), pi / 2, 'AbsTol', 1e-9);        % ends heading +y
            testCase.verifyEqual(ref.thetaUnwrapped(end) - ref.thetaUnwrapped(1), pi / 2, 'AbsTol', 1e-9);
            testCase.verifyEqual(ref.omega, ref.v .* ref.kappa, 'AbsTol', 1e-12);
        end

        function referencePassesThroughWaypointsAndSamplesByTime(testCase)
            cfg = trackingConfig();
            path = [2 2; 8 2; 8 8];
            ref = generateReferenceTrajectory(path, cfg);
            for k = 1:size(path, 1)
                testCase.verifyLessThan(min(hypot(ref.x - path(k, 1), ref.y - path(k, 2))), 1e-9);
            end
            q = sampleReferenceAtTime(ref, [0, ref.duration / 2, ref.duration, ref.duration + 5]);
            testCase.verifyEqual([q.x(1), q.y(1)], path(1, :), 'AbsTol', 1e-12);
            testCase.verifyEqual([q.x(3), q.y(3)], path(end, :), 'AbsTol', 1e-9);
            testCase.verifyEqual([q.x(4), q.y(4)], path(end, :), 'AbsTol', 1e-9);      % clamped
            testCase.verifyGreaterThan(q.s(2), q.s(1));
            testCase.verifyLessThan(q.s(2), q.s(3));
        end

        function referenceRejectsDegenerateAndInvalidPaths(testCase)
            cfg = trackingConfig();
            testCase.verifyError(@() generateReferenceTrajectory([0 0; NaN 1], cfg), 'amrStep3:invalidPath');
            testCase.verifyError(@() generateReferenceTrajectory([0 0; Inf 1], cfg), 'amrStep3:invalidPath');
            testCase.verifyError(@() generateReferenceTrajectory([1 1], cfg), 'amrStep3:degeneratePath');
            testCase.verifyError(@() generateReferenceTrajectory([1 1; 1 1; 1 1], cfg), 'amrStep3:degeneratePath');
            ref = generateReferenceTrajectory([0 0; 0 0; 3 0], cfg);       % repeated point is dropped
            testCase.verifyEqual(ref.waypointsXY, [0 0; 3 0]);
        end

        %% Tracking scenarios
        function straightPathTracking(testCase)
            [sim, m] = testStep3.track([2 2; 9 2], [2 2 0], trackingConfig(), testStep3.freeEnv());
            testCase.verifyEqual(sim.terminationReason, 'goal_reached');
            testCase.verifyTrue(m.goalReached);
            % The only cross-track error is the few millimetres by which the stop passes the end point
            testCase.verifyLessThan(m.maxCrossTrackM, 0.01);
            testCase.verifyLessThanOrEqual(m.finalPositionErrorM, 0.1);
            testCase.verifyEqual(m.finalSpeedMps, 0);
            testCase.verifyEqual(m.distanceTravelledM, 7, 'AbsTol', 0.02);
            testCase.verifyTrue(m.overallPass);
        end

        function turningPathTracking(testCase)
            [sim, m, ref] = testStep3.track([2 2; 8 2; 8 8], [2 2 0], trackingConfig(), testStep3.freeEnv());
            testCase.verifyEqual(sim.terminationReason, 'goal_reached');
            testCase.verifyLessThan(m.maxCrossTrackM, 0.25);               % observed 0.154 m; divergence limit is 0.5 m
            testCase.verifyLessThan(m.rmsCrossTrackM, m.maxCrossTrackM);
            testCase.verifyLessThan(m.distanceTravelledM, ref.length);     % the lookahead cuts the corner
            testCase.verifyLessThanOrEqual(m.maxHeadingErrorRad, pi);      % wrapped
            testCase.verifyLessThan(m.maxHeadingErrorRad, deg2rad(45));
            testCase.verifyLessThanOrEqual(m.finalPositionErrorM, 0.1);
            testCase.verifyTrue(m.allFinite);
        end

        function initialHeadingErrorIsRecovered(testCase)
            env = testStep3.freeEnv();
            cfg = trackingConfig();
            for theta0 = [pi / 4, pi / 2, pi, -2.5]
                [sim, m] = testStep3.track([2 2; 9 2], [2 2 theta0], cfg, env);
                testCase.verifyEqual(sim.terminationReason, 'goal_reached', sprintf('heading offset %.2f rad', theta0));
                testCase.verifyLessThanOrEqual(m.finalPositionErrorM, 0.1);
                testCase.verifyLessThan(m.maxCrossTrackM, 0.3);
                testCase.verifyEqual(abs(sim.log.headingError(1)), abs(wrapAngle(0 - theta0)), 'AbsTol', 1e-9);
                if abs(wrapAngle(theta0)) > cfg.controller.rotateThreshold
                    testCase.verifyTrue(any(sim.log.rotating > 0), 'turns on the spot for large errors');
                end
            end
        end

        function lateralOffsetAndHeadingErrorConverge(testCase)
            [sim, m] = testStep3.track([2 2; 9 2], [2 2.3 pi / 4], trackingConfig(), testStep3.freeEnv());
            testCase.verifyEqual(sim.terminationReason, 'goal_reached');
            testCase.verifyEqual(abs(sim.log.crossTrack(1)), 0.3, 'AbsTol', 1e-9);
            testCase.verifyLessThan(abs(sim.log.crossTrack(round(end / 2))), 0.02);   % converged by mid-run
            testCase.verifyLessThanOrEqual(m.finalPositionErrorM, 0.1);
        end

        function runsAreDeterministic(testCase)
            env = testStep3.freeEnv();
            a = testStep3.track([2 2; 8 2; 8 8], [2 2 0.3], trackingConfig(), env);
            b = testStep3.track([2 2; 8 2; 8 8], [2 2 0.3], trackingConfig(), env);
            testCase.verifyTrue(isequaln(a.log, b.log));
        end

        %% Goal convergence, braking and termination
        function robotBrakesToStandstillWithinAccelerationLimit(testCase)
            cfg = trackingConfig();
            [sim, m] = testStep3.track([2 2; 8 2; 8 8], [2 2 0], cfg, testStep3.freeEnv());
            testCase.verifyEqual(sim.terminationReason, 'goal_reached');
            testCase.verifyEqual(sim.log.v(end), 0);                       % ends at rest, not an instant stop
            testCase.verifyEqual(sim.log.omega(end), 0);
            tol = 1e-9;
            testCase.verifyLessThanOrEqual(m.maxLinearAccelMps2, cfg.robot.maxLinearAccel + tol);
            testCase.verifyLessThanOrEqual(m.maxAngularAccelRadps2, cfg.robot.maxAngularAccel + tol);
            % The last step must also respect the limit (it used to be excluded)
            lastDrop = abs(sim.log.v(end) - sim.log.v(end - 1)) / cfg.sim.timeStep;
            testCase.verifyLessThanOrEqual(lastDrop, cfg.robot.maxLinearAccel + tol);
        end

        function goalIsReportedOnlyAtStandstillInsideTolerance(testCase)
            cfg = trackingConfig();
            [sim, m] = testStep3.track([2 2; 9 2], [2 2 0], cfg, testStep3.freeEnv());
            testCase.verifyTrue(sim.goalReached);
            testCase.verifyEqual(m.finalSpeedMps, 0);
            testCase.verifyLessThanOrEqual(m.finalPositionErrorM, cfg.controller.goalTolerance);
            testCase.verifyLessThan(sim.log.t(end), sim.timeoutS);
        end

        function stoppingOutsideToleranceIsMissedGoal(testCase)
            % The reference brakes at 0.5 m/s^2 but this robot can only brake at 0.3 m/s^2
            ref = generateReferenceTrajectory([2 2; 9 2], trackingConfig());
            weak = trackingConfig('MaxLinearAccelMps2', 0.3);
            sim = simulateUnicycleTracking(ref, [2 2 0], weak, testStep3.freeEnv());
            m = computeTrackingMetrics(sim, ref, testStep3.freeEnv(), [9 2], weak);
            testCase.verifyEqual(sim.terminationReason, 'missed_goal');
            testCase.verifyFalse(sim.goalReached);
            testCase.verifyFalse(m.overallPass);
            testCase.verifyGreaterThan(m.finalPositionErrorM, weak.controller.goalTolerance);
            testCase.verifyEqual(m.finalSpeedMps, 0);                      % it still stopped, safely
            testCase.verifyLessThanOrEqual(m.maxLinearAccelMps2, weak.robot.maxLinearAccel + 1e-9);
        end

        %% Velocity-limit enforcement
        function limitVelocityCommandEnforcesAllLimits(testCase)
            robot = trackingConfig().robot;
            [v, ~] = limitVelocityCommand(5, 0, robot);
            testCase.verifyEqual(v, robot.maxLinearSpeed);
            [v, ~] = limitVelocityCommand(-1, 0, robot);
            testCase.verifyEqual(v, 0);                                    % no reversing
            % Yaw-rate limit: speed is reduced and the commanded curvature is preserved
            [v, w] = limitVelocityCommand(0.6, 6, robot);
            testCase.verifyEqual(abs(w), robot.maxAngularSpeed, 'AbsTol', 1e-12);
            testCase.verifyEqual(w / v, 6 / 0.6, 'RelTol', 1e-9);
            % Wheel-speed limit
            tight = robot;  tight.maxWheelSpeed = 0.62;
            [v, w] = limitVelocityCommand(0.6, 1.4, tight);
            testCase.verifyLessThanOrEqual(abs(v) + abs(w) * tight.trackWidth / 2, tight.maxWheelSpeed + 1e-12);
            testCase.verifyEqual(w / v, 1.4 / 0.6, 'RelTol', 1e-9);
            % Turning on the spot is limited by the yaw rate only
            [v, w] = limitVelocityCommand(0, -9, robot);
            testCase.verifyEqual([v w], [0, -robot.maxAngularSpeed], 'AbsTol', 1e-12);
        end

        function simulatedVelocitiesStayWithinConfiguredLimits(testCase)
            cfg = trackingConfig('MaxLinearSpeedMps', 0.3, 'MaxAngularSpeedRadps', 0.5, 'MaxLinearAccelMps2', 0.2, ...
                'MaxAngularAccelRadps2', 1.0, 'MaxWheelSpeedMps', 0.35, 'MinTurnSpeedMps', 0.1, 'MinSpeedMps', 0.03);
            [~, m] = testStep3.track([2 2; 8 2; 8 8], [2 2 1.2], cfg, testStep3.freeEnv());
            tol = 1e-9;
            testCase.verifyLessThanOrEqual(m.maxLinearSpeedMps, cfg.robot.maxLinearSpeed + tol);
            testCase.verifyLessThanOrEqual(m.maxAngularSpeedRadps, cfg.robot.maxAngularSpeed + tol);
            testCase.verifyLessThanOrEqual(m.maxWheelSpeedMps, cfg.robot.maxWheelSpeed + tol);
            testCase.verifyLessThanOrEqual(m.maxLinearAccelMps2, cfg.robot.maxLinearAccel + tol);
            testCase.verifyLessThanOrEqual(m.maxAngularAccelRadps2, cfg.robot.maxAngularAccel + tol);
            testCase.verifyTrue(m.goalReached);
        end

        function controllerOutputIsLimitedForExtremeInputs(testCase)
            cfg = trackingConfig();
            ref = generateReferenceTrajectory([2 2; 8 2; 8 8], cfg);
            poses = [2 2 pi; 5 4 -pi / 2; 8 2 0.2; 30 30 1];
            for k = 1:size(poses, 1)
                cmd = purePursuitController(poses(k, :), ref, 0, 0.6, cfg);
                testCase.verifyTrue(all(isfinite(cmd)));
                testCase.verifyGreaterThanOrEqual(cmd(1), 0);
                testCase.verifyLessThanOrEqual(cmd(1), cfg.robot.maxLinearSpeed);
                testCase.verifyLessThanOrEqual(abs(cmd(2)), cfg.robot.maxAngularSpeed + 1e-12);
            end
        end

        %% Controller geometry and angle handling
        function crossTrackAndHeadingErrorSignsAndWrapping(testCase)
            cfg = trackingConfig();
            ref = generateReferenceTrajectory([2 2; 9 2], cfg);              % path along +x
            [~, left] = purePursuitController([4 2.2 0], ref, 2, 0, cfg);   % sPrev = 2 m: the robot is 2 m along the path
            [~, right] = purePursuitController([4 1.8 0], ref, 2, 0, cfg);
            testCase.verifyEqual(left.crossTrack, 0.2, 'AbsTol', 1e-9);      % left of the path = positive
            testCase.verifyEqual(right.crossTrack, -0.2, 'AbsTol', 1e-9);
            [~, tilted] = purePursuitController([4 2 0.3], ref, 2, 0, cfg);
            testCase.verifyEqual(tilted.headingError, -0.3, 'AbsTol', 1e-9);  % reference heading minus robot heading
            % Wrapping: path heading is +pi, robot heading is -pi + 0.1 -> error -0.1, not +6.18
            refWest = generateReferenceTrajectory([9 2; 2 2], cfg);
            [~, w] = purePursuitController([5 2 -pi + 0.1], refWest, 4, 0, cfg);
            testCase.verifyEqual(abs(w.headingError), 0.1, 'AbsTol', 1e-9);
            a = wrapAngle([3 * pi / 2, -3 * pi / 2, 2 * pi + 0.1, -7]);
            testCase.verifyTrue(all(abs(a) <= pi + 1e-12));
            testCase.verifyEqual(a(1), -pi / 2, 'AbsTol', 1e-12);
            testCase.verifyEqual(a(3), 0.1, 'AbsTol', 1e-12);
        end

        function zeroDistanceLookaheadIsHandled(testCase)
            cfg = trackingConfig();
            ref = generateReferenceTrajectory([2 2; 9 2], cfg);
            [cmd, info] = purePursuitController([9 2 0], ref, ref.s(end), 0, cfg);   % robot sits on the final point
            testCase.verifyTrue(all(isfinite(cmd)));
            testCase.verifyEqual(info.kappa, 0);
            testCase.verifyEqual(info.distToGoal, 0, 'AbsTol', 1e-12);
        end

        %% Obstacle and clearance validation
        function clearanceAtPointsIsExact(testCase)
            g = false(60, 60);  g(30:35, 30:35) = true;                    % block spans x 7.25-8.75, top edge y = 7.75
            env = makeClearanceEnvironment(g, robotConfig());
            [c, inMap] = clearanceAtPoints(env, [8 7.97; 8 7.75; 8 7.0; 6 7.0; -1 5]);
            testCase.verifyEqual(c(1), 0.22, 'AbsTol', 1e-12);
            testCase.verifyEqual(c(2), 0, 'AbsTol', 1e-12);                % on the edge
            testCase.verifyEqual(c(3), 0, 'AbsTol', 1e-12);                % inside the block
            testCase.verifyEqual(c(4), 1.25, 'AbsTol', 1e-12);             % 1.25 m left of the block
            testCase.verifyEqual(inMap.', [true true true true false]);
            empty = makeClearanceEnvironment(false(4, 4), robotConfig());
            testCase.verifyEqual(clearanceAtPoints(empty, [0.5 0.5]), Inf);
        end

        function trajectoryWithEnoughClearancePasses(testCase)
            g = false(60, 60);  g(30:35, 30:35) = true;
            env = makeClearanceEnvironment(g, robotConfig());
            y = 7.75 + 0.5;
            [sim, m] = testStep3.track([3 y; 13 y], [3 y 0], trackingConfig(), env);
            testCase.verifyEqual(sim.terminationReason, 'goal_reached');
            testCase.verifyEqual(m.minClearanceM, 0.5, 'AbsTol', 1e-6);
            testCase.verifyFalse(m.collision);
            testCase.verifyFalse(m.clearanceViolation);
            testCase.verifyEqual(m.numViolatingSamples, 0);
            testCase.verifyTrue(m.overallPass);
        end

        function clearanceBelowRequirementIsFlaggedButNotACollision(testCase)
            g = false(60, 60);  g(30:35, 30:35) = true;
            mapCfg = robotConfig();                                           % radius 0.20, required 0.25
            env = makeClearanceEnvironment(g, mapCfg);
            y = 7.75 + 0.22;
            [sim, m] = testStep3.track([3 y; 13 y], [3 y 0], trackingConfig(), env);
            testCase.verifyEqual(sim.terminationReason, 'goal_reached');
            testCase.verifyEqual(m.minClearanceM, 0.22, 'AbsTol', 1e-6);
            testCase.verifyFalse(m.collision);                                % 0.22 >= 0.20
            testCase.verifyTrue(m.clearanceViolation);                        % 0.22 < 0.25
            testCase.verifyGreaterThan(m.numViolatingSamples, 0);
            testCase.verifyFalse(m.overallPass);
        end

        function drivingIntoTheRobotRadiusIsACollision(testCase)
            g = false(60, 60);  g(30:35, 30:35) = true;
            env = makeClearanceEnvironment(g, robotConfig());
            y = 7.75 + 0.10;
            [sim, m] = testStep3.track([3 y; 13 y], [3 y 0], trackingConfig(), env);
            testCase.verifyEqual(sim.terminationReason, 'collision');
            testCase.verifyFalse(sim.goalReached);
            testCase.verifyTrue(m.collision);
            testCase.verifyLessThan(m.minClearanceM, 0.2);
            testCase.verifyFalse(m.overallPass);
        end

        function checksTheRealMapNotTheReferencePath(testCase)
            % A reference that goes straight through an obstacle: the real robot trajectory
            % must be judged against the obstacle, whatever the reference says
            g = false(60, 60);  g(30:35, 30:35) = true;
            env = makeClearanceEnvironment(g, robotConfig());
            [sim, m] = testStep3.track([3 7.0; 13 7.0], [3 7.0 0], trackingConfig(), env);   % block spans y 6.25-7.75
            testCase.verifyEqual(sim.terminationReason, 'collision');
            testCase.verifyTrue(m.collision);
        end

        %% Invalid parameters and failure handling
        function invalidConfigurationIsRejected(testCase)
            testCase.verifyError(@() trackingConfig('MaxLinearSpeedMps', -1), 'MATLAB:validators:mustBePositive');
            testCase.verifyError(@() trackingConfig('TimeStepS', 0), 'MATLAB:validators:mustBePositive');
            testCase.verifyError(@() trackingConfig('GoalToleranceM', Inf), 'MATLAB:validators:mustBeFinite');
            testCase.verifyError(@() trackingConfig('LookaheadGainS', -0.1), 'MATLAB:validators:mustBeNonnegative');
            testCase.verifyError(@() trackingConfig('LookaheadMinM', 2, 'LookaheadMaxM', 1), 'amrStep3:invalidConfig');
            testCase.verifyError(@() trackingConfig('MinSpeedMps', 1), 'amrStep3:invalidConfig');
            testCase.verifyError(@() trackingConfig('MaxWheelSpeedMps', 0.3), 'amrStep3:invalidConfig');
            testCase.verifyError(@() trackingConfig('RotateThresholdRad', 4), 'amrStep3:invalidConfig');
        end

        function invalidPosesAreRejectedOrReported(testCase)
            cfg = trackingConfig();
            ref = generateReferenceTrajectory([2 2; 9 2], cfg);
            testCase.verifyError(@() purePursuitController([NaN 2 0], ref, 0, 0, cfg), 'amrStep3:invalidPose');
            testCase.verifyError(@() purePursuitController([2 2], ref, 0, 0, cfg), 'amrStep3:invalidPose');
            testCase.verifyError(@() simulateUnicycleTracking(ref, [2 2], cfg, testStep3.freeEnv()), 'MATLAB:validation:IncompatibleSize');
            sim = simulateUnicycleTracking(ref, [NaN 2 0], cfg, testStep3.freeEnv());
            testCase.verifyEqual(sim.terminationReason, 'numerical_failure');
            testCase.verifyFalse(sim.goalReached);
        end

        function timeoutStallDivergenceAndLeavingTheMapAreSafeFailures(testCase)
            env = testStep3.freeEnv();
            path = [2 2; 9 2];
            ref = generateReferenceTrajectory(path, trackingConfig());
            cases = {
                trackingConfig('TimeoutFactor', 0.2, 'TimeoutPadS', 0), [2 2 0], 'timeout'
                trackingConfig('MinSpeedMps', 0),                       [2 2 0], 'stalled'
                trackingConfig(),                                       [2 3 0], 'diverged'
                trackingConfig(),                                       [-1 2 0], 'left_map'};
            for k = 1:size(cases, 1)
                sim = simulateUnicycleTracking(ref, cases{k, 2}, cases{k, 1}, env);
                m = computeTrackingMetrics(sim, ref, env, path(end, :), cases{k, 1});
                testCase.verifyEqual(sim.terminationReason, cases{k, 3});
                testCase.verifyFalse(sim.goalReached);
                testCase.verifyFalse(m.goalReached);
                testCase.verifyFalse(m.overallPass);
                testCase.verifyLessThanOrEqual(sim.log.t(end), sim.timeoutS + cases{k, 1}.sim.timeStep);
            end
        end

        function metricsAreConsistentWithTheLog(testCase)
            [sim, m] = testStep3.track([2 2; 8 2; 8 8], [2 2 0.4], trackingConfig(), testStep3.freeEnv());
            L = sim.log;
            testCase.verifyEqual(m.rmsCrossTrackM, sqrt(mean(L.crossTrack .^ 2)), 'AbsTol', 1e-15);
            testCase.verifyEqual(m.maxCrossTrackM, max(abs(L.crossTrack)), 'AbsTol', 1e-15);
            testCase.verifyEqual(m.distanceTravelledM, sum(hypot(diff(L.x), diff(L.y))), 'AbsTol', 1e-12);
            testCase.verifyEqual(m.durationS, L.t(end));
            testCase.verifyEqual(m.finalPositionErrorM, hypot(L.x(end) - 8, L.y(end) - 8), 'AbsTol', 1e-12);
            testCase.verifyTrue(all(isfinite([L.x; L.y; L.theta; L.v; L.omega; L.crossTrack; L.headingError])));
            testCase.verifyTrue(all(abs(L.headingError) <= pi + 1e-12));
        end
    end
end
