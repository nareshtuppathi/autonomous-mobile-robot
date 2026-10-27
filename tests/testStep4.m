classdef testStep4 < matlab.unittest.TestCase
%TESTSTEP4 Tests for Step 4: the Simulink model, its block functions and the MATLAB-versus-Simulink comparison.
%   Run with: runtests('tests/testStep4.m')   (takes about two minutes: it runs the model several times)
%   The tests separate three things: the model can be BUILT, it can be SIMULATED, and its results
%   agree NUMERICALLY with the MATLAB baseline within step4Tolerances.

    properties (Access = private)
        scn
        tol
        simM
        simS
    end

    methods (TestClassSetup)
        function buildModelAndRunNominalScenario(testCase)
            srcDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(srcDir, 'IncludeSubfolders', true));
            buildStep4Model();
            testCase.addTeardown(@closeModel);
            testCase.scn  = buildStep3Scenario();
            testCase.tol = step4Tolerances();
            testCase.simM = simulateUnicycleTracking(testCase.scn.ref, testCase.scn.startPose, testCase.scn.tcfg, testCase.scn.env);
            testCase.simS = runSimulinkTracking(testCase.scn.ref, testCase.scn.startPose, testCase.scn.tcfg, testCase.scn.env);
        end
    end

    methods (Test)
        %% BUILD
        function modelFileIsBuiltWithTheExpectedStructure(testCase)
            modelFile = buildStep4Model();
            testCase.verifyTrue(isfile(modelFile));
            [~, name] = fileparts(modelFile);
            load_system(modelFile);
            blocks = find_system(name, 'SearchDepth', 1, 'Type', 'Block');
            blocks = cellfun(@(p) erase(p, [name '/']), blocks(~strcmp(blocks, name)), 'UniformOutput', false);
            expected = {'PoseState', 'ProgressState', 'ActuatorState', 'Environment_Clearance', 'Controller_PurePursuit', ...
                'Supervisor_GoalStop', 'Actuator_Limits', 'Plant_UnicycleKinematics', 'Logging', 'sl_log', 'StopOnTermination'};
            testCase.verifyTrue(all(ismember(expected, blocks)));
            testCase.verifyEqual(get_param(name, 'Solver'), 'FixedStepDiscrete');
            testCase.verifyEqual(get_param(name, 'FixedStep'), 's4dt');
            testCase.verifyEqual(get_param(name, 'AlgebraicLoopMsg'), 'error');
        end

        %% SIMULATION
        function nominalRunTerminatesCleanlyWithoutWarnings(testCase)
            testCase.verifyEqual(testCase.simS.terminationReason, 'goal_reached');
            testCase.verifyEmpty(testCase.simS.warnings.message);
            dt = testCase.scn.tcfg.sim.timeStep;
            testCase.verifyLessThan(max(abs(diff(testCase.simS.log.t) - dt)), 1e-12);
            testCase.verifyEqual(testCase.simS.log.t(1), 0);
        end

        %% NUMERICAL VALIDATION
        function nominalScenarioMatchesMatlabWithinTolerance(testCase)
            c = compareTrackingRuns(testCase.simM, testCase.simS, testCase.tol, testCase.scn.ref, testCase.scn.tcfg);
            testCase.verifyTrue(c.sameSteps);
            testCase.verifyTrue(c.sameReason);
            testCase.verifyTrue(c.sameBrakingOnset);
            testCase.verifyTrue(c.sameRotating);
            testCase.verifyTrue(all(c.table.Pass), 'a logged signal differs by more than its tolerance');
            testCase.verifyLessThanOrEqual(c.maxAbsDiffOverall, 1e-9);
        end

        function summaryMetricsMatchMatlab(testCase)
            sc = testCase.scn;
            mM = computeTrackingMetrics(testCase.simM, sc.ref, sc.env, sc.goalWorld, sc.tcfg);
            mS = computeTrackingMetrics(testCase.simS, sc.ref, sc.env, sc.goalWorld, sc.tcfg);
            numeric = {'durationS', 'distanceTravelledM', 'finalPositionErrorM', 'maxCrossTrackM', 'rmsCrossTrackM', ...
                'maxHeadingErrorRad', 'maxLinearSpeedMps', 'maxAngularSpeedRadps', 'maxLinearAccelMps2', ...
                'maxAngularAccelRadps2', 'finalSpeedMps', 'minClearanceM'};
            for k = 1:numel(numeric)
                testCase.verifyEqual(mS.(numeric{k}), mM.(numeric{k}), 'AbsTol', testCase.tol.metric, numeric{k});
            end
            testCase.verifyEqual(mS.goalReached, mM.goalReached);
            testCase.verifyEqual(mS.collision, mM.collision);
            testCase.verifyEqual(mS.clearanceViolation, mM.clearanceViolation);
            testCase.verifyTrue(mS.overallPass);
        end

        function simulinkGoalStopBrakesWithinTheAccelerationLimit(testCase)
            sc = testCase.scn;
            m = computeTrackingMetrics(testCase.simS, sc.ref, sc.env, sc.goalWorld, sc.tcfg);
            L = testCase.simS.log;
            testCase.verifyEqual(m.finalSpeedMps, 0);
            testCase.verifyLessThanOrEqual(m.maxLinearAccelMps2, sc.tcfg.robot.maxLinearAccel + 1e-9);
            testCase.verifyLessThanOrEqual(m.maxAngularAccelRadps2, sc.tcfg.robot.maxAngularAccel + 1e-9);
            onset = find(L.braking > 0.5, 1, 'first');
            testCase.verifyNotEmpty(onset);
            testCase.verifyGreaterThan(L.v(onset), 0.1);                     % it was still moving when braking began
            testCase.verifyLessThanOrEqual(abs(L.v(end) - L.v(end - 1)) / sc.tcfg.sim.timeStep, sc.tcfg.robot.maxLinearAccel + 1e-9);
            testCase.verifyLessThanOrEqual(m.finalPositionErrorM, sc.tcfg.controller.goalTolerance);
        end

        function simulinkTrajectoryIsCheckedAgainstTheRealMap(testCase)
            sc = testCase.scn;
            [clr, inMap] = clearanceAtPoints(sc.env, [testCase.simS.log.x, testCase.simS.log.y]);
            testCase.verifyTrue(all(inMap));
            testCase.verifyGreaterThanOrEqual(min(clr), sc.mapCfg.inflationRadiusM - 1e-9);
            testCase.verifyEqual(clr, testCase.simS.log.clearanceM, 'AbsTol', 1e-12);
            testCase.verifyTrue(all(isfinite([testCase.simS.log.x; testCase.simS.log.y; testCase.simS.log.theta; testCase.simS.log.v; testCase.simS.log.omega])));
        end

        function failureAndStopScenariosMatchMatlab(testCase)
            sc = testCase.scn;
            p0 = sc.startPose;
            n = [-sin(p0(3)), cos(p0(3))];
            cand = [p0(1:2) + 1.5 * n; p0(1:2) - 1.5 * n];
            [clr, inMap] = clearanceAtPoints(sc.env, cand);
            clr(~inMap) = -1;
            [~, best] = max(clr);
            cases = {
                'timeout',           trackingConfig('TimeoutFactor', 0.2, 'TimeoutPadS', 0), p0, 'timeout'
                'stalled',           trackingConfig('MinSpeedMps', 0),                        p0, 'stalled'
                'missed goal',       trackingConfig('MaxLinearAccelMps2', 0.3),               p0, 'missed_goal'
                'diverged',          sc.tcfg,                                                 [cand(best, :), p0(3)], 'diverged'
                'collision',         sc.tcfg,                                                 [3.85, 6.0, 0], 'collision'
                'left map',          sc.tcfg,                                                 [-1, 5, 0], 'left_map'
                'numerical failure', sc.tcfg,                                                 [NaN, 5, 0], 'numerical_failure'
                'heading error 120', sc.tcfg,                                                 [p0(1:2), p0(3) + 2 * pi / 3], 'goal_reached'};
            for k = 1:size(cases, 1)
                sM = simulateUnicycleTracking(sc.ref, cases{k, 3}, cases{k, 2}, sc.env);
                sS = runSimulinkTracking(sc.ref, cases{k, 3}, cases{k, 2}, sc.env);
                c = compareTrackingRuns(sM, sS, testCase.tol, sc.ref, cases{k, 2});
                testCase.verifyEqual(sS.terminationReason, cases{k, 4}, cases{k, 1});
                testCase.verifyEqual(sM.terminationReason, cases{k, 4}, cases{k, 1});
                testCase.verifyTrue(c.allWithinTolerance, cases{k, 1});
                testCase.verifyEqual(sS.goalReached, strcmp(cases{k, 4}, 'goal_reached'), cases{k, 1});
            end
        end

        function comparisonDetectsARealDifference(testCase)
            sc = testCase.scn;
            cfgP = trackingConfig('LookaheadMinM', 0.5);
            sP = runSimulinkTracking(sc.ref, sc.startPose, cfgP, sc.env);
            c = compareTrackingRuns(testCase.simM, sP, testCase.tol, sc.ref, sc.tcfg);
            testCase.verifyFalse(c.allWithinTolerance);
            testCase.verifyGreaterThan(c.maxAbsDiffOverall, 1e-3);
        end

        function simulinkRunsAreDeterministic(testCase)
            sc = testCase.scn;
            again = runSimulinkTracking(sc.ref, sc.startPose, sc.tcfg, sc.env);
            testCase.verifyTrue(isequaln(again.log, testCase.simS.log));
        end

        function loggedPosesSatisfyTheUnicycleOde(testCase)
            r = verifyPlantAgainstOde(testCase.simS, testCase.scn.tcfg.sim.timeStep);
            testCase.verifyLessThanOrEqual(r.maxPositionErrorM, testCase.tol.plantOde);
            testCase.verifyLessThanOrEqual(r.maxHeadingErrorRad, testCase.tol.plantOde);
            testCase.verifyEqual(r.numSteps, testCase.simS.numSteps - 1);
        end

        %% Block functions and helpers (no Simulink run needed)
        function logLayoutAndReasonNamesAreConsistent(testCase)
            names = slLogLayout();
            testCase.verifyEqual(numel(unique(names)), numel(names));
            row = slLogPack([1; 2; 3], (1:16).', [4; 5], [6; 7], 8, 9, 10, 11);
            testCase.verifyEqual(numel(row), numel(names));
            testCase.verifyEqual(row(strcmp(names, 'x')), 1);
            testCase.verifyEqual(row(strcmp(names, 'sProj')), 1);
            testCase.verifyEqual(row(strcmp(names, 'kappa')), 16);
            testCase.verifyEqual(row(strcmp(names, 'vDes')), 4);
            testCase.verifyEqual(row(strcmp(names, 'v')), 6);
            testCase.verifyEqual(row(strcmp(names, 'clearanceM')), 8);
            testCase.verifyEqual(row(strcmp(names, 'reasonCode')), 11);
            reasons = slReasonNames();
            testCase.verifyEqual(numel(reasons), 8);
            for code = 1:8
                m = zeros(2, numel(names));  m(:, strcmp(names, 'reasonCode')) = [0; code];
                s = slLogToSim(m, [0; 0.02], [0 0 0], 1);
                testCase.verifyEqual(s.terminationReason, reasons{code});
            end
            m = zeros(2, numel(names));
            testCase.verifyEqual(slLogToSim(m, [0; 0.02], [0 0 0], 1).terminationReason, 'none');
        end

        function supervisorImplementsTheGoalStopLogic(testCase)
            sc = testCase.scn;
            tolG = sc.tcfg.controller.goalTolerance;
            sEnd = sc.ref.s(end);
            info = @(dist, s, cte) [s; cte; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; dist; 0];
            run = @(clr, inMap, finite, cmdFinite, have, inf_, vel) slSupervisorStep(clr, inMap, finite, cmdFinite, have, inf_, vel, sc.ref, sc.tcfg, sc.env);

            clear slSupervisorStep
            [~, code, stop] = run(1, 1, 1, 1, 1, info(5, 1, 0), [0; 0]);
            testCase.verifyEqual([code stop], [0 0]);                                         % running
            clear slSupervisorStep
            [~, code] = run(0.1, 1, 1, 1, 1, info(5, 1, 0), [0; 0]);
            testCase.verifyEqual(code, 6);                                                    % collision
            clear slSupervisorStep
            [~, code] = run(1, 0, 1, 1, 1, info(5, 1, 0), [0; 0]);
            testCase.verifyEqual(code, 7);                                                    % left map
            clear slSupervisorStep
            [~, code] = run(NaN, 1, 0, 0, 0, nan(16, 1), [0; 0]);
            testCase.verifyEqual(code, 8);                                                    % non-finite pose
            clear slSupervisorStep
            [~, code] = run(1, 1, 1, 1, 1, info(5, 1, 0.9), [0; 0]);
            testCase.verifyEqual(code, 5);                                                    % diverged

            % Goal: enter the tolerance (braking starts), then standstill inside / outside the tolerance
            clear slSupervisorStep
            [braking, code] = run(1, 1, 1, 1, 1, info(0.09, sEnd - 0.05, 0), [0.3; 0]);
            testCase.verifyEqual([braking code], [1 0]);
            [braking, code] = run(1, 1, 1, 1, 1, info(0.05, sEnd - 0.02, 0), [0.2; 0]);
            testCase.verifyEqual([braking code], [1 0]);                                      % still braking
            [~, code, stop] = run(1, 1, 1, 1, 1, info(0.01, sEnd, 0), [0; 0]);
            testCase.verifyEqual([code stop], [1 1]);                                         % goal_reached
            clear slSupervisorStep
            run(1, 1, 1, 1, 1, info(tolG - 0.01, sEnd - 0.05, 0), [0.3; 0]);
            [~, code] = run(1, 1, 1, 1, 1, info(tolG + 0.1, sEnd, 0), [0; 0]);
            testCase.verifyEqual(code, 2);                                                    % missed_goal
            clear slSupervisorStep
        end

        function supervisorDetectsTimeoutAndStall(testCase)
            sc = testCase.scn;
            cfg = trackingConfig('TimeoutFactor', 0.2, 'TimeoutPadS', 0);
            info = [0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 5; 0];
            tMaxSteps = ceil(0.2 * sc.ref.duration / cfg.sim.timeStep) + 1;
            clear slSupervisorStep
            code = 0;  k = 0;
            while code == 0 && k < tMaxSteps + 5
                k = k + 1;
                info(1) = 0.5 + 0.01 * k;                                                     % progressing, so not stalled
                [~, code] = slSupervisorStep(1, 1, 1, 1, 1, info, [0.3; 0], sc.ref, cfg, sc.env);
            end
            testCase.verifyEqual(code, 3);
            testCase.verifyEqual(k, tMaxSteps);                                              % first step with t >= tMax
            clear slSupervisorStep
            stallSteps = round(sc.tcfg.sim.stallWindow / sc.tcfg.sim.timeStep);
            code = 0;  k = 0;
            info(1) = 0.5;                                                                    % no progress at all
            while code == 0 && k < stallSteps + 5
                k = k + 1;
                [~, code] = slSupervisorStep(1, 1, 1, 1, 1, info, [0.3; 0], sc.ref, sc.tcfg, sc.env);
            end
            testCase.verifyEqual(code, 4);
            testCase.verifyEqual(k, stallSteps + 1);
            clear slSupervisorStep
        end

        function actuatorBlockAppliesBrakingAndLimits(testCase)
            cfg = testCase.scn.tcfg;
            dt = cfg.sim.timeStep;
            [vel, logged] = slActuatorStep([0.6; 0], 1, 0, [0.6; 0], cfg);                   % braking: command forced to zero
            testCase.verifyEqual(vel(1), 0.6 - cfg.robot.maxLinearAccel * dt, 'AbsTol', 1e-15);
            testCase.verifyEqual(logged, vel);
            [vel, logged] = slActuatorStep([0.6; 0], 1, 1, [0.6; 0], cfg);                   % final sample: previous values logged
            testCase.verifyEqual(logged, [0.6; 0]);
            testCase.verifyEqual(vel(1), 0.6 - cfg.robot.maxLinearAccel * dt, 'AbsTol', 1e-15);
            vel = slActuatorStep([0.6; 5], 0, 0, [0; 0], cfg);                               % acceleration limits from rest
            testCase.verifyEqual(vel(1), cfg.robot.maxLinearAccel * dt, 'AbsTol', 1e-15);
            testCase.verifyEqual(vel(2), min(cfg.robot.maxAngularAccel * dt, cfg.robot.maxAngularSpeed), 'AbsTol', 1e-15);
        end

        function plantAndClearanceBlocksMatchTheStep3Functions(testCase)
            sc = testCase.scn;
            pose = [3; 4; 0.7];
            testCase.verifyEqual(slUnicycleStep(pose, [0.4; 0.3], sc.tcfg), stepUnicycle(pose.', 0.4, 0.3, sc.tcfg.sim.timeStep).', 'AbsTol', 1e-15);
            [c, inMap, finite] = slClearanceStep([8; 6], sc.env);
            [c0, in0] = clearanceAtPoints(sc.env, [8 6]);
            testCase.verifyEqual([c, inMap, finite], [c0, double(in0), 1]);
            [c, ~, finite] = slClearanceStep([NaN; 6], sc.env);
            testCase.verifyTrue(isnan(c));
            testCase.verifyEqual(finite, 0);
            [cmd, info, sNext, have, cmdFinite] = slPurePursuitStep([NaN; 1; 0], 0.3, [0; 0], sc.ref, sc.tcfg);
            testCase.verifyTrue(all(isnan(cmd)) && all(isnan(info)));
            testCase.verifyEqual([sNext have cmdFinite], [0.3 0 0]);
        end

        function compareTrackingRunsFlagsDifferencesAndHandlesWrapping(testCase)
            a = testCase.simM;
            b = a;  b.log.x(10) = b.log.x(10) + 1e-6;
            c = compareTrackingRuns(a, b, testCase.tol, testCase.scn.ref, testCase.scn.tcfg);
            testCase.verifyFalse(c.table.Pass(strcmp(c.table.Field, 'x')));
            testCase.verifyFalse(c.allWithinTolerance);
            b = a;  b.log.theta(5) = a.log.theta(5);  a2 = a;  a2.log.theta(5) = pi - 1e-12;  b.log.theta(5) = -pi + 1e-12;
            c = compareTrackingRuns(a2, b, testCase.tol, testCase.scn.ref, testCase.scn.tcfg);
            testCase.verifyTrue(c.table.Pass(strcmp(c.table.Field, 'theta')));              % 2e-12 after wrapping, not 2*pi
            b = a;  b.log.y(7) = NaN;
            c = compareTrackingRuns(a, b, testCase.tol, testCase.scn.ref, testCase.scn.tcfg);
            testCase.verifyEqual(c.table.MaxAbsDiff(strcmp(c.table.Field, 'y')), Inf);
            b = a;  b.numSteps = a.numSteps - 1;  b.terminationReason = 'timeout';
            c = compareTrackingRuns(a, b, testCase.tol, testCase.scn.ref, testCase.scn.tcfg);
            testCase.verifyFalse(c.sameSteps);
            testCase.verifyFalse(c.sameReason);
        end

        function headingErrorAnalysisClassifiesCauses(testCase)
            sc = testCase.scn;
            r = analyzeHeadingErrorJumps(testCase.simM, sc.ref, sc.tcfg);
            testCase.verifyEqual(r.events.Step.', [261 361 1011]);
            testCase.verifyTrue(all(strcmp(r.events.Cause, 'projection_jump')));
            testCase.verifyEqual(r.summary.numUnexplained, 0);
            testCase.verifyTrue(r.summary.wrapNeverActive);
            testCase.verifyLessThan(r.summary.maxStepCounterfactualRad, r.thresholdRad);
            testCase.verifyEqual(r.events.PredictedFlipM, r.events.DeltaProgressM, 'RelTol', 0.2);
            testCase.verifyLessThan(r.summary.maxRefHeadingStepPerSampleRad, 0.2);          % reference heading is continuous
            % Synthetic wrap: the true error passes through pi, so the wrapped error steps by about 2*pi
            ref = generateReferenceTrajectory([0 0; 10 0], sc.tcfg);
            n = 40;
            theta = linspace(-3.0, -3.3, n).';
            L.t = (0:n - 1).' * 0.02;  L.theta = wrapAngle(theta);  L.sProj = ones(n, 1);  L.thetaRef = zeros(n, 1);
            L.headingError = wrapAngle(0 - theta);  L.crossTrack = zeros(n, 1);  L.rotating = zeros(n, 1);
            L.v = zeros(n, 1);  L.omega = zeros(n, 1);  L.omegaDes = zeros(n, 1);
            s.log = L;
            rw = analyzeHeadingErrorJumps(s, ref, sc.tcfg);
            testCase.verifyEqual(height(rw.events), 1);
            testCase.verifyEqual(rw.events.Cause{1}, 'angle_wrap');
            testCase.verifyFalse(rw.summary.wrapNeverActive);
        end

        function tolerancesAreExplicitAndPositive(testCase)
            t = step4Tolerances();
            f = fieldnames(t);
            testCase.verifyEqual(sort(f).', sort({'lengthM'; 'angleRad'; 'velocity'; 'metric'; 'plantOde'}).');
            for k = 1:numel(f)
                testCase.verifyGreaterThan(t.(f{k}), 0);
                testCase.verifyLessThanOrEqual(t.(f{k}), 1e-6);
            end
        end
    end
end

function closeModel()
    if bdIsLoaded('amr_step4_tracking')
        close_system('amr_step4_tracking', 0);
    end
end
