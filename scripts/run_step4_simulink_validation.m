%% run_step4_simulink_validation.m
% Step 4 - Simulink version of the Step 3 simulation and MATLAB-versus-Simulink validation.
%
% Three different things are reported separately:
%   BUILD       the .slx file is created and the diagram compiles
%   SIMULATION  the model runs to a termination without errors or warnings
%   VALIDATION  the Simulink results agree numerically with the MATLAB baseline,
%               within tolerances fixed in advance (step4Tolerances)
% Everything is deterministic. This is a kinematic simulation; no dynamic or hardware validation.

clear; clc; close all;

scriptDir   = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
addpath(genpath(fullfile(projectRoot, 'src')));

figDir = fullfile(projectRoot, 'results', 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end
metricsCsv  = fullfile(projectRoot, 'results', 'step4_comparison.csv');
scenarioCsv = fullfile(projectRoot, 'results', 'step4_scenarios.csv');
simLogCsv   = fullfile(projectRoot, 'results', 'step4_simulink_log.csv');
eventsCsv   = fullfile(projectRoot, 'results', 'step4_heading_error_events.csv');
checks = struct('name', {}, 'pass', {});

%% 1. BUILD
modelName = 'amr_step4_tracking';
modelFile = buildStep4Model();
sc  = buildStep3Scenario();
tol = step4Tolerances();
tcfg = sc.tcfg;
dt = tcfg.sim.timeStep;

fprintf('=== BUILD ===\n');
fprintf('Model file : %s\n', modelFile);
load_system(modelFile);
blockNames = find_system(modelName, 'SearchDepth', 1, 'Type', 'Block');
blockNames = cellfun(@(p) erase(p, [modelName '/']), blockNames(~strcmp(blockNames, modelName)), 'UniformOutput', false);
fprintf('Top-level blocks (%d): %s\n', numel(blockNames), strjoin(blockNames.', ', '));
expected = {'PoseState', 'ProgressState', 'ActuatorState', 'Environment_Clearance', 'Controller_PurePursuit', ...
    'Supervisor_GoalStop', 'Actuator_Limits', 'Plant_UnicycleKinematics', 'Logging', 'sl_log', 'StopOnTermination'};
fprintf('Solver     : %s, fixed step = %s, stop time = %s\n', get_param(modelName, 'Solver'), get_param(modelName, 'FixedStep'), get_param(modelName, 'StopTime'));
checks = addCheck(checks, '[BUILD] Model file exists', isfile(modelFile));
checks = addCheck(checks, '[BUILD] Model contains the planned state, subsystem, logging and stop blocks', all(ismember(expected, blockNames)));
checks = addCheck(checks, '[BUILD] Fixed-step discrete solver using the Step 3 time step', ...
    strcmp(get_param(modelName, 'Solver'), 'FixedStepDiscrete') && strcmp(get_param(modelName, 'FixedStep'), 's4dt'));
try
    pngFile = fullfile(figDir, 'step4_00_model_diagram.png');
    print(['-s' modelName], '-dpng', '-r150', pngFile);
    fprintf('Diagram    : %s\n', pngFile);
catch printError
    fprintf('Diagram image could not be exported: %s\n', printError.message);
end

%% 2. SIMULATION and Scenario A comparison
startPose = sc.startPose;
simM = simulateUnicycleTracking(sc.ref, startPose, tcfg, sc.env);
simS = runSimulinkTracking(sc.ref, startPose, tcfg, sc.env);
fprintf('\n=== SIMULATION (Scenario A: nominal start) ===\n');
fprintf('MATLAB   : %s after %d steps (%.2f s)\n', simM.terminationReason, simM.numSteps, simM.log.t(end));
fprintf('Simulink : %s after %d steps (%.2f s)\n', simS.terminationReason, simS.numSteps, simS.log.t(end));
checks = addCheck(checks, '[SIMULATION] Model compiles and runs to a termination without error', ~strcmp(simS.terminationReason, 'none'));
checks = addCheck(checks, '[SIMULATION] No warnings were raised during the run', isempty(simS.warnings.message));
checks = addCheck(checks, '[SIMULATION] Simulink time vector advances in steps of the control period', max(abs(diff(simS.log.t) - dt)) < 1e-12);
checks = addCheck(checks, '[SIMULATION] Scenario A reaches the goal in Simulink', simS.goalReached);

mM = computeTrackingMetrics(simM, sc.ref, sc.env, sc.goalWorld, tcfg);
mS = computeTrackingMetrics(simS, sc.ref, sc.env, sc.goalWorld, tcfg);
cmp = compareTrackingRuns(simM, simS, tol, sc.ref, tcfg);

fprintf('\n=== VALIDATION: tolerances (fixed in advance) ===\n');
fprintf('Lengths/times %.0e m|s | angles %.0e rad | velocities %.0e | scalar metrics %.0e | ODE one-step %.0e\n', ...
    tol.lengthM, tol.angleRad, tol.velocity, tol.metric, tol.plantOde);
fprintf('Integer quantities (step count, termination reason, braking onset step, rotating flag) must match exactly.\n');

fprintf('\n=== VALIDATION: sample-by-sample differences, Scenario A (MATLAB vs Simulink) ===\n');
fprintf('%-14s %14s %12s %s\n', 'Signal', 'Max |diff|', 'Tolerance', 'Pass');
for k = 1:height(cmp.table)
    fprintf('%-14s %14.3e %12.0e %s\n', cmp.table.Field{k}, cmp.table.MaxAbsDiff(k), cmp.table.Tolerance(k), passText(cmp.table.Pass(k)));
end
fprintf('Steps: MATLAB %d, Simulink %d | termination: %s / %s | braking starts at step %d / %d\n', simM.numSteps, simS.numSteps, ...
    simM.terminationReason, simS.terminationReason, cmp.brakingOnsetStepMatlab, cmp.brakingOnsetStepSimulink);
fprintf('Largest difference over all signals and samples: %.3e\n', cmp.maxAbsDiffOverall);

rows = metricRows(mM, mS, tol);
fprintf('\n=== VALIDATION: summary metrics, Scenario A ===\n');
fprintf('%-30s %14s %14s %11s %9s %s\n', 'Metric', 'MATLAB', 'Simulink', '|diff|', 'Tol', 'Pass');
for k = 1:height(rows)
    fprintf('%-30s %14.6g %14.6g %11.2e %9.0e %s\n', rows.Metric{k}, rows.Matlab(k), rows.Simulink(k), rows.AbsDiff(k), rows.Tolerance(k), passText(rows.Pass(k)));
end
fprintf('Termination reason: MATLAB %s, Simulink %s\n', mM.terminationReason, mS.terminationReason);
writetable(rows, metricsCsv);

checks = addCheck(checks, '[VALIDATION] Same number of steps', cmp.sameSteps);
checks = addCheck(checks, '[VALIDATION] Same termination reason (goal_reached)', cmp.sameReason && strcmp(simS.terminationReason, 'goal_reached'));
checks = addCheck(checks, '[VALIDATION] Braking starts at the same step in both, and it matches the Simulink braking flag', ...
    cmp.sameBrakingOnset && ~isempty(cmp.brakingOnsetStepSimulink) && cmp.brakingOnsetStepSimulink == find(simS.log.braking > 0.5, 1, 'first'));
checks = addCheck(checks, '[VALIDATION] Every logged signal agrees within tolerance (pose, reference, commands, errors, clearance)', all(cmp.table.Pass) && cmp.sameRotating);
checks = addCheck(checks, '[VALIDATION] Every summary metric agrees within tolerance', all(rows.Pass));
limitA = tcfg.robot.maxLinearAccel;  limitW = tcfg.robot.maxAngularAccel;
checks = addCheck(checks, '[VALIDATION] Simulink acceleration limits respected over all samples including the final braking', ...
    mS.maxLinearAccelMps2 <= limitA + 1e-9 && mS.maxAngularAccelRadps2 <= limitW + 1e-9);
checks = addCheck(checks, '[VALIDATION] Simulink run ends at rest inside the goal tolerance', mS.finalSpeedMps < 1e-9 && mS.finalPositionErrorM <= tcfg.controller.goalTolerance);
checks = addCheck(checks, '[VALIDATION] Simulink trajectory is collision-free and keeps the required clearance from the real map', ...
    ~mS.collision && ~mS.clearanceViolation && mS.minClearanceM >= sc.mapCfg.inflationRadiusM - 1e-9);
checks = addCheck(checks, '[VALIDATION] No NaN or Inf in the Simulink log (clearance Inf allowed only without obstacles)', mS.allFinite);

% Determinism and an independent plant check
simS2 = runSimulinkTracking(sc.ref, startPose, tcfg, sc.env);
checks = addCheck(checks, '[VALIDATION] Two Simulink runs give identical logs (deterministic)', isequaln(simS.log, simS2.log));
ode = verifyPlantAgainstOde(simS, dt);
fprintf('\nPlant check against ode45 (one step at a time, %d steps): max position error %.2e m, max heading error %.2e rad\n', ...
    ode.numSteps, ode.maxPositionErrorM, ode.maxHeadingErrorRad);
checks = addCheck(checks, '[VALIDATION] Logged Simulink poses satisfy the unicycle ODE (ode45 cross-check)', ...
    ode.maxPositionErrorM <= tol.plantOde && ode.maxHeadingErrorRad <= tol.plantOde);

% Sensitivity: the comparison must be able to see a real difference
cfgP = trackingConfig('LookaheadMinM', 0.5);
simMP = simulateUnicycleTracking(sc.ref, startPose, cfgP, sc.env);
simSP = runSimulinkTracking(sc.ref, startPose, cfgP, sc.env);
cmpEquiv = compareTrackingRuns(simMP, simSP, tol, sc.ref, cfgP);
cmpSens  = compareTrackingRuns(simM, simSP, tol, sc.ref, tcfg);
fprintf('\nSensitivity: lookahead minimum 0.30 -> 0.50 m. MATLAB vs Simulink with the same change: max diff %.2e (equivalent: %d).\n', ...
    cmpEquiv.maxAbsDiffOverall, cmpEquiv.allWithinTolerance);
fprintf('Baseline MATLAB vs changed Simulink: max diff %.3e (the comparison must flag this).\n', cmpSens.maxAbsDiffOverall);
checks = addCheck(checks, '[VALIDATION] Equivalence also holds with a changed controller parameter', cmpEquiv.allWithinTolerance);
checks = addCheck(checks, '[VALIDATION] The comparison detects a real difference (it is not vacuous)', ~cmpSens.allWithinTolerance && cmpSens.maxAbsDiffOverall > 1e-3);

%% 3. Other scenarios: initial heading error, goal-stop and failure behaviour
n = [-sin(startPose(3)), cos(startPose(3))];
cand = [startPose(1:2) + 1.5 * n; startPose(1:2) - 1.5 * n];
[candClr, candIn] = clearanceAtPoints(sc.env, cand);
candClr(~candIn) = -1;
[~, best] = max(candClr);
offsetPose = [cand(best, :), startPose(3)];
scen = { ...
    'B: 120 deg initial heading error', tcfg, [startPose(1:2), startPose(3) + 2 * pi / 3], 'goal_reached'; ...
    'timeout (time limit 0.2 x reference)', trackingConfig('TimeoutFactor', 0.2, 'TimeoutPadS', 0), startPose, 'timeout'; ...
    'missed_goal (brakes at 0.3 m/s^2)', trackingConfig('MaxLinearAccelMps2', 0.3), startPose, 'missed_goal'; ...
    'stalled (no creep speed)', trackingConfig('MinSpeedMps', 0), startPose, 'stalled'; ...
    'diverged (start 1.5 m off the path)', tcfg, offsetPose, 'diverged'; ...
    'collision (start 0.10 m from a wall)', tcfg, [3.85, 6.0, 0], 'collision'; ...
    'left_map (start outside the map)', tcfg, [-1, 5, 0], 'left_map'; ...
    'numerical_failure (NaN pose)', tcfg, [NaN, 5, 0], 'numerical_failure'};
numScen = size(scen, 1);
names = cell(numScen, 1);  expectReason = cell(numScen, 1);  reasonM = cell(numScen, 1);  reasonS = cell(numScen, 1);
stepsM = zeros(numScen, 1);  stepsS = zeros(numScen, 1);  maxDiff = zeros(numScen, 1);  pass = false(numScen, 1);
finalSpeedS = zeros(numScen, 1);  maxAccS = zeros(numScen, 1);
fprintf('\n=== VALIDATION: other scenarios (MATLAB vs Simulink) ===\n');
fprintf('%-38s %-18s %-18s %6s %6s %11s %s\n', 'Scenario', 'MATLAB', 'Simulink', 'StepsM', 'StepsS', 'Max |diff|', 'Pass');
for q = 1:numScen
    cfgQ = scen{q, 2};
    sM = simulateUnicycleTracking(sc.ref, scen{q, 3}, cfgQ, sc.env);
    sS = runSimulinkTracking(sc.ref, scen{q, 3}, cfgQ, sc.env);
    c = compareTrackingRuns(sM, sS, tol, sc.ref, cfgQ);
    names{q} = scen{q, 1};  expectReason{q} = scen{q, 4};  reasonM{q} = sM.terminationReason;  reasonS{q} = sS.terminationReason;
    stepsM(q) = sM.numSteps;  stepsS(q) = sS.numSteps;  maxDiff(q) = c.maxAbsDiffOverall;
    pass(q) = c.allWithinTolerance && strcmp(sS.terminationReason, scen{q, 4});
    finalSpeedS(q) = abs(sS.log.v(end));
    if sS.numSteps > 1, maxAccS(q) = max(abs(diff([0; sS.log.v]))) / dt; end
    fprintf('%-38s %-18s %-18s %6d %6d %11.2e %s\n', names{q}, reasonM{q}, reasonS{q}, stepsM(q), stepsS(q), maxDiff(q), passText(pass(q)));
    checks = addCheck(checks, sprintf('[VALIDATION] Scenario ''%s'': Simulink ends with %s and matches MATLAB', scen{q, 1}, scen{q, 4}), pass(q));
    if q == 1
        simSB = sS;  simMB = sM;  mSB = computeTrackingMetrics(sS, sc.ref, sc.env, sc.goalWorld, cfgQ);
    end
    if strcmp(scen{q, 4}, 'missed_goal')
        mMiss = computeTrackingMetrics(sS, sc.ref, sc.env, sc.goalWorld, cfgQ);
    end
end
T = table(names, expectReason, reasonM, reasonS, stepsM, stepsS, maxDiff, finalSpeedS, maxAccS, pass, ...
    'VariableNames', {'Scenario', 'Expected', 'MatlabReason', 'SimulinkReason', 'MatlabSteps', 'SimulinkSteps', ...
    'MaxAbsDiff', 'SimulinkFinalSpeedMps', 'SimulinkMaxLinearAccelMps2', 'Pass'});
writetable(T, scenarioCsv);
checks = addCheck(checks, '[VALIDATION] Goal-stop: missed_goal stops at rest outside the tolerance and is not reported as success', ...
    strcmp(mMiss.terminationReason, 'missed_goal') && mMiss.finalSpeedMps < 1e-9 && mMiss.finalPositionErrorM > tcfg.controller.goalTolerance && ~mMiss.overallPass);
checks = addCheck(checks, '[VALIDATION] Scenario B: Simulink ends at rest inside the tolerance, clearance kept, accelerations within limits', ...
    mSB.goalReached && mSB.finalSpeedMps < 1e-9 && ~mSB.collision && ~mSB.clearanceViolation && ...
    mSB.maxLinearAccelMps2 <= limitA + 1e-9 && mSB.maxAngularAccelRadps2 <= limitW + 1e-9);
odeB = verifyPlantAgainstOde(simSB, dt);
checks = addCheck(checks, '[VALIDATION] Scenario B poses satisfy the unicycle ODE (ode45 cross-check)', odeB.maxPositionErrorM <= tol.plantOde && odeB.maxHeadingErrorRad <= tol.plantOde);

%% 4. Heading-error jump investigation
resM = analyzeHeadingErrorJumps(simM, sc.ref, tcfg);
resS = analyzeHeadingErrorJumps(simS, sc.ref, tcfg);
resB = analyzeHeadingErrorJumps(simSB, sc.ref, tcfg);
fprintf('\n=== Heading-error jump investigation ===\n');
fprintf('Event threshold: |step of heading error| > %.4f rad per %.2f s step (largest change smooth motion can produce)\n', resS.thresholdRad, dt);
for pass_ = 1:2
    if pass_ == 1, res = resS;  label = 'Scenario A (Simulink log)'; else, res = resB;  label = 'Scenario B (Simulink log)'; end
    s = res.summary;
    fprintf('\n%s: %d events, projection jumps %d, wrapping %d, robot motion %d, controller mode switches %d, unexplained %d\n', ...
        label, s.numEvents, s.numProjectionJumps, s.numWrap, s.numRobotMotion, s.numModeSwitch, s.numUnexplained);
    for k = 1:height(res.events)
        e = res.events(k, :);
        fprintf('  step %4d (t = %6.2f s): d(error) = %+.3f rad, d(ref heading) = %+.3f, d(robot heading) = %+.3f, d(progress) = %.3f m (robot moves <= %.3f m/step), predicted flip %.3f m, corner %d -> %s\n', ...
            e.Step, e.TimeS, e.DeltaHeadingError, e.DeltaRefHeading, e.DeltaRobotHeading, e.DeltaProgressM, e.MaxTravelPerStepM, e.PredictedFlipM, e.Corner, e.Cause{1});
    end
    fprintf('  max |heading error| %.1f deg; largest step %.3f rad; with rate-limited progress %.3f rad\n', ...
        rad2deg(s.maxAbsErrorRad), s.maxStepRad, s.maxStepCounterfactualRad);
    fprintf('  reference heading: largest change per sample %.3f rad (max curvature %.2f 1/m): continuous, no jumps\n', s.maxRefHeadingStepPerSampleRad, s.maxRefCurvature);
    fprintf('  robot heading: largest change per step %.4f rad | desired omega step up to %.3f rad/s, applied omega step %.4f rad/s per step\n', ...
        s.maxRobotHeadingStepRad, s.maxDesiredOmegaStepRadps, s.maxAppliedOmegaStepRadps);
end
fprintf('\nConclusion: every abrupt step of the heading error is a jump of the projected progress sProj (the nearest point on the path flips between two segments when the robot cuts inside a corner).\n');
fprintf('The geometric prediction 2*d*tan(phi/2) matches the measured progress jumps. Angle wrapping is never active, the reference heading is continuous, and the robot heading stays smooth.\n');
fprintf('The jump also reaches the controller (the lookahead point moves, so the desired omega steps), but the actuator rate limit smooths it before it reaches the robot.\n');

eventsOut = [addvars(resS.events, repmat({'A'}, height(resS.events), 1), 'Before', 1, 'NewVariableNames', 'Scenario'); ...
             addvars(resB.events, repmat({'B'}, height(resB.events), 1), 'Before', 1, 'NewVariableNames', 'Scenario')];
writetable(eventsOut, eventsCsv);
checks = addCheck(checks, '[VALIDATION] Heading-error events are identical in the MATLAB and Simulink logs', isequal(resM.events.Step, resS.events.Step) && isequal(resM.events.Cause, resS.events.Cause));
checks = addCheck(checks, '[VALIDATION] Every heading-error event in Scenario A is explained (all are projection jumps)', resS.summary.numEvents > 0 && resS.summary.numUnexplained == 0 && resS.summary.numProjectionJumps == resS.summary.numEvents);
checks = addCheck(checks, '[VALIDATION] Angle wrapping is not the cause (never active, error stays below pi)', resS.summary.wrapNeverActive && resB.summary.numWrap == 0);
checks = addCheck(checks, '[VALIDATION] Without the progress jump the heading error is smooth (below the event threshold)', resS.summary.maxStepCounterfactualRad < resS.thresholdRad);
checks = addCheck(checks, '[VALIDATION] Geometric flip prediction is within 20% of each measured progress jump', ...
    all(abs(resS.events.PredictedFlipM - resS.events.DeltaProgressM) <= 0.2 * resS.events.DeltaProgressM));

%% 5. Figures and logs
plotStep4Comparison(sc, simM, simS, cmp, figDir);
plotHeadingErrorInvestigation(simS, tcfg, resS, 'Scenario A: why the heading error jumps', fullfile(figDir, 'step4_04_heading_error_investigation.png'));
plotHeadingErrorInvestigation(simSB, tcfg, resB, 'Scenario B (120 deg initial heading error): heading error steps', fullfile(figDir, 'step4_05_heading_error_investigation_B.png'));
writetable(struct2table(simS.log), simLogCsv);
figNames = dir(fullfile(figDir, 'step4_*.png'));
fprintf('\nFigures: %s\n', strjoin({figNames.name}, ', '));

%% 6. Summary
fprintf('\n=== Check summary ===\n');
for k = 1:numel(checks)
    fprintf('[%s] %s\n', passText(checks(k).pass), checks(k).name);
end
fprintf('\n%d of %d checks passed.\n', nnz([checks.pass]), numel(checks));
close_system(modelName, 0);
if ~all([checks.pass])
    error('run_step4:checksFailed', 'One or more checks failed.');
end

%% Local helpers
function checks = addCheck(checks, name, pass)
    checks(end + 1) = struct('name', name, 'pass', logical(pass));
end

function s = passText(pass)
    if pass, s = 'PASS'; else, s = 'FAIL'; end
end

function rows = metricRows(mM, mS, tol)
    numeric = {'durationS', 'distanceTravelledM', 'finalPositionErrorM', 'maxCrossTrackM', 'rmsCrossTrackM', ...
        'maxHeadingErrorRad', 'maxHeadingErrorAfterAlignRad', 'maxLinearSpeedMps', 'maxAngularSpeedRadps', ...
        'maxWheelSpeedMps', 'maxLinearAccelMps2', 'maxAngularAccelRadps2', 'finalSpeedMps', 'minClearanceM'};
    exact = {'numViolatingSamples', 'goalReached', 'collision', 'clearanceViolation', 'overallPass', 'allFinite'};
    names = [numeric, exact];
    a = zeros(numel(names), 1);  b = a;  t = a;
    for k = 1:numel(names)
        a(k) = double(mM.(names{k}));  b(k) = double(mS.(names{k}));
        if k <= numel(numeric), t(k) = tol.metric; else, t(k) = 0; end
    end
    d = abs(a - b);
    rows = table(names(:), a, b, d, t, d <= t, 'VariableNames', {'Metric', 'Matlab', 'Simulink', 'AbsDiff', 'Tolerance', 'Pass'});
end
