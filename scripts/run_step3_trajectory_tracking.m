%% run_step3_trajectory_tracking.m
% Step 3 - reference trajectory generation and Pure Pursuit path tracking.
%
% Takes the Step 2 smoothed, collision-checked path, builds a time-parameterised
% reference, and tracks it with a Pure Pursuit controller in a kinematic unicycle
% simulation. The simulated robot path is checked against the ORIGINAL occupied
% map and the required clearance. Everything is deterministic (fixed map, no random
% numbers). This is a kinematic simulation, not a dynamic or hardware validation.

clear; clc; close all;

scriptDir   = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
addpath(genpath(fullfile(projectRoot, 'src')));

figDir  = fullfile(projectRoot, 'results', 'figures');
trajCsv = fullfile(projectRoot, 'results', 'step3_trajectory_log.csv');
metCsv  = fullfile(projectRoot, 'results', 'step3_metrics.csv');
if ~exist(figDir, 'dir'), mkdir(figDir); end
checks = struct('name', {}, 'pass', {});

%% 1. Reuse Step 1 and Step 2: map, inflation, A*, smoothing, validation
mapCfg = robotConfig();
[grid, startCell, goalCell] = createOccupancyMap('default');
gridSize = size(grid);
cspace = planInConfigurationSpace(grid, startCell, goalCell, mapCfg);
smooth = smoothPath(cspace.pathCells, cspace.inflatedGrid, grid, mapCfg);
vStep2 = validatePathClearance(grid, cspace.inflatedGrid, smooth.pathCells, startCell, goalCell, mapCfg);

fprintf('=== Step 3 inputs (from Steps 1 and 2) ===\n');
fprintf('Map: %d x %d cells at %.2f m/cell; robot radius %.2f m + margin %.2f m = %.2f m required clearance\n', ...
    gridSize(1), gridSize(2), mapCfg.cellSizeM, mapCfg.robotRadiusM, mapCfg.safetyMarginM, mapCfg.inflationRadiusM);
fprintf('Step 2 smoothed path: %d waypoints, %.4f m, min clearance %.4f m, validation pass = %d\n', ...
    vStep2.numWaypoints, vStep2.lengthM, vStep2.minClearanceM, vStep2.isValid);
checks = addCheck(checks, 'Step 2 smoothed path passes its own validation', vStep2.isValid);

pathWorld = gridCoordsToWorld(smooth.pathCells, gridSize, mapCfg.cellSizeM);   % [x y] in metres
startWorld = gridCoordsToWorld(startCell, gridSize, mapCfg.cellSizeM);
goalWorld  = gridCoordsToWorld(goalCell, gridSize, mapCfg.cellSizeM);
checks = addCheck(checks, 'World path starts at the start cell centre and ends at the goal cell centre', ...
    norm(pathWorld(1, :) - startWorld) < 1e-12 && norm(pathWorld(end, :) - goalWorld) < 1e-12);

%% 2. Tracking configuration and reference trajectory
tcfg = trackingConfig();
fprintf('\n=== Tracking parameters ===\n');
fprintf('Robot  : v <= %.2f m/s, omega <= %.2f rad/s, accel <= %.2f m/s^2, yaw accel <= %.2f rad/s^2, track %.2f m, wheel <= %.2f m/s\n', ...
    tcfg.robot.maxLinearSpeed, tcfg.robot.maxAngularSpeed, tcfg.robot.maxLinearAccel, tcfg.robot.maxAngularAccel, tcfg.robot.trackWidth, tcfg.robot.maxWheelSpeed);
fprintf('Reference: lateral accel limit %.2f m/s^2, min turn speed %.2f m/s, sample spacing %.2f m, corner blend %.2f m\n', ...
    tcfg.reference.maxLateralAccel, tcfg.reference.minTurnSpeed, tcfg.reference.sampleSpacing, tcfg.reference.cornerBlend);
fprintf('Pure Pursuit: lookahead %.2f + %.2f*v m (clipped to [%.2f, %.2f]), rotate in place above %.0f deg, min speed %.2f m/s, goal tolerance %.2f m\n', ...
    tcfg.controller.lookaheadMin, tcfg.controller.lookaheadGain, tcfg.controller.lookaheadMin, tcfg.controller.lookaheadMax, ...
    rad2deg(tcfg.controller.rotateThreshold), tcfg.controller.minSpeed, tcfg.controller.goalTolerance);
fprintf('Simulation: dt %.3f s, timeout %.1f x reference duration + %.0f s, divergence at %.2f m cross-track\n', ...
    tcfg.sim.timeStep, tcfg.sim.timeoutFactor, tcfg.sim.timeoutPad, tcfg.sim.maxCrossTrack);

ref = generateReferenceTrajectory(pathWorld, tcfg);
limits = checkReferenceLimits(ref, tcfg);
env = makeClearanceEnvironment(grid, mapCfg);
[refClr, refInMap] = clearanceAtPoints(env, [ref.x, ref.y]);
waypointGap = zeros(size(pathWorld, 1), 1);
for k = 1:size(pathWorld, 1)
    waypointGap(k) = min(hypot(ref.x - pathWorld(k, 1), ref.y - pathWorld(k, 2)));
end

fprintf('\n=== Reference trajectory ===\n');
fprintf('Samples %d | length %.4f m | duration %.2f s | mean speed %.3f m/s\n', numel(ref.s), ref.length, ref.duration, ref.length / ref.duration);
fprintf('Max speed %.3f m/s | max |curvature| %.2f 1/m | max |omega| %.3f rad/s | max |accel| %.3f m/s^2\n', ...
    limits.maxSpeed, max(abs(ref.kappa)), limits.maxOmega, limits.maxAccel);
fprintf('Max wheel speed %.3f m/s | max lateral accel %.3f m/s^2 (limit %.2f; the speed floor can exceed it)\n', ...
    limits.maxWheelSpeed, limits.maxLateralAccel, tcfg.reference.maxLateralAccel);
fprintf('Reference min clearance to real obstacles: %.4f m\n', min(refClr));
checks = addCheck(checks, 'Reference respects speed, acceleration, yaw-rate and wheel-speed limits, starts and ends at rest', limits.passesAll);
checks = addCheck(checks, 'Reference passes exactly through every smoothed waypoint', all(waypointGap < 1e-9));
checks = addCheck(checks, 'Reference length equals the polyline length', abs(ref.length - computePathLength(pathWorld)) < 1e-9);
checks = addCheck(checks, 'Reference keeps the required clearance from real obstacles', all(refInMap) && min(refClr) >= mapCfg.inflationRadiusM - 1e-9);

%% 3. Scenario A: start at the Step 2 start cell, heading along the first path segment
startPoseA = [startWorld, ref.theta(1)];
simA = simulateUnicycleTracking(ref, startPoseA, tcfg, env);
mA = computeTrackingMetrics(simA, ref, env, goalWorld, tcfg);
printMetrics('Scenario A (nominal start)', mA, mapCfg, ref);

L = simA.log;
% Acceleration is measured over ALL logged samples (including the first step from rest and the
% last sample). The only allowance is a numerical tolerance for floating-point rounding.
accelTol = 1e-9;   % m/s^2 and rad/s^2; the applied change per step is accel*dt up to rounding of order 1e-14
fprintf('Applied acceleration: max %.6f m/s^2 (limit %.2f), max yaw acceleration %.6f rad/s^2 (limit %.2f), tolerance %g\n', ...
    mA.maxLinearAccelMps2, tcfg.robot.maxLinearAccel, mA.maxAngularAccelRadps2, tcfg.robot.maxAngularAccel, accelTol);
fprintf('Final speed %.3g m/s (the robot braked to a standstill)\n', mA.finalSpeedMps);
budget = min(refClr) - mapCfg.inflationRadiusM;
fprintf('Cross-track budget that would GUARANTEE the required clearance (reference min clearance - required) = %.3f m.\n', budget);
fprintf('  Max cross-track was %.3f m; the exact clearance check above is what decides pass/fail.\n', mA.maxCrossTrackM);

checks = addCheck(checks, 'A: goal reached', mA.goalReached);
checks = addCheck(checks, 'A: final position error within the goal tolerance', mA.finalPositionErrorM <= tcfg.controller.goalTolerance + 1e-9);
checks = addCheck(checks, 'A: no collision with the original occupied map', ~mA.collision);
checks = addCheck(checks, 'A: required clearance (radius + margin) kept at every step', ~mA.clearanceViolation);
checks = addCheck(checks, 'A: no NaN or Inf in any logged signal', mA.allFinite);
checks = addCheck(checks, 'A: linear, angular and wheel speed limits respected', ...
    mA.maxLinearSpeedMps <= tcfg.robot.maxLinearSpeed + 1e-9 && mA.maxAngularSpeedRadps <= tcfg.robot.maxAngularSpeed + 1e-9 && ...
    mA.maxWheelSpeedMps <= tcfg.robot.maxWheelSpeed + 1e-9);
checks = addCheck(checks, 'A: acceleration limits respected over all samples, including the final stop', ...
    mA.maxLinearAccelMps2 <= tcfg.robot.maxLinearAccel + accelTol && mA.maxAngularAccelRadps2 <= tcfg.robot.maxAngularAccel + accelTol);
checks = addCheck(checks, 'A: robot ends at rest inside the goal tolerance (reason goal_reached)', ...
    strcmp(simA.terminationReason, 'goal_reached') && mA.finalSpeedMps < 1e-9);
checks = addCheck(checks, 'A: finished before the timeout', L.t(end) < simA.timeoutS);

simA2 = simulateUnicycleTracking(ref, startPoseA, tcfg, env);
checks = addCheck(checks, 'A: repeated run is bit-for-bit identical (deterministic)', isequaln(simA.log, simA2.log));

%% 4. Scenario B: 120 degree initial heading error (robot starts facing away from the path)
startPoseB = [startWorld, wrapAngle(ref.theta(1) + deg2rad(120))];
simB = simulateUnicycleTracking(ref, startPoseB, tcfg, env);
mB = computeTrackingMetrics(simB, ref, env, goalWorld, tcfg);
printMetrics('Scenario B (120 deg initial heading error)', mB, mapCfg, ref);
checks = addCheck(checks, 'B: goal reached from a 120 deg initial heading error', mB.goalReached);
checks = addCheck(checks, 'B: no collision and required clearance kept', ~mB.collision && ~mB.clearanceViolation);
checks = addCheck(checks, 'B: no NaN or Inf in any logged signal', mB.allFinite);
checks = addCheck(checks, 'B: acceleration limits respected over all samples', ...
    mB.maxLinearAccelMps2 <= tcfg.robot.maxLinearAccel + accelTol && mB.maxAngularAccelRadps2 <= tcfg.robot.maxAngularAccel + accelTol);
checks = addCheck(checks, 'B: robot ends at rest inside the goal tolerance', mB.finalSpeedMps < 1e-9 && mB.finalPositionErrorM <= tcfg.controller.goalTolerance + 1e-9);
checks = addCheck(checks, 'B: initial heading error is recovered (first logged error is 120 deg)', ...
    abs(rad2deg(simB.log.headingError(1)) - 120) < 1e-6 || abs(rad2deg(simB.log.headingError(1)) + 120) < 1e-6);

%% 5. Failure handling (each run must end safely with the documented reason, never as success)
fprintf('\n=== Failure handling ===\n');
% Timeout: only 20%% of the reference duration is allowed
tcfgTimeout = trackingConfig('TimeoutFactor', 0.2, 'TimeoutPadS', 0);
simT = simulateUnicycleTracking(ref, startPoseA, tcfgTimeout, env);
fprintf('Timeout case   : reason = %s, goal reached = %d\n', simT.terminationReason, simT.goalReached);
checks = addCheck(checks, 'Timeout is detected and is not reported as success', strcmp(simT.terminationReason, 'timeout') && ~simT.goalReached);

% Missed goal: the reference decelerates at 0.5 m/s^2 but this robot can only brake at 0.3 m/s^2,
% so it enters the goal tolerance too fast, stops past it and must NOT be reported as success
tcfgWeakBrake = trackingConfig('MaxLinearAccel', 0.3);
simM = simulateUnicycleTracking(ref, startPoseA, tcfgWeakBrake, env);
mM = computeTrackingMetrics(simM, ref, env, goalWorld, tcfgWeakBrake);
fprintf('Missed-goal case: reason = %s, final error %.3f m (tolerance %.2f m), final speed %.3g m/s, min clearance %.3f m\n', ...
    simM.terminationReason, mM.finalPositionErrorM, tcfg.controller.goalTolerance, mM.finalSpeedMps, mM.minClearanceM);
checks = addCheck(checks, 'Stopping outside the goal tolerance is reported as missed_goal, not success', ...
    strcmp(simM.terminationReason, 'missed_goal') && ~simM.goalReached && mM.finalPositionErrorM > tcfg.controller.goalTolerance && mM.finalSpeedMps < 1e-9);

% Stall: with no creep speed the robot cannot leave the zero-speed start of the reference
tcfgStall = trackingConfig('MinSpeedMps', 0);
simS = simulateUnicycleTracking(ref, startPoseA, tcfgStall, env);
fprintf('Stall case     : reason = %s after %.1f s\n', simS.terminationReason, simS.log.t(end));
checks = addCheck(checks, 'Stalled robot is detected', strcmp(simS.terminationReason, 'stalled') && ~simS.goalReached);

% Divergence: start 1 m away from the path
offPose = [startWorld + [0, 1.0], ref.theta(1)];
simD = simulateUnicycleTracking(ref, offPose, tcfg, env);
fprintf('Divergence case: reason = %s\n', simD.terminationReason);
checks = addCheck(checks, 'Large cross-track error is detected as divergence', strcmp(simD.terminationReason, 'diverged') && ~simD.goalReached);

% Collision: start 0.1 m from a wall (inside the robot radius)
wallPose = [mapCfg.cellSizeM * (15 - 0.5) - 0.5 * mapCfg.cellSizeM - 0.10, (gridSize(1) - 10 + 0.5) * mapCfg.cellSizeM, 0];
simC = simulateUnicycleTracking(ref, wallPose, tcfg, env);
fprintf('Collision case : reason = %s\n', simC.terminationReason);
checks = addCheck(checks, 'Starting inside the robot radius of an obstacle is detected as a collision', strcmp(simC.terminationReason, 'collision') && ~simC.goalReached);

%% 6. Outputs: figures, CSV logs
plotTrackingResults(grid, cspace.inflatedGrid, mapCfg, startCell, goalCell, ref, simA, tcfg, fullfile(figDir, 'step3_'), true);
plotTrackingResults(grid, cspace.inflatedGrid, mapCfg, startCell, goalCell, ref, simB, tcfg, fullfile(figDir, 'step3_B_'), false);

T = struct2table(rmfield(simA.log, {'vDes', 'omegaDes', 'tRef', 'xLook', 'yLook', 'alpha', 'distToGoal'}));
writetable(T, trajCsv);
names = {'A nominal start'; 'B 120 deg initial heading error'};
mm = [mA; mB];
M = table(names, {mm.terminationReason}.', [mm.goalReached].', [mm.durationS].', [mm.distanceTravelledM].', [mm.finalPositionErrorM].', ...
    [mm.maxCrossTrackM].', [mm.rmsCrossTrackM].', rad2deg([mm.maxHeadingErrorRad]).', [mm.maxLinearSpeedMps].', [mm.maxAngularSpeedRadps].', ...
    [mm.minClearanceM].', [mm.collision].', [mm.clearanceViolation].', [mm.maxLinearAccelMps2].', [mm.maxAngularAccelRadps2].', [mm.finalSpeedMps].', ...
    'VariableNames', {'Scenario', 'Termination', 'GoalReached', 'DurationS', 'DistanceM', 'FinalErrorM', 'MaxCrossTrackM', 'RmsCrossTrackM', ...
    'MaxHeadingErrorDeg', 'MaxLinearSpeedMps', 'MaxAngularSpeedRadps', 'MinClearanceM', 'Collision', 'ClearanceViolation', 'MaxLinearAccelMps2', 'MaxAngularAccelRadps2', 'FinalSpeedMps'});
writetable(M, metCsv);
fprintf('\n=== Metrics table (saved to results/step3_metrics.csv) ===\n');
disp(M);

%% 7. Summary
fprintf('\n=== Check summary ===\n');
for k = 1:numel(checks)
    fprintf('[%s] %s\n', passText(checks(k).pass), checks(k).name);
end
fprintf('\n%d of %d checks passed.\n', nnz([checks.pass]), numel(checks));
if ~all([checks.pass])
    error('run_step3:checksFailed', 'One or more checks failed.');
end

%% Local helpers
function checks = addCheck(checks, name, pass)
    checks(end + 1) = struct('name', name, 'pass', logical(pass));
end

function s = passText(pass)
    if pass, s = 'PASS'; else, s = 'FAIL'; end
end

function printMetrics(label, m, mapCfg, ref)
    fprintf('\n=== %s ===\n', label);
    fprintf('Termination reason          : %s\n', m.terminationReason);
    fprintf('Goal reached                : %d\n', m.goalReached);
    fprintf('Duration                    : %.2f s (reference %.2f s)\n', m.durationS, m.referenceDurationS);
    fprintf('Distance travelled          : %.4f m (reference path %.4f m)\n', m.distanceTravelledM, m.referenceLengthM);
    fprintf('Final position error        : %.4f m\n', m.finalPositionErrorM);
    fprintf('Cross-track error  max / RMS: %.4f m / %.4f m  (signed distance to the reference polyline)\n', m.maxCrossTrackM, m.rmsCrossTrackM);
    fprintf('Max heading error (wrapped) : %.2f deg  (%.4f rad); after first within 30 deg (t = %.2f s): %.2f deg\n', ...
        rad2deg(m.maxHeadingErrorRad), m.maxHeadingErrorRad, m.timeToAlignS, rad2deg(m.maxHeadingErrorAfterAlignRad));
    fprintf('Max applied accel          : %.4f m/s^2 linear, %.4f rad/s^2 angular; final speed %.3g m/s\n', ...
        m.maxLinearAccelMps2, m.maxAngularAccelRadps2, m.finalSpeedMps);
    fprintf('Max linear / angular speed  : %.3f m/s / %.3f rad/s (max wheel %.3f m/s)\n', m.maxLinearSpeedMps, m.maxAngularSpeedRadps, m.maxWheelSpeedMps);
    fprintf('Min clearance (real map)    : %.4f m (collision below %.2f m, required %.2f m)\n', m.minClearanceM, mapCfg.robotRadiusM, mapCfg.inflationRadiusM);
    fprintf('Collision                   : %d\n', m.collision);
    fprintf('Clearance violation         : %d (%d samples below the required clearance)\n', m.clearanceViolation, m.numViolatingSamples);
    fprintf('All logged values finite    : %d\n', m.allFinite);
    fprintf('Overall pass                : %d\n', m.overallPass);
    assert(ref.duration > 0);
end
