function modelFile = buildStep4Model()
%BUILDSTEP4MODEL Create simulink/amr_step4_tracking.slx, the Simulink version of the Step 3 simulation.
%   modelFile = buildStep4Model() builds the model programmatically and saves it.
%
%   Architecture (one fixed step of cfg.sim.timeStep, discrete states only)
%     State blocks (Unit Delay): PoseState [x;y;theta], ProgressState (arc length along the path),
%                                ActuatorState [v;omega] applied at the previous step
%     Environment_Clearance    clearance of the robot centre to the real obstacles, map-bounds flag
%     Controller_PurePursuit   the existing purePursuitController (no re-implementation)
%     Supervisor_GoalStop      goal braking, goal_reached/missed_goal and all failure terminations
%     Actuator_Limits          acceleration limits, then limitVelocityCommand
%     Plant_UnicycleKinematics x_dot = v cos(theta), y_dot = v sin(theta), theta_dot = omega
%     Logging                  packs every signal into one logged matrix (slLogLayout)
%     StopOnTermination        Stop Simulation block driven by the supervisor
%   The block bodies are thin calls to the functions in src/simulink, which call the same
%   Step 3 functions as the MATLAB simulation. Parameters come from workspace variables:
%   s4ref (reference), s4cfg (trackingConfig), s4env (clearance environment),
%   s4startPose ([x y theta]), s4dt (time step), s4stopTime.

    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    modelName = 'amr_step4_tracking';
    modelFile = fullfile(projectRoot, 'simulink', [modelName '.slx']);
    if ~isfolder(fileparts(modelFile)), mkdir(fileparts(modelFile)); end
    if bdIsLoaded(modelName), close_system(modelName, 0); end
    new_system(modelName);

    set_param(modelName, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', 's4dt', 'StartTime', '0', 'StopTime', 's4stopTime', ...
        'SaveTime', 'on', 'SaveOutput', 'off', 'SignalLogging', 'off', ...
        'ReturnWorkspaceOutputs', 'on', 'AlgebraicLoopMsg', 'error', ...
        'UnconnectedInputMsg', 'error', 'UnconnectedOutputMsg', 'error', 'UnconnectedLineMsg', 'error');

    % --- State blocks ---------------------------------------------------------
    addBlock(modelName, 'simulink/Discrete/Unit Delay', 'PoseState', [60 40 130 80], ...
        {'InitialCondition', 's4startPose(:)', 'SampleTime', 's4dt'});
    addBlock(modelName, 'simulink/Discrete/Unit Delay', 'ProgressState', [60 140 130 180], ...
        {'InitialCondition', '0', 'SampleTime', 's4dt'});
    addBlock(modelName, 'simulink/Discrete/Unit Delay', 'ActuatorState', [60 240 130 280], ...
        {'InitialCondition', '[0; 0]', 'SampleTime', 's4dt'});

    % --- Subsystems ------------------------------------------------------------
    env = makeSubsystem(modelName, 'Environment_Clearance', [260 20 440 100], {'pose'}, {'clearanceM', 'inMap', 'poseFinite'});
    addMatlabFunction(env, 'ClearanceStep', {'pose'}, {'clearanceM', 'inMap', 'poseFinite'}, {'s4env'}, ...
        '[clearanceM, inMap, poseFinite] = slClearanceStep(pose, s4env);');

    ctl = makeSubsystem(modelName, 'Controller_PurePursuit', [260 130 440 230], {'pose', 'progress', 'velPrev'}, ...
        {'cmdDesired', 'info', 'progressNext', 'haveInfo', 'cmdFinite'});
    addMatlabFunction(ctl, 'PurePursuitStep', {'pose', 'progress', 'velPrev'}, ...
        {'cmdDesired', 'info', 'progressNext', 'haveInfo', 'cmdFinite'}, {'s4ref', 's4cfg'}, ...
        '[cmdDesired, info, progressNext, haveInfo, cmdFinite] = slPurePursuitStep(pose, progress, velPrev, s4ref, s4cfg);');

    sup = makeSubsystem(modelName, 'Supervisor_GoalStop', [520 20 720 160], ...
        {'clearanceM', 'inMap', 'poseFinite', 'cmdFinite', 'haveInfo', 'info', 'velPrev'}, {'braking', 'reasonCode', 'stop'});
    addMatlabFunction(sup, 'SupervisorStep', {'clearanceM', 'inMap', 'poseFinite', 'cmdFinite', 'haveInfo', 'info', 'velPrev'}, ...
        {'braking', 'reasonCode', 'stop'}, {'s4ref', 's4cfg', 's4env'}, ...
        '[braking, reasonCode, stop] = slSupervisorStep(clearanceM, inMap, poseFinite, cmdFinite, haveInfo, info, velPrev, s4ref, s4cfg, s4env);');

    act = makeSubsystem(modelName, 'Actuator_Limits', [520 200 720 300], {'cmdDesired', 'braking', 'stop', 'velPrev'}, {'velApplied', 'velLogged'});
    addMatlabFunction(act, 'ActuatorStep', {'cmdDesired', 'braking', 'stop', 'velPrev'}, {'velApplied', 'velLogged'}, {'s4cfg'}, ...
        '[velApplied, velLogged] = slActuatorStep(cmdDesired, braking, stop, velPrev, s4cfg);');

    plant = makeSubsystem(modelName, 'Plant_UnicycleKinematics', [820 200 1020 280], {'pose', 'velApplied'}, {'poseNext'});
    addMatlabFunction(plant, 'UnicycleStep', {'pose', 'velApplied'}, {'poseNext'}, {'s4cfg'}, ...
        'poseNext = slUnicycleStep(pose, velApplied, s4cfg);');

    logSys = makeSubsystem(modelName, 'Logging', [820 20 1020 160], ...
        {'pose', 'info', 'cmdDesired', 'velLogged', 'clearanceM', 'inMap', 'braking', 'reasonCode'}, {'row'});
    addMatlabFunction(logSys, 'LogPack', {'pose', 'info', 'cmdDesired', 'velLogged', 'clearanceM', 'inMap', 'braking', 'reasonCode'}, ...
        {'row'}, {}, 'row = slLogPack(pose, info, cmdDesired, velLogged, clearanceM, inMap, braking, reasonCode);');
    addBlock(modelName, 'simulink/Sinks/To Workspace', 'sl_log', [1100 70 1190 110], ...
        {'VariableName', 'sl_log', 'SaveFormat', 'Array', 'MaxDataPoints', 'inf', 'SampleTime', 's4dt'});
    addBlock(modelName, 'simulink/Sinks/Stop Simulation', 'StopOnTermination', [820 120 900 150], {});

    % --- Wiring ----------------------------------------------------------------
    connect(modelName, 'PoseState/1', 'Environment_Clearance/1', 'pose');
    connect(modelName, 'PoseState/1', 'Controller_PurePursuit/1', '');
    connect(modelName, 'PoseState/1', 'Plant_UnicycleKinematics/1', '');
    connect(modelName, 'PoseState/1', 'Logging/1', '');
    connect(modelName, 'ProgressState/1', 'Controller_PurePursuit/2', 'progress');
    connect(modelName, 'ActuatorState/1', 'Controller_PurePursuit/3', 'velPrev');
    connect(modelName, 'ActuatorState/1', 'Supervisor_GoalStop/7', '');
    connect(modelName, 'ActuatorState/1', 'Actuator_Limits/4', '');
    connect(modelName, 'Environment_Clearance/1', 'Supervisor_GoalStop/1', 'clearanceM');
    connect(modelName, 'Environment_Clearance/1', 'Logging/5', '');
    connect(modelName, 'Environment_Clearance/2', 'Supervisor_GoalStop/2', 'inMap');
    connect(modelName, 'Environment_Clearance/2', 'Logging/6', '');
    connect(modelName, 'Environment_Clearance/3', 'Supervisor_GoalStop/3', 'poseFinite');
    connect(modelName, 'Controller_PurePursuit/1', 'Actuator_Limits/1', 'cmdDesired');
    connect(modelName, 'Controller_PurePursuit/1', 'Logging/3', '');
    connect(modelName, 'Controller_PurePursuit/2', 'Supervisor_GoalStop/6', 'info');
    connect(modelName, 'Controller_PurePursuit/2', 'Logging/2', '');
    connect(modelName, 'Controller_PurePursuit/3', 'ProgressState/1', 'progressNext');
    connect(modelName, 'Controller_PurePursuit/4', 'Supervisor_GoalStop/5', 'haveInfo');
    connect(modelName, 'Controller_PurePursuit/5', 'Supervisor_GoalStop/4', 'cmdFinite');
    connect(modelName, 'Supervisor_GoalStop/1', 'Actuator_Limits/2', 'braking');
    connect(modelName, 'Supervisor_GoalStop/1', 'Logging/7', '');
    connect(modelName, 'Supervisor_GoalStop/2', 'Logging/8', 'reasonCode');
    connect(modelName, 'Supervisor_GoalStop/3', 'Actuator_Limits/3', 'stop');
    connect(modelName, 'Supervisor_GoalStop/3', 'StopOnTermination/1', '');
    connect(modelName, 'Actuator_Limits/1', 'Plant_UnicycleKinematics/2', 'velApplied');
    connect(modelName, 'Actuator_Limits/1', 'ActuatorState/1', '');
    connect(modelName, 'Actuator_Limits/2', 'Logging/4', 'velLogged');
    connect(modelName, 'Plant_UnicycleKinematics/1', 'PoseState/1', 'poseNext');
    connect(modelName, 'Logging/1', 'sl_log/1', 'logRow');

    note = Simulink.Annotation(modelName, ['Step 4: Simulink version of the Step 3 unicycle + Pure Pursuit simulation.' newline ...
        'Kinematic model only; one fixed step per control period; state in Unit Delay blocks.' newline ...
        'Parameters: s4ref, s4cfg, s4env, s4startPose, s4dt, s4stopTime (workspace variables).']);
    note.Position = [60 330];

    save_system(modelName, modelFile);
    close_system(modelName, 0);
end

function addBlock(model, libPath, name, position, params)
    path = [model '/' name];
    add_block(libPath, path, 'Position', position);
    for k = 1:2:numel(params)
        set_param(path, params{k}, params{k + 1});
    end
end

function sys = makeSubsystem(model, name, position, inNames, outNames)
    sys = [model '/' name];
    add_block('built-in/Subsystem', sys, 'Position', position);
    delete_line_if_any(sys);
    for k = 1:numel(inNames)
        add_block('built-in/Inport', [sys '/' inNames{k}], 'Port', num2str(k), 'Position', [30 40 * k 60 40 * k + 14]);
    end
    for k = 1:numel(outNames)
        add_block('built-in/Outport', [sys '/' outNames{k}], 'Port', num2str(k), 'Position', [400 40 * k 430 40 * k + 14]);
    end
end

function delete_line_if_any(sys)
    % A new subsystem starts with one Inport and one Outport joined by a line; remove them
    lines = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
    for k = 1:numel(lines)
        delete_line(lines(k));
    end
    blocks = find_system(sys, 'SearchDepth', 1, 'Type', 'Block');
    for k = 1:numel(blocks)
        if ~strcmp(blocks{k}, sys)
            delete_block(blocks{k});
        end
    end
end

function addMatlabFunction(sys, name, inNames, outNames, paramNames, body)
    blk = [sys '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', [180 40 330 40 + 30 * max(numel(inNames), numel(outNames))]);
    script = sprintf('function [%s] = fcn(%s)\n%s\nend\n', strjoin(outNames, ', '), strjoin(inNames, ', '), body);
    chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
    chart.Script = script;
    % Explicit output sizes and types: the model has feedback loops, so inference alone cannot resolve them
    for k = 1:numel(outNames)
        d = chart.find('-isa', 'Stateflow.Data', 'Name', outNames{k});
        d.Props.Array.Size = outputSize(outNames{k});
        d.DataType = 'double';
    end
    for k = 1:numel(paramNames)
        d = Stateflow.Data(chart);
        d.Name = paramNames{k};
        d.Scope = 'Parameter';
    end
    for k = 1:numel(inNames)
        add_line(sys, [inNames{k} '/1'], [name '/' num2str(k)]);
    end
    for k = 1:numel(outNames)
        add_line(sys, [name '/' num2str(k)], [outNames{k} '/1']);
    end
end

function connect(model, src, dst, signalName)
    h = add_line(model, src, dst, 'autorouting', 'on');
    if ~isempty(signalName)
        set_param(h, 'Name', signalName);
    end
end

function sz = outputSize(name)
%OUTPUTSIZE Width of each block output signal (1 means scalar).
    switch name
        case 'cmdDesired',  sz = '2';
        case 'info',        sz = '16';
        case 'velApplied',  sz = '2';
        case 'velLogged',   sz = '2';
        case 'poseNext',    sz = '3';
        case 'row',         sz = num2str(numel(slLogLayout()));
        otherwise,          sz = '1';
    end
end
