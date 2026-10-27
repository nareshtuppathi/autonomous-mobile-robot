function result = runSimulinkTracking(ref, startPose, tcfg, env)
%RUNSIMULINKTRACKING Run amr_step4_tracking.slx and return a result shaped like simulateUnicycleTracking.
%   result = runSimulinkTracking(ref, startPose, tcfg, env) builds the model if the .slx file does not
%   exist, passes the reference, configuration, clearance environment and start pose to it as
%   workspace variables for this run only, simulates, and converts the logged matrix back to the
%   struct layout of the MATLAB simulation, so computeTrackingMetrics works on both.
%   Extra fields of result.log: kappa, inMap, braking, reasonCode (Simulink only).
%   terminationReason is 'none' if the model stopped at its stop time without a supervisor decision.
%   Warnings raised by the run are returned in result.warnings.

    arguments
        ref (1, 1) struct
        startPose (1, 3) double
        tcfg (1, 1) struct
        env (1, 1) struct
    end

    modelName = 'amr_step4_tracking';
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    modelFile = fullfile(projectRoot, 'simulink', [modelName '.slx']);
    if ~isfile(modelFile)
        buildStep4Model();
    end
    load_system(modelFile);

    dt = tcfg.sim.timeStep;
    tMax = tcfg.sim.timeoutFactor * ref.duration + tcfg.sim.timeoutPad;
    in = Simulink.SimulationInput(modelName);
    in = in.setVariable('s4ref', ref);
    in = in.setVariable('s4cfg', tcfg);
    in = in.setVariable('s4env', env);
    in = in.setVariable('s4startPose', [startPose(1:2), wrapAngle(startPose(3))]);
    in = in.setVariable('s4dt', dt);
    in = in.setVariable('s4stopTime', tMax + 3 * dt);

    lastwarn('');
    out = sim(in);
    [warnMsg, warnId] = lastwarn;
    if ~isempty(out.ErrorMessage)
        error('runSimulinkTracking:simulationFailed', '%s', out.ErrorMessage);
    end
    result = slLogToSim(out.sl_log, out.tout, startPose, tMax);
    result.warnings = struct('message', warnMsg, 'identifier', warnId);
end
