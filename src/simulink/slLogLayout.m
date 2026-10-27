function names = slLogLayout()
%SLLOGLAYOUT Column order of the log vector written by the Simulink model (one row per step).
%   The same names are used by slLogToSim to rebuild a struct that matches the log of
%   simulateUnicycleTracking. 'v' and 'omega' are the applied velocities over the step that
%   starts at that sample (the last applied values on the final sample).
    names = {'x', 'y', 'theta', 'vDes', 'omegaDes', 'sProj', 'crossTrack', 'headingError', ...
             'xRef', 'yRef', 'thetaRef', 'vRef', 'omegaRef', 'tRef', 'lookahead', 'xLook', ...
             'yLook', 'alpha', 'rotating', 'distToGoal', 'kappa', 'clearanceM', 'inMap', ...
             'braking', 'reasonCode', 'v', 'omega'};
end
