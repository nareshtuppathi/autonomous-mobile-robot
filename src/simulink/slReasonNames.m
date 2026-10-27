function names = slReasonNames()
%SLREASONNAMES Termination reasons indexed by the reason code used inside the Simulink model.
%   Code 0 means the simulation is still running; codes 1-8 match the reason strings of
%   simulateUnicycleTracking.
    names = {'goal_reached', 'missed_goal', 'timeout', 'stalled', 'diverged', 'collision', ...
             'left_map', 'numerical_failure'};
end
