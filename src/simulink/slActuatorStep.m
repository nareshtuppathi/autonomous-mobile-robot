function [velNew, velLog] = slActuatorStep(cmd, braking, stop, velPrev, cfg)
%SLACTUATORSTEP Actuator model of the Step 4 Simulink model.
%   Block body of Actuator_Limits. Same order as simulateUnicycleTracking: braking overrides the
%   command with zero, then the acceleration (slew-rate) limits, then limitVelocityCommand
%   (speed, yaw-rate and wheel-speed limits).
%   velNew = [v; omega] applied over the coming step. velLog is what is logged for this
%   sample: velNew normally, but the previous values on the final sample (stop = 1).
    cmd = cmd(:);
    if braking > 0.5
        cmd = [0; 0];
    end
    dt = cfg.sim.timeStep;
    dvMax = cfg.robot.maxLinearAccel * dt;
    dwMax = cfg.robot.maxAngularAccel * dt;
    v = velPrev(1) + min(max(cmd(1) - velPrev(1), -dvMax), dvMax);
    w = velPrev(2) + min(max(cmd(2) - velPrev(2), -dwMax), dwMax);
    [v, w] = limitVelocityCommand(v, w, cfg.robot);
    velNew = [v; w];
    if stop > 0.5
        velLog = [velPrev(1); velPrev(2)];
    else
        velLog = velNew;
    end
end
