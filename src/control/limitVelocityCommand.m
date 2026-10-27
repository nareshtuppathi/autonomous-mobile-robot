function [v, w] = limitVelocityCommand(v, w, robotCfg)
%LIMITVELOCITYCOMMAND Enforce linear, angular and wheel-speed limits on a command.
%   [v, w] = limitVelocityCommand(v, w, robotCfg) with robotCfg = trackingConfig().robot.
%   1. v is clipped to [0, maxLinearSpeed] (no reversing).
%   2. If |w| exceeds maxAngularSpeed while driving, v is scaled down by the same
%      factor as w, so the commanded curvature w/v is preserved (the robot keeps
%      its arc instead of cutting the turn); w is then clipped.
%   3. If a wheel (v -/+ w*trackWidth/2) would exceed maxWheelSpeed, both v and w
%      are scaled by the same factor, again preserving curvature.
    v = min(max(v, 0), robotCfg.maxLinearSpeed);

    if abs(w) > robotCfg.maxAngularSpeed
        if v > 0
            v = v * robotCfg.maxAngularSpeed / abs(w);
        end
        w = sign(w) * robotCfg.maxAngularSpeed;
    end

    wheelMax = max(abs(v + w * robotCfg.trackWidth / 2), abs(v - w * robotCfg.trackWidth / 2));
    if wheelMax > robotCfg.maxWheelSpeed
        scale = robotCfg.maxWheelSpeed / wheelMax;
        v = v * scale;
        w = w * scale;
    end
end
