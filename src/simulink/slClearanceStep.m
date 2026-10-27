function [clearanceM, inMap, poseFinite] = slClearanceStep(pose, env)
%SLCLEARANCESTEP Clearance of the robot centre to the real (uninflated) obstacles.
%   Block body of Environment_Clearance in the Step 4 Simulink model. Calls the same
%   clearanceAtPoints as the MATLAB simulation. Outputs are doubles (inMap, poseFinite are 0/1).
%   If the pose is not finite the clearance is NaN, as in the MATLAB simulation.
    pose = pose(:).';
    poseFinite = double(all(isfinite(pose)));
    clearanceM = NaN;
    inMap = 1;
    if poseFinite > 0
        [c, m] = clearanceAtPoints(env, pose(1:2));
        clearanceM = c;
        inMap = double(m);
    end
end
