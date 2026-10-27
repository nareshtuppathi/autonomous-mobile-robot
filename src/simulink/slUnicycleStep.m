function poseNext = slUnicycleStep(pose, velNew, cfg)
%SLUNICYCLESTEP Unicycle kinematics for one step of the Step 4 Simulink model.
%   Block body of Plant_UnicycleKinematics:
%       x_dot = v*cos(theta),  y_dot = v*sin(theta),  theta_dot = omega
%   integrated exactly for the constant (v, omega) of the step, by calling stepUnicycle.
    pose = pose(:).';
    poseNext = stepUnicycle(pose, velNew(1), velNew(2), cfg.sim.timeStep);
    poseNext = poseNext(:);
end
