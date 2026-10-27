function [cmd, infoVec, sProgNext, haveInfo, cmdFinite] = slPurePursuitStep(pose, sProg, velPrev, ref, cfg)
%SLPUREPURSUITSTEP One Pure Pursuit step for the Step 4 Simulink model.
%   Block body of Controller_PurePursuit. It calls the existing purePursuitController
%   unchanged and only packs its outputs into plain numeric vectors.
%     pose    - [x; y; theta];  sProg - progress along the path at the previous step
%     velPrev - [v; omega] applied at the previous step (v sets the lookahead distance)
%   infoVec order: sProj, crossTrack, headingError, xRef, yRef, thetaRef, vRef, omegaRef,
%   tRef, lookahead, xLook, yLook, alpha, rotating, distToGoal, kappa (16 values).
%   haveInfo is 0 (and cmd, infoVec are NaN) if the pose is not finite, as in the MATLAB loop.
    pose = pose(:).';
    cmd = NaN(2, 1);
    infoVec = NaN(16, 1);
    sProgNext = sProg;
    haveInfo = 0;
    cmdFinite = 0;
    if all(isfinite(pose))
        [c, info] = purePursuitController(pose, ref, sProg, velPrev(1), cfg);
        cmd = c(:);
        infoVec = [info.sProj; info.crossTrack; info.headingError; info.xRef; info.yRef; ...
                   info.thetaRef; info.vRef; info.omegaRef; info.tRef; info.lookahead; ...
                   info.xLook; info.yLook; info.alpha; double(info.rotating); ...
                   info.distToGoal; info.kappa];
        sProgNext = info.sProj;
        haveInfo = 1;
        cmdFinite = double(all(isfinite(c)));
    end
end
