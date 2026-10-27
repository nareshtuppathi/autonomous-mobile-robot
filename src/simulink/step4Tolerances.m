function tol = step4Tolerances()
%STEP4TOLERANCES Numerical tolerances for the MATLAB-versus-Simulink comparison (set before running).
%   Both implementations call the same Step 3 functions with IEEE double arithmetic, so they
%   should agree to rounding. The tolerances sit far above rounding noise and far below any
%   physically meaningful difference (1e-9 m is a nanometre):
%     lengthM      m (also s)    positions, progress, cross-track error, clearance, times
%     angleRad     rad           headings, heading error, lookahead angle (compared after wrapping)
%     velocity     m/s, rad/s    commanded and applied velocities
%     metric       any           scalar summary metrics (final error, RMS, distance, accelerations ...)
%     plantOde     m, rad        one-step pose error against an independent ode45 integration
%   Integer quantities (step count, termination reason, braking onset step, rotating flag) must
%   match exactly.
    tol.lengthM = 1e-9;
    tol.angleRad = 1e-9;
    tol.velocity = 1e-9;
    tol.metric = 1e-9;
    tol.plantOde = 1e-9;
end
