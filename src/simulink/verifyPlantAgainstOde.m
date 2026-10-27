function res = verifyPlantAgainstOde(sim, dt)
%VERIFYPLANTAGAINSTODE Check the logged poses against an independent numerical ODE solution.
%   res = verifyPlantAgainstOde(sim, dt) takes each logged pose and the velocities logged for that
%   step, integrates x_dot = v*cos(theta), y_dot = v*sin(theta), theta_dot = omega over dt with
%   ode45 (RelTol 1e-12, AbsTol 1e-13) and compares the result with the next logged pose.
%   This tests the closed-form arc used by the plant block (stepUnicycle) against a different
%   method, one step at a time. The last sample is skipped because no step follows it.
%   res fields: maxPositionErrorM, maxHeadingErrorRad, numSteps.
    L = sim.log;
    n = numel(L.t);
    opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-13);
    errPos = zeros(n - 1, 1);
    errTh = zeros(n - 1, 1);
    for k = 1:n - 1
        v = L.v(k);
        w = L.omega(k);
        rhs = @(~, q) [v * cos(q(3)); v * sin(q(3)); w];
        [~, Y] = ode45(rhs, [0 dt], [L.x(k); L.y(k); L.theta(k)], opts);
        qEnd = Y(end, :);
        errPos(k) = hypot(qEnd(1) - L.x(k + 1), qEnd(2) - L.y(k + 1));
        errTh(k) = abs(wrapAngle(qEnd(3) - L.theta(k + 1)));
    end
    res.maxPositionErrorM = max(errPos);
    res.maxHeadingErrorRad = max(errTh);
    res.numSteps = n - 1;
end
