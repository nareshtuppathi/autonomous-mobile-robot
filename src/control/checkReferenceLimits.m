function report = checkReferenceLimits(ref, cfg)
%CHECKREFERENCELIMITS Check a reference trajectory against the configured limits.
%   report = checkReferenceLimits(ref, cfg) returns logical fields
%     allFinite, strictlyIncreasing (s and t), startsAtRest, endsAtRest,
%     speedWithinLimit, accelWithinLimit, omegaWithinLimit, wheelSpeedWithinLimit,
%     passesAll (all of the above)
%   and numeric fields maxSpeed, maxAccel, maxOmega, maxWheelSpeed, maxLateralAccel.
%   Lateral acceleration is reported but not part of passesAll: the speed floor
%   in tight turns can push it above cfg.reference.maxLateralAccel.
    tol = 1e-9;
    r = cfg.robot;
    report.allFinite = all(isfinite([ref.s; ref.t; ref.x; ref.y; ref.theta; ref.v; ref.omega; ref.kappa]));
    report.strictlyIncreasing = all(diff(ref.s) > 0) && all(diff(ref.t) > 0);
    report.startsAtRest = ref.v(1) == 0;
    report.endsAtRest = ref.v(end) == 0;
    wheel = abs(ref.v) + abs(ref.omega) * r.trackWidth / 2;
    report.maxSpeed = max(ref.v);
    report.maxAccel = max(abs(ref.accelSeg));
    report.maxOmega = max(abs(ref.omega));
    report.maxWheelSpeed = max(wheel);
    report.maxLateralAccel = ref.maxLateralAccel;
    report.speedWithinLimit = report.maxSpeed <= r.maxLinearSpeed + tol && all(ref.v >= 0);
    report.accelWithinLimit = report.maxAccel <= r.maxLinearAccel + 1e-6;
    report.omegaWithinLimit = report.maxOmega <= r.maxAngularSpeed + tol;
    report.wheelSpeedWithinLimit = report.maxWheelSpeed <= r.maxWheelSpeed + tol;
    report.passesAll = report.allFinite && report.strictlyIncreasing && report.startsAtRest && ...
        report.endsAtRest && report.speedWithinLimit && report.accelWithinLimit && ...
        report.omegaWithinLimit && report.wheelSpeedWithinLimit;
end
