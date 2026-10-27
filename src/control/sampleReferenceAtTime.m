function q = sampleReferenceAtTime(ref, tq)
%SAMPLEREFERENCEATTIME Reference state at given times (clamped to [0, duration]).
%   q = sampleReferenceAtTime(ref, tq) returns a struct with fields x, y, theta,
%   v, omega, s at the query times tq. Linear interpolation of the stored samples.
    tq = min(max(tq(:), 0), ref.duration);
    q.s     = interp1(ref.t, ref.s, tq, 'linear');
    q.x     = interp1(ref.t, ref.x, tq, 'linear');
    q.y     = interp1(ref.t, ref.y, tq, 'linear');
    q.theta = wrapAngle(interp1(ref.t, ref.thetaUnwrapped, tq, 'linear'));
    q.v     = interp1(ref.t, ref.v, tq, 'linear');
    q.omega = interp1(ref.t, ref.omega, tq, 'linear');
end
