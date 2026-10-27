function a = wrapAngle(a)
%WRAPANGLE Wrap angles in radians to (-pi, pi].
    a = atan2(sin(a), cos(a));
end
