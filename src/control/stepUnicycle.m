function pose = stepUnicycle(pose, v, w, dt)
%STEPUNICYCLE Advance the unicycle model exactly over one step of constant v and w.
%   Model:  x_dot = v*cos(theta),  y_dot = v*sin(theta),  theta_dot = w
%   pose = [x y theta] (m, m, rad); v in m/s; w in rad/s; dt in s.
%   For constant v and w the model has a closed-form solution (a circular arc),
%   so there is no integration error. For |w| ~ 0 the arc degenerates to a line.
    x = pose(1);  y = pose(2);  th = pose(3);
    if abs(w) < 1e-9
        x = x + v * dt * cos(th);
        y = y + v * dt * sin(th);
        th = th + w * dt;
    else
        thNew = th + w * dt;
        x = x + (v / w) * (sin(thNew) - sin(th));
        y = y - (v / w) * (cos(thNew) - cos(th));
        th = thNew;
    end
    pose = [x, y, wrapAngle(th)];
end
