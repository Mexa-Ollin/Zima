function ctl = make_scheduler(pid)
th = wrapToPi([pid.heading]);
Kp = [pid.Kp]; Ki = [pid.Ki]; Kd = [pid.Kd];

mask = isfinite(Kp) & isfinite(Ki) & isfinite(Kd);
th = th(mask); Kp = Kp(mask); Ki = Ki(mask); Kd = Kd(mask);
[th, ix] = sort(th); Kp = Kp(ix); Ki = Ki(ix); Kd = Kd(ix);

ctl.mode  = 'scheduled';
ctl.theta = th; ctl.Kp = Kp; ctl.Ki = Ki; ctl.Kd = Kd;
ctl.query = @(theta) deal( ...
    interp1(th, Kp, wrapToPi(theta), 'linear', 'extrap'), ...
    interp1(th, Ki, wrapToPi(theta), 'linear', 'extrap'), ...
    interp1(th, Kd, wrapToPi(theta), 'linear', 'extrap'));
end