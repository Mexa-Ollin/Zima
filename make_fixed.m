function ctl = make_fixed(pid)
Kp = median([pid.Kp], "omitnan");
Ki = median([pid.Ki], "omitnan");
Kd = median([pid.Kd], "omitnan");
ctl.mode  = 'fixed';
ctl.gains = [Kp, Ki, Kd];
end