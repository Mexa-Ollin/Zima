function T = compute_metrics(sim)
% COMPUTE_METRICS  Basic performance numbers for one sim run.
% Returns a table with RMS error, settling time, overshoot, and effort.

t   = sim.t(:);
err = sim.err_h(:);          % already wrapped in simulate_controller
u   = sim.u(:);              % guide command (rad)
des = sim.des_h(:);

% 1) RMS heading error (whole run)
rms_err = sqrt(mean(err.^2,'omitnan'));

% 2) Settling time after the final change in desired heading
dDes = abs([0; diff(des)]);
idx_change = find(dDes > deg2rad(1), 1, 'last');   % ~1° threshold
if isempty(idx_change), idx_change = 1; end

epsb = deg2rad(5);     % settling band ±5°
ts   = NaN;
for k = idx_change:numel(t)
    seg  = err(k:end);
    cond = abs(seg) < epsb | isnan(seg);   % ignore NaNs
    if all(cond)
        ts = t(k) - t(idx_change);
        break;
    end
end

% 3) Overshoot: max |error| in 10 s after last change
win = (t >= t(idx_change)) & (t <= t(idx_change)+10);
overshoot = max(abs(err(win)),[],'omitnan');

% 4) Effort proxies
J_u = trapz(t, abs(u));
J_f = NaN;
if isfield(sim,'Mf') && ~isempty(sim.Mf)
    Mf = sim.Mf(:);
    n  = min(numel(Mf), numel(t));
    J_f = trapz(t(1:n), abs(Mf(1:n)));
end

T = table(rms_err, ts, overshoot, J_u, J_f, ...
    'VariableNames', {'rms_err','settle_time_s','overshoot_rad','guide_effort','fin_effort'});
end