function pid = PID_Design_From_Results(results, zeta, wn, alpha)
% PID_Design_From_Results
% Compute PID gains from identified (K,T) for each experiment.
%   zeta  : desired damping (e.g., 0.8)
%   wn    : natural frequency [rad/s] (e.g., 6)
%   alpha : sets 3rd pole p3 = alpha*wn (e.g., 4 → p3 ≈ 4*wn)
%
% Returns struct array 'pid' with fields Kp, Ki, Kd and copies heading, K, T.

if nargin < 2 || isempty(zeta),  zeta = 0.8; end
if nargin < 3 || isempty(wn),    wn   = 6.0; end
if nargin < 4 || isempty(alpha), alpha= 4.0; end

p3  = alpha*wn;
pid = results;

fprintf('\n=== PID design from identified (K,T) ===\n');
fprintf('Target: zeta=%.2f, wn=%.2f rad/s, p3=%.2f rad/s (alpha=%.1f)\n', zeta, wn, p3, alpha);
fprintf('%10s %10s %10s %10s %10s %10s %10s\n', 'Heading°','|K|','T[s]','Kp','Ki','Kd','Note');

for k = 1:numel(results)
    K = results(k).K;
    T = results(k).T;

    if ~isfinite(K) || ~isfinite(T) || T <= 0
        pid(k).Kp = NaN; pid(k).Ki = NaN; pid(k).Kd = NaN;
        note = 'skip (bad ID)';
    else
        Km = abs(K);
        Kd = (T*(2*zeta*wn + p3) - 1) / Km;
        Kp = (T*(wn^2 + 2*zeta*wn*p3)) / Km;
        Ki = (T*(wn^2*p3)) / Km;

        pid(k).Kp = Kp; pid(k).Ki = Ki; pid(k).Kd = Kd;
        note = '';
    end

    fprintf('%10.1f %10.4f %10.3f %10.3f %10.3f %10.3f %10s\n', ...
        rad2deg(results(k).heading), abs(results(k).K), results(k).T, ...
        pid(k).Kp, pid(k).Ki, pid(k).Kd, note);
end

plot_PID_vs_heading(pid);
end

% --- local helper for this file ---
function plot_PID_vs_heading(pid)
theta = [pid.heading];
Kp = [pid.Kp]; Ki = [pid.Ki]; Kd = [pid.Kd];

figure('Name','PID gains vs Heading');
subplot(3,1,1);
plot(rad2deg(theta), Kp, 'o-','LineWidth',1.3); grid on;
ylabel('K_P'); title('PID gains vs Heading');

subplot(2,1,2);
plot(rad2deg(theta), Ki, 's-','LineWidth',1.3); grid on;
ylabel('K_I');

figure('Name','PID gains vs Heading (K_D)');
plot(rad2deg(theta), Kd, 'd-','LineWidth',1.3); grid on;
ylabel('K_D'); xlabel('Heading (deg)');
end