function results = StepID_Program()
IDENT_MODE = true;
U0_deg     = 7;
t_step     = 5.0;
t_hold     = 30.0;

wp_list = [ 3,0; 4,0; 4,1; 3,2; 2,2 ];
make_step_overlays = true;

n = size(wp_list,1);
results = struct('wp',cell(n,1),'heading',[],'K',[],'T',[], ...
                 'r0',[],'rinf',[],'t0',[],'U0',[], 't',[], 'u',[], 'r',[]);
for k = 1:n
    wp = wp_list(k,:);
    out = Fs_Step_Response(wp, IDENT_MODE, U0_deg, t_step, t_hold);
    results(k).wp      = wp;
    results(k).heading = wrapToPi(out.heading_to_wp);
    results(k).K       = out.K;
    results(k).T       = out.T;
    results(k).r0      = out.r0;
    results(k).rinf    = out.rinf;
    results(k).t0      = out.t0;
    results(k).U0      = out.U0;
    results(k).t       = out.t;
    results(k).u       = out.u;
    results(k).r       = out.r;

    fprintf('WP (%.2f, %.2f): heading %.1f deg  →  K=%.4f, T=%.3f s\n', ...
        wp(1), wp(2), rad2deg(results(k).heading), results(k).K, results(k).T);

    if make_step_overlays
        step_overlay(out, sprintf('WP(%.1f,%.1f)', wp));
    end
end

plot_KT_vs_heading(results);
plot_heading_map(results);
end

function out = Fs_Step_Response(wp, IDENT_MODE, U0_deg, t_step, t_hold)
%% Simulation parameters
Vl.t_end = 60;             % simulation duration (s)
Vl.dt    = 0.02;           % time step (s)
Vl.t     = 0:Vl.dt:Vl.t_end;
N        = numel(Vl.t);
world_scale = 1.5;

if nargin < 1 || isempty(wp),        wp = [3,1]; end
if nargin < 2 || isempty(IDENT_MODE),IDENT_MODE = true; end
if nargin < 3 || isempty(U0_deg),    U0_deg = 7; end
if nargin < 4 || isempty(t_step),    t_step = 5.0; end
if nargin < 5 || isempty(t_hold),    t_hold = 15.0; end
U0 = deg2rad(U0_deg);

%% Geometry and craft shape
CrL = 0.5; CW = 0.25;
boatShape = [-CrL/2, -CW/2;
             -CrL/2,  CW/2;
              CrL/2,  CW/2;
              CrL/2, -CW/2];
finLength   = 0.2;
guideLength = 0.25;

%% Physical parameters - ADJUSTED
m   = 1.45;    Iz = 0.2;
X_u = 0.5;  Y_v = 0.75; N_r = 1.5;
X_du = 0.3; Y_dv = 0.6;  N_dr = 0.05;

% Reduced drag coefficients
params.X_uu   = 1.0;
params.Y_vv   = 1.5;
params.N_rr   = 0.5;
params.Cd_hull  = 0.2;
params.A_wetted = 0.05;

% Fin parameters
params.finLength     = finLength;
params.chord         = 0.03;
params.N_elements    = 10;
params.rho           = 1000;
params.flap_freq     = 1.0;
params.phi_fin       = pi/2;
params.base_flap_amp = deg2rad(25);
params.stroke_asym   = 0.1;
params.CL_max        = 3.5;
params.CD_min        = 0.01;
params.CrL           = CrL;
params.CW            = CW;
params.guideLength   = guideLength;
params.finDepth      = 1.0;
prev_fAng = 0;

%% State initialization
nu         = zeros(3,N);    % body velocity [u; v; r]
eta        = zeros(3,N);    % pose [x; y; psi]
omega      = zeros(1,N);
finForce   = zeros(2,N);
finMoment  = zeros(1,N);
guide_cmd  = zeros(1,N);

% Base propulsion oscillation parameters
base_amp   = deg2rad(15);
base_freq  = params.flap_freq;
theta_const = deg2rad(8);

%% Control parameters (only used if IDENT_MODE=false)
Kp_R = 0.5; Ki_R = 0.05; Kd_R = 0.2;
cmd_smooth = 0.7;
int_err=0; prev_err=0;

%% Mass and damping matrices
MRB   = diag([m, m, Iz]);
MA    = [X_du,0,0; 0,Y_dv,Y_dv; 0,Y_dv,N_dr];
M_tot = MRB + MA;
Damp = @(nu)[ X_u + params.X_uu*abs(nu(1))   0                    0;
              0      Y_v + params.Y_vv*abs(nu(2))   -0.05*nu(2);
              0      -0.05*nu(1)          N_r + params.N_rr*abs(nu(3)) ];

%% Visualization (optional – kept lightweight)
figure('Name',sprintf('Sim WP(%.1f,%.1f)',wp)); hold on; axis equal; grid on;
xlim([-7/world_scale,7/world_scale]); ylim([-7/world_scale,7/world_scale]);
plot(wp(1),wp(2),'ro','MarkerSize',8);
hCraft = fill(boatShape(:,1), boatShape(:,2), [0.2,0.6,1.0], 'EdgeColor','k');
hGuide = line([0,0],[0,0],'LineWidth',2,'Color',[0.5,0.5,0.5]);
hFin   = line([0,0],[0,0],'LineWidth',2,'Color','k');
trail  = animatedline('LineStyle','--','Color','b');

guide_state = struct('phase',0,'last_freq',params.flap_freq,'last_guide',0);
ID_reduce_noise = true;

%% Logs for ID
u_input = zeros(1,N);     % applied guide (rad)
r_log   = zeros(1,N);     % yaw rate (rad/s)
psi_log = zeros(1,N);     % heading (rad)

%% Simulation loop
for i = 2:N
    % Desired heading (computed for information)
    des_h = atan2(wp(2)-eta(2,i-1), wp(1)-eta(1,i-1));
    err_h = wrapToPi(des_h - eta(3,i-1)) + (ID_reduce_noise*0);

    % Control / Identification logic
    if IDENT_MODE
        % Clean step on guide_cmd; PID is bypassed
        if Vl.t(i) < t_step
            guide_cmd(i) = 0;
        else
            guide_cmd(i) = U0;
        end
    else
        % PID tracking (not used during ID mode)
        if abs(guide_cmd(i-1)) < deg2rad(29.5)
            int_err = int_err + err_h * Vl.dt;
        end
        d_err = (err_h - prev_err) / Vl.dt; prev_err = err_h;
        u_cmd = Kp_R*err_h + Ki_R*int_err + Kd_R*d_err;
        u_cmd = max(min(u_cmd,deg2rad(30)),-deg2rad(30));
        guide_cmd(i) = -(cmd_smooth*u_cmd + (1-cmd_smooth)*guide_cmd(i-1));
    end

    % Guide & fin angles
    [gAng, totalAng, ~, ~, guide_state] = ...
        calculate_guide_wave(guide_cmd,[],Vl.t,Vl.dt,i,params,guide_state,cmd_smooth);

    % Add both oscillatory and constant bias
    theta_base = base_amp * sin(2*pi*base_freq*Vl.t(i));
    fAng = totalAng + theta_base + theta_const;

    % Fin angular velocity
    omega(i) = (fAng - prev_fAng) / Vl.dt;
    prev_fAng = fAng;
    % (approximate; exact previous fAng not stored—ok for our force model below)

    % Slow down near waypoint (keeps sim stable/realistic)
    dist_to_wp = norm(wp - eta(1:2,i-1)');
    if dist_to_wp < 0.8
        speed_factor = min(1, dist_to_wp/0.5);
        theta_const = deg2rad(8) * speed_factor;
        base_amp = deg2rad(15) * speed_factor;
    else
        theta_const = deg2rad(8);
        base_amp = deg2rad(15);
    end

    % Fin forces (steady) - MODIFIED FORCE CALCULATION
    [Ff, Mf] = compute_fin_force_corrected(eta(:,i-1), nu(:,i-1), fAng, omega(i), params);

    % Yaw control moment (used only if PID on; harmless otherwise)
    Kp_y = 1.0; Kd_y = 0.3;
    My = Kp_y * err_h + Kd_y * (-nu(3,i-1));

    % Hull resistance - REDUCED
    Rh = 0.5*params.rho*params.Cd_hull*params.A_wetted * abs(nu(1,i-1)) * nu(1,i-1);

    % Net tau
    stabilization_gain = 0.2;
    tau = [Ff(1) - Rh - stabilization_gain*nu(1,i-1);
           Ff(2) - stabilization_gain*nu(2,i-1);
           Mf + My];

    % Integrate dynamics
    C_rb   = coriolis_rb( nu(:,i-1), m, CrL, CW );
    C_a    = coriolis_am( nu(:,i-1), X_du, Y_dv );
    nu_dot = M_tot \ (tau - (C_rb + C_a + Damp(nu(:,i-1)))*nu(:,i-1));
    nu(:,i) = nu(:,i-1) + nu_dot * Vl.dt;

    % Update pose
    psi = eta(3,i-1);
    Rb  = [cos(psi),-sin(psi); sin(psi),cos(psi)];
    eta(1:2,i) = eta(1:2,i-1) + Rb * nu(1:2,i-1) * Vl.dt;
    eta(3,i)   = psi + nu(3,i-1) * Vl.dt;

    % Animate (lightweight)
    bodyWorld = (boatShape * Rb') + eta(1:2,i)';
    set(hCraft, 'XData', bodyWorld(:,1), 'YData', bodyWorld(:,2));
    gb = eta(1:2,i) + Rb*[-CrL/2; 0];
    gt = gb + Rb*[cos(gAng),-sin(gAng); sin(gAng),cos(gAng)]*[-guideLength;0];
    set(hGuide, 'XData', [gb(1),gt(1)], 'YData', [gb(2),gt(2)]);
    fb = gt;
    finDir = [cos(gAng+fAng), -sin(gAng+fAng);
              sin(gAng+fAng),  cos(gAng+fAng)];
    ft = fb + Rb * finDir * [-finLength;0];
    set(hFin, 'XData', [fb(1), ft(1)], 'YData', [fb(2), ft(2)]);
    addpoints(trail, eta(1,i), eta(2,i));
    drawnow limitrate;

    % Logs for ID
    u_input(i) = guide_cmd(i);
    r_log(i)   = nu(3,i);
    psi_log(i) = eta(3,i);

    % Early stop after hold time
    if IDENT_MODE && Vl.t(i) >= (t_step + t_hold)
        N = i;  %#ok<FXSET>
        break;
    end
end

% 63% identification
t = Vl.t(1:N).'; u = u_input(1:N).'; r = r_log(1:N).';

% Step detection
du = [0; diff(u)./max(eps, diff(t))];
[~, idx0] = max(abs(du)); t0 = t(idx0);
pre_idx  = t < t0;
post_idx = t > (t0 + 2*median(diff(t)));

r0   = mean(r(pre_idx),  'omitnan');
U0h  = mean(u(post_idx), 'omitnan') - mean(u(pre_idx), 'omitnan');
rinf = mean(r(post_idx), 'omitnan');

K_est = (rinf - r0) / max(U0h, 1e-9);
target = r0 + 0.632*(rinf - r0);
if rinf >= r0
    ix63 = find(t >= t0 & r >= target, 1, 'first');
else
    ix63 = find(t >= t0 & r <= target, 1, 'first');
end

if isempty(ix63)
    T_est = NaN;
else
    T_est = t(ix63) - t0;
end

% Heading to waypoint at step time (for "heading vs K/T" plots)
xy_err = wp(:) - eta(1:2, max(idx0-1,1));
heading_to_wp = atan2(xy_err(2), xy_err(1));

% Return everything
out = struct('t',t,'u',u,'r',r,'psi',psi_log(1:N).', ...
             'K',K_est,'T',T_est,'r0',r0,'rinf',rinf,'t0',t0,'U0',U0h, ...
             'wp',wp,'heading_to_wp',heading_to_wp);
end

function [gAng, totalAng, thetaP, fFreq, st] = calculate_guide_wave(cmd,~,t,dt,i,params,st,sm)
% Guide angle = commanded bias + base oscillation; asymmetry on stroke

f     = params.flap_freq;
phase = st.phase + 2*pi*f*dt;
A     = params.base_flap_amp;

% Use the command (already smoothed before call), fall back to 0
if ~isempty(cmd) && numel(cmd) >= i
    gBias = cmd(i);           % <-- this wires your PID/step into the plant
else
    gBias = 0;
end

% Oscillatory stroke component
thetaP = A * cos(phase);

% Total fin “wave” relative to guide with asymmetry
if sin(phase) >= 0
    asym = 1 + params.stroke_asym;
else
    asym = 1 - params.stroke_asym;
end
rawAng    = -thetaP;
totalAng  = asym * rawAng;   % fin motion around the guide
gAng      = gBias;           % the guide itself points at the commanded bias
fFreq     = f;

st.phase     = phase;
st.last_freq = f;
end

function C = coriolis_rb(nu,m,~,~)
u=nu(1);v=nu(2);
C = [ 0    0    -m*v;
    0    0     m*u;
    m*v -m*u    0  ];
end

function C = coriolis_am(nu,X_du,Y_dv)
u=nu(1);v=nu(2);
C = [ 0             0              Y_dv*v;
    0             0             -X_du*u;
    -Y_dv*v        X_du*u         0     ];
end

function [F_fin, M_fin] = compute_fin_force_corrected(eta, nu, finAng, omega, params)
% Enhanced fin force model with velocity scaling
if isfield(params,'N_elements') && ~isempty(params.N_elements)
    N = params.N_elements;
else
    N = 10;
end
span = params.finLength;
rho  = params.rho;
c    = params.chord;

% Velocity-dependent thrust scaling factor
vel_scaling = max(0.5, 1 - 0.1*norm(nu(1:2)));

base = [-params.CrL/2; 0];
psi  = eta(3);
Rb = [cos(psi), -sin(psi); sin(psi), cos(psi)];
U  = nu(1:2);

F = [0; 0]; M = 0;

for j = 1:N
    y = span * (j - 0.5) / N;
    vrot = [0; omega*y];

    Rf = [cos(finAng), -sin(finAng); sin(finAng), cos(finAng)];
    Urel = Rf' * (Rb' * U + vrot);
    Umag = norm(Urel);
    if Umag < 1e-4, continue; end

    a = atan2(Urel(2), Urel(1));

    % stall-aware coefficients
    if abs(a) <= deg2rad(15)
        CL = 1.5*sin(2*a);
        CD = params.CD_min + 0.5*(1 - cos(2*a));
    else
        CL = sign(a)*params.CL_max*sin(a)^2;
        CD = 1.1 - 0.5*cos(2*a);
    end

    area = c * (span/N) * params.finDepth;
    dL = 0.5 * rho * Umag^2 * area * CL * vel_scaling;
    dD = 0.5 * rho * Umag^2 * area * CD;

    % Local force in fin frame
    f = [cos(a)*dD - sin(a)*dL;    % thrust-like component
        cos(a)*dL + sin(a)*dD];   % lift-like component

    fb = Rb * Rf * f;              % to body frame
    r = base + Rf * [0; y];        % arm in body frame
    M = M + (r(1)*fb(2) - r(2)*fb(1));
    F = F + fb;
end

F_fin = -F;
M_fin = -M;
end

function step_overlay(out, nameTag)
t=out.t; u=out.u; r=out.r; r0=out.r0; rinf=out.rinf; t0=out.t0; K=out.K; T=out.T; U0=out.U0;
H = double(t >= t0);
r_fit = r0 + K*U0 * (1 - exp(-(t - t0)/max(T,1e-6))) .* H;

figure('Name',['Step overlay - ' nameTag]);
subplot(2,1,1);
plot(t, rad2deg(u), 'LineWidth',1.2); grid on; ylabel('u (deg)');
title(['Step Input – ' nameTag]);

subplot(2,1,2);
plot(t, r, 'k', 'LineWidth',1.0); hold on;
plot(t, r_fit, 'r--', 'LineWidth',1.5);
xline(t0,':'); yline(r0,':b'); yline(rinf,':g');
grid on; xlabel('t (s)'); ylabel('r (rad/s)');
legend('measured','fit','t_0','r_0','r_\infty','Location','best');
title(sprintf('Fit: K=%.4f, T=%.3fs', K, T));
end

function plot_KT_vs_heading(R)
theta = [R.heading];
K     = [R.K];
T     = [R.T];

figure('Name','K and T vs Heading');
subplot(2,1,1);
plot(rad2deg(theta), K, 'o-','LineWidth',1.3); grid on;
xlabel('Heading to waypoint (deg)'); ylabel('K  (rad/s)');
title('Identified K vs Heading');

subplot(2,1,2);
plot(rad2deg(theta), T, 's-','LineWidth',1.3); grid on;
xlabel('Heading to waypoint (deg)'); ylabel('T (s)');
title('Identified T vs Heading');
end

function plot_heading_map(R)
figure('Name','Heading Map of Experiments'); hold on; axis equal; grid on;
title('Experiment Heading Map (K variation)'); xlabel('x (m)'); ylabel('y (m)');

Ks   = [R.K];
Kabs = abs(Ks);
if all(Kabs==0), Kabs = ones(size(Kabs)); end
maxL = 2; L = maxL * (Kabs / max(Kabs));

for k = 1:numel(R)
    th = R(k).heading;
    quiver(0,0, L(k)*cos(th), L(k)*sin(th), 0, 'LineWidth',1.8,'MaxHeadSize',0.4);
    text(1.05*L(k)*cos(th), 1.05*L(k)*sin(th), sprintf('%.0f°', rad2deg(th)), 'FontSize',9);
end
colormap('parula'); caxis([min(Ks) max(Ks)]);
cb = colorbar; cb.Label.String = 'K (rad/s)';

wps = vertcat(R.wp);
scatter(wps(:,1), wps(:,2), 40, 'k', 'filled', 'DisplayName','Waypoints');
legend('Location','best');
end


% PID design from identified (K,T)
function pid = PID_Design_From_Results(results, zeta, wn, alpha)
if nargin < 2,  zeta = 0.8; end
if nargin < 3,  wn   = 6.0; end
if nargin < 4,  alpha= 4.0; end
p3 = alpha*wn;

pid = results;
fprintf('\n=== PID design from identified (K,T) ===\n');
fprintf('Target: zeta=%.2f, wn=%.2f rad/s, p3=%.2f rad/s\n', zeta, wn, p3);
fprintf('%10s %10s %10s %10s %10s %10s %10s\n', 'Heading°','|K|','T[s]','Kp','Ki','Kd','Note');

for k = 1:numel(results)
    K = results(k).K;
    T = results(k).T;
    if ~isfinite(K) || ~isfinite(T) || T <= 0
        pid(k).Kp = NaN; pid(k).Ki = NaN; pid(k).Kd = NaN; note='skip';
    else
        Km = abs(K);
        Kd = (T*(2*zeta*wn + p3) - 1) / Km;
        Kp = (T*(wn^2 + 2*zeta*wn*p3)) / Km;
        Ki = (T*(wn^2*p3)) / Km;
        pid(k).Kp = Kp; pid(k).Ki = Ki; pid(k).Kd = Kd; note='';
    end
    fprintf('%10.1f %10.4f %10.3f %10.3f %10.3f %10.3f %10s\n', ...
        rad2deg(results(k).heading), abs(results(k).K), results(k).T, ...
        pid(k).Kp, pid(k).Ki, pid(k).Kd, note);
end
plot_PID_vs_heading(pid);
end

function plot_PID_vs_heading(pid)
theta = [pid.heading]; Kp=[pid.Kp]; Ki=[pid.Ki]; Kd=[pid.Kd];
figure('Name','PID gains vs Heading');
subplot(3,1,1); plot(rad2deg(theta),Kp,'o-','LineWidth',1.3); grid on; ylabel('K_P');
title('PID gains vs Heading');
subplot(3,1,2); plot(rad2deg(theta),Ki,'s-','LineWidth',1.3); grid on; ylabel('K_I');
subplot(3,1,3); plot(rad2deg(theta),Kd,'d-','LineWidth',1.3); grid on; ylabel('K_D'); xlabel('Heading (deg)');
end

% Post-ID utilities
function pid = flag_outliers(pid)
for k=1:numel(pid)
    pid(k).flag = "";
    if ~isfinite(pid(k).T) || pid(k).T<=0, pid(k).flag = pid(k).flag + " T_bad"; end
    if ~isfinite(pid(k).K), pid(k).flag = pid(k).flag + " K_bad"; end
    if isfinite(pid(k).K) && pid(k).K<0, pid(k).flag = pid(k).flag + " K_sign"; end
end
end

function ctl = make_fixed(pid)
% Use the median of finite gains.
Kp = median([pid.Kp],"omitnan");
Ki = median([pid.Ki],"omitnan");
Kd = median([pid.Kd],"omitnan");
ctl.mode = 'fixed';
ctl.gains = [Kp,Ki,Kd];
end

function ctl = make_scheduler(pid)
% Linear interpolation vs heading (wrap-aware).
th = wrapToPi([pid.heading]);
Kp = [pid.Kp]; Ki=[pid.Ki]; Kd=[pid.Kd];
% remove NaNs
mask = isfinite(Kp)&isfinite(Ki)&isfinite(Kd);
th=th(mask); Kp=Kp(mask); Ki=Ki(mask); Kd=Kd(mask);
[th,ix]=sort(th); Kp=Kp(ix); Ki=Ki(ix); Kd=Kd(ix);
ctl.mode='scheduled';
ctl.theta = th; ctl.Kp=Kp; ctl.Ki=Ki; ctl.Kd=Kd;
ctl.query = @(theta) deal( ...
    interp1(th,Kp,wrapToPi(theta),'linear','extrap'), ...
    interp1(th,Ki,wrapToPi(theta),'linear','extrap'), ...
    interp1(th,Kd,wrapToPi(theta),'linear','extrap'));
end

function ctl = make_adaptive_defaults()
% Simple online estimator (filtered ratio) to update gains.
ctl.mode = 'adaptive';
ctl.zeta = 0.8; ctl.wn = 6.0; ctl.alpha = 4.0;  % design targets
ctl.est = struct('K',0.05,'T',0.3,'r0',0,'u0',0,'tau',0.5); % initial guesses
end


% Controller + scenarios sim
function sim = simulate_controller(scn, ctl, id_results)
% scn.wps : [N x 2] waypoints or [1x2] single
% ctl     : struct from make_fixed/make_scheduler/make_adaptive_defaults
% id_results: only used to reuse low-level dynamics (fin etc.) from Fs_Step_Response

Vl.t_end = 60; Vl.dt=0.02; Vl.t=0:Vl.dt:Vl.t_end; N=numel(Vl.t);
CrL=0.5; CW=0.25;
finLength=0.2; guideLength=0.25;

% Physical (same as your ID sim)
m=1.45; Iz=0.2; X_u=0.5; Y_v=0.75; N_r=1.5;
X_du=0.3; Y_dv=0.6; N_dr=0.05;

params.X_uu=1.0; params.Y_vv=1.5; params.N_rr=0.5;
params.Cd_hull=0.2; params.A_wetted=0.05;

params.finLength=finLength; params.chord=0.03; params.N_elements=10;
params.rho=1000; params.flap_freq=1.0; params.phi_fin=pi/2;
params.base_flap_amp=deg2rad(25); params.stroke_asym=0.1;
params.CL_max=3.5; params.CD_min=0.01;
params.CrL=CrL; params.CW=CW; params.guideLength=guideLength;
params.finDepth=1.0;

MRB=diag([m,m,Iz]); MA=[X_du,0,0; 0,Y_dv,Y_dv; 0,Y_dv,N_dr]; M_tot=MRB+MA;
Damp = @(nu)[ X_u + params.X_uu*abs(nu(1))   0                    0;
              0      Y_v + params.Y_vv*abs(nu(2))   -0.05*nu(2);
              0      -0.05*nu(1)          N_r + params.N_rr*abs(nu(3)) ];

% State
nu=zeros(3,N); eta=zeros(3,N);
guide_cmd=zeros(1,N);
omega=zeros(1,N); prev_fAng=0;
int_err=0; prev_err=0;

% base flap
base_amp=deg2rad(15); theta_const=deg2rad(8);

guide_state=struct('phase',0,'last_freq',params.flap_freq,'last_guide',0);
cmd_smooth=0.7;

% disturbance handle
D = scn.dist;

% logging
des_h = zeros(1,N); err_h=zeros(1,N); Mf_log=zeros(1,N); Ff_log=zeros(2,N);

% waypoint manager
wps = scn.wps; if size(wps,1)==1, wps=[wps; wps]; end
wpi=1; wp_current=wps(wpi,:);

for i=2:N
    % switch waypoint when close
    if norm(wp_current - eta(1:2,i-1)') < 0.3 && wpi < size(wps,1)
        wpi=wpi+1; wp_current=wps(wpi,:);
    end

    % desired heading and error
    des_h(i) = atan2(wp_current(2)-eta(2,i-1), wp_current(1)-eta(1,i-1));
    err_h(i) = wrapToPi(des_h(i) - eta(3,i-1));

    % PID gains
    switch ctl.mode
        case 'fixed'
            [Kp,Ki,Kd] = deal(ctl.gains(1),ctl.gains(2),ctl.gains(3));

        case 'scheduled'
            [Kp,Ki,Kd] = ctl.query(err_h(i));  % schedule vs relative heading

        case 'adaptive'
            % quick-and-dirty online K,T estimate
            % filtered ratio r/u for K (avoid divide-by-small)
            u_prev = max(min(guide_cmd(i-1),deg2rad(30)),-deg2rad(30));
            r_prev = nu(3,i-1);
            Khat   = (1-ctl.est.tau)*ctl.est.K + ctl.est.tau * (r_prev / max(1e-3, u_prev));
            Khat   = max(1e-3, abs(Khat));  % use magnitude
            That   = max(0.05, min(1.0, ctl.est.T)); % keep T in a sane band

            ctl.est.K = Khat; ctl.est.T = That;

            % design fresh gains from (K,T)
            z=ctl.zeta; wn=ctl.wn; p3=ctl.alpha*wn; Km=Khat;
            Kd = (That*(2*z*wn + p3) - 1) / Km;
            Kp = (That*(wn^2 + 2*z*wn*p3)) / Km;
            Ki = (That*(wn^2*p3)) / Km;

        otherwise
            error('Unknown mode');
    end

    % PID law (on heading error)
    if abs(guide_cmd(i-1)) < deg2rad(29.5)
        int_err = int_err + err_h(i) * (Vl.dt);
    end
    d_err = (err_h(i) - prev_err) / Vl.dt; prev_err = err_h(i);
    u_cmd = Kp*err_h(i) + Ki*int_err + Kd*d_err;
    u_cmd = max(min(u_cmd,deg2rad(30)),-deg2rad(30));
    guide_cmd(i) = -(cmd_smooth*u_cmd + (1-cmd_smooth)*guide_cmd(i-1));

    % angles
    [gAng, totalAng, ~, ~, guide_state] = ...
        calculate_guide_wave(guide_cmd,[],Vl.t,Vl.dt,i,params,guide_state,cmd_smooth);
    theta_base = base_amp * sin(2*pi*params.flap_freq*Vl.t(i));
    fAng = totalAng + theta_base + theta_const;

    % angular speed
    omega(i) = (fAng - prev_fAng)/Vl.dt; prev_fAng=fAng;

    % force & moment from fin
    [Ff, Mf] = compute_fin_force_corrected(eta(:,i-1), nu(:,i-1), fAng, omega(i), params);
    Ff_log(:,i) = Ff; Mf_log(i)=Mf;

    % damping + hull resistance
    Rh = 0.5*params.rho*params.Cd_hull*params.A_wetted * abs(nu(1,i-1)) * nu(1,i-1);

    % disturbances (body frame)
    tau_dist = D.tau(Vl.t(i), eta(:,i-1), nu(:,i-1));
    tau = [Ff(1)-Rh; Ff(2); Mf] + tau_dist;

    % integrate
    C_rb = coriolis_rb(nu(:,i-1), m, CrL, CW);
    C_a  = coriolis_am(nu(:,i-1), X_du, Y_dv);
    nu_dot = M_tot \ (tau - (C_rb + C_a + Damp(nu(:,i-1)))*nu(:,i-1));
    nu(:,i) = nu(:,i-1) + nu_dot * Vl.dt;

    % pose
    psi = eta(3,i-1); Rb=[cos(psi),-sin(psi); sin(psi),cos(psi)];
    eta(1:2,i) = eta(1:2,i-1) + Rb * nu(1:2,i) * Vl.dt;
    eta(3,i)   = psi + nu(3,i) * Vl.dt;
end

% package result
sim.t=Vl.t; sim.eta=eta; sim.nu=nu; sim.u=guide_cmd;
sim.des_h=des_h; sim.err_h=err_h; sim.wps=wps;
sim.Ff=Ff_log; sim.Mf=Mf_log; sim.mode=ctl.mode; sim.name=scn.name;
end

% Disturbance models
function D = dNone()
D.tau = @(t,eta,nu)[0;0;0];
end

function D = dCurrentNoise(bias_u, sigma_r)
% bias_u : constant body-x drag-like bias (N)
% sigma_r: white noise on yaw moment
D.tau = @(t,eta,nu) [ -bias_u; 0; sigma_r*randn ];
end

% Metrics
function T = compute_metrics(sim)
t=sim.t; err=wrapToPi(sim.err_h); u=sim.u;
psi = sim.eta(3,:); des = sim.des_h;

% RMS error over whole run
rms_err = sqrt(mean(err.^2));

% Settling time to within 5 degrees after last waypoint change
eps = deg2rad(5);
idx_change = find([false, diff(des)~=0], 1, 'last');
if isempty(idx_change), idx_change = 1; end
ts = NaN;
for k = idx_change:numel(t)
    win = k:numel(t);
    if all(abs(err(win)) < eps), ts = t(k)-t(idx_change); break; end
end

% Overshoot: max |psi - des| in first 10 s after last change minus final abs error
win = t >= t(idx_change) & t <= (t(idx_change)+10);
overshoot = max(abs(err(win)),[],'omitnan');

% Effort proxy (integral absolute guide + fin moment)
J_u = trapz(t, abs(u));
J_f = trapz(t, abs(sim.Mf));

T = table(rms_err, ts, overshoot, J_u, J_f, ...
    'VariableNames', {'rms_err','settle_time_s','overshoot_rad','guide_effort','fin_effort'});
end

% Saving plots
function files = save_core_plots(sim, tag)
t=sim.t; eta=sim.eta; u=sim.u; des=sim.des_h; err=wrapToPi(sim.err_h);

% 1) Trajectory
f1=figure('Name',[tag ' Trajectory']); hold on; axis equal; grid on;
plot(eta(1,:),eta(2,:),'b-'); scatter(sim.wps(:,1),sim.wps(:,2),40,'r','filled');
xlabel('x'); ylabel('y'); title(['Trajectory - ' strrep(tag,'_','\_')]);
files{1} = [tag '_traj.png']; saveas(f1, files{1});

% 2) Heading vs desired
f2=figure('Name',[tag ' Heading']); 
plot(t, unwrap(eta(3,:)),'k','LineWidth',1.2); hold on;
plot(t, unwrap(des),'r--'); grid on;
xlabel('t (s)'); ylabel('\psi (rad)'); legend('actual','desired');
title(['Heading - ' strrep(tag,'_','\_')]);
files{2} = [tag '_heading.png']; saveas(f2, files{2});

% 3) Error and command
f3=figure('Name',[tag ' Error & Command']);
yyaxis left; plot(t, err,'LineWidth',1.2); ylabel('e_\psi (rad)');
yyaxis right; plot(t, rad2deg(u),'LineWidth',1.0); ylabel('u (deg)');
grid on; xlabel('t (s)'); title(['Error & Command - ' strrep(tag,'_','\_')]);
files{3} = [tag '_error_cmd.png']; saveas(f3, files{3});

end



