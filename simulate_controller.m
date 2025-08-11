function sim = simulate_controller(scn, ctl, id_results) %#ok<INUSD>

%% --- basic sim setup (matches your ID sim) ---
Vl.t_end = 60; 
Vl.dt    = 0.02; 
Vl.t     = 0:Vl.dt:Vl.t_end; 
N        = numel(Vl.t);

CrL=0.5; CW=0.25;
finLength=0.2; guideLength=0.25;

% Physical
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

MRB=diag([m,m,Iz]); 
MA =[X_du,0,0; 0,Y_dv,Y_dv; 0,Y_dv,N_dr]; 
M_tot=MRB+MA;

Damp = @(nu)[ X_u + params.X_uu*abs(nu(1))   0                    0;
              0      Y_v + params.Y_vv*abs(nu(2))   -0.05*nu(2);
              0      -0.05*nu(1)          N_r + params.N_rr*abs(nu(3)) ];

% State/logging
nu=zeros(3,N); eta=zeros(3,N);
guide_cmd=zeros(1,N);
omega=zeros(1,N); prev_fAng=0;
des_h=zeros(1,N); err_h=zeros(1,N);
Mf_log=zeros(1,N); Ff_log=zeros(2,N);

% base flap
base_amp=deg2rad(15); theta_const=deg2rad(8);

guide_state=struct('phase',0,'last_freq',params.flap_freq,'last_guide',0);
cmd_smooth=0.7;

% disturbance
D = scn.dist;

% waypoint manager
wps = scn.wps; if size(wps,1)==1, wps=[wps; wps]; end
wpi=1; wp_current=wps(wpi,:);

% PID integrator state
int_err=0; prev_err=0;

%% --- main loop ---
for i=2:N
    % switch waypoint when close
    if norm(wp_current - eta(1:2,i-1)') < 0.3 && wpi < size(wps,1)
        wpi = wpi+1;
        wp_current = wps(wpi,:);
        int_err = 0;            % reset I on waypoint switch
        prev_err = 0;
    end

    % desired heading + error
    des_h(i) = atan2(wp_current(2)-eta(2,i-1), wp_current(1)-eta(1,i-1));
    err_h(i) = wrapToPi_local(des_h(i) - eta(3,i-1));

    dist_to_wp = norm(wp_current - eta(1:2,i-1)');
    if dist_to_wp < 0.8
        speed_factor = min(1, dist_to_wp/0.5);
        theta_const  = deg2rad(8)  * speed_factor;
        base_amp     = deg2rad(15) * speed_factor;
    else
        theta_const  = deg2rad(8);
        base_amp     = deg2rad(15);
    end
    switch ctl.mode
        case 'fixed'
            Kp=ctl.gains(1); Ki=ctl.gains(2); Kd=ctl.gains(3);

        case 'scheduled'
            [Kp,Ki,Kd] = ctl.query(err_h(i)); % schedule vs relative heading

        case 'adaptive'
            % quick online K,T estimate -> fresh gains
            u_prev = max(min(guide_cmd(i-1),deg2rad(30)),-deg2rad(30));
            r_prev = nu(3,i-1);
            Khat   = (1-ctl.est.tau)*ctl.est.K + ctl.est.tau * (r_prev / max(1e-3, u_prev));
            Khat   = max(1e-3, abs(Khat));
            That   = max(0.05, min(1.0, ctl.est.T));
            ctl.est.K = Khat; ctl.est.T = That;

            z=ctl.zeta; wn=ctl.wn; p3=ctl.alpha*wn; Km=Khat;
            Kd = (That*(2*z*wn + p3) - 1) / Km;
            Kp = (That*(wn^2 + 2*z*wn*p3)) / Km;
            Ki = (That*(wn^2*p3)) / Km;

        otherwise
            error('Unknown controller mode: %s', ctl.mode);
    end

    % PID law on heading error -> guide command
    if abs(guide_cmd(i-1)) < deg2rad(29.5)
        int_err = int_err + err_h(i) * Vl.dt;
    end
    d_err = (err_h(i) - prev_err) / Vl.dt; prev_err = err_h(i);
    u_cmd = Kp*err_h(i) + Ki*int_err + Kd*d_err;

    % anti-windup + small leak
    if abs(u_cmd) >= deg2rad(30) || sign(err_h(i)) ~= sign(prev_err)
        int_err = 0.9 * int_err;
    end

    u_cmd = max(min(u_cmd,deg2rad(30)),-deg2rad(30));
    guide_cmd(i) = -(cmd_smooth*u_cmd + (1-cmd_smooth)*guide_cmd(i-1));

    % angles
    [gAng, totalAng, ~, ~, guide_state] = ...
        calculate_guide_wave_local(guide_cmd,[],Vl.t,Vl.dt,i,params,guide_state,cmd_smooth);
    theta_base = base_amp * sin(2*pi*params.flap_freq*Vl.t(i));
    fAng = totalAng + theta_base + theta_const;

    % angular speed
    omega(i) = (fAng - prev_fAng)/Vl.dt; prev_fAng=fAng;

    % fin forces
    [Ff, Mf] = compute_fin_force_corrected_local(eta(:,i-1), nu(:,i-1), fAng, omega(i), params);
    Ff_log(:,i) = Ff; Mf_log(i)=Mf;

    % hull resistance
    Rh = 0.5*params.rho*params.Cd_hull*params.A_wetted * abs(nu(1,i-1)) * nu(1,i-1);

    % disturbances
    tau_dist = D.tau(Vl.t(i), eta(:,i-1), nu(:,i-1));

    % net generalized force
    tau = [Ff(1)-Rh; Ff(2); Mf] + tau_dist;

    % integrate
    C_rb = coriolis_rb_local(nu(:,i-1), m, CrL, CW);
    C_a  = coriolis_am_local(nu(:,i-1), X_du, Y_dv);
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

% local helpers (self-contained)
function [gAng, totalAng, thetaP, fFreq, st] = calculate_guide_wave_local(cmd,~,t,dt,i,params,st,sm) %#ok<INUSD,INUSL>
f     = params.flap_freq;
phase = st.phase + 2*pi*f*dt;
A     = params.base_flap_amp;

% commanded guide bias
if ~isempty(cmd) && numel(cmd) >= i
    gBias = cmd(i);
else
    gBias = 0;
end

thetaP = A * cos(phase);
if sin(phase) >= 0
    asym = 1 + params.stroke_asym;
else
    asym = 1 - params.stroke_asym;
end
rawAng    = -thetaP;
totalAng  = asym * rawAng;
gAng      = gBias;
fFreq     = f;

st.phase     = phase;
st.last_freq = f;
end

function C = coriolis_rb_local(nu,m,~,~)
u=nu(1);v=nu(2);
C = [ 0    0    -m*v;
      0    0     m*u;
      m*v -m*u    0  ];
end

function C = coriolis_am_local(nu,X_du,Y_dv)
u=nu(1);v=nu(2);
C = [ 0        0         Y_dv*v;
      0        0        -X_du*u;
     -Y_dv*v   X_du*u     0     ];
end

function [F_fin, M_fin] = compute_fin_force_corrected_local(eta, nu, finAng, omega, params)
if isfield(params,'N_elements') && ~isempty(params.N_elements)
    N = params.N_elements;
else
    N = 10;
end
span = params.finLength;
rho  = params.rho;
c    = params.chord;

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

    if abs(a) <= deg2rad(15)
        CL = 1.5*sin(2*a);
        CD = params.CD_min + 0.5*(1 - cos(2*a));
    else
        CL = sign(a)*params.CL_max*(sin(a))^2;
        CD = 1.1 - 0.5*cos(2*a);
    end

    area = c * (span/N) * params.finDepth;
    dL = 0.5 * rho * Umag^2 * area * CL * vel_scaling;
    dD = 0.5 * rho * Umag^2 * area * CD;

    f = [cos(a)*dD - sin(a)*dL;    % thrust-like
         cos(a)*dL + sin(a)*dD];   % lift-like

    fb = Rb * Rf * f;
    r  = base + Rf * [0; y];
    M  = M + (r(1)*fb(2) - r(2)*fb(1));
    F  = F + fb;
end

F_fin = -F;
M_fin = -M;
end

function y = wrapToPi_local(x)
% simple wrap to [-pi,pi]
y = mod(x+pi, 2*pi) - pi;
end