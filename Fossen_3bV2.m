% Fossen-style vehicle simulation with rudder + flapping fin
clear; clc;

%% Time setup
Vl.t_end = 30;
Vl.dt = 0.01;
Vl.t = 0:Vl.dt:Vl.t_end;
N = length(Vl.t);

%% Fin parameters
finLength = 0.15;
A_fin = deg2rad(25);
F_fin = 1.0;
phi_fin = deg2rad(90);
thetaP = A_fin * sin(2 * F_fin * Vl.t + phi_fin);
omega = gradient(thetaP, Vl.dt);
F_thrust = 0.2 * omega.^2;

%% Rudder parameters
rudderLength = 0.2;
L_r = 0.35;  % Rudder offset from CG
rudder_cmd = zeros(1, N);

%% Vehicle parameters
m = 5;
Iz = 0.2;
X_u = -5; Y_v = -10; N_r = -2;

M = diag([m, m, Iz]);  % Inertia matrix
D = @(nu) diag([-X_u*abs(nu(1)), -Y_v*abs(nu(2)), -N_r*abs(nu(3))]);

%% Initial state
nu = zeros(3, N);   % [u; v; r]
eta = zeros(3, N);  % [x; y; psi]

%% Waypoints
waypoints = [3  2; 6 -1; 7  2; 5 4];
wp_index = 1;
wp_tol = 0.3;

%% Control gains
k_psi = 1.0;   % Heading proportional gain
k_r = 2.0;     % Yaw rate tracking gain
rudder_max = deg2rad(30);

%% Geometry for plotting
CL = 0.7; CW = 0.1; boatScale = 1.0;
boatShape = [-CL/2, -CW/2; -CL/2, CW/2; CL/2, CW/2; CL/2, -CW/2];

%% Plot setup
figure;
hold on; axis equal; grid on;
xlim([-8, 8]); ylim([-8, 8]);
plot(waypoints(:,1), waypoints(:,2), 'rx--', 'LineWidth', 1.2);
xlabel('X'); ylabel('Y'); title('Yaw-Rate Controlled Craft');

R0 = eye(2);
boatWorld = (boatShape * boatScale) * R0' + eta(1:2,1)';
hCraft = fill(boatWorld(:,1), boatWorld(:,2), [0.2 0.6 1.0], 'EdgeColor','k');
hRudder = line([0,0],[0,0],'LineWidth',2,'Color',[0.5 0.5 0.5]);
hFin = line([0,0],[0,0],'LineWidth',2,'Color','k');
trail = animatedline('Color','b','LineWidth',1.2,'LineStyle','--');

rho = 1000; A_r = 0.02;

%% Main simulation loop
for i = 2:N
    % Compute waypoint-based heading
    if wp_index <= size(waypoints,1)
        wp = waypoints(wp_index,:)';
        dx = wp(1) - eta(1,i-1);
        dy = wp(2) - eta(2,i-1);
        dist_to_wp = norm([dx; dy]);

        if dist_to_wp < wp_tol && wp_index < size(waypoints,1)
            wp_index = wp_index + 1;
        end

        psi_d = atan2(dy, dx);  % desired heading
    else
        psi_d = eta(3,i-1);  % hold heading
    end

    psi = eta(3,i-1);
    r = nu(3,i-1);

    e_psi = wrapToPi(psi_d - psi);
    r_d = k_psi * e_psi;
    delta = k_r * (r_d - r);
    delta = max(min(delta, rudder_max), -rudder_max);
    rudder_cmd(i) = delta;

    % Hydrodynamic rudder force and moment
    U = norm(nu(1:2,i-1));
    C_lift = 5 * sin(delta);
    F_r = 0.5 * rho * A_r * C_lift * U^2;

    tau = [F_thrust(i); -F_r; -F_r * L_r];

    % Rigid body dynamics
    nu_dot = M \ (tau - D(nu(:,i-1)) * nu(:,i-1));
    nu(:,i) = nu(:,i-1) + nu_dot * Vl.dt;

    % Kinematics
    psi = eta(3,i-1);
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    eta(1:2,i) = eta(1:2,i-1) + R * nu(1:2,i) * Vl.dt;
    eta(3,i) = eta(3,i-1) + nu(3,i) * Vl.dt;

    % Update visuals
    psi = eta(3,i);
    R_heading = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    boatWorld = (boatShape * boatScale) * R_heading' + eta(1:2,i)';
    set(hCraft, 'XData', boatWorld(:,1), 'YData', boatWorld(:,2));

    R_rudder = [cos(delta), -sin(delta); sin(delta), cos(delta)];
    R_fin = [cos(thetaP(i)), -sin(thetaP(i)); sin(thetaP(i)), cos(thetaP(i))];

    rudder_local = [-CL/2; 0];
    rudderBase = eta(1:2,i) + R_heading * rudder_local;
    rudderTip = rudderBase + rudderLength * (R_heading * R_rudder * [-1;0]);
    set(hRudder, 'XData', [rudderBase(1), rudderTip(1)], 'YData', [rudderBase(2), rudderTip(2)]);

    finBase = rudderTip;
    finTip = finBase + finLength * (R_heading * R_rudder * R_fin * [-1;0]);
    set(hFin, 'XData', [finBase(1), finTip(1)], 'YData', [finBase(2), finTip(2)]);

    addpoints(trail, eta(1,i), eta(2,i));
    drawnow;
end

%% Results
disp(['Max surge speed: ', num2str(max(nu(1,:)))]);
disp(['Max sway speed: ', num2str(max(nu(2,:)))]);
disp(['Max yaw rate: ', num2str(rad2deg(max(nu(3,:))))]);

% Plot diagnostics
figure;
subplot(3,1,1);
plot(Vl.t, rad2deg(rudder_cmd));
ylabel('Rudder (deg)'); title('Rudder Command'); grid on;

subplot(3,1,2);
plot(Vl.t, rad2deg(nu(3,:)));
ylabel('Yaw Rate (deg/s)'); title('Yaw Rate'); grid on;

subplot(3,1,3);
plot(Vl.t, F_thrust);
xlabel('Time (s)'); ylabel('Thrust (N)'); title('Thrust Over Time'); grid on;