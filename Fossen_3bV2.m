% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
% Fossen 3b - Waypoint (X,Y)
% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-


Vl.t_end = 60;              % Length of experiment
Vl.dt = 0.01;               % Time step
Vl.t = 0:Vl.dt:Vl.t_end;    % Time evolution
N = length(Vl.t);           % Vector for time use

finLength = 0.15;           % Length of fin head
rudderLength = 0.2;         % Length of rudder head
L_r = 0.35;                 % Length from rudder base to CG (CM)    


A_fin = deg2rad(25);                    % Amplitude (rads) of fin 
F_fin = 1.0;                            % Fin motion frequency
phi_fin = deg2rad(90);                  % Phase of fin
thetaP = A_fin * sin(2* F_fin * Vl.t + phi_fin); % Pitch angle for fin (Theta)
omega = gradient(thetaP, Vl.dt);        % Angular veloctiy of fin motion (Assumption of perfe

% Generate Rudder logic
A_rudder = deg2rad(30);     % Amplitude (rads) of Rudder 
f_rudder = 1.0;             % Rudder motion frequency
% rudderAngle = A_rudder * sin(2 * pi * f_rudder * Vl.t); 

% Vehicle parameters
m = 5;

Iz = 0.2;
X_u = -5;       % Surge damping
Y_v = -10;      % Sway damping
N_r = -2;       % Yaw damping

% Mass matrix
M = [m , 0, 0; ...
     0 , m, 0; ...
     0 , 0, Iz];

% Damping matrix
D = @(nu) diag([-X_u * abs(nu(1)), -Y_v * abs(nu(2)), -N_r * abs(nu(3))]);

k_thrust = 0.2;
F_thrust = k_thrust * omega.^2;

% Control Parameters
waypoints = [1.5  1;
             2.5 0.5;
             4  1;
             5 2]; % Waypoint cordinates

wp_index = 1;       % First waypoint following 
wp_tolerance = 0.3; % Radius to switch before
wp_Cond = 0;

CL = 0.7; % Craft length
CW = 0.1; % Craft width
boatScale = 1.0;
boatShape = [-CL/2, -CW/2;
             -CL/2,  CW/2;
              CL/2,  CW/2;
              CL/2, -CW/2];

nu = zeros(3,N);       % Velocity matrix
eta = zeros(3,N);      % Position matrix


% waypoints_deg = [10, -20, 40]; % Target heading(degrees)
% waypoints_rad = deg2rad(waypoints_deg);

Kp_R = 1.5; % Proportional gain of rudder
Kd_R = 0.8; % Derivative coefficient
rudder_cmd = zeros(size(Vl.t)); % Allocate rudder command array

% Graphics setup
figure;
hold on;
axis equal;
xlim([-6, 6]);
ylim([-6, 6]);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Waypoint tracking');
plot(waypoints(:,1), waypoints(:,2), 'rx--', 'LineWidth', 1.2);

% Initial craft plotting
R0 = eye(2);
boatWorld = (boatShape * boatScale) * R0' + [0, 0];
hCraft = fill(boatWorld(:,1), boatWorld(:,2), [0.2 0.6 1.0], 'EdgeColor','k','LineWidth',1.5);

% Rudder and fin
hRudder = line([0,0],[0,0],'LineWidth',2,'Color',[0.5 0.5 0.5]);
hFin = line([0,0],[0,0],'LineWidth',2,'Color','k');

% Trail
trail = animatedline('Color','b','LineWidth',1.2,'LineStyle','--');

% Animation Loop
for i = 2:N
    
    if wp_index <= size(waypoints, 1)
        
        % Change rate between x and y coordinates
        dx = waypoints(wp_index,1) - eta(1,i-1);
        dy = waypoints(wp_index,2) - eta(2,i-1);
        des_heading = atan2(dy,dx);
        heading_error = wrapToPi(des_heading - eta(3,i-1));
        


        % PD goes within
        % Dynamics for heading
        rudder_raw = Kp_R * heading_error + Kd_R * (-nu(3, i-1));       % Rudder raw angle
        rudder_raw = max(min(rudder_raw, deg2rad(30)), deg2rad(-30));   % Bound data to 30°
        alpha = 0.5;                                                    % saturation index
        rudder_cmd(i) = alpha * rudder_raw + (1 - alpha) * rudder_cmd(i-1); % Saturated and filtered


        dist_to_wp = sqrt(dx^2 + dy^2); % Distance to waypoint
        
        % Tracking of reach
        if dist_to_wp < wp_tolerance && wp_index < size(waypoints, 1)
            wp_index = wp_index + 1;
        end
    else
        % None saturated dynamics
        des_heading = atan2(waypoints(end,2) - eta(2, i-1), ...
                            waypoints(end,1) - eta(1, i-1));
        heading_error = wrapToPi(des_heading - eta(3,i-1));
        rudder_cmd(i) = Kp_R * heading_error + Kd_R * (-nu(3, i-1));
        rudder_cmd(i) = max(min(rudder_cmd(i), deg2rad(30)), deg2rad(-30));        
        
    end
        

    % Update turning dynamics
    U = norm(nu(1:2, i-1));
    delta = rudder_cmd(i);                  % Store angle position
    C_lift = 5* sin(delta);
    A_r = 0.02;                             % Rudder amplitude
    rho = 1000;
    F_r = 0.5 * rho * A_r * C_lift * U^2;
    
    tau_X = F_thrust(i);
    tau_Y = F_r * cos(delta);
    tau_N = F_r * L_r * cos(delta);

    tau = [tau_X; tau_Y; tau_N];            

    nu_dot = M \ (tau - D(nu(:,i-1)) * nu(:,i-1));   
    nu(:,i) = nu(:,i-1) + nu_dot * Vl.dt;           % Velocity matrix

    % RT_Kinematics update
    psi = eta(3,i-1);
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    eta(1:2,i) = eta(1:2,i-1) + R * nu(1:2,i) * Vl.dt;
    eta(3,i) = eta(3,i-1) + nu(3,i) * Vl.dt;
    
    % Animation 
    psi = eta(3,i);

    R_heading = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    boatWorld = (boatShape * boatScale) * R_heading' + eta(1: 2,i)';
    set(hCraft, 'XData', boatWorld(:,1), 'YData', boatWorld(:,2));
    
    % Rudder roatiton
    R_rudder = [cos(rudder_cmd(i)), -sin(rudder_cmd(i));
                sin(rudder_cmd(i)),  cos(rudder_cmd(i))];


    % Fin rotation
    R_fin = [cos(thetaP(i)), -sin(thetaP(i));
             sin(thetaP(i)),  cos(thetaP(i))];
    
    % Rudder position
    rudder_local = [-(CL/2); 0];
    rudderBase = eta(1:2,i) + R_heading * rudder_local;
    rudderDir_local = [-1; 0];
    rudderDir_world = R_heading * R_rudder * rudderDir_local;
    rudderTip = rudderBase + rudderLength * rudderDir_world;
    set(hRudder, 'XData', [rudderBase(1), rudderTip(1)], ...
                 'YData', [rudderBase(2), rudderTip(2)]);
    
    % Fin flapping
    finDir_local = [-1;0];
    finDir_world = R_heading * R_rudder * R_fin * finDir_local;
    finBase = rudderTip;
    finTip = finBase + finLength * finDir_world;
    set(hFin, 'XData', [finBase(1), finTip(1)], ...
              'YData', [finBase(2), finTip(2)]);
    
    % Trail update
    addpoints(trail, eta(1,i), eta(2,i));
    
    drawnow;
    pause(Vl.dt * 0.5);
end
