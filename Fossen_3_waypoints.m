% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
% Fossen 2 - Waypoint heading
% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

% Vehicle parameters
m = 5;
Iz = 0.2;
X_u = -5;
Y_v = -10;
N_r = -2;

% Mass matrix
M = [m , 0, 0; ...
     0 , m, 0; ...
     0 , 0, Iz];

% Damping matrix
D = @(nu) diag([-X_u * abs(nu(1)), -Y_v * abs(nu(2)), -N_r * abs(nu(3))]);

CL = 0.7; % Craft length
CW = 0.1; % Craft width
boatShape = [-CL/2, -CW/2;
             -CL/2,  CW/2;
              CL/2,  CW/2;
              CL/2, -CW/2];

N = length(Vl.t);
nu = zeros(3,N);
eta = zeros(3,N);


k_thrust = 0.2;
F_thrust = k_thrust * omega.^2;


% Control Parameters
waypoints_deg = [10, -20, 40]; % Target heading(degrees)
waypoints_rad = deg2rad(waypoints_deg);
wp_Cond = 0;
index_wp = 1;
tolerance = deg2rad(2);


Kp_R = 1.5; % Proportional gain of rudder
Kd_R = 0.8; % Derivative coefficient
rudder_cmd = zeros(size(Vl.t)); % Allocate rudder command array

% Turning dynamics
heading = zeros(1,N);
angVel = zeros(1,N);
posX = zeros(1,N);
posY = zeros(1,N);

% Moment of inertia & torque gain
I = 0.1; 
rudderTorqueGain = 0.3;

% Graphics setup
figure;
hold on;
axis equal;
xlim([-5, 5]);
ylim([-5, 5]);
grid on;
xlabel('Displacement (m)');
ylabel('Y (m)');
title('Craft with Flapping Fin and Vortex Wake');

% Craft Shape
boatScale = 1;
rudderLength = 0.2;
L_r = 0.35;

% Initial craft plotting
R0 = eye(2);
boatWorld = (boatShape * boatScale) * R0' + [posX(1), posY(1)];
hCraft = fill(boatWorld(:,1), boatWorld(:,2), [0.2 0.6 1.0], 'EdgeColor','k','LineWidth',1.5);

% Rudder and fin
hRudder = line([0,0],[0,0],'LineWidth',2,'Color',[0.5 0.5 0.5]);
hFin = line([0,0],[0,0],'LineWidth',2,'Color','k');
finLength = 0.15;

% Trail
trail = animatedline('Color','b','LineWidth',1.2,'LineStyle','--');

% Generate rudder sway angle array
rudderAngle = A_rudder * sin(2 * pi * f_rudder * Vl.t);



% Animation Loop
for i = 2:N
    
    if index_wp <= length(waypoints_rad)
        current_target = waypoints_rad(min(index_wp, length(waypoints_rad)));
        heading_error = wrapToPi(current_target - eta(3, i-1));

        % PD goes within
         % Dynamics for heading
        rudder_cmd(i) = Kp_R * heading_error + Kd_R * (-nu(3, i-1));
        rudder_cmd(i) = max(min(rudder_cmd(i), deg2rad(15)), deg2rad(-15));
        
        if abs(heading_error) < tolerance
            wp_Cond = wp_Cond + 1;
            if wp_Cond >= 10 % Check for 10 constant errors before passing logic
                index_wp = index_wp + 1;
                wp_Cond = 0;
            end 
        else 
            wp_Cond = 0;
        end
    else
        current_target = waypoints_rad(end);
        heading_error = wrapToPi(current_target - eta(3, i-1));
        rudder_cmd(i) = Kp_R * heading_error + Kd_R * (-nu(3, i-1));
        rudder_cmd(i) = max(min(rudder_cmd(i), deg2rad(15)), deg2rad(-15));
    end
        


   

    % Update turning dynamics
    U = norm(nu(1:2, i-1));
    delta = rudder_cmd(i);
    C_lift = 5* sin(delta);
    A_r = 0.02;
    rho = 1000;
    F_r = 0.5 * rho * A_r * C_lift * U^2;
    
    tau_X = F_thrust(i);
    tau_Y = F_r * cos(delta);
    tau_N = F_r * L_r * cos(delta);

    tau = [tau_X; tau_Y; tau_N];

    nu_dot = M \ (tau - D(nu(:,i-1)) * nu(:,i-1));
    nu(:,i) = nu(:,i-1) + nu_dot * Vl.dt;

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
    

    R_rudder = [cos(rudder_cmd(i)), -sin(rudder_cmd(i));
                sin(rudder_cmd(i)),  cos(rudder_cmd(i))];



    R_fin = [cos(-thetaP(i)), -sin(-thetaP(i));
             sin(-thetaP(i)),  cos(-thetaP(i))];
    
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