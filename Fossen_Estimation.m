m = 10;
I_z = 0.21;
x_g = 0.05;
rho = 1000;
A = 0.04;
C_L = 0.9;
Theta0 = deg2rad(16);
f = 2;
L_pivot = 0.25;


X_d = -2; Y_d = -4; N_d = -0.8; N_r = -1.2; % Estimates from gpt

FossenEqs = @(t, state)[
    (1/m) * ( (0.5 * rho * A * C_L * (0.4 * (2*pi*f*Theta0) * cos(2*pi*f*t))^2) * cos(Theta0*sin(2*pi*f*t)) - X_d*state(1) );
    
    % Sway acceleration
    (1/m) * ( (0.5 * rho * A * C_L * (0.4 * (2*pi*f*Theta0) * cos(2*pi*f*t))^2) * sin(Theta0*sin(2*pi*f*t)) - Y_d*state(2) );
    
    % Yaw acceleration
    (1/I_z) * ( ((0.5 * rho * A * C_L * (0.4 * (2*pi*f*Theta0) * cos(2*pi*f*t))^2) * sin(Theta0*sin(2*pi*f*t))) * L_pivot - N_d*state(3) - N_r*state(3) );
]

% Initial conditions.
initState = [0;0;0];

% Solve the ODE system
[t, sol] = ode45(FossenEqs, [0 10], initState);

% Extract velocity components
u = sol(:,1); v = sol(:,2); r = sol(:,3);

% Plot results
figure;
subplot(3,1,1);
plot(t, u, 'b', 'LineWidth', 2);
ylabel('Surge velocity (m/s)'); grid on;

subplot(3,1,2);
plot(t, v, 'r', 'LineWidth', 2);
ylabel('Sway velocity (m/s)'); grid on;

subplot(3,1,3);
plot(t, rad2deg(r), 'g', 'LineWidth', 2);
ylabel('Yaw rate (deg/s)'); xlabel('Time (s)'); grid on;
title('Fossen’s Equations for Small Vessel');