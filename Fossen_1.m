% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o
% Fossen_1
% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

% Craft elements
Mass = 5; % (Kg)
I_z = 0.5; % Yaw Inertia (kg*m^2)
    
% Mass matrix
M = [mass,    0, 0;
     0   , mass, 0;
     0   ,    0, mass];

% Damping Coefficient
X_u = -10;
Y_v = -20;
N_r = -5;

% Damping matrix
D = @(nu) diag([X_u * abs(nu(1)), Y_v * abs(nu(2)), N_r * abs(nu(3))]);

nu = [0;0;0];
eta = [0;0;0];

% Force vectors (Tau)
F_thrust = k_thrust * omega.^2;
tau_x = F_thrust(i);

% Control Force
rho = 1000;
A_r = 0.02; 
k_r = 5;
l_r = 0.35;

% Inside simulation loop
delta = rudder_cmd(i);
U = vel(i);
CL = k_r * delta;
F_r = 0.5 * rho * A_r * CL * U^2;

% Force in body-fixed frame (perpendicular to surge)
tau_Y = -F_r * cos(delta);
tau_N = -F_r * l_r * cos(delta);

tau = [F_thrust(i); tau_Y; tau_N];

% Vehicle parameters
m = 5;
Iz = 0.2;
X_u = -5;
Y_v = -10;
N_r = -2;

M = diag([m , m, Iz]);
D = diag([-X_u, -Y_v, -N_r]);

N = length(Vl.t);
nu = [u; v; r];
eta = [x; y; psi];

nu_dot = M \ (tau - D * nu);
nu = nu + nu_dot * dt;

R = [cos(eta(3)), -sin(eta(3)); sin(eta(3)), cos(eta(3))];
eta(1:2) = eta(1:2) + R * nu(1:2) *dt;
eta(3) = eta(3) + nu(3) * dt;
