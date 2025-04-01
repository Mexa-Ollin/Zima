%% 1st test, Fin motion simulator for the consideration of design in robot implementation.

% Governing behavior (Sine wave)
% y(x,t) = A* sin (lambda * x - omega * t)
% A - Amplitude of fin's motion
% k - wave number (in relation to wave length - lambda)
% omega - Angular frequency (omega = 2 *pi * f)
% x - Position along the fin
% t - time :v

% Global parameters.
A = 1;
lambda = 2; % Reduction could aid with positioning
f = 1; % Increase related to motion a.w.t. speed, consider an upper limit. 
omega = 2 * pi * f;
k = 2 * pi / lambda; % Wavenumber
L = 5; % Length of fin
x = linspace(0, L, 50); % Discretize fin along its length

% Time evolution
t_end = 5; % Time duration of experiment
dt = 0.05; % Time step
t = 0:dt:t_end; % Time vector

% Initialize the plot
figure;
h = plot(x,A * sin(k*x)); 
axis([0 L -A A]);
xlabel('Fin length');
ylabel('Displacement');
title('Simulation of fin motion');

% Animation loop
for i = 1:length(t)
    y = A * sin(k * x - omega * t(i));
    set(h,'yData', y);
    pause(0.05);
end



