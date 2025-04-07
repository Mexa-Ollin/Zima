%% Fin Motion Simulator with Real-Time Adjustments (Template)

A = 1;         % Initial amplitude
lambda = 2;    % Wavelength
f = 1;         % Initial frequency
C = 0;         % Initial vertical shift
omega = 2 * pi * f;
k = 2 * pi / lambda;
L = 5;                  % Length of fin
x = linspace(0, L, 50); % Discretization of fin

% Time evolution parameters
t_end = 15;         % Duration
dt = 0.05;          % Time step
t = 0:dt:t_end;     % Time vector

% Initialize figure
figure;
h = plot(x, A * sin(k * x) + C); 
axis([0 L -2 2]);   % Adjusted for visibility
xlabel('Fin length');
ylabel('Displacement');
title('Simulation of fin motion');

% Add UI controls for interactive parameter changes
uicontrol('Style', 'text', 'String', 'Amplitude', 'Position', [10, 120, 100, 20]);
sA = uicontrol('Style', 'slider', 'Min', 0.1, 'Max', 2, 'Value', A, ...
               'Position', [10, 100, 100, 20]);

uicontrol('Style', 'text', 'String', 'Frequency', 'Position', [10, 80, 100, 20]);
sf = uicontrol('Style', 'slider', 'Min', 0.1, 'Max', 3, 'Value', f, ...
               'Position', [10, 60, 100, 20]);

uicontrol('Style', 'text', 'String', 'Vertical Shift', 'Position', [10, 40, 100, 20]);
sC = uicontrol('Style', 'slider', 'Min', -1, 'Max', 1, 'Value', C, ...
               'Position', [10, 20, 100, 20]);

% Animation loop
for i = 1:length(t)
    % Read updated values from sliders
    A = get(sA, 'Value');
    f = get(sf, 'Value');
    C = get(sC, 'Value'); % Read vertical shift
    
    % Update dependent parameters
    omega = 2 * pi * f;
    
    % Compute displacement
    y = A * sin(k * x - omega * t(i)) + C;
    
    % Update plot
    set(h, 'YData', y);
    pause(0.05);
end
