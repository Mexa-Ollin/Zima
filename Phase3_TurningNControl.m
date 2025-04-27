% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
% Phase 3 - Turning (Almost there)
% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

% Rudder setup
A_rudder = deg2rad(10);  % Rudder sway amplitude
f_rudder = 1.0;          % Rudder sway frequency
rudderLength = 0.2;      % Length of rudder (m)

rudderAngle = A_rudder * sin(2 * pi * f_rudder * Vl.t); % Swaying rudder profile

% Fin setup (linked to rudder, partially synchronized)
syncFactor = 0.5;         % 0 = independent, 1 = locked
Vl.A = deg2rad(20);       % Fin sway amplitude
Vl.fp = f_rudder;         % Flapping frequency same as rudder
Vl.phi1 = deg2rad(10);    % Phase offset

thetaP = Vl.A * ((1-syncFactor)*sin(2*pi*Vl.fp*Vl.t + Vl.phi1) + syncFactor*sin(2*pi*f_rudder*Vl.t));
omega = gradient(thetaP, Vl.dt);  % Angular velocity of fin

% Forward dynamics setup
m = 5;              % Mass (kg)
k_thrust = 0.2;     % Thrust gain
rho = 1000;         % Water density
cd = 1.0;           % Drag coefficient
A_front = 0.025 * 0.07;  % Frontal area (m²)

F_thrust = k_thrust * omega.^2;
acc = zeros(1,N);
vel = zeros(1,N);
dispX = zeros(1,N);

for i = 2:N
    F_drag = 0.5 * rho * cd * A_front * vel(i-1)^2;
    F_net = F_thrust(i) - F_drag * sign(vel(i-1));
    acc(i) = F_net / m;
    vel(i) = vel(i-1) + acc(i) * Vl.dt;
    dispX(i) = dispX(i-1) + vel(i) * Vl.dt;
end

% Turning dynamics
heading = zeros(1,N);
angVel = zeros(1,N);
posX = zeros(1,N);
posY = zeros(1,N);

% Moment of inertia & torque gain
I = 0.1; 
rudderTorqueGain = 0.3;

%% === Vortex Wake Initialization ===
Fw.vortex_speed = 0.05;
Fw.max_vortices = 100;
Fw.arrowLength = 0.1;
Fw.vortices = [];
Fw.vortex_signs = [];
Fw.colors = [1.0 0.3 0.3; 0.1 0.6 1.0];
Fw.vortexHandles = gobjects(Fw.max_vortices, 1);
Fw.last_sign = sign(omega(1));

%% === Graphics setup ===

figure;
hold on;
axis equal;
xlim([-1, max(dispX)*3]);
ylim([-1 1]);
grid on;
xlabel('Displacement (m)');
ylabel('Y (m)');
title('Craft with Flapping Fin and Vortex Wake');

% Craft Shape
boatShape = [-1 0; -1 0.5; -1 -0.5];
boatScale = 0.4;

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

%% === Animation Loop ===
for i = 2:N

    % Update turning dynamics
    torque = rudderTorqueGain * sin(rudderAngle(i));
    angAcc = torque / I;
    angVel(i) = angVel(i-1) + angAcc * Vl.dt;
    heading(i) = heading(i-1) + angVel(i) * Vl.dt;
    
    % Update position
    posX(i) = posX(i-1) + vel(i) * cos(heading(i)) * Vl.dt;
    posY(i) = posY(i-1) + vel(i) * sin(heading(i)) * Vl.dt;
    
    % Move craft (rotation + translation)
    R = [cos(heading(i)) -sin(heading(i));
         sin(heading(i))  cos(heading(i))];
    boatWorld = (boatShape * boatScale) * R' + [posX(i), posY(i)];
    set(hCraft, 'XData', boatWorld(:,1), 'YData', boatWorld(:,2));
    
    % Rudder position
    rudderBase = [posX(i) - cos(heading(i))*0.35, posY(i) - sin(heading(i))*0.35];
    rudderDir = [cos(heading(i) + rudderAngle(i)), sin(heading(i) + rudderAngle(i))];
    rudderTip = rudderBase + rudderDir * rudderLength;
    set(hRudder, 'XData', [rudderBase(1), rudderTip(1)], 'YData', [rudderBase(2), rudderTip(2)]);
    
    % Fin flapping
    finAngle = heading(i) + rudderAngle(i) + thetaP(i);
    finDir = [cos(finAngle), sin(finAngle)];
    finBase = rudderTip;
    finTip = finBase + finDir * finLength;
    set(hFin, 'XData', [finBase(1), finTip(1)], 'YData', [finBase(2), finTip(2)]);
    
    % Trail update
    addpoints(trail, posX(i), posY(i));
    
    % Vortex shedding
    current_sign = sign(omega(i));
    if current_sign ~= Fw.last_sign && current_sign ~= 0
        Fw.vortices(end+1,:) = finTip;
        sign_flip = (-1)^length(Fw.vortex_signs);
        Fw.vortex_signs(end+1) = sign_flip;
        if size(Fw.vortices,1) > Fw.max_vortices
            Fw.vortices(1,:) = [];
            Fw.vortex_signs(1) = [];
        end
        Fw.last_sign = current_sign;
    end

    % Vortex move and plot
    for j = 1:size(Fw.vortices,1)
        Fw.vortices(j,1) = Fw.vortices(j,1) - Fw.vortex_speed;
        angle = pi/2 * Fw.vortex_signs(j);
        dx = Fw.arrowLength * cos(angle);
        dy = Fw.arrowLength * sin(angle);
        
        if isgraphics(Fw.vortexHandles(j))
            delete(Fw.vortexHandles(j));
        end
        
        colorIndex = (Fw.vortex_signs(j)+1)/2 + 1;
        Fw.vortexHandles(j) = quiver(Fw.vortices(j,1), Fw.vortices(j,2), dx, dy, ...
            0, 'LineWidth', 1.5, 'Color', Fw.colors(colorIndex,:), 'MaxHeadSize', 2);
    end

    drawnow;
    pause(Vl.dt * 0.5);
end