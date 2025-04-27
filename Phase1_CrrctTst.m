%%%%
% Phase 1_ Fin Motion
%%%%

%  Time Setup  
%{
Vl.dt = 0.01;
Vl.t_end = 5;
Vl.t = 0:Vl.dt:Vl.t_end;
%}

%  Fin & Rudder Parameters  
%{
Vl.L = 0.15;        % Fin length
%}

%Setup
Mt = struct('N', 0,'RL', 0, 'Ar', 0, 'Fr', 0, 'phi_fin',...
             0, 'Ra', 0, 'Fa', 0);


Mt.N = length(Vl.t);
Mt.RL = 0.2; % Rudder length (m)

Mt.Ar = deg2rad(15);  % Rudder sway amplitude
Mt.Fr = 1.0;          % Sway frequency (Hz)


Mt.phi_fin = deg2rad(90);   % Phase offset — pitches into stroke

%  Fin Kinematics  
Mt.Ra = Mt.Ar * sin(2*pi* Mt.Fr * Vl.t); % RudderAngle (rad)
% thetaP = Vl.A * sin(2*pi* Vl.fp * Vl.t + phi_fin);  % Flapping angle
Mt.Fa = -(Mt.Ra + Vl.thetaP);

%  Visualization Setup  
figure;
axis equal;
axis([-0.5 0.5 -0.4 0.4]);
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Animated Flapping Foil (Rudder + Fin)');

% Graphics handles
rudderLine = line([0, 0], [0, 0], 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);
finLine = line([0, 0], [0, 0], 'LineWidth', 3, 'Color', 'b');
trail = animatedline('Color','k','LineStyle', '--');

% Animation Loop 
for i = 1:N
    base = [0, 0];  % Fixed rudder hinge
    rudderDir = [cos(rudderAngle(i)), sin(rudderAngle(i))];
    rudderTip = base + rudderDir * rudderLength;

    finDir = [cos(finAngle(i)), sin(finAngle(i))];
    finTip = rudderTip + finDir * Vl.L;

    set(rudderLine, 'XData', [base(1), rudderTip(1)], ...
                    'YData', [base(2), rudderTip(2)]);
    set(finLine, 'XData', [rudderTip(1), finTip(1)], ...
                 'YData', [rudderTip(2), finTip(2)]);

    addpoints(trail, finTip(1), finTip(2));

    drawnow;
    pause(Vl.dt * 0.5);
end

%% === Vortex Wake Visualization ===
%{
Fw.vortex_speed = 0.05;
Fw.max_vortices = 100;
Fw.arrowLength = 0.1;
Fw.vortices = [];
Fw.vortex_signs = [];
Fw.colors = [1.0 0.3 0.3; 0.1 0.6 1.0]; % Red = CCW, Blue = CW
Fw.vortexHandles = gobjects(Fw.max_vortices, 1);
Fw.last_sign = sign(omega(1));
%}

figure;
axis equal;
axis([-Vl.L Vl.L+1 -Vl.L Vl.L]*1.5);
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Vortex Wake');

hold on;
finLine2 = line([0, 0], [0, 0], 'LineWidth', 3, 'Color', 'b');
rudderLine2 = line([0, 0], [0, 0], 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);

for i = 1:N
    rudderDir = [cos(rudderAngle(i)), sin(rudderAngle(i))];
    rudderTip = rudderDir * rudderLength;

    angle = finAngle(i);
    finDir = [cos(angle), sin(angle)];
    finTip = rudderTip + finDir * Vl.L;

    set(rudderLine2, 'XData', [0, rudderTip(1)], 'YData', [0, rudderTip(2)]);
    set(finLine2, 'XData', [rudderTip(1), finTip(1)], 'YData', [rudderTip(2), finTip(2)]);

    % Vortex shedding on sign flip
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

    % Update vortex arrows
    for j = 1:size(Fw.vortices,1)
        Fw.vortices(j,1) = Fw.vortices(j,1) + Fw.vortex_speed;

        vortex_angle = pi/2 * Fw.vortex_signs(j);
        dx = Fw.arrowLength * cos(vortex_angle);
        dy = Fw.arrowLength * sin(vortex_angle);

        if isgraphics(Fw.vortexHandles(j))
            delete(Fw.vortexHandles(j));
        end

        colorIdx = (Fw.vortex_signs(j)+1)/2 + 1;
        Fw.vortexHandles(j) = quiver(Fw.vortices(j,1), Fw.vortices(j,2), dx, dy, ...
            0, 'LineWidth', 1.5, ...
            'Color', Fw.colors(colorIdx, :), ...
            'MaxHeadSize', 2);
    end

    drawnow;
    pause(Vl.dt * 0.1);
end


% Parameters
m = 5;              % Mass of craft in kg
k_thrust = 0.2;     % Thrust gain factor
rho = 1000;         % Water density (kg/m^3)
cd = 1.0;           % Drag coefficient (estimated, blunt body)
A = 0.025 * 0.07    % Frontal area in m^2 (Approx 2.5cm * 7cm foil)

% Compute thrust from angular velocity
F_thrust = k_thrust * omega.^2;

% Motion Array
acc = zeros(size(Vl.t));
vel = zeros(size(Vl.t));
dispX = zeros(size(Vl.t));

% Integration loop with drag

for i = 2:length(Vl.t)

    % Drag force
    F_drag = 0.5 * rho * cd * A * vel(i-1)^2;

    % Net force
    F_net = F_thrust(i) - F_drag * sign(vel(i-1));

    % Acceleration
    acc(i) = F_net / m;

    %Integrate to get velocity and position
    vel(i) = vel(i-1) + acc(i) * Vl.dt;
    dispX(i) = dispX(i-1) + vel(i) * Vl.dt;
end


% Plot results
figure;
subplot(3,1,1);
plot(Vl.t, F_thrust, 'g', 'LineWidth', 1.5);
ylabel('Thrust (N)');
title('Generated Thrust from Flapping Fin');
grid on;

subplot(3,1,2);
plot(Vl.t, vel, 'b', 'LineWidth', 1.5);
ylabel('Velocity (m/s)');
title('Craft Forward Velocity');
grid on;

subplot(3,1,3);
plot(Vl.t, dispX, 'k', 'LineWidth', 1.5);
ylabel('Displacement (m)');
xlabel('Time (s)');
title('Craft Position Over Time');
grid on;
