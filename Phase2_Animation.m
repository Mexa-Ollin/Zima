% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
% Phase 2
% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

% Compute Vehicle Displacement (Forward Motion)

% Parameters
m = 5;              % Mass of craft in kg
k_thrust = 0.2;     % Thrust gain factor (empirical)
rho = 1000;         % Water density (kg/m^3)
cd = 1.0;           % Drag coefficient (approx)
A_front = 0.025 * 0.07;  % Cross-sectional area (m²)

% Thrust from fin motion
F_thrust = k_thrust * omega.^2;

% Initialize motion arrays
acc = zeros(size(Vl.t));
vel = zeros(size(Vl.t));
dispX = zeros(size(Vl.t));

% Integrate motion
for i = 2:length(Vl.t)
    F_drag = 0.5 * rho * cd * A_front * vel(i-1)^2;
    F_net = F_thrust(i) - F_drag * sign(vel(i-1));
    acc(i) = F_net / m;
    vel(i) = vel(i-1) + acc(i) * Vl.dt;
    dispX(i) = dispX(i-1) + vel(i) * Vl.dt;
end

figure;
hold on;
axis equal;
xlim([-1, max(dispX)*3.2]);
ylim([-0.8 0.8]);
grid on;
xlabel('Displacement (m)');
ylabel('Y (m)');
title('Craft with Flapping Fin and Vortex Wake');


% Craft design
CL = 0.7;     % (m)
CW = 0.1;       % (m)

% Initial posiiton
CX = dispX(1);  
CY = 0;         


% Plot craft
hCraft = rectangle('Position', [CX - CL/2, ...
                                CY - CW/2, ...
                                CL, CW], ...
                   'FaceColor', [0.2 0.6 1.0], ...
                   'EdgeColor', 'k', ...
                   'LineWidth', 1.5);
% Fin parameters
finLength = 0.15;
rudderLength = .2;
hFin = line([0, 0], [0, 0], 'LineWidth', 2, 'Color', 'k');
hRudder = line([0, 0], [0,0], 'Linewidth', 2, 'Color', [0.5 0.5 0.5]);


% Trail
trail = animatedline('Color', 'b', 'LineWidth',...
                      1.2, 'LineStyle', '--');


% Generate rudder sway angle array
rudderAngle = A_rudder * sin(2 * pi * f_rudder * Vl.t);

% Animation loop
for i = 1:length(Vl.t)
    DX = dispX(i);

    % Move craft
    set (hCraft, 'Position', [DX - CL/2, ...
                             CY - CW/2, ...
                             CL, CW]);
    % Update trail
    addpoints(trail, DX, CY);

    %{
    angle = thetaP(i);  % angle of flapping fin
    finBase = [DispX - CL/2 - rudderlength, CY];  % rear center of craft
    finTip = finBase - [cos(angle), sin(angle)] * finLength; % Point backward
    %}
    
    rudderBase = [DX - CL/2, CY];
    rudderDir = [cos(rudderAngle(i)), sin(rudderAngle(i))];
    rudderTip = rudderBase - rudderDir * rudderLength;
    set(hRudder, 'XData', [rudderBase(1),rudderTip(1)], ...
                 'YData', [rudderBase(2),rudderTip(2)]);
    
    finAngleGlobal = -(rudderAngle(i) + thetaP(i));
    finDir = -[cos(finAngleGlobal), sin(finAngleGlobal)];
    finBase = rudderTip;
    finTip = finBase + finDir * finLength;

    set(hFin, 'XData', [finBase(1), finTip(1)], ...
              'YData', [finBase(2), finTip(2)]);

    % Vortex shedding
    current_sign = sign(omega(i));
    if current_sign ~= Fw.last_sign && current_sign ~= 0
        Fw.vortices(end+1,:) = finTip;
        sign_flip = (-1)^length(Fw.vortex_signs);
        Fw.vortex_signs(end+1) = sign_flip;

        if size(Fw.vortices,1) > Fw.max_vortices
            Fw.vortices(1,:) = []; Fw.vortex_signs(1) = [];
        end
        Fw.last_sign = current_sign;
    end

    % Update vortices
    for j = 1:size(Fw.vortices,1)
        Fw.vortices(j,1) = Fw.vortices(j,1) - Fw.vortex_speed;  % drift left with time

        angle = pi/2 * Fw.vortex_signs(j);  % vortex rotation
        dx = Fw.arrowLength * cos(angle);
        dy = Fw.arrowLength * sin(angle);

        if isgraphics(Fw.vortexHandles(j))
            delete(Fw.vortexHandles(j));
        end

        colorIndex = (Fw.vortex_signs(j)+1)/2 + 1;
        Fw.vortexHandles(j) = quiver(Fw.vortices(j,1), Fw.vortices(j,2), dx, dy, ...
            0, 'LineWidth', 1.5, ...
            'Color', Fw.colors(colorIndex, :), ...
            'MaxHeadSize', 2);
    end


    drawnow;
    pause(Vl.dt * 0.25);
end

figure;
sgtitle('Motion Analysis');

subplot(3,1,1);
plot (Vl.t, F_thrust, 'LineWidth',1.5, 'Color','g');
ylabel('Thrust(N)');
title('Generated Thrust');
grid on;

subplot(3,1,2);
plot (Vl.t, vel, 'LineWidth',1.5, 'Color','b');
ylabel('Velocity (m/s)');
title('Surge speed');
grid on;

subplot(3,1,3);
plot (Vl.t, dispX, 'LineWidth',1.5, 'Color','r');
ylabel('Displacement (m)');
xlabel('Time (s)');
title('Craft Displacement');
grid on;
