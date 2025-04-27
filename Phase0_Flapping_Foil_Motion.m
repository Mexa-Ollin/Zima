%%%% Attempt 2, Control for a flapping foil

% Vessel parameters (Design)
% Lenght of craft = 70 cm
% Width of craft = 25 cm
% Mass (material dependant) = 5kg maybe

clc;clear;

% Main calculation variables (Vl)
Vl = struct('t_end', 0,'dt', 0, 't', 0, 'L',...
            0, 'K', 0, 'A', 0, 'fp', 0, ...
            'phi1', 0, 'thetaP', 0, 'omega', 0);

% Fin and wake animation values (Fw)
Fw = struct('arrowLength', 0, 'max_vortices', 0,...
            'vortex_speed', 0, 'vortices', 0, 'vortex_signs', 0,...
            'vortexHandles', 0, 'colors', 0, 'last_sign', 0);


% Pre op
Vl.t_end = 5;             % Length of test
Vl.dt = 0.01;             % Time step
Vl.t = 0:Vl.dt:Vl.t_end;  
Vl.L = 0.5;               % Length from hinge to tip
Vl.k = 1;


% Global variables
Vl.A = deg2rad(20); % Amplitude of fin yaw (From tip to tip, Figure 3.a)  
Vl.fp = 0.75;       % Pitching frequency (Hz), variates with speed in relation to wake


Vl.phi1 = 0;        % Phase offset for pitch

% Calculations
Vl.thetaP = Vl.A * sin(2*pi*Vl.fp* Vl.t + Vl.phi1); % Angle of pitch

Vl.omega = gradient(Vl.thetaP,Vl.dt);     % Angular velocity

%{
disp(rad2deg(thetaP(1:10)));  % Print first 10 angle values in degrees
% Tip position in 2D (for animation/plotting)
%}

% Plot results
figure;
x = Vl.L * cos(Vl.thetaP);
y = Vl.L * sin(Vl.thetaP);

subplot(3,1,1);
plot(Vl.t, rad2deg(Vl.thetaP),'b','LineWidth',1.5);
ylabel('Angle (deg)');
title('Fin Angular Position');
grid on;

subplot(3,1,2);
plot(Vl.t, rad2deg(Vl.omega), 'r', 'LineWidth', 1.5);
ylabel('Angular Velocity (deg/s)');
title('Fin Angular Velocity');
grid on;

subplot(3,1,3);
plot(x, y, 'k', 'LineWidth', 1.5);
axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Fin Tip Path');
grid on;

% Animation of flapping motion for craft
figure;
axis equal;
axis([-Vl.L Vl.L -Vl.L Vl.L] * 1.2);
grid on;
xlabel('x(m)');
ylabel('y(m)');
title('Animated Fin Flapping');

base = [0,0]; % Origin point for fin

finLine1 = line([0, x(1)], [0, y(1)], 'LineWidth', 3, 'Color', 'b');
trail = animatedline('Color','k','LineStyle', '--');

% Animation loop
for i = 1:length(Vl.t)
    set(finLine1, 'XData', [base(1),x(i)], 'YData', [base(2),y(i)]);

    addpoints(trail, x(i), y(i)); % Trail, useful for tracking
 
    drawnow;
    pause(Vl.dt * 0.5); % Modulate speed
end

% Vortex Shedding
% To do: Make the shed_interval proportional to the swaying of the fin

shed_interval = 10;         % Frame rate
Fw.vortex_speed = 0.05;        % Drift per frame
Fw.max_vortices = 50;         % Cap on vortex count
Fw.arrowLength= 0.2;

Fw.vortices = [];                  % Vector por point
Fw.vortex_signs = [];              % Matrix for orientation

Fw.colors = [1.0 0.3 0.3;          % red (CCW)
             0.1 0.6 1.0];         % blue (CW)
Fw.vortexHandles = gobjects(Fw.max_vortices, 1);


Fw.last_sign = -sign(Vl.omega(1));         % Track reversal

figure;
axis equal;
axis([-Vl.L Vl.L+1 -Vl.L Vl.L]*1.5);
grid on;
xlabel('x(m)');
ylabel('y(m)');
title('Vortex wake');

hold on;

finLine2 = line([0, x(1)], [0, y(1)], 'LineWidth', 3, 'Color', 'b');

% Plot for vortex visualization
for i = 1:length(Vl.t)
    
    if isgraphics(finLine2)
        set(finLine2, 'XData', [0, x(i)], 'YData', [0, y(i)]);
    end

    current_sign = sign(Vl.omega(i));
    if current_sign ~= Fw.last_sign && current_sign ~= 0
        Fw.vortices(end+1, :) = [x(i), y(i)];          % Position
        sign_flip = (-1) ^ length(Fw.vortex_signs);    % Alternate Right/Left
        Fw.vortex_signs(end+1) = sign_flip;
        


        if size(Fw.vortices,1) > Fw.max_vortices  % To avoid oversaturation, limit vorteces
            Fw.vortices(1,:) = [];
            Fw.vortex_signs(1) = [];
        end

        Fw.last_sign = current_sign;
    end
    

    % Move vortex
    for j = 1:size(Fw.vortices,1)
        Fw.vortices(j,1) = Fw.vortices(j,1) + Fw.vortex_speed;
        
        %{
        offset = 0.02 * vortex_signs(j)
        colorIndex = (vortex_signs(j) +1)/ 2 + 1;
        set(vortexHandles(j),...
            'xData', vortices(j,1),...
            'yData', vortices(j,2),...
            'Color', colors(colorIndex, :));
        %}
        

        % Calculate rotational direction
        angle = pi/2 * Fw.vortex_signs(j); % +- 90° rotation (change of direction)
        dx = Fw.arrowLength * cos(angle);
        dy = Fw.arrowLength * sin(angle);

        if isgraphics(Fw.vortexHandles(j))     % Remove previous angles
            delete(Fw.vortexHandles(j));
        end
        colorIndex = (Fw.vortex_signs(j) + 1)/2 + 1;  % map -1/+1 → 1/2 
        Fw.vortexHandles(j) = quiver(Fw.vortices(j,1), Fw.vortices(j,2), dx, dy,...
            0, 'LineWidth', 1.5, ...
            'Color', Fw.colors((Fw.vortex_signs(j)+1)/2 + 1, :), 'MaxHeadSize', 2);

    end
    drawnow;
    pause(Vl.dt * 0.1);
end

