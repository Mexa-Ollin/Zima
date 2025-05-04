% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-
% Phase 4a  -Better Control!
% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-

% Control Parameters
planHeading = deg2rad(30);
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

% Animation Loop
for i = 2:N
    
    
    heading_error = wrapToPi(planHeading - heading(i - 1));
    rudder_cmd(i) = Kp_R * heading_error + (Kd_R * (-angVel(i)));
    rudder_cmd(i) = max(min(rudder_cmd(i), deg2rad(15)), deg2rad(-15));

    % Update turning dynamics
    torque = rudderTorqueGain * sin(rudder_cmd(i));
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
    rudderDir = -[cos(heading(i) + rudder_cmd(i)), sin(heading(i) + rudder_cmd(i))];
    rudderTip = rudderBase + rudderDir * rudderLength;
    set(hRudder, 'XData', [rudderBase(1), rudderTip(1)], 'YData', [rudderBase(2), rudderTip(2)]);
    
    % Fin flapping
    finAngle = -(heading(i) + rudder_cmd(i) + thetaP(i));
    finDir = -[cos(finAngle), sin(finAngle)];
    finBase = rudderTip;
    finTip = finBase + finDir * finLength;
    set(hFin, 'XData', [finBase(1), finTip(1)], 'YData', [finBase(2), finTip(2)]);
    
    % Trail update
    addpoints(trail, posX(i), posY(i));
 
    drawnow;
    pause(Vl.dt * 0.5);
end