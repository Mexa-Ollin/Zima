% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o
% Phase 4b - Waypoint tracking
% -o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o-o

waypoints = [30, 60 ;-20, 0];
waypoints_rad = deg2rad(waypoints);
indx_Wp = 1;

tolerance = deg2rad(2)

% Initialize matrices
heading = zeros(1,N);
angVel = zeros(1,N);
posX = zeros(1,N);
posY = zeros(1,N);
rudder_cmd = zeros(1,N);

% Setup graphics
figure;
hold on;
axis equal;
xlim([-max(dispX) *2, max(dispX) * 2]);
ylim([-max(dispX) *1.5, max(dispX) * 1.5]);
grid on;
xlabel ('Displacement (m)');
ylabel ('Y(m)');
title('Fin motion');

% Craft parameters
boatShape = [-1 0; -1 0.5; -1 -0.5];
boatScale = 0.4;
R0 = eye(2);
boatWorld = (boatShape * boatScale) * R0' + [posX(1), posY(1)];
hCraft = fill(boatWorld(:,1), boatWorld(:,2), [0.2 0.6 1.0],...
            'EdgeColor','k','LineWidth', 1.5);

hRudder = line([0, 0], [0, 0], 'LineWidth', 2, 'Color', [0.5 0.5 0.5]);
hFin = line([0, 0], [0,0], 'LineWidth', 2, 'Color', 'K');
trail = animatedline('Color', 'b', 'LineWidth', 1.2, 'LineStyle','--');

for i = 2:N
    if indx_Wp <= length(waypoints_rad)
        des_heading = waypoints_rad(indx_Wp);                   %Planned heading
        heading_error = wrapToPi(des_heading - heading(i-1));   %Error validation

        % Ignore if heading error is small enough
        if abs(heading_error) <= tolerance
           indx_Wp = indx_Wp +1;
           if indx_Wp >= length(waypoints_rad)
                des_heading = waypoints_rad(indx_Wp);
           end
        end
    else
        des_heading = waypoints_rad(end);
    end
    
    % Rudder control law
    rudder_cmd(i) = Kp_R * heading_error + Kd_R * (-angVel(i-1));
    rudder_cmd(i) = max(min(rudder_cmd(i), deg2rad(15)), deg2rad(-15));  % saturate
    
    % Dynamics update
    torque = rudderTorqueGain * sin(rudder_cmd(i));
    angAcc = torque / I;
    angVel(i) = angVel(i-1) + angAcc * Vl.dt;
    heading(i) = heading(i-1) + angVel(i) * Vl.dt;

    posX(i) = posX(i-1) + vel(i) * cos(heading(i)) * Vl.dt;
    posY(i) = posY(i-1) + vel(i) * sin(heading(i)) * Vl.dt;

    % Rotations for boat/rudder/fin
    R_heading = [cos(heading(i)), -sin(heading(i)); ...
                 sin(heading(i)),  cos(heading(i))];

    R_rudder = [cos(rudder_cmd(i)), -sin(rudder_cmd(i)); ...
                sin(rudder_cmd(i)),  cos(rudder_cmd(i))];

    R_fin = [cos(thetaP(i)), -sin(thetaP(i)); ...
             sin(thetaP(i)),  cos(thetaP(i))];

    % Boat body update
    boatWorld = (boatShape * boatScale) * R_heading' + [posX(i), posY(i)];
    set(hCraft, 'XData', boatWorld(:,1), 'YData', boatWorld(:,2));

    % Rudder position
    rudderBase = [posX(i) - cos(heading(i)) * 0.35, posY(i) - sin(heading(i)) * 0.35];
    rudderDir_local = [-1; 0];  % local rudder points backwards
    rudderDir_world = R_heading * (R_rudder * rudderDir_local);
    rudderTip = rudderBase + rudderLength * rudderDir_world';

    set(hRudder, 'XData', [rudderBase(1), rudderTip(1)], ...
                 'YData', [rudderBase(2), rudderTip(2)]);

    % Fin position
    finDir_local = [-1; 0];
    finDir_world = R_heading * (R_rudder * (R_fin * finDir_local));
    finBase = rudderTip;
    finTip = finBase + finLength * finDir_world';

    set(hFin, 'XData', [finBase(1), finTip(1)], ...
              'YData', [finBase(2), finTip(2)]);

    % Trail update
    addpoints(trail, posX(i), posY(i));
 
    drawnow;
    pause(Vl.dt * 0.5);
end