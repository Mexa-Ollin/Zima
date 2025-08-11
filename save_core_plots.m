function files = save_core_plots(sim, tag, outDir)
% SAVE_CORE_PLOTS  Save trajectory, heading and error/command plots to outDir.

if nargin < 3 || isempty(outDir), outDir = pwd; end
if ~exist(outDir,'dir'), mkdir(outDir); end

t   = sim.t;
eta = sim.eta;         % [x;y;psi]
u   = sim.u;           % guide command (rad)
des = sim.des_h;       % desired heading (rad)
err = sim.err_h;       % already wrapped in simulate_controller

% safe filename base
base = regexprep(tag,'\s+','_');
files = cell(1,3);

% 1) Trajectory
f1 = figure('Name',[tag ' Trajectory']);
hold on; axis equal; grid on;
plot(eta(1,:), eta(2,:), 'b-','LineWidth',1.2);
if isfield(sim,'wps') && ~isempty(sim.wps)
    scatter(sim.wps(:,1), sim.wps(:,2), 40, 'r', 'filled');
end
xlabel('x'); ylabel('y');
title(['Trajectory - ' strrep(tag,'_','\_')]);
files{1} = fullfile(outDir, sprintf('%s_traj.png', base));
try, saveas(f1, files{1}); catch, end

% 2) Heading vs desired
f2 = figure('Name',[tag ' Heading']);
plot(t, unwrap(eta(3,:)), 'k','LineWidth',1.2); hold on;
plot(t, unwrap(des), 'r--','LineWidth',1.2);
grid on; xlabel('t (s)'); ylabel('\psi (rad)');
legend('actual','desired','Location','best');
title(['Heading - ' strrep(tag,'_','\_')]);
files{2} = fullfile(outDir, sprintf('%s_heading.png', base));
try, saveas(f2, files{2}); catch, end

% 3) Error and command
f3 = figure('Name',[tag ' Error & Command']);
yyaxis left;  plot(t, err, 'LineWidth',1.2);      ylabel('e_\psi (rad)');
yyaxis right; plot(t, rad2deg(u), 'LineWidth',1); ylabel('u (deg)');
grid on; xlabel('t (s)');
title(['Error & Command - ' strrep(tag,'_','\_')]);
files{3} = fullfile(outDir, sprintf('%s_error_cmd.png', base));
try, saveas(f3, files{3}); catch, end
% after plot(...):
plot(eta(1,1), eta(2,1), 'g*', 'MarkerSize', 8);           % start
if ~isempty(sim.wps)
    scatter(sim.wps(end,1), sim.wps(end,2), 60, 'r', 'filled'); % final WP
   
end
end
