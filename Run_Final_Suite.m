function summary = Run_Final_Suite()

% output folder
rootOut = fullfile(pwd, 'output');
if ~exist(rootOut, 'dir'), mkdir(rootOut); end
stamp   = datestr(now,'yyyymmdd_HHMMSS');
outDir  = fullfile(rootOut, stamp);
plotsDir= fullfile(outDir, 'plots');
mkdir(outDir); mkdir(plotsDir);

% 1) Identification 
results = StepID_Program();   % runs multi-heading ID and draws plots

% Save ID summary as CSV
R = struct2table(struct( ...
    'wp_x', arrayfun(@(s)s.wp(1), results), ...
    'wp_y', arrayfun(@(s)s.wp(2), results), ...
    'heading_deg', rad2deg([results.heading]).', ...
    'K', [results.K].', ...
    'T', [results.T].' ));
writetable(R, fullfile(outDir,'results_id.csv'));

% Save ID plots that StepID_Program() created
saveFigByName({'K and T vs Heading','Heading Map of Experiments'}, plotsDir);

%% ---------- 2) PID design + outliers ----------
pid = PID_Design_From_Results(results);   % draws PID vs heading plot
pid = flag_outliers(pid);

% Save PID table and plot
P = struct2table(struct( ...
    'heading_deg', rad2deg([pid.heading]).', ...
    'K', [pid.K].', 'T', [pid.T].', ...
    'Kp', [pid.Kp].', 'Ki', [pid.Ki].', 'Kd', [pid.Kd].', ...
    'flags', string({pid.flag}).' ));
writetable(P, fullfile(outDir,'pid_gains.csv'));
saveFigByName({'PID gains vs Heading'}, plotsDir);

% Controllers
ctl_fixed = make_fixed(pid);
ctl_sched = make_scheduler(pid);
ctl_adapt = make_adaptive_defaults();

modes = { ...
    struct('name','A_fixed','ctl',ctl_fixed), ...
    struct('name','B_scheduled','ctl',ctl_sched), ...
    struct('name','C_adaptive','ctl',ctl_adapt) ...
};

%  Scenarios 
scenarios = { ...
   struct('name','S1_single','wps',[4,1],'dist',dNone()), ...
   struct('name','S2_sequence','wps',[3,0; 4,0; 4,1; 3,2; 2,2],'dist',dNone()), ...
   struct('name','S3_disturbed','wps',[4,1],'dist',dCurrentNoise(0.4,0.01)) ...
};

% Run sims, save plots, collect metrics 
allRows = [];
for ms = 1:numel(modes)
    for sc = 1:numel(scenarios)
        tag = [modes{ms}.name '_' scenarios{sc}.name];
        fprintf('\n=== Running %s ===\n', tag);

        sim = simulate_controller(scenarios{sc}, modes{ms}.ctl, results);

        % Save core plots under plotsDir
        files = save_core_plots(sim, tag, plotsDir)
        % Metrics and per-case CSV
        M = compute_metrics(sim);
        T = addvars(M, string(modes{ms}.name), string(scenarios{sc}.name), ...
            'Before',1,'NewVariableNames',{'mode','scenario'});
        writetable(T, fullfile(outDir, ['metrics_' tag '.csv']));

        allRows = [allRows; T]; %#ok<AGROW>
    end
end
writetable(allRows, fullfile(outDir,'metrics_all.csv'));

% Save MAT bundle 
save(fullfile(outDir,'suite_bundle.mat'), 'results','pid','modes','scenarios','allRows');

% Save any open figures just in case
saveAllOpenFigs(plotsDir);

% return summary 
summary = struct('outDir',outDir, 'plotsDir',plotsDir, ...
                 'results_table', fullfile(outDir,'results_id.csv'), ...
                 'pid_table',     fullfile(outDir,'pid_gains.csv'), ...
                 'metrics_all',   fullfile(outDir,'metrics_all.csv'));
fprintf('\nDone. Outputs in: %s\n', outDir);
end

% ===== helpers for saving figures =====
function saveFigByName(names, outDir)
for i=1:numel(names)
    hs = findall(0,'Type','figure','Name',names{i});
    if ~isempty(hs)
        fn = regexprep(names{i},'\s+','_');
        for hh = reshape(hs,1,[])
            saveas(hh, fullfile(outDir, sprintf('%s_%d.png', fn, hh.Number)));
        end
    end
end
end

function saveAllOpenFigs(outDir)
hs = findall(0,'Type','figure');
for k=1:numel(hs)
    nm = get(hs(k),'Name');
    if isempty(nm), nm = sprintf('Figure_%d', hs(k).Number); end
    fn = regexprep(nm,'\s+','_');
    try
        saveas(hs(k), fullfile(outDir, [fn '.png']));
    catch
    end
end
end

function D = dNone()
% No disturbance
D.tau = @(t,eta,nu) [0; 0; 0];
end

function D = dCurrentNoise(bias_u, sigma_r)
% Constant surge bias + white-noise yaw moment
if nargin < 1, bias_u = 0.4; end
if nargin < 2, sigma_r = 0.01; end
D.tau = @(t,eta,nu) [ -bias_u; 0; sigma_r*randn ];
end

