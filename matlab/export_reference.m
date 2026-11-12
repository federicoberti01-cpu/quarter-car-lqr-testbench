function export_reference()
% EXPORT_REFERENCE  Writes the MATLAB reference data used by the Python tests.
% Only this function writes into reference/. Never edit those files by hand.

    %% 0. Paths (independent of the current working folder)
    here   = fileparts(mfilename('fullpath'));
    outDir = fullfile(here, '..', 'reference');
    if ~exist(outDir, 'dir'), mkdir(outDir); end

    %% 1. Load from the single source of truth
    p     = suspension_params();
    plant = build_plant(p);                         
    cfg   = scenario_config();                      
    [K, cl] = lqr_design(plant, cfg.weights.q1, cfg.weights.q2, cfg.weights.q3, cfg.weights.r);   

    scen = ["ramp","bump","rough"];                
    downsample = struct('ramp',1, 'bump',1, 'rough',10);

    %% 2. plant_nominal.json
    s = struct();
    s.parameters   = p;
    s.A = plant.A;
    s.B_u = plant.B(:,1);                           % actuator column
    s.B_w = plant.B(:,2);                           % road-velocity column
    s.C = plant.C;
    s.D_u = plant.D(:,1);
    s.D_w = plant.D(:,2);
    s.state_names  = {'z_s - z_u','dz_s','z_u - z_r','dz_u'};   
    s.output_names = {'ddz_s','z_s - z_u','z_u - z_r','u'};     
    s.input_names  = {'u','dz_r'};
    write_json(fullfile(outDir,'plant_nominal.json'), s);

    %% 3. controller.json
    ctrl = struct();
    ctrl.K = K;
    ctrl.sign_convention = 'u = -K*x';
    ctrl.weights = struct('q1',cfg.weights.q1,'q2',cfg.weights.q2,'q3',cfg.weights.q3,'r',cfg.weights.r);

    W = diag([cfg.weights.q1,cfg.weights.q2,cfg.weights.q3,0]);         % Cost diagonal matrix
    ctrl.Q = s.C'*W*s.C;
    ctrl.R = s.D_u'*W*s.D_u + cfg.weights.r;
    ctrl.N = s.C'*W*s.D_u;
    % TODO: Q, R, N (return them from design_lqr, or rebuild them here)
    ctrl.closed_loop_poles_real = real(eig(cl.A));
    ctrl.closed_loop_poles_imag = imag(eig(cl.A));
    write_json(fullfile(outDir,'controller.json'), ctrl);

    %% 4. scenarios.json
    sc = struct('schema_version', 1, ...
            'sim', cfg.sim, ...
            'scenarios', cfg.scenarios);
    write_json(fullfile(outDir, 'scenarios.json'), sc);
    write_json(fullfile(outDir, 'requirements.json'), cfg.requirements);

    %% 5. rough_road_components.json
    res = road_inputs(cfg.scenarios.rough.type,cfg.scenarios.rough.t_end, cfg.sim.dt, false);
    comps = res.comps;
    write_json(fullfile(outDir,'rough_road_components.json'), comps);
    %% 6. metrics_nominal.jsom

    passive_ss = plant(:,2);                           % TODO: confirm input order [u, road]
    active_ss  = cl;                                   % closed loop from design_lqr / apply_gain
    results = run_scenarios(passive_ss, active_ss, cfg);
    
    for i = 1:numel(results)
        r = results(i);
        k = downsample.(r.scenario);                   % field names must equal scenario names
        for sysName = ["passive","active"]
            y = r.(sysName);
            T = table(r.t, r.z_r_dot, y(:,1), y(:,2), y(:,3), y(:,4), ...
                'VariableNames', {'t','zr_dot','ddz_s','defl_susp','defl_tire','u'});
            writetable(T(1:k:end,:), fullfile(outDir, ...
                sprintf('sim_%s_%s.csv', r.scenario, sysName)));
        end
    end
    
    m = compute_metrics(results);              
    write_json(fullfile(outDir,'metrics_nominal.json'), m);

    %% 8. manifest.json (written last)
    mf = struct();
    mf.generated = char(datetime('now','Format','yyyy-MM-dd''T''HH:mm:ss'));
    mf.matlab_release = version;
    mf.files = {dir(fullfile(outDir,'*')).name};    % TODO: filter out '.' and '..'
    write_json(fullfile(outDir,'manifest.json'), mf);

    %% 9. Verification: read back and compare
    chk = jsondecode(fileread(fullfile(outDir,'plant_nominal.json')));
    assert(norm(chk.A - plant.A, 'fro') < 1e-9, 'A mismatch after round trip');

    fprintf('Reference exported to %s\n', outDir);
end

function write_json(path, s)
    try
        txt = jsonencode(s, 'PrettyPrint', true);   % newer releases
    catch
        txt = jsonencode(s);                        % fallback
    end
    fid = fopen(path, 'w');
    assert(fid > 0, 'Cannot open %s', path);
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, txt, 'char');
end