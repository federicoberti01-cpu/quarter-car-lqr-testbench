function results = run_scenarios(passive_ss, active_ss, cfg)
% RUN_SCENARIOS  Simulates both systems on every scenario defined in cfg.
%   Systems come in as arguments; scenario names, durations and dt come from cfg.

    names = fieldnames(cfg.scenarios);              % order = order in scenario_config
    dt    = cfg.sim.dt;
    results = struct('scenario',{}, 't',{}, 'z_r',{}, 'z_r_dot',{}, ...
                     'x0',{}, 'passive',{}, 'active',{});

    for idx = 1:numel(names)
        name = names{idx};
        sc   = cfg.scenarios.(name);

        % TODO: adapt to your road_inputs signature (see the note below)
        road = road_inputs(name, sc.t_end, dt, false);

        t  = road.t;
        x0 = zeros(size(passive_ss.A,1), 1);        % all scenarios start at rest now

        results(idx).scenario = name;               % char, not string: it is used in file names
        results(idx).t        = t;
        results(idx).z_r      = road.z_r;           % TODO: your field name
        results(idx).z_r_dot  = road.z_r_dot;
        results(idx).x0       = x0;
        results(idx).passive  = lsim(passive_ss, road.z_r_dot, t, x0, 'foh');
        results(idx).active   = lsim(active_ss,  road.z_r_dot, t, x0, 'foh');
    end
end