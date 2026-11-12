function cfg = scenario_config()
% SCENARIO_CONFIG  Single source of truth for weights, scenarios,
% requirements and thresholds. Nothing below may be hardcoded elsewhere.

    %% Simulation
    cfg.sim.dt = 1e-3;                       % [s] fixed step for every scenario

    %% Controller weights (see docs/weights_fixing.md)
    cfg.weights.q1 = 0.1;                    
    cfg.weights.q2 = 100;
    cfg.weights.q3 = 3e3;
    cfg.weights.r  = 1e-8;

    %% Scenarios
    % Common fields: t_end [s], event_start [s], event_end [s], window_start [s]
    cfg.scenarios.ramp.type        = 'ramp';
    cfg.scenarios.ramp.height_m    = 0.03;   
    cfg.scenarios.ramp.speed_mps   = 5;      
    cfg.scenarios.ramp.length_m    = 0.3;    
    cfg.scenarios.ramp.shape       = 'linear';
    cfg.scenarios.ramp.lead_in_s   = 0.5;
    cfg.scenarios.ramp.t_end       = 10;
    cfg.scenarios.ramp.window_start = 0.5;   % metric window starts at the event

    cfg.scenarios.bump.type        = 'bump';
    cfg.scenarios.bump.height_m    = 0.05;   
    cfg.scenarios.bump.length_m    = 3.0;     
    cfg.scenarios.bump.speed_mps   = 5;
    cfg.scenarios.bump.lead_in_s   = 0.5;
    cfg.scenarios.bump.t_end       = 10;
    cfg.scenarios.bump.window_start = 0.5;

    cfg.scenarios.rough.type       = 'rough';
    cfg.scenarios.rough.iso_class  = 'D';
    cfg.scenarios.rough.Gd_n0      = 512e-6;    
    cfg.scenarios.rough.n0         = 0.1;    % [cycles/m]
    cfg.scenarios.rough.waviness   = 2;
    cfg.scenarios.rough.speed_mps  = 10;     
    cfg.scenarios.rough.n_min      = 0.025;  % [cycles/m]
    cfg.scenarios.rough.n_max      = 1;
    cfg.scenarios.rough.n_components = 500;
    cfg.scenarios.rough.seed       = 42;
    cfg.scenarios.rough.t_end      = 30;
    cfg.scenarios.rough.window_start = 2;    % discard the start-up transient

    %% Requirements
    % kind: 'performance' (relative to passive at the same mass) or 'safety' (absolute)
    cfg.requirements.R1 = struct('kind','performance','metric','rms_body_accel', ...
        'type','ratio_to_passive','threshold',0.70, ...
        'applies_to',{{'ramp','bump','rough'}});          
    cfg.requirements.R2 = struct('kind','safety','metric','max_abs_susp_defl', ...
        'type','absolute','threshold',0.08,'units','m', ...
        'applies_to',{{'ramp','bump','rough'}});         
    cfg.requirements.R3 = struct('kind','safety','metric','max_tire_load_ratio', ...
        'type','absolute','threshold',1, ...
        'applies_to',{{'ramp','bump','rough'}});
    cfg.requirements.R4 = struct('kind','performance','metric','settling_time', ...
        'type','absolute','threshold',1.5, 'units', 's', ...
        'band_m',0.008,'applies_to',{{'ramp','bump'}});          
    cfg.requirements.R5 = struct('kind','safety','metric','max_abs_force', ...
        'type','absolute','threshold',1500,'units','N', ...
        'applies_to',{{'ramp','bump','rough'}});
    
    names = fieldnames(cfg.scenarios);
    for i = 1:numel(names)
        s = cfg.scenarios.(names{i});
        if isfield(s, 'length_m')                      % ramp and bump
            s.event_start_s = s.lead_in_s;
            s.event_end_s   = s.lead_in_s + s.length_m / s.speed_mps;
        end
        cfg.scenarios.(names{i}) = s;
    end

    validate_config(cfg);
end

function validate_config(cfg)
    assert(cfg.sim.dt > 0, 'dt must be positive');
    names = fieldnames(cfg.scenarios);
    for i = 1:numel(names)
        s = cfg.scenarios.(names{i});
        assert(s.t_end > s.window_start, '%s: window outside record', names{i});
    end
    assert(~isnan(cfg.scenarios.rough.Gd_n0), 'Fill in the ISO 8608 roughness value');
end