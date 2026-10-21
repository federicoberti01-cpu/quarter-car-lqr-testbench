function c = compute_metrics(results)
    % Computes the metrics specified in requirements.md for further
    % processing. Accepts a result struct obtained after
    % simulation in run_prototype.m.
    % Returns: - R1: RMS_pass/RMS_act in % of \ddot{z_s} over a time window specific for
    %            each scenario, for active and passive system(see requirements.md)
    %          - R2: max |z_s - z_u|
    %          - R3: max k_t*|z_u-z_r| / (m_s+m_u)*g
    %          - R4: t_s of |z_s-z_u| (only for step scenario)
    %          - R5: max |F_a|
    % For each scenario in the struct c

    g = 9.81;
    par = suspension_params();
    m_s = par.ms;
    m_u = par.mu;
    k_t = par.kt;
    
    scen = ["step","bump","rough"];
    
    for i = 1:numel(scen)
        scenarioName = scen(i);
        scenarioResults = results(i);
        t = scenarioResults.t;
        pass_y = scenarioResults.passive;
        act_y = scenarioResults.active;
        
        % R1 
        ddot_zs_pass = pass_y(:,1);
        ddot_zs_act = act_y(:,1);
        if strcmp(scenarioName, 'bump')
            win_start = 5.0/10.0;       % Window starts at event start (v/s)
        else 
            win_start = 1;
        end

        win_end = t(end);
        windowMask = t >= win_start & t <= win_end;
        rms_pass = rms(ddot_zs_pass(windowMask));
        rms_act = rms(ddot_zs_act(windowMask));
        c(i).R1 = rms_pass/rms_act * 100;

        % R2 
        susp_defl = act_y(:,2);
        c(i).R2 = max(susp_defl);

        % R3
        tire_defl = act_y(:,3);
        dyn_tire_load = k_t*tire_defl;
        stat_tire_load = (m_s+m_u)*g;
        max_tire_load_ratio = max(dyn_tire_load/stat_tire_load);
        c(i).R3 = max_tire_load_ratio;

        % R4
        if strcmp(scenarioName, 'step')
            defl_final = 0.06;               % step steady value (from road_inputs.m)
            tol = 0.008;                     % absolute tolerance
            inside = abs(susp_defl - defl_final) <= tol;
            
            % To estimate settling time, we require that all remaining samples stay inside
            lastOutside = find(~inside, 1, 'last');
            if isempty(lastOutside)
                ts = t(1);                   % always inside
            elseif lastOutside == numel(t)
                ts = NaN;                    % never settles within record
            else
                ts = t(lastOutside + 1);
            end
            c(i).R4 = ts;
        else
            c(i).R4 = NaN;
        end
        
        % R5
        force_act = act_y(:,4);
        c(i).R5 = max(abs(force_act));
    end
    
end