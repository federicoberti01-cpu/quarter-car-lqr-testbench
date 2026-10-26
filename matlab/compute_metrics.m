function c = compute_metrics(results)
    % Computes the metrics specified in requirements.md for further
    % processing and compares them to a set of pre-defined threshold.
    % Accepts a result struct obtained after simulation in run_prototype.m.
    % Returns a struct containing:
    %          - R1: RMS_pass/RMS_act in % of \ddot{z_s} over a time window specific for
    %            each scenario, for active and passive system(see requirements.md)
    %          - R2: max |z_s - z_u|
    %          - R3: max k_t*|z_u-z_r| / (m_s+m_u)*g
    %          - R4: t_s of |z_s-z_u| (only for step scenario)
    %          - R5: max |F_a|
    % and a table object reporting the pass/fail results.
    
    % Thresholds from requirements.md
    thr_R1 = 80;         % percentage
    thr_R2 = 0.10;       
    thr_R3 = 1;          % threshold less than 1 ensures tire contact is kept
    thr_R4 = 2.5;    
    thr_R5 = 1500;

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
        if strcmp(scenarioName, "bump")
            win_start = 5.0/5.0;       % Window starts at event start (v/s)
        elseif strcmp(scenarioName, "rough")
            win_start = 2;              % Discard first 2 seconds for rough road
        else
            win_start = 0;
        end

        win_end = t(end);
        windowMask = t >= win_start & t <= win_end;
        rms_pass = rms(ddot_zs_pass(windowMask));
        rms_act = rms(ddot_zs_act(windowMask));
        c(i).R1 = rms_act/rms_pass * 100;

        % R2 
        susp_defl = act_y(:,2);
        c(i).R2 = max(abs(susp_defl));

        % R3
        tire_defl = act_y(:,3);
        dyn_tire_load = k_t*tire_defl;
        stat_tire_load = (m_s+m_u)*g;
        max_tire_load_ratio = max(dyn_tire_load/stat_tire_load);
        c(i).R3 = max_tire_load_ratio;

        % R4
        if strcmp(scenarioName, 'step')
            defl_final = 0;               % step steady value (from road_inputs.m)
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

        % Pass/fail checks against the specified thresholds
        c(i).pass = [c(i).R1 <= thr_R1, ...
                     c(i).R2 <= thr_R2, ...
                     c(i).R3 < thr_R3,  ...      % strict inequality
                     isnan(c(i).R4) || c(i).R4 <= thr_R4, ...
                     c(i).R5 <= thr_R5];
        c(i).allPass = all(c(i).pass);

        % Add a table printing metrics, value and threshold
        thr_names = ["R1";"R2";"R3";"R4";"R5"];
        vals = [c(i).R1;c(i).R2;c(i).R3;c(i).R4;c(i).R5];
        thr_vals = [thr_R1;thr_R2;thr_R3;thr_R4;thr_R5];
        c(i).table_results = table(thr_names,vals,thr_vals, 'VariableNames', {'Requirement', 'Value', 'Threshold'});
    end
end