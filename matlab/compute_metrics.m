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
    thr_R1 = 70;         % percentage
    thr_R2 = 0.08;       
    thr_R3 = 1;          % threshold less than 1 ensures tire contact is kept
    thr_R4 = 1.5;    
    thr_R5 = 1500;

    g = 9.81;
    par = suspension_params();
    m_s = par.ms;
    m_u = par.mu;
    k_t = par.kt;
    
    scen = ["ramp","bump","rough"];
    
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
            win_start = 0.5;            % Discard first 0.5 for ramp scenario
        end

        win_end = t(end);
        windowMask = t >= win_start & t <= win_end;
        rms_pass = rms(ddot_zs_pass(windowMask));
        rms_act = rms(ddot_zs_act(windowMask));
        c(i).R1 = rms_act/rms_pass * 100;
        c(i).passR1 = 100;

        % R2 
        susp_defl = act_y(:,2);
        susp_defl_pass = pass_y(:,2);
        c(i).R2 = max(abs(susp_defl));
        c(i).passR2 = max(abs(susp_defl_pass));

        % R3
        tire_defl = act_y(:,3);
        dyn_tire_load = k_t*tire_defl;
        stat_tire_load = (m_s+m_u)*g;
        max_tire_load_ratio = max(dyn_tire_load/stat_tire_load);
        c(i).R3 = max_tire_load_ratio;
        
        % Passive baseline
        pass_tire_defl = pass_y(:,3);
        pass_dyn_tire_load = k_t*pass_tire_defl;
        pass_stat_tire_load = (m_s+m_u)*g;
        pass_max_tire_load_ratio = max(pass_dyn_tire_load/pass_stat_tire_load);
        c(i).passR3 = pass_max_tire_load_ratio;


        % R4
        v = 5.0;
        x = v*t;
        if strcmp(scenarioName, 'ramp')
            defl_final = 0;               % step steady value (from road_inputs.m)
            tol = 0.008;                     % absolute tolerance
            ramp_start_dist = 1.0;        % from road_inputs.m
            rampLength = 0.3;

            % Ramp end
            ramp_end_pos = ramp_start_dist + rampLength;
            post_ramp_mask = (x > ramp_end_pos);   % Consideriamo il tratto successivo alla salita
            
            if any(post_ramp_mask)
                t_post = t(post_ramp_mask);
                defl_post = susp_defl(post_ramp_mask);
                defl_pass_post = susp_defl_pass(post_ramp_mask);
        
                inside = abs(defl_post - defl_final) <= tol;
                pass_inside = abs(defl_pass_post - defl_final) <= tol;
                
                % Active loop
                lastOutside = find(~inside, 1, 'last');
                if isempty(lastOutside)
                    ts = t_post(1);                   
                elseif lastOutside == numel(t_post)
                    ts = t(end);                    
                else
                    ts = t_post(lastOutside + 1);
                end
                
                % Passive baseline loop
                lastOutsidePass = find(~pass_inside, 1, 'last');
                if isempty(lastOutsidePass)
                    ts_pass = t_post(1);
                elseif lastOutsidePass == numel(t_post)
                    ts_pass = NaN;
                else
                    ts_pass = t_post(lastOutsidePass + 1);
                end
            end
                c(i).R4 = ts;
                c(i).passR4 = ts_pass;

        elseif strcmp(scenarioName, 'bump')
            defl_final = 0;                    % Valore di regime a valle del dosso
            tol = 0.008;                       % Absolute tolerance
            bump_start_dist = 1.0;
            bumpLength = 3.0;
            
            % Bump end
            bump_end_pos = bump_start_dist + bumpLength;
            post_bump_mask = (x > bump_end_pos);
            
            if any(post_bump_mask)
                % Extract post-bump vectors
                t_post = t(post_bump_mask);
                defl_post = susp_defl(post_bump_mask);
                defl_pass_post = susp_defl_pass(post_bump_mask);
        
                %  Deflection inside the band
                inside = abs(defl_post - defl_final) <= tol;
                pass_inside = abs(defl_pass_post - defl_final) <= tol;
        
                % --- Active loop (Post-bump) ---
                lastOutside = find(~inside, 1, 'last');
                if isempty(lastOutside)
                    ts = t_post(1);             % Già dentro fin dall'uscita del dosso
                elseif lastOutside == numel(t_post)
                    ts = NaN;                   % Non si assesta mai nel record
                else
                    ts = t_post(lastOutside + 1);
                end
                
                % --- Passive baseline loop (Post-bump) ---
                lastOutsidePass = find(~pass_inside, 1, 'last');
                if isempty(lastOutsidePass)
                    ts_pass = t_post(1);
                elseif lastOutsidePass == numel(t_post)
                    ts_pass = NaN;
                else
                    ts_pass = t_post(lastOutsidePass + 1);
                end
            end
            % Log settling time for bump
            c(i).R4 = ts;
            c(i).passR4 = ts_pass;

        else    %Rough scenario case
            c(i).R4 = NaN;
            c(i).passR4 = NaN;
        end
        
        % R5
        force_act = act_y(:,4);
        pass_force_act = pass_y(:,4);
        c(i).R5 = max(abs(force_act));
        c(i).passR5 = max(abs(pass_force_act));

        % Pass/fail checks against the specified thresholds
        c(i).pass = [c(i).R1 <= thr_R1, ...
                     c(i).R2 <= thr_R2, ...
                     c(i).R3 < thr_R3,  ...      % strict inequality
                     isnan(c(i).R4) || c(i).R4 <= thr_R4, ...
                     c(i).R5 <= thr_R5];
        c(i).allPass = all(c(i).pass);

        % Add a table printing metrics, value and threshold
        thr_names = ["R1";"R2";"R3";"R4";"R5"];
        safety_thr_names = thr_names([2,3,5]);
        performance_thr_names = thr_names([1,4]);

        vals = [c(i).R1;c(i).R2;c(i).R3;c(i).R4;c(i).R5];
        safety_vals = vals([2,3,5]);
        performance_vals = vals([1,4]);

        pass_vals = [c(i).passR1;c(i).passR2;c(i).passR3;c(i).passR4;c(i).passR5;];
        safety_pass_vals = pass_vals([2,3,5]);
        performance_pass_vals = pass_vals([1,4]);

        thr_vals = [thr_R1;thr_R2;thr_R3;thr_R4;thr_R5];
        safety_thr_vals = thr_vals([2,3,5]);
        performance_thr_vals = thr_vals([1,4]);

        c(i).safety_results = table(safety_thr_names,safety_vals,safety_pass_vals,safety_thr_vals, 'VariableNames', {'Requirement', 'Controlled System', 'Passive System','Threshold'});
        c(i).perf_results = table(performance_thr_names,performance_vals,performance_pass_vals,performance_thr_vals, 'VariableNames', {'Requirement', 'Controlled System', 'Passive System','Threshold'});
    end
end