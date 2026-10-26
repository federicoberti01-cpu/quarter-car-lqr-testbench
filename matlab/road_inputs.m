function r = road_inputs(scenario_type, t_end, dt, plot_profiles)

    % Designs signals for the road input velocity \dot{z_r} and the road
    % input height z_r.
    % Accepted scenario types are
    %   - Step: Simulates an increase in road height h with no variations in
    %           road speed. It is implemented by imposing 
    %           as i.c. z_u-z_r = -h
    %   - Bump: Simulates a bump of height h and length L at speed v, with
    %           consequent variation of road velocity.
    %   - Rough Road: Generates a rough road profile according to ISO 8608
    %                 with a class D profile.
    % User provides the simulation duration and the (constant) time step.
    % Returns a struct containing the time vector t, road height and
    % velocity and the initial state.
    if nargin < 4 || isempty(plot_profiles)
        plot_profiles = False;
    end

    t = (0:dt:t_end)';
    z_r = zeros(size(t));
    z_r_dot = zeros(size(t));
    x0 = zeros(4,1);


    switch lower(scenario_type)
        case 'step'
            stepHeight = 0.03;                 % Standard step of 3cm     
            x0 = [0; 0; -stepHeight; 0];

            % Populate z_r and z_r_dot for visualization/plotting
            z_r(:) = stepHeight;             % Road stays elevated after step_time
        
        case 'bump'
            bumpHeight = 0.05;                 % Standard bump of 5cm
            bumpLength = 3;                    % Standard length of 3m
            v = 5;                             % Car velocity
            bump_start_dist = 5.0;  
            
            x = v*t;                           % Distance vector

            % Check samples where the vehicle is above the bump
            bump_mask = (x >= bump_start_dist) & (x <= (bump_start_dist + bumpLength));

            if any(bump_mask)
                x_in_bump = x(bump_mask) - bump_start_dist;

                % Raised cosine profile
                z_r(bump_mask) = (bumpHeight/2) * (1 - cos(2*pi * x_in_bump/bumpLength));
                % Velocity profile is the derivative of the height profile
                z_r_dot(bump_mask) = (bumpHeight * pi * v/bumpLength) * sin(2*pi* x_in_bump/bumpLength);
            end

        case 'rough'
            Gd_n0 = 512e-6;                     % Roughness coefficient for D class road [m^3]
            n0 = 0.1;                          % Reference spacial frequency [cycles/m]
            w = 2.0;                            % Waviness exponent
            v = 10;                            % Car velocity 
            x = v*t;

            % Spatial frequency bounds
            n_min = 0.025; % [cycles/m]
            n_max = 1.0; % [cycles/m]
            N_freq = 500; % Number of harmonic components
            
            n_vec = linspace(n_min, n_max, N_freq)';
            dn = n_vec(2) - n_vec(1);
            
            % Random phase angles for each frequency component [0, 2*pi]
            s = RandStream("mt19937ar","Seed",42);       %fix the random seed for tests reproducibility
            phases = 2 * pi * rand(s,N_freq, 1);
        
            % Harmonic superposition for displacement (z_r) and velocity (z_dot_r)
            z_r = zeros(size(t));
            z_r_dot = zeros(size(t));
        
            for i = 1:N_freq
                n_i = n_vec(i);
                % PSD magnitude at spatial frequency n_i
                Gd_i = Gd_n0 * (n_i / n0)^(-w);
                amplitude = sqrt(2 * Gd_i * dn);
                
                % Accumulate elevation and time derivative (dx/dt = v)
                z_r = z_r + amplitude * cos(2 * pi * n_i * x + phases(i));
                z_r_dot = z_r_dot - amplitude * (2 * pi * n_i * v) * sin(2 * pi * n_i * x + phases(i));
            end

        otherwise
            error('Unknown scenario type: %s. Use ''step'', ''bump'', or ''rough''.');
    end

    r.t = t; 
    r.z_r = z_r;
    r.z_r_dot = z_r_dot;
    r.x0 = x0;

    if plot_profiles
        figure;
        subplot(2,1,1);
        plot(t, z_r, 'LineWidth', 1.2);
        xlabel('Time [s]'); ylabel('Road height [m]');
        title(['Scenario: ', upper(scenario_type)]);
        grid on;
        subplot(2,1,2);
        plot(t, z_r_dot, 'LineWidth', 1.2);
        xlabel('Time [s]'); ylabel('Road velocity [m/s]');
        grid on;
    end
end