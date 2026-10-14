function p = suspension_params()
    % Returns the physical parameters of the quarter-car model.
    % References: Rajamani Vehicle Dynamics (Chapter 10)
    
    p.ms = 320;    % Sprung mass (body quarter) [kg]
    p.mu = 40;     % Unsprung mass (wheel/axle) [kg]
    p.ks = 20000;  % Suspension stiffness (spring) [N/m]
    p.bs = 1000;   % Suspension damping coefficient [Ns/m]
    p.kt = 200000; % Tire stiffness [N/m]
    p.bt = 0;      % Tire damping coefficient (negligible/optional) [Ns/m]
end