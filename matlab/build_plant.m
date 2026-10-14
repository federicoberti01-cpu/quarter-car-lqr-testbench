function sys = build_plant(p)
    % Defines the state space model for the quarter-car active suspension
    % system. Returns matrices A (state), B (input), C(output),
    % D(feedthrough). 
    % States are x = [z_s-z_u, \dot{z_s}, z_u-z_r, \dot{z_u}]
    % Input are u = [F_a, \dot{z_r}]; Outputs are y = [\ddot{z_s}, z_s-z_u, z_u-z_r,Fa];
    % Disturbance is \dot{z_r}.
    
    ms = p.ms; mu = p.mu; ks = p.ks; bs = p.bs; kt = p.kt; bt = p.bt;
    A = [
            0,  1,  0,  -1;
            -ks/ms, -bs/ms, 0, bs/ms;
            0,  0,  0,  1;
            ks/mu, bs/mu, -kt/mu, -(bs + bt)/mu
        ];
    B = [0 ,   0;
        1/ms,  0;
        0,    -1;
        -1/mu, bt/mu];
    C = [
            -ks/ms, -bs/ms, 0, bs/ms;
            1, 0, 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 0
          ];
    D = [1/ms, 0;
         0,    0;
         0,    0;
         1,    0];

    sys = ss(A,B,C,D);
    sys.StateName = {'Suspension Deflection', 'Body Velocity', 'Tire Deflection', 'Wheel Velocity'};
    sys.InputName = {'Actuator Force (Fa)', 'Road Velocity (z_dot_r)'};
    sys.OutputName = {'Body Acceleration', 'Suspension Travel', 'Tire Deflection', 'Control Force'};
end
