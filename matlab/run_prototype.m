% This script takes up all the processes from the building of the plant to
% the evaluation of the requirements, through the other functions in the
% folder.

clear; clc; close all;
%% Parameter Loading and Plant 

params = suspension_params();
plant = build_plant(params);

% Obtain weights for cost matrix through Bryson Rule. See
% docs/bryson_weights_calculation for a detailed motivation.
q1 = 1/(0.08)^2;            % Suspension deflection cost
q2 = 1/(1.0)^2;             % Sprung velocity cost, estimate
q3 = 1/(0.0176)^2;          % Tire deflection cost 
r = 1/(1500)^2;             % Actuator cost

%% Open Loop Checks

% Evaluate open-loop stability and basic plant properties.
openLoopPoles = eig(plant.A);
isOpenLoopStable = all(real(openLoopPoles) < 0);
fprintf('Open-loop stable: %d\n', isOpenLoopStable);

% Study natural frequencies for body and wheel
[wn, zeta, p] = damp(plant);
fprintf('Natural frequencies (Hz):\n');
disp(unique(wn/(2*pi)));
fprintf('Damping ratios:\n');
disp(unique(zeta));

% Check Controllability 
isCtrb = (rank(ctrb(plant.A, plant.B(:,1))) == 4);      %Controllability is checked for the actuator input only
fprintf('Controllable: %d\n', isCtrb);

%% LQR Controller Design
[K, closedLoop_ss] = lqr_design(plant,q1,q2,q3,r);

closedLoopPoles = eig(closedLoop_ss.A);
isClosedLoopStable = all(real(closedLoopPoles) < 0);

fprintf('Closed-loop stable: %d\n', isClosedLoopStable);
fprintf('Obtained Feedback Matrix K: ');
disp(K)

% Plot poles shift due to control
figure; hold on; grid on; 
plot(real(openLoopPoles), imag(openLoopPoles), 'bo', 'MarkerSize',10, 'LineWidth',1.5) % open-loop
plot(real(closedLoopPoles), imag(closedLoopPoles), 'r+', 'MarkerSize',10, 'LineWidth',1.5) % closed-loop
xlabel('Real Axis'); ylabel('Imag Axis')
legend('Open-loop poles','Closed-loop poles','Location','best')

% Optionally connect closest pairs to show movement
for i = 1:min(numel(openLoopPoles),numel(closedLoopPoles))
    % find nearest closed-loop pole to each open-loop pole
    [~, idx] = min(abs(closedLoopPoles - openLoopPoles(i)));
    h = plot([real(openLoopPoles(i)), real(closedLoopPoles(idx))], [imag(openLoopPoles(i)), imag(closedLoopPoles(idx))], 'g--');
    h.HandleVisibility = 'off';
end

title('Pole Movement: Open-loop → Closed-loop')

%% Passive vs. active suspension systems

passive_ss = plant(:,2);
active_ss = closedLoop_ss;

%% Simulation Loop

scenarios = ["step","bump","rough"];
sim_duration = [5, 6, 30];                 % Simulation times differ for each scenario
dt = 1e-3;

results = struct();
for idx = 1:numel(scenarios)
    scenario = scenarios(idx);
    roadInput = road_inputs(scenario,sim_duration(idx),dt,false);
    t = roadInput.t;
    % Loop for bump and rough scenario where lsim is used
    if  ~strcmp(scenario,'step')
        roadVelocityProfile = roadInput.z_r_dot;
        passiveResponse = lsim(passive_ss, roadVelocityProfile, t,'foh');
        activeResponse = lsim(active_ss, roadVelocityProfile, t, 'foh');

    % Loop for step scenario where initial is used
    else
        initCond = roadInput.x0;
        passiveResponse = initial(passive_ss,initCond,t);
        activeResponse = initial(active_ss,initCond,t);
    end
    results(idx).scenario = scenario;
    results(idx).t = t;
    results(idx).passive = passiveResponse;
    results(idx).active = activeResponse;
end

%% Metrics retrieval and processing

metrics = compute_metrics(results);
