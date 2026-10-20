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