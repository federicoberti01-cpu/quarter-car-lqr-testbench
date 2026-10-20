% This file consists of sanity checks for the functions developed in
% matlab/
clc 
close all
clear all

addpath('C:/Users/feder/OneDrive/Desktop/Projects/quarter-car-lqr-testbench/matlab')

% Parameters loading check
p = suspension_params();
disp("Sprung mass: " + p.ms + " Kg")
disp("Tire stiffness: " + p.kt + " N/m")

% State-space model construction check
sys = build_plant(p);
disp("State matrix size: " + mat2str(size(sys.A)))
disp("Input matrix size: " + mat2str(size(sys.B)))
disp("State Names: " + sys.StateName)

t = 0:0.01:8;
u = ones(length(t),2);
[y, tOut, x] = lsim(sys, u, t);
disp("Open-loop simulation completed: " + numel(tOut) + " samples")
%plot(t,y)

disp("Eigenvalues of the open loop system: " + eig(sys.A))

% LQR controller design check
q1 = 1e2; q2 = 1e4; q3 = 1e3;
r = 1e4;
[K,cl_ss] = lqr_design(sys,q1,q2,q3,r);
disp("LQR gain size: " + mat2str(size(K)))
disp("Closed loop eigenvalues: " + eig(cl_ss.A))

% Road profiles check
r = road_inputs('rough',5.0,1e-3,false);

sim_time = r.t;
u = r.z_r_dot;
[yClosedLoop, tClosedLoop, ~] = lsim(cl_ss, u, sim_time);
disp("Closed-loop simulation completed: " + numel(tClosedLoop) + " samples")
plot(sim_time,yClosedLoop);

r = road_inputs('step',2.0,1e-3,false);
[yClosedLoop, tClosedLoop, ~] = initial(cl_ss, r.x0, r.t);
disp("Closed-loop simulation completed: " + numel(tClosedLoop) + " samples")
plot(r.t,yClosedLoop);