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

disp(eig(sys.A))
