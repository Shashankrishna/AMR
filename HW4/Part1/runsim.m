close all;
clear;

% Setting the reference height
% The reference height can be controlled in runsim.m by setting the z_des variable:

% Hover
% z_des = 0;           % sets the reference height to zero

% Step
% z_des = 1;         % sets the reference height to one

% Given trajectory generator
trajhandle = @(t) fixed_set_point(t, z_des);

% This is your controller
controlhandle = @controller;

% Run simulation with given trajectory generator and controller
[t, z] = height_control(trajhandle, controlhandle);
