clc;
clear all;
close all;
%% Parameters

% Workspace Dimension
xlim([0 100])           % X Limit
ylim([0 100])           % Y Limit

% Landmarks at Locations
LX = [25 25 70 70 10 80];   % Landmark X coordinates
LY = [25 70 25 70 40 60];   % Landmark Y coordinates

% Velocity 
vel = 5;             % Maximum Speed m/s
vin = 0;             % Initial Speed m/s

% Steering angle
steering = 0.2;     % Maximum Steering Angle

% Robot in workspace, in x and y range
x = [];
y = [];
theta = [];
v = []; 
%% Robot Initial Pose
% Place your robot in a random position and orientation in middle of
% the environment.

% Inital Position 
x(1) = 10;             % Fixed x and y Position
y(1) = 10; 
% x(1) = randi([0 100]);   % Random position in environment.
% y(1) = randi([0 100]);
            
% Initial Orientation 
theta(1) = 0; 
% theta(1) = (2*pi).*rand();
%% Particle Section

% Create 1000 particles scattered uniformly through the environment.
% Each particle should contain (random) x,y position and orientation.

Particle_size = 1;
Particle_x = 100*rand([1 1000]); 
Particle_y = 100*rand([1 1000]);
Particle = (2*pi)*rand([1 1000]);

% Guess an initial pose of your robot in the environment
R_X = [];
R_Y = [];
R = [];

R_X(1) = mean(Particle_x);
R_Y(1) = mean(Particle_y);
R(1) = mean(Particle);
%% Initial Plot
hold on;
plot(LX, LY, 'rO') 
% Landmark Points: Red circle

quiver(Particle_x, Particle_y, Particle_size*cos(Particle), Particle_size*sin(Particle), 0, 'k'); 
% Particles: Black arrow

quiver(x(1), y(1), 5*cos(theta(1)), 5*sin(theta(1)), 0, 'b', 'Marker', 'p', 'LineWidth', 2); 
% Robot Initial position [10,10] or Random Position

quiver(R_X(1), R_Y(1), 5*cos(R(1)), 5*sin(R(1)), 0, 'g', 'Marker', 's', 'LineWidth', 2); 
% Robot Guess position
hold off;
xlim([0 100]); ylim([0 100]); 
legend('Landmarks', 'Particles', 'Robot Initial', 'Robot Guess');
title('Particle Filter');