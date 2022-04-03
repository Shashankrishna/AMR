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
% x(1) = 10;             % Fixed x and y Position
% y(1) = 10; 
x(1) = randi([0 100]);   % Random location in environment.
y(1) = randi([0 100]);
            
% Initial Orientation 
% theta(1) = 0; 
theta(1) = (2*pi).*rand();
%% Particle Section

% Create 1000 particles scattered uniformly through the environment.
% Each particle should contain (random) x,y position and orientation.

Particle_size = 1;
Particle_x = 100*rand([1 1000]); 
Particle_y = 100*rand([1 1000]);
Particle = (2*pi)*rand([1 1000]);

% Guess an initial pose of your robot in the environment.
R_X = [];
R_Y = [];
R = [];

R_X(1) = mean(Particle_x);
R_Y(1) = mean(Particle_y);
R(1) = mean(Particle);

i = 1;
dt = 0.1;

while 1

% Compute Weights: From Lecture Notes.
% Reference: Pseudo code to calculate weights from lecture notes.

% Measure distance between initial position and landmark points.
LM1 = Distance(x(i), y(i), LX(1), LY(1));
LM2 = Distance(x(i), y(i), LX(2), LY(2));
LM3 = Distance(x(i), y(i), LX(3), LY(3));
LM4 = Distance(x(i), y(i), LX(4), LY(4));
LM5 = Distance(x(i), y(i), LX(5), LY(5));
LM6 = Distance(x(i), y(i), LX(6), LY(6));

% Measure distance between particle and landmark points.
P1 = Distance(Particle_x, Particle_y, LX(1), LY(1));
P2 = Distance(Particle_x, Particle_y, LX(2), LY(2));
P3 = Distance(Particle_x, Particle_y, LX(3), LY(3));
P4 = Distance(Particle_x, Particle_y, LX(4), LY(4));
P5 = Distance(Particle_x, Particle_y, LX(5), LY(5));
P6 = Distance(Particle_x, Particle_y, LX(6), LY(6));

% Compute Particle Probabilites: 
% From lecture notes: For each Particle the Probability = 1. 
Prob_Part = ones([1 1000]);

% Prob(particle) = gaussian(di,mi,noise/sigma)
% From lecture notes: Formula
Prob_Part_L1 = (1/sqrt(2*pi*8))*exp((-0.5*(P1 - LM1).^2)/8);
Prob_Part_L2 = (1/sqrt(2*pi*8))*exp((-0.5*(P2 - LM2).^2)/8);
Prob_Part_L3 = (1/sqrt(2*pi*8))*exp((-0.5*(P3 - LM3).^2)/8);
Prob_Part_L4 = (1/sqrt(2*pi*8))*exp((-0.5*(P4 - LM4).^2)/8);
Prob_Part_L5 = (1/sqrt(2*pi*8))*exp((-0.5*(P5 - LM5).^2)/8);
Prob_Part_L6 = (1/sqrt(2*pi*8))*exp((-0.5*(P6 - LM6).^2)/8);

% Return Prob(p)
Prob_Part = Prob_Part.*Prob_Part_L1;
Prob_Part = Prob_Part.*Prob_Part_L2;
Prob_Part = Prob_Part.*Prob_Part_L3;
Prob_Part = Prob_Part.*Prob_Part_L4;
Prob_Part = Prob_Part.*Prob_Part_L5;
Prob_Part = Prob_Part.*Prob_Part_L6;

% Weight Function: wp = Prob(p)
wp = Prob_Part;

% Resampling Section is sourced from Lecture notes.

W = sum(wp);     % Weights = Summation of weights calculated
alpha = wp/W;    % Normalized Weights

index = randi([1 1000]);
beta = 0;

Particle_X_New = zeros([1 1000]);
Particle_Y_New = zeros([1 1000]);
Particle_New = zeros([1 1000]);

% Referring Resampling Pseudo code from Lecture notes.
% Reference: Pseudo code to calculate Resampling from lecture notes.

for j = 1:1000
   beta = beta + unifrnd(0, 2*max(alpha));
   while(alpha(index) < beta)
      beta = beta - alpha(index);
      index = index + 1;
      if (index > 1000)
          index = 1;
      end
   end
   Particle_X_New(j) = Particle_x(index);
   Particle_Y_New(j) = Particle_y(index);
   Particle_New(j) = Particle(index);
end

Particle_x = Particle_X_New;
Particle_y = Particle_Y_New;
Particle = Particle_New;


% As mentioned in lecture notes "Consideration" section we will add noise
% and observe the behavior.

% New Particles and Robot Movement.
% The robot forward and turning motion noise is v = N(0,0.5)

Vel_noise = vel + normrnd(0, 0.5, [1 1000]);
Particle_x = Particle_x + Vel_noise.*cos(Particle).*dt;
Particle_y = Particle_y + Vel_noise.*sin(Particle).*dt;

Steering_noise = steering + normrnd(0, 0.5, [1 1000]);
Particle = Particle + Steering_noise.*dt;

Steering_noise = steering + normrnd(0, 0.5);
theta(i+1) = theta(i) + Steering_noise * dt;

Vel_noise = vel+normrnd(0, 0.5);
x(i+1) = x(i) + Vel_noise * cos(theta(i)) * dt;
y(i+1) = y(i) + Vel_noise * sin(theta(i)) * dt;

% Calculated New Guess position, updated.
R_X(i+1) = mean(Particle_x);
R_Y(i+1) = mean(Particle_y);
R(i+1) = mean(Particle);

i = i+1;

cla;
hold on;
plot(LX, LY, 'rO') % Landmark Points: Red circle

quiver(Particle_x, Particle_y,Particle_size*cos(Particle), Particle_size*sin(Particle), 0, 'k'); 
% Particles: Black arrow

quiver(x(i), y(i), 5.*cos(theta(i)), 5.*sin(theta(i)), 0, 'b', 'Marker', 'p', 'LineWidth', 2); 
% Robot Actual position

quiver(R_X(i), R_Y(i), 5.*cos(R(i)), 5.*sin(R(i)), 0, 'g', 'Marker', 's', 'LineWidth', 2);
% Robot Guess position
hold off;
xlim([0 100]); ylim([0 100]); 
legend('Landmarks', 'Particles', 'Actual Position', 'Estimated Position');
title('Particle Filter');

if (i > 100) % number of steps
   break 
end

pause(0.01);

end

% Function to calculate the Distance: From Lecture 9 Particle Filter Board Notes
function [Distance] = Distance(Particle_X, Particle_Y , Landmark_X, Landmark_Y)
Distance = sqrt((Particle_X-Landmark_X).^2 + (Particle_Y-Landmark_Y).^2);
end
