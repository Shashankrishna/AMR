% Homogeneous Transformation

clear all
clc

%% Parameters

% Workspace Dimension
xlim([0 200])           % X Limit
ylim([0 200])           % Y Limit

%Velocity 
vel = 5;          % Maximum Speed m/s
vin = 0;          % Initial Speed m/s

%Steering angle
steering = pi/4;  % Maximum Steering Angle

%Robot in workspace, in x and y range.
x=[]; 
y=[];
%% Robot Initial Pose

% Initial Position 
x(1) = 100;               % Setting the initial position x,y to 100,100.
y(1) = 100;

% Initial Orientation 
r = (2*pi)*rand(); % Random angle 
theta(1) = r;

% Robot Model
robot = SquareRobot(x,y,theta(1));    
hold on;
plot(robot(:,1),robot(:,2),'-');       % Robot position 
xlim([0 200])
ylim([0 200])    
%% Transformation 
    
x(2) = 120;        % Desired end position, x axis.
y(2) = 130;        % Desired end position, y axis.
theta(2) = pi/6;   % Desired rotating angle.

robot = SquareRobot(x(2),y(2),theta(2));
plot(robot(:,1),robot(:,2),'-',x,y,'-');
xlim([0 200])
ylim([0 200])  
    