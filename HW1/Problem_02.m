% Go to goal

clear all
clc

%% Parameters

% Workspace Dimension
xlim([0 200])           % X Limit
ylim([0 200])           % Y Limit

%Velocity 
vel = 5;             % Maximum Speed m/s
vin = 0;             % Initial Speed m/s

%Steering angle
steering = pi/4;     % Maximum Steering Angle

%Robot in workspace, in x and y range
x=[]; 
y=[];
%% Robot Initial Pose

% Initial Position 
x(1) = 120;              % From question 1, X and Y coordinates is obtained.
y(1) = 130;              % x(1) = x(2) and y(1) = y(2).

% Initial Orientation 
theta(1) = pi/6;         % theta(1) = theta(2)

% Robot Model
robot = SquareRobot(x,y,theta(1));
% hold on;
plot(robot(:,1),robot(:,2),'-');    % Robot position 
xlim([0 200])
ylim([0 200])
%% Go To Goal

xg = 10;              % Desired end position, x axis, Goal x.
yg = 20;              % Desired end position, y axis, Goal y.

% Time step
i = 1;                % First time Step
dt = 0.5;

% Controller Parameter
Kv = 0.05;          % Velocity Gain
Kp = 0.8;           % Proportional Gain

velD = vin;         

while 1
    
    % Velocity proportional to its distance to the goal
    velD(end+1) = min(Kv*(sqrt(((xg - x(i))^2 )+ ((yg - y(i))^2))),vel);        
    % Choose the minimum value i.e. the velocity is set at max 5 m/s so it should not cross the limit.
    
%     velD(end+1) = Kv*(sqrt(((xg - x(i))^2 )+ ((yg - y(i))^2))); 
     % The velocity calculated is used.
    
    thetaD = atan2(yg-y(i),xg-x(i));    % Steer toward the goal
    gamma = Kp*(thetaD-theta(i));      % Proportional controller
    disp(velD)
    
    x(i+1) = x(i) + velD(end) * cos(theta(i)) * dt;
    y(i+1) = y(i) + velD(end) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + gamma * dt;

    robot = SquareRobot(x(i),y(i),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-',xg,yg,'o');
    xlim([0 200])
    ylim([0 200])
    
    i = i+1;
    
    if (abs(x(i)-xg) <= 1.0) && (abs(y(i)-yg) <= 1.0)
        hold on;
        robot = SquareRobot(x(1),y(1),theta(1));
        plot(robot(:,1),robot(:,2),'-');
        hold off;
        break
    end
    
    pause(0.01)
   
end

% Velocity Vs Time Plot
% y = [velD()] 
% plot(velD)
% grid on
% xlabel('Time')
% ylabel('Velocity')
% ylim([0 10])

% X Position Vs Time Plot
% y = [x()] 
% plot(x())
% grid on
% xlabel('Time')
% ylabel('X Position')
% ylim([0 200])

% Y Position Vs Time Plot
% y = [y()] 
% plot(y())
% grid on
% xlabel('Time')
% ylabel('Y Position')
% ylim([0 200])