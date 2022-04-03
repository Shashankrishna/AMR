clc;
clear all;
close all;
%% Parameters

% Workspace Dimension
xlim([0 100])           % X Limit
ylim([0 100])           % Y Limit

%Velocity 
vel = 5;             % Maximum Speed m/s
vin = 0;             % Initial Speed m/s

%Steering angle
steering = pi/4;     % Maximum Steering Angle

%Robot in workspace, in x and y range
x = []; 
y = [];
theta = [];
v = []; 
%% Robot Initial Pose

% Initial Position 
x(1) = 20;             
y(1) = 20;              

% Initial Orientation 
theta(1) = pi/6;         

% Inital Velocity
v(1) = 0;

% Point Mass Robot
xlim([0 100])
ylim([0 100])
%% Go To Goal

xg = 70;        % Desired end position, x axis, Goal x.
yg = 20;        % Desired end position, y axis, Goal y.

% Time step
i = 1;                % First time Step
dt = 0.5;

% PID Controller Parameters
Kp = 0.1;         % Proportional Gain
Ki = 0.001;       % Integral Gain
Kd = 0.03;        % Derivative Gain

% Predefined Values
error = 0;             
previous_error = 0;
integral = 0;
setpoint = 3; % (Vref = 3 m/s)
measured_value = 0;
output = 3;
 
while 1
   
    % Velocity Control
    error = setpoint - vel; 
    integral = integral + error*dt;
    derivative = (error - previous_error)/dt;
    
    output = Kp*error + Ki*integral + Kd*derivative; % Output of controller

    previous_error = error;
    
    vel = vel + output - 0.01*vel;
           
    % Angular Control
    thetaD = atan2(yg-y(i),xg-x(i));    % Steer toward the goal
    E = atan2(sin(thetaD - theta(i)),cos(thetaD - theta(i)));  % e' = atan2(sin(e),cos(e)) ; e = thetaD - theta(i)
    steerA = Kp*E;
    
    if (abs(E) > steerA)
        E = sign(E)*(pi/4);
    end
   
    x(i+1) = x(i) + vel * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + E * dt;
    
    cla;
    hold on;
    plot(x(i), y(i), 'ok');
    plot(x,y,'b');
    hold off;
    
    i = i+1;
    
    if (abs(x(i)-xg) <= 1.0) && (abs(y(i)-yg) <= 1.0)
        cla;
        hold on;
        plot(x(i), y(i), 'ok');
        plot(x,y,'k');
        hold off;
        break
    end    
    pause(0.01)   
end