% follow a wall

clear all
clc

%% Parameters

% Workspace Dimension
xlim([0 200])           % X Limit
ylim([0 200])           % Y Limit

%Velocity 
vel = 5;            % Maximum Speed m/s
vin = 0;            % Initial Speed m/s

%Steering angle and Minimum Distance
steerA = -pi/4;    % Maximum Steering Angle
dmin = 10;         % Minimum Distance between robot and line in meter.

%Robot in workspace, in x and y range
x=[]; 
y=[];
theta=[];
X_Line = 1:200;     % Line limit, it must not exceed the workspace  
%% Robot Initial Pose

% Initial Position 
r = 180*rand();
x(1) = 110;
y(1) = 160;

% Initial Orientation 
theta(1) = -pi/4;

% Robot Model
robot = SquareRobot(x,y,theta(1));
% hold on;
plot(robot(:,1),robot(:,2),'-');     % Robot position 
xlim([0 200])
ylim([0 200])
%% Follow The Line

% Two controllers are needed for this task.

% Line Equation
% a*x + b*y + c = 0;
% y = -(c/b) - (a/b)*x {[y = mx + c] m is slope}

% Time step
i = 1;
dt = 0.1;

% Line parameters
a = 1;
b = -1;
c = 1;

% Controller Parameter
Kt = 0.8;    % Proportional gain to turn the robot towards the line.
Kh = 0.8;    % Proportional gain to adjust the heading angle to be paralle to the line.
 
d = [];

while 1
    
    d(i) = (a*x(i) + b*y(i) + c)/sqrt(a^2 + b^2);  % Normal Distance

    thetaD = atan2(-a, b);
    gamma = -Kt*(d(i) - 10) + Kh*(thetaD-theta(i));      % Combining two controllers
    
    E = atan2(sin(gamma), cos(gamma));   
    
    if (abs(E) > steerA)
        E = sign(E)*(pi/4);
    end
    
    theta(i+1) = theta(i) + E * dt;
    
    x(i+1) = x(i) + vel * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel * sin(theta(i)) * dt;
    
    robot = SquareRobot(x(i),y(i),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    hold on;
    plot(X_Line, a*X_Line + c, 'k--');
    hold off;
    xlim([0 200])
    ylim([0 200])
    
    i = i+1;
    
    if (x(i) < 1) || (x(i) > 199) || (y(i) < 1)  || (y(i) > 199)  
        hold on;
        robot = SquareRobot(x(1),y(1),theta(1));
        plot(robot(:,1),robot(:,2),'-');
        plot(X_Line, a*X_Line + c, 'k--');
        hold off;
        break
    end
    
    pause(0.01)

end

% Relative Distance Vs Time Plot
y = [d] 
plot(d)
grid on
xlabel('Time')
ylabel('Relative Distance')
% ylim([])