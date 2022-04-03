% Cruise Controller

clear all
clc
%% Parameters

% Workspace Dimension
xlim([0 200])           % X Limit
ylim([0 200])           % Y Limit

%Velocity 
vel(1) = 5;            % Maximum Speed m/s
vin = 0;               % Initial Speed m/s
vref = 3;              % Reference Speed m/s

%Steering angle
steerA = pi/4;    % Maximum Steering Angle

%Robot in workspace, in x and y range
x=[]; 
y=[];
theta=[];
%% Robot Initial Pose

% Initial Position 
x(1) = 100;             % Setting the initial position x,y to 100,100.
y(1) = 100;

% Initial Orientation 
theta(1) = pi/6;

% Robot Model
robot = SquareRobot(x,y,theta(1));
% hold on;
plot(robot(:,1),robot(:,2),'-');     % Robot position 
xlim([0 200])
ylim([0 200])
%% Cruise Controller

xg = [10 150 190];             % Goal on x axis, 3 locations.
yg = [50 140 190];             % Goal on y axis, 3 locations.

% Time step
i = 1;
dt = 0.3;

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
output = vref;

j = 1;          % xg and yg have three locations, so to switch to each we have defined j = 1.

 while 1
    
    % Velocity Control
    error = setpoint - vel(i); 
    integral = integral + error*dt;
    derivative = (error - previous_error)/dt;
    
    output = Kp*error + Ki*integral + Kd*derivative; % Output of controller

    previous_error = error;
    
    vel(i+1) = vel(i)+output - 0.01*vel(i);
    
    if (((x(i) - xg(j))^2 + (y(i) - yg(j))^2) < 1 )   % Loop: switch to three locations.
        j = j+1;
          if j > 3
              break
          end
    end 
        
    % Angular Control
    thetaD = atan2(yg(j)-y(i),xg(j)-x(i));    % Steer toward the goal
    E = atan2(sin(thetaD - theta(i)),cos(thetaD - theta(i)));  % e' = atan2(sin(e),cos(e)) ; e = thetaD - theta(i)
    steerA = Kp*E;
    
    if (abs(E) > steerA)
        E = sign(E)*(pi/4);
    end
           
    x(i+1) = x(i) + vel(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel(i) * sin(theta(i)) * dt; 
    theta(i+1) = theta(i) + E * dt;
    
    robot = SquareRobot(x(i),y(i),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-',xg(1),yg(1),'o',xg(2),yg(2),'o',xg(3),yg(3),'o');
    xlim([0 200])
    ylim([0 200])
    
     i = i+1;
    
     if (abs(x(i)-xg(j)) <= 1.0) && (abs(y(i)-yg(j)) <= 1.0)
         hold on;
         robot = SquareRobot(x(1),y(1),theta(1));
        plot(robot(:,1),robot(:,2),'-');
        hold off;
%         break
    end
    
    pause(0.01)
  end

% Velocity Vs Time Plot
% y = [vel] 
% plot(vel)
% grid on
% xlabel('Time')
% ylabel('Velocity')
% ylim([0 10])
