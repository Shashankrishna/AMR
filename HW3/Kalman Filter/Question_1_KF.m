% Estimate the position and velocity of a point mass robot using Kalman Filter.
% The robot is initially located at [x,y] = [20,20] and moves with constant
% velocity 5 m/s to [x,y] = [70,20].

clc;
clear all;
close all;

%% Parameters

% Workspace Dimension
xlim([0 100])           % X Limit
ylim([0 100])           % Y Limit

% Initial Position 
x(1) = 20;             % X initial
y(1) = 20;             % Y initial
initial = [20, 20];

% Final Destination
xg = 70;
yg = 20;
goal = [70, 20];

i = 0;
dt = 0.1;
plot(x(1), y(1), '.');


%Velocity 
vel = 5;             % Maximum Speed m/s
vin = 0;             % Initial Speed m/s
vel_buffer = [0];

%Steering angle
steering = pi/4;     % Maximum Steering Angle

% Position Change
delta_x = x(1) - xg;
delta_y = y(1) - yg;
previous_error = 0;
error_sum = 0;

% PID Controller Parameters
Kp = 0.1;         % Proportional Gain
Ki = 0.001;       % Integral Gain
Kd = 0.03;        % Derivative Gain

% From lecture notes we define the parameters.

% Prediction Phase
% x = Estimate
% u = Motion vector
% F = State transition matrix
% P = Uncertainity covariance
% Q = Process noise

% Measurement Update Phase
% Z = Measurement 
% H = Measurement function
% R = Measurement noise
% S = Innovation
% Y = Error
% K = Kalman gain

x_t = [10; 5; 10; 5];

F_t = [1 dt 0 0; 
       0 1  0 0; 
       0 0  1 dt; 
       0 0  0 1];
   
H_t = [1 0 0 0;
       0 0 1 0; 
       1 0 0 0; 
       0 0 1 0];
   
P_t = [10 0 0 0; 
        0 1 0 0; 
        0 0 10 0; 
        0 0 0 1];
   
Q_t = [0 0 0 0; 
       0 1 0 0; 
       0 0 0 0; 
       0 0 0 1];
   
R_t = [6 0 0 0; 
       0 6 0 0; 
       0 0 4 0;
       0 0 0 4];

I = eye(4);
Z_t = [0; 0; 0; 0;];

velocity =[];
display = [];
Distance = [];
total_display = [];
Sensor1 = [];
Sensor2 = [];

while (abs(delta_x) >= 1.5 || abs(delta_y) >= 1.5)
    i = i+1;
   
    % Measurement Update Phase:-
    % normrnd(mu, sigma)
    sensor1_noise_x = normrnd(0, sqrt(6));         % For sensor 1 = N(0,6)
    sensor1_noise_y = normrnd(0, sqrt(6));         % mu(1x) = mu(1y)
    sensor2_noise_x = normrnd(0, sqrt(4));         % For sensor 2 = N(0.4)
    sensor2_noise_y = normrnd(0, sqrt(4));         % mu(2x) = mu(2y)
     
    % New Measurement Z_t  
    Z_t(1) = x(1) + sensor1_noise_x;
    Z_t(2) = y(1) + sensor1_noise_y;
    Z_t(3) = x(1) + sensor2_noise_x;
    Z_t(4) = y(1) + sensor2_noise_y;
    
    % Error Function
    Y_t = Z_t - (H_t * x_t);
    
    % Innovation
    S_t = (H_t * P_t * H_t') + R_t; 
    
    % Kalman Gain
    K_t = (P_t * H_t') * (inv(S_t));
    
    % State Estimation
    x_t = x_t + (K_t * Y_t);
    Dist = 0;
    Dist = norm(goal - [x_t(1), x_t(3)] );  % norm([goal] - [x_t(1), x_t(3)])
    Distance = [Distance, Dist];
    
    P_t = (I - (K_t * H_t)) * P_t;
    
    % Motion Update: Prediction Phase
    x_t = F_t * x_t;
    P_t = (F_t * P_t * F_t') + Q_t;

    display = [display, i];
    velocity = [velocity, x_t(2)];
    
    delta_x = xg - x(1);
    delta_y = yg - y(1);
    theta_d = atan2(delta_y, delta_x);
    
    % PID Control: Sourced from Question_1_Intro.m
    error = vel - vel_buffer(i);    % error = setpoint - input
    error_sum = error_sum + error;
    derivative = (error - previous_error)/dt; 
    integral = (error_sum * dt);    
    
    previous_error = error; 
    output = (Kp * error) + (Ki * integral)  + (Kd * derivative);    % Output of controller
    vel_buffer(i + 1) = vel_buffer(i) + output - (0.01 * vel_buffer(i)) ; 
    velos = vel_buffer(i+1);
    
    % Position update:
    x(1) =  x(1) + velos * cos(theta_d) * dt;
    y(1) =  y(1) + velos * sin(theta_d) * dt;
    total_display = [total_display; [x_t(1), x_t(3)]];
    Sensor1 = [Sensor1; [Z_t(1), Z_t(2)]];
    Sensor2 = [Sensor2; [Z_t(3), Z_t(4)]];
    
    % Plot_1: Robot_Movements: Current Estimate Vs Robot Position Vs Goal 
%     plot(x_t(1), x_t(3), 'o', x(:,1), y(:,1), '+', xg, yg, '*');
%     legend('Current Estimate', 'Robot Position', 'Goal');
    
    % Plot_2: Sensor Estimates: Sensor_1 Vs Sensor_2 Vs Sensor_Total
    plot(total_display(:, 1), total_display(:, 2), '-', Sensor1(:, 1), Sensor1(:, 2), '.', Sensor2(:, 1), Sensor2(:, 2), 'x')
    legend('Combined Sensor Estimate', 'Sensor One Estimate', 'Sensor Two Estimate')
    
    
    xlim([0 100]);
    ylim([0 100]);
    pause(0.1);
end

% Plot: Velocity Vs Time
figure;
plot(display, velocity);
title('Velocity Vs Time');
xlabel('Time(s)');
ylabel('Velocity(m/s)');

% Plot: Distance Vs Time
figure;
plot(display, Distance);
title('Distance Vs Time');
xlabel('Time(s)');
ylabel('Distance (m)');
