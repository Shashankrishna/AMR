%% Project Sim Code

clear;
close all;
clc;

% Read Camera Data in from processing
inputData = readtable('data.csv', 'Delimiter', ',', 'ReadVariableNames', true);

% Start
xr = inputData.RobotX(1);
yr = inputData.RobotY(1);

% Goal
xg = inputData.GoalX(1);
yg = inputData.GoalY(1);

% Obst
obstX = inputData.ObstX;
obstY = inputData.ObstY;

clear inputData; % Remove unneeded table

%% Parameters
Width = 400;
Height = 400;

% Workspace Size
xlim([0 Width]);
ylim([0 Height]);

% Workspace Point Grid
x = 0:0.25:Width;
y = 0:0.25:Height;
[X, Y] = meshgrid(x, y); % Create matrix for 3D plotting

%% Generate Fields
% Attractive Potential Field (Goal)
field_1 = 1/2*((X-xg).^2 + (Y-yg).^2);

% Scale Field
k = max(field_1,[],'all');
field_1 = field_1./k;

% Constants for Point Fields
A = 1;
sig = [70, 70];

% Create Camera Repulsive Field
mu = [obstX(1), obstY(1)];
field_2 = A.*exp( -1.* ( (((X-mu(1)).^2) ./(2.*sig(1))) + (((Y-mu(2)).^2) ./(2.*sig(2))) ) );

for j = 2:length(obstX)
    mu = [obstX(j), obstY(j)];
    field_2 = field_2 + A.*exp( -1.* ( (((X-mu(1)).^2) ./(2.*sig(1))) + (((Y-mu(2)).^2) ./(2.*sig(2))) ) );
end

% Scale Field
k = max(field_2,[],'all');
field_2 = field_2./k;

% Global Frame Field, includes camera data
setField = field_1 + field_2;

%% Robot Movement

i = 1;

pX = []; % Robot position X
pY = []; % Robot position Y

pX(i) = xr; % Set initial
pY(i) = yr;

vel = []; % Robot velocity
vel(i) = 0; % Set initial

theta = []; % Robot angle
theta(i) = 0; % Set initial

dt = 0.1; % Time Step

Kp = 1000; % Vel Prop Const
Kp_angle = 1; % Ang Vel Prop Const

max_velocity = 5;
max_steering = pi/4;

while sqrt( (xg-pX(i))^2 + (yg-pY(i))^2 ) > 0.1 % Compute path
    
    [ranges, lidarX, lidarY] = LiDARSim(pX(i), pY(i), theta(i)); % Get LiDAR
    
    % Create LiDAR Repulsive Field
    mu = [lidarX(1), lidarY(1)];
    field_3 = A.*exp( -1.* ( (((X-mu(1)).^2) ./(2.*sig(1))) + (((Y-mu(2)).^2) ./(2.*sig(2))) ) );    
    
    for j = 2:length(lidarX)
        mu = [lidarX(j), lidarY(j)];
        field_3 = field_3 + A.*exp( -1.* ( (((X-mu(1)).^2) ./(2.*sig(1))) + (((Y-mu(2)).^2) ./(2.*sig(2))) ) );
    end
    
    % Scale Field
    k = max(field_3,[],'all');
    field_3 = field_3./k;
    
    % Combine all Fields
    f = setField + field_3;
    
    [FX, FY] = gradient(f);

    % Invert Gradient
    FX = -1.*FX;
    FY = -1.*FY;
    
    tmpX = round(pX(i)*4)/4; % Find closest known point on grid to current
    tmpY = round(pY(i)*4)/4; % location (determined by workspace grid spacing)
    
    indx = find(x==tmpX); % Get index in of point
    indy = find(y==tmpY); % in workspace arrays
    
    grad = [FX(indy, indx) FY(indy, indx)]; % Get gradient at closest location
    
    g_len = sqrt(grad(1)^2 + grad(2)^2); % Length of gradient
    
    % Normalized gradient
    norm_x = grad(1)/g_len;
    norm_y = grad(2)/g_len;
    
    vel(i) = Kp * g_len; % Compute velocity from gradient length
    
    if abs(vel(i)) > max_velocity % Range check
       vel(i) = sign(vel(i))*max_velocity; 
    end
    
    if abs(vel(i)) < 0.25 % Range check
       vel(i) = sign(vel(i))*0.25; 
    end
    
    thetaTarget = atan2(norm_y, norm_x); % Target angle from gradient
    
    gamma = Kp_angle*(thetaTarget-theta(i)); % Steering
    
    gamma = atan2(sin(gamma), cos(gamma)); % Coordinate Transform
    
    if (abs(gamma) > max_steering) % Range check
        gamma = sign(gamma)*(max_steering);
    end
    
    theta(i+1) = theta(i) + gamma * dt; % Calculate new anglei
    
    pX(i+1) = pX(i) + vel(i) * cos(theta(i)) * dt; % Calculate new position
    pY(i+1) = pY(i) + vel(i) * sin(theta(i)) * dt;
    
    % Plot and Save Keyframes (100 steps apart)
    if (i == 1)
        VideoPlot(x,y,X,Y,f,xr,yr,xg,yg,pX,pY,vel(i),obstX,obstY,lidarX,lidarY);
    end
    
    if (mod(i,100) == 0)
        VideoPlot(x,y,X,Y,f,xr,yr,xg,yg,pX,pY,vel(i),obstX,obstY,lidarX,lidarY);
        disp(i); 
    end

    % Plot and Save Frames
    %VideoPlot(x,y,X,Y,f,xr,yr,xg,yg,pX,pY,vel(i),obstX,obstY,lidarX,lidarY);
    
    i = i+1; % Iterate
        
end
