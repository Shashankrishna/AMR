clear;
close all;
clc;

% Workspace Dimension
xlim = 100;           % X Limit
ylim = 100;           % Y Limit
q_goal = [80 20];        % Goal = [80,20];

% Parameters

x_Init = 10;             % X Start Point
y_Init = 80;             % Y Start Point

% Obstacle Coordinates 
x_CO = [30 50 50 30];    % X Coordinate Points
y_CO = [50 50 70 70];    % Y Coordinate Points

x_CE = 40;               % X Center Point in Square
y_CE = 60;               % Y Center Point in Square

x = 0:0.5:100;
y = 0:0.5:100;

[X, Y] = meshgrid(x, y);

% Attractive potential function
Xi = 0.001;
u_att = 1/2 * Xi *((X - q_goal(1)).^2 + (Y - q_goal(2)).^2);

% Distance 
d = sqrt((y_CE - Y).^2 + (x_CE - X).^2);
d2 = (d/10) + 2;

% Repulsive potential function
nu = 100;
d0 = 10;

u_rep = nu*((1./d2 - 1/d0).^2);
u_rep(d2 > d0) = 0;

u_total = u_att + u_rep;

% Radial repulsive potential

figure(1);
mesh(X, Y, u_total);
hold on;
plot3(q_goal(1), q_goal(2), 0, 'xw');
plot3(x_CO, y_CO, [8 8 8 8], 'xk');
plot3(x_Init, y_Init, u_total(find(y == y_Init), find(x == x_Init)), 'xm');
hold off;

figure(2);
contour(X, Y, u_total, 20);
hold on;
plot3(q_goal(1), q_goal(2), 0, 'xk');
plot3(x_CE, y_CE, 0, 'xk');
plot3(x_CO, y_CO, [0 0 0 0], 'xk');
plot3(x_Init, y_Init, u_total(find(y == y_Init), find(x == x_Init)), 'xm');
hold off;
