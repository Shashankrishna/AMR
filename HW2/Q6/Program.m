clear;
close all;
clc;

% Parameters
q_goal = [80 20];         % Goal = [80,20];
u_att = zeros(100,100);   % Attractive Potential Field
u_rep = zeros(100,100);   % Repulsive Potential Field
q_start = [10 80];        % Start Position of Robot
v_max = 5;                % Maximum Velocity 

% Obstacle Coordinates    % obstacle (50:70, 30:50) [Square Coordinates]
obs_x_min = 30;
obs_x_max = 50;
obs_y_min = 50;
obs_y_max = 70;

obs_x_len = obs_x_max - obs_x_min + 1;
obs_y_len = obs_y_max - obs_y_min + 1;

for i = 1:obs_x_len
   for j = 1:obs_y_len
       q_obs{obs_x_len*(i-1)+j} = [obs_x_min+i-1 obs_y_min+j-1];
   end
end

% u_att: Attractive Potential Function
for i = 1:100
   for j = 1:100
       u_att(i,j) = U_att([i,j], q_goal);
   end
end

figure(1);
mesh(u_att.')
title ('Attractive Potential');

% U_rep: Repulsive Potential Function
for i = 1:100
   for j = 1:100
       u_rep(i,j) = U_rep(Dist_obs([i,j], q_obs));
   end
end

figure(2);
mesh(u_rep.')
title ('Repulsive Potential');

% F_att: Attractive Potential Force
for i = 1:100
   for j = 1:100
       f_att{i,j} = F_att([i,j], q_goal);
   end
end

% F_rep: Repulsive Potential Force
for i = 1:100
   for j = 1:100
       [min_dist,point_loc] = Dist_obs([i,j], q_obs);
       f_rep{i,j} = F_rep([i,j],min_dist,point_loc);
   end
end

u_total = u_att + u_rep;    % Total Potential: Attractive + Repulsive

figure(3);
mesh (u_total.');
title ('Total Potential');

for i = 1:100
    for j = 1:100
        x(i,j)=i;
        y(i,j)=j;
        dx(i,j) = f_att{i,j}(1);
        dy(i,j) = f_att{i,j}(2);
    end
end

figure(4);
quiver(x,y,dx,dy)
title ('Attractive Force');

for i = 1:100
    for j = 1:100
        dx(i,j) = f_rep{i,j}(1);
        dy(i,j) = f_rep{i,j}(2);
    end
end

figure(5);
quiver(x,y,dx,dy)
title ('Repulsive Force');

for i = 1:100
    for j = 1:100
        f_total{i,j} = f_att{i,j} + f_rep{i,j};
        dx(i,j) = f_total{i,j}(1);
        dy(i,j) = f_total{i,j}(2);
    end
end

figure(6);
quiver(x,y,dx,dy)
title ('Total Force');

% Non-holonomic robot: Square Robot from HW#1.
x = [];
y = [];
vel = [];
theta = [];

x(1) = q_start(1);
y(1) = q_start(2);
vel(1) = 0;
theta(1) = 2*pi*rand();
nstep = 3000;
kp = 5;               % Position Gain
dt = 0.01;
a_limit = 10;         % Acceleration Limit
L = 8;
steering = pi/4;
R1 = L/tan(steering);

for i = 1:nstep
    dx = f_total{round(x(i)),round(y(i))}(1);
    dy = f_total{round(x(i)),round(y(i))}(2);
    len_d = (dx^2 + dy^2)^0.5;
    if len_d > 0
        dir_x = dx/len_d;
        dir_y = dy/len_d;
    end
    
    x(i+1) = x(i) + vel(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel(i) * sin(theta(i)) * dt;
    
    theta_goal = atan2(dir_y,dir_x);
    theta_error = theta_goal - theta(i);
    theta_error = atan2(sin(theta_error),cos(theta_error));
    
    distance = ((q_goal(1) - x(i))^2 + (q_goal(2) - y(i))^2)^0.5;
    
    theta(i+1) = theta(i) + theta_error;
    vel(i+1) = kp * distance;
    
    if (abs(vel(i+1) - vel(i)) > a_limit * dt)
        if (vel(i+1) - vel(i) > 0)
            vel(i+1) = vel(i) +  a_limit * dt;
        else
            vel(i+1) = vel(i) -  a_limit * dt;
        end
    end
    
    if vel(i+1) > v_max
        vel(i+1) = v_max;
    elseif vel(i+1) < 0
        vel(i+1) = 0;
    end
    
    if (abs(theta(i+1) - theta(i))/dt > vel(i)/R1)
        if (theta(i+1) - theta(i) > 0)
            theta(i+1) = theta(i) + vel(i)/R1 * dt;
        else
            theta(i+1) = theta(i) - vel(i)/R1 * dt;
        end
    end
    
    robot = SquareRobot(x(i),y(i),theta(i));
    figure(7)
    contour(u_total.')
    hold on
    plot(x,y,'-',robot(:,1),robot(:,2),'-',q_goal(1),q_goal(2),'r>');
    hold off
    xlim([0 100])
    ylim([0 100])
    pause(0.01)
    
    if distance < 0.1
       break; 
    end
end

%% Functions

% Attractive Function [From Lecture Notes]
function [u_att] = U_att(q, q_goal)
    Xi = 0.001;
    u_att = 1/2*Xi*((q(1) - q_goal(1))^2 + (q(2) - q_goal(2))^2);
end

% Attractive Potential Force [From Lecture Notes]
function [f_att] = F_att(q, q_goal)
    Xi = 0.001;
    f_att = -Xi.*[q(1) - q_goal(1) q(2) - q_goal(2)];
end

% Repulsive Function [From Lecture Notes]
function [u_rep] = U_rep(Rho)
    d0 = 20;
    nu = 500;
    if Rho > d0
        u_rep = 0;
    elseif Rho == 0
        u_rep = 1/2*nu*(1 - 1/d0)^2;
    else
        u_rep = 1/2*nu*(1/Rho - 1/d0)^2;
    end
    if u_rep > 5
        u_rep = 5;
    end
end

% Repulsive Potential Force [From Lecture Notes]
function [f_rep] = F_rep(q, Rho, point_loc)
    d0 = 20;
    nu = 20;
    dis = ((q(1) - point_loc(1))^2+(q(2) - point_loc(2))^2)^0.5;
    if Rho > d0
        f_rep = [0 0];
    elseif Rho == 0
        f_rep = [0 0];
    else
        f_rep = nu*(1/Rho - 1/d0)*(1/Rho)^2/dis*[(q(1) - point_loc(1)) (q(2) - point_loc(2))];
    end
end

% Measure Distance to Obstacle
function [min_dist,point_loc] = Dist_obs(q, q_obs)
    min_dist = inf;
    for i = 1:size(q_obs,2)
        dis = ((q(1) - q_obs{i}(1))^2+(q(2) - q_obs{i}(2))^2)^0.5;
        if min_dist > dis
            min_dist = dis;
            point_loc = q_obs{i};
        end
    end
end

% Square Robot Function 
function [robot] = SquareRobot(x,y,theta)
       
center = [x y];

a = [0 -2];
b = [2 0];
c = [0 2];
d = [-2 0];
e = [0 -2];

% Rotation Matrix

rotmat = [cos(theta) -sin(theta); sin(theta) cos(theta)];

rota = (rotmat * (a'));
rotb = (rotmat * (b'));
rotc = (rotmat * (c'));
rotd = (rotmat * (d'));
rote = (rotmat * (e'));

% Final Robot Configuration after transformation

robot1 = [rota(1) + center(1), rota(2) + center(2)];
robot2 = [rotb(1) + center(1), rotb(2) + center(2)];
robot3 = [rotc(1) + center(1), rotc(2) + center(2)];
robot4 = [rotd(1) + center(1), rotd(2) + center(2)];
robot5 = [rote(1) + center(1), rote(2) + center(2)];

robot = [robot1;robot2;robot3;robot4;robot5];
 
end

