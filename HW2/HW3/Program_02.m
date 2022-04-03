clear;
close all;
clc;

% Workspace Dimension
xlim = 100;           % X Limit
ylim = 100;           % Y Limit
[x,y] = meshgrid (1:ylim, 1:xlim);

% Parameters
q_goal = [80 20];         % Goal = [80,20];
u_rep = zeros(100,100);   % Repulsive Potential Field

% Obstacle Coordinates       % obstacle (50:70, 30:50) [Square Coordinates]
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

% U_rep: Repulsive Potential Function
for i = 1:100
   for j = 1:100
       u_rep(i,j) = U_rep(Dist_obs([i,j], q_obs));
   end
end

figure;
mesh (u_rep.');
title ('Repulsive Potential');

%% Function

% Repulsive Function [From Lecture Notes]
function [u_rep] = U_rep(Rho)
    d0 = 20;
    nu = 50;
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

% Distance to Obstacle
function [min_dist,point_loc] = Dist_obs(q, q_obs)
    min_dist = inf;
    for i = 1:size(q_obs,2)
        distance = ((q(1) - q_obs{i}(1))^2 + (q(2) - q_obs{i}(2))^2);
        if min_dist > distance
            min_dist = distance;
            point_loc = q_obs{i};
        end
    end
end