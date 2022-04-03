clear;
close all;
clc;

% Workspace Dimension
xlim = 100;           % X Limit
ylim = 100;           % Y Limit
[x,y] = meshgrid (1:ylim, 1:xlim);

% Parameters
q_goal = [80 20];         % Goal = [80,20];
u_att = zeros(100,100);   % Attractive Potential Field

% u_att: Attractive Potential Function

% for i = 1:100
%    for j = 1:100
%        u_att(i,j) = U_att([i,j], q_goal);
%    end
% end

Xi = 0.001;
u_att = 1/2 * Xi * ((x - q_goal(1)).^2 + (y - q_goal(2)).^2);

figure;
m = mesh (u_att);
title ('Attractive Potential');

% Use this when calculating u_att with function. 
% figure;
% m = mesh (u_att.');
% title ('Attractive Potential');

%% Function

% Attractive Function [From Lecture Notes]
function [u_att] = U_att(q, q_goal)
    Xi = 0.001;
    u_att = 1/2*Xi*((q(1) - q_goal(1))^2 + (q(2) - q_goal(2))^2);
end
