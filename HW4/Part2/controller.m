function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
% Control the motion of the quadrotor in the Y-Z plane.

m = params.mass;         % robot mass
g = params.gravity;      % acceleration due to gravity
Ixx = params.Ixx;        % moment of inertia 
phi = state.rot;
phid = state.omega;

Dy =  des_state.pos(1);     % Y-axis Position
Dz =  des_state.pos(2);     % Z-axis Position

Dyd =  des_state.vel(1);    % Y-axis Velocity / First Derivative
Dzd =  des_state.vel(2);    % Z-axis Velocity / First Derivative

Dydd =  des_state.acc(1);   % Y-axis Acceleration / Second Derivative
Dzdd =  des_state.acc(2);   % Z-axis Acceleration / Second Derivative
       

Kvy = 5;             % Velocity Gain in Y-axis
Kvz = 2;             % Velocity Gain in Z-axis
Kvphi = 10;          % Velocity Gain for roll angle (phi): Orientation

Kpy = 20;            % Position Gain in Y-axis
Kpz = 60;            % Position Gain in Z-axis
Kpphi = 1000;        % Position Gain for roll angle (phi): Orientation

y = state.pos(1);    % Y-axis Position
z = state.pos(2);    % Z-axis Position
yd = state.vel(1);   % Y-axis Velocity / First Derivative
zd = state.vel(2);   % Z-axis Velocity / First Derivative

% From instructions pdf: Hover Controller

u1 = m * (g + Dzdd + (Kvz * (Dzd - zd)) + (Kpz * (Dz - z)));

phic = (-1/g) * (Dydd + Kvy * (Dyd - yd) + (Kpy * (Dy - y)));

phic_d = 0;      % First Derivative
phic_dd = 0;     % Second Derivative

u2 = Ixx * (phic_dd + Kvphi * (phic_d  - phid) + Kpphi * (phic - phi));

end
