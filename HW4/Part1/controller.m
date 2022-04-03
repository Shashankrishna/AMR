function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;


% FILL IN YOUR CODE HERE

% From instructions pdf: Accessing states and robot params

% The position and velocity are stored in the s variable, which is a 2 × 1 vector. 
% They can be accessed stored into the variables pos and vel as follows:

pos = s(1);
vel = s(2);

% The desired position and velocity can be stored into the pos_des and
% vel_des variables by accessing them in the s_des variable:

pos_des = s_des(1);
vel_des = s_des(2);

g = params.gravity;                       % acceleration due to gravity
m = params.mass;                          % robot mass
Kp = 50;                                  % Proportional Gain 
Kv = 10;                                  % Velocity Gain 
e_pos = pos_des - pos;                    % position error
e_vel = vel_des - vel;                    % velocity error
Z_des_dot_dot = 0;

% Control input for a PD controller:
u = m*(Z_des_dot_dot + (Kp*e_pos) + (Kv*e_vel) + g);  

end

