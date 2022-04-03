% Square Robot Function 

function [robot] = SquareRobot(x,y,theta)
       
center = [x y];

a = [0 -1];
b = [1 0];
c = [0 1];
d = [-1 0];
e = [0 -1];

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

