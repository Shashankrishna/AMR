% Sensor Information
% sensor measure: generate a random number from the normal distribution with mean 
% parameter mu(0) and standard deviation parameter sigma.

% Sensor 1 x Position = N(0,6), sizeof(x)
% Sensor 1 y Position = N(0,6), sizeof(y)

% Sensor 2 x Position = N(0,4), sizeof(x)
% Sensor 2 y Position = N(0,4), sizeof(y)

s_x = normrnd(0, 2.4, [1,32]); % Calculate sigma from Lecture notes
s_y = normrnd(0, 2.4, [1,32]); % sigma = 1/((1/4) + (1/6)) = 2.4

xF = x + s_x; 
yF = y + s_y;

plot(xF, yF, '.')
axis square
xlim([0 100])
ylim([0 100])