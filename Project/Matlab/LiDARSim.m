function [ranges, xLoc, yLoc] = LiDARSim(RobotX, RobotY, RobotT)
% Simulates LiDAR Sensor for an Obstacle

% Init variables
ranges = Inf.*ones(1,720); % Holds range for each angle
xLoc = []; % Detections in X
yLoc = []; % Detections in Y 
itr = 1;

angles = 0:0.5:(360-0.5); % Angles around sensor (0.5 deg seperation)

if isnan(RobotT)
    RobotT = 0; % Err check for robot angle
end

for i = 1:720

    lidarAngle = RobotT + angles(i); % Get global angle of LiDAR beam

    r = 0; % Radius
    x = 0; % X pos
    y = 0; % Y pos

    while true

        % Calculate X and Y from radius and angle
        x = RobotX + r*cosd(lidarAngle);
        y = RobotY + r*sind(lidarAngle);

        % Workspace Check: If outside workspace, stop
        if (x < -5) || (x > 405) || (y < -5) || (y > 405)
            break;
        end

        % Obst. Check: If at/inside obstacle, stop
        if (x < 145) && (x > 25) && (y < 130) &&(y > 100)
            break;
        end

        r = r+0.1; % Increase radius

    end

    ranges(i) = r; % Store ranged data

    % Round x,y pos inside workspace to coordinate frame and store values
    if (x >= 0) && (x <= 400) && (y >= 0) && (y <= 400)
        xLoc(itr) = round(x);
        yLoc(itr) = round(y);
        itr = itr+1;
    end

end

% Remove duplicate xy points
points = [xLoc' yLoc'];
points = unique(points, 'rows', 'stable');

% Return data
xLoc = points(:,1)';
yLoc = points(:,2)';
    
end

