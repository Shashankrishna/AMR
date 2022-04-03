function VideoPlot(x,y,X,Y,f,xr,yr,xg,yg,pX,pY,v,obstX,obstY,lidarX,lidarY)
% Displays/Saves plots as png files

persistent itr
if isempty(itr)
   itr = 0;
end

% For graphics, compute 'height' of each point along robot path
path_Z = [];
for i = 1:length(pX)-1 
        tmpX = round(pX(i)*4)/4; % Find closest known 
        tmpY = round(pY(i)*4)/4; % point on grid to current location (determined by workspace grid)

        indx = find(x==tmpX); % Get index in of point
        indy = find(y==tmpY); % in workspace arrays

        path_Z(i) = f(indy, indx); % Get height of points along path
end
path_Z = path_Z+0.01;

% Plot Field as 3D
figure(1);
cla;

mesh(X, Y, f);

hold on;
plot3(xg, yg, 0, 'xw'); % Goal
plot3(xr, yr, f(find(y==yr), find(x==xr)), 'xm'); % Robot Start
plot3(pX(end-1), pY(end-1), path_Z(end), 'ok'); % Robot Current
plot3(pX(1:end-1), pY(1:end-1), path_Z, 'k'); % Path
hold off;

xlabel("X");
ylabel("Y");
view(67, 41.5);

filename = sprintf('%s%d%s','imgs/img3D_',itr,'.png');
saveas(gcf,filename);

% Plot field contour
figure(2);
cla;

contour(X, Y, f, 20);

hold on;
%for i = 1:length(lidarX)
%   plot([pX(end) lidarX(i)], [pY(end) lidarY(i)], 'Color', [0.8 0.8 0.8]); % LiDAR Ray
%end
plot(xr, yr, 'xk'); % Robot Start
plot(pX(end), pY(end), 'ok'); % Robot Current
plot(xg, yg, 'xk'); % Goal
plot(pX, pY, 'k'); % Path
plot(obstX, obstY, '.k'); % Camera Obst
plot(lidarX, lidarY, '.k'); % LiDAR Obst
hold off;

axis square;
xlabel("X");
ylabel("Y");

filename = sprintf('%s%d%s','imgs/img_',itr,'.png');
saveas(gcf,filename);

itr = itr+1;
end

