clc
clear
close all

%% Initial experiment with STL-files

% Load object and extract points
object = stlread('plate.stl');
points = object.Points;

% Plot the figure
xvals = points(:,1);
yvals = points(:,2);
zvals = points(:,3);
plot3(xvals,yvals,zvals,'.');
xlabel('x [mm]');
ylabel('y [mm]');
zlabel('z [mm]');

% Get axis dimensions
xmax = max(xvals);
xmin = min(xvals);
xdiff = xmax-xmin;
ymax = max(yvals);
ymin = min(yvals);
ydiff = ymax-ymin;
zmax = max(zvals);
zmin = min(zvals);
zdiff = zmax-zmin;
axis([xmin-0.1*xdiff xmax+0.1*xdiff ymin-0.1*ydiff ymax+0.1*ydiff zmin-0.1*zdiff zmax+0.1*zdiff])

%% Trying to load the generated point cloud of the object and visualizing it

% Plate dimensions: 100*100*1 [cm]
cloud = pcread('platePoints.ply');

% For visualization of the point cloud
pcshow(cloud)

% Choose a random point along the x-axis as the starting point for the
% polishing path
xRand = randi([0,100]);
startpoint = [xRand 0];
endpoint = [xRand 100];

% Discretize the points on the line between the two points in order to simulate datapoints
% collected from LfD
numberOfPoints = 10; % The number of points to sample on the line
% Each row is a seperate point, with the first column being the x-coordinate and the second column being the y-coordinate
datapoints = [zeros(numberOfPoints,2) ones(numberOfPoints,1)]; 
lineVec = endpoint - startpoint; % Vector of the line
for i = 1:numberOfPoints
    datapoints(i,1:2) = [lineVec(1)/numberOfPoints*i+startpoint(1) lineVec(2)/numberOfPoints*i+startpoint(2)];
end

% Plotting the point cloud and the line in the same figure
plot3(datapoints(:,1),datapoints(:,2),datapoints(:,3))
hold on
plot3(cloud.Location(:,1),cloud.Location(:,2),cloud.Location(:,3),'o')

%% Modelling of contact area

% I need to translate this disc along the line i have created at some
% contact depth
% I need to locally fit a plane to the point cloud to use for modelling (IS this nessecary when just doing a plane?)

% Model is in cm in the graph

% Disc parameters
R = 5; % [cm] radius
H = 1; % [cm] height
depth = 0.1; % [cm] Depth at which the disc is below the surface of the object
discArea = pi * R^2;
discRadiusStep = 1; % [cm] Step-size of the radius of the disc
discCircleStep = 36; % [degree] Step-size of the angle of the circle
discRadiusRes = R / discRadiusStep; % [cm] Resolution of the disc radius
discCircleRes = 360 / discCircleStep; % [degree] Resoultion of the disc circle
numberOfDiscPoints = discCircleRes * discRadiusRes;
unitArea = discArea/numberOfDiscPoints; % Area per discretized circle point

% Generate circle points
circlePoints = [zeros(discRadiusRes*discCircleRes+1,2) zeros(discRadiusRes*discCircleRes+1,1)-depth];
radiusIndex = 0;
for j = discRadiusStep:discRadiusStep:R % Iterate through the points on the radius of the circle
    circleIndex = 1;
    for k = 0:discCircleStep:360 % Iterate through all points in the sub-circle
        xval = cos(deg2rad(k))*j;
        circlePoints(radiusIndex*(360/discCircleStep)+circleIndex,1) = xval;
        yval = sin(deg2rad(k))*j;
        circlePoints(radiusIndex*(360/discCircleStep)+circleIndex,2) = yval;
        circleIndex = circleIndex + 1;
    end
    radiusIndex = radiusIndex + 1;
end

% Plot the circle
plot3(circlePoints(:,1),circlePoints(:,2),circlePoints(:,3),'o');


%% Moving the circle along the trajectory.
% This is done by adding the diffrence between the current frame and the
% previous frame to all circle points, with the initial frame being the
% world frame

% Initialize copy of circle points to translate.
% This is done to avoid having to run multiple sections while testing
circleCopy = circlePoints;

% Iterate through the points in the trajectory
for i = 1:size(datapoints,1)
    % Calculate the difference between points on the trajectory
    % The first point doesn't differ, hence the if statement
    if i > 1
        pointDiff = datapoints(i,:)-datapoints(i-1,:);
    else
        pointDiff = datapoints(i,:);
    end
    % Loop trough all circle points and add the current point difference
    for j = 1:size(circlePoints,1)
        circleCopy(j,:) = circleCopy(j,:) + pointDiff;
    end
    % Save figure
    figure(1)
    hold on
    if mod(i,2) == 0
        plot3(circleCopy(:,1),circleCopy(:,2),circleCopy(:,3),'or')
    else
        plot3(circleCopy(:,1),circleCopy(:,2),circleCopy(:,3),'ob')
    end
    plot3(datapoints(:,1),datapoints(:,2),datapoints(:,3),'k')
    xlim([-20 120]);
    ylim([-20 120]);
    zlim([0.5 1.5]);
    view(45,45); % Change the view of the 3D figure. This is around the x-axis
    hold off
    F(i) = getframe(gcf);
end

% Make an illustration video
videoWriterObj = VideoWriter('circleTranslation.avi');
videoWriterObj.FrameRate = 10;
open(videoWriterObj);
for i=1:length(F)
 frame = F(i);
 writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);

%% Computation of points in contact with the object

% This should be 100% for this plane

% Selecting all points in the point cloud that are above the depth, and 

% The unit area varies from time to time, so has to be calculated for each
% frame. First, calculate for the x-y conditions for the unit area, and
% then find out which are in contact by applying the z-condition. How does
% this apply to the cylinder? It has to be in relation to the z-axis of the
% frame, so i might have to work with that

% There might be an issue once we move away from a plane to more complex
% objects like a cylinder