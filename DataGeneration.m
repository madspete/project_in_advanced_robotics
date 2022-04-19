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

% Disc parameters
R = 5; % [cm] radius
H = 1; % [cm] height
depth = 0.1; % [cm] Depth at which the disc is below the surface of the object

% Plate dimensions: 100*100*1 [cm]
cloud = pcread('platePoints.ply');

% For visualization of the point cloud
% If regular plotting at the same time, use seperate figures
%pcshow(cloud)

% MAYBE LINSPACE CAN BE USED HERE INSTEAD?
% Choose a random point along the x-axis as the starting point for the
% polishing path
xRand = randi([0,100]);
startpoint = [xRand 0];
endpoint = [xRand 100];

% Discretize the points on the line between the two points in order to simulate datapoints
% collected from LfD
numberOfPoints = 10; % The number of points to sample on the line
% Each row is a seperate point, with the first column being the x-coordinate and the second column being the y-coordinate
datapoints = [zeros(numberOfPoints,2) zeros(numberOfPoints,1)+H-depth]; 
lineVec = endpoint - startpoint; % Vector of the line
for i = 1:numberOfPoints
    datapoints(i,1:2) = [lineVec(1)/numberOfPoints*i+startpoint(1) lineVec(2)/numberOfPoints*i+startpoint(2)];
end

% Plotting the point cloud and the line in the same figure
plot3(datapoints(:,1),datapoints(:,2),datapoints(:,3))
hold on
plot3(cloud.Location(:,1),cloud.Location(:,2),cloud.Location(:,3),'o')
view(-45,2); % Change the view of the 3D figure. This is around the x-axis

%% Modelling of contact area

% I need to translate this disc along the line i have created at some
% contact depth
% I need to locally fit a plane to the point cloud to use for modelling (IS this nessecary when just doing a plane?)

% Model is in cm in the graph

discArea = pi * R^2;
discRadiusStep = 1; % [cm] Step-size of the radius of the disc
discCircleStep = 36; % [degree] Step-size of the angle of the circle
discRadiusRes = R / discRadiusStep; % [cm] Resolution of the disc radius
discCircleRes = 360 / discCircleStep; % [degree] Resoultion of the disc circle
numberOfDiscPoints = discCircleRes * discRadiusRes;
unitArea = discArea/numberOfDiscPoints; % Area per discretized circle point

% Generate circle points
circlePoints = [zeros(discRadiusRes*discCircleRes+1,2) zeros(discRadiusRes*discCircleRes+1,1)];
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
    % The first point is the translation from the circle center point to 
    % the start of the trajectory
    if i > 1
        pointDiff = datapoints(i,:)-datapoints(i-1,:);
    else
        pointDiff = datapoints(i,:);
        if pointDiff(3) >= 0 % Subtract disc depth from the trajectory
            pointDiff(3) = pointDiff(3) - depth;
        else % Add disc depth
            pointDiff(3) = pointDiff(3) + depth;
        end
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

clc
close all
xRand

% 1. Fit plane to points of circle
% 2. Calculate normal/orthogonal vector of plane
% 3. Find all points in point cloud which are a maximum distance of
%    disc radius from the normal vector to the plane. This is the amount of point cloud
%    points in the disc at this point. Divide the area with the number of
%    points to get the area per point in this disc configuration
% 4. From this, determine the amount of points above or in the plane. 
%    Multiply this amount of points with the area per points found in step
%    3 to get the contact area estimation
% 5. Continue for the remainding points on the trajectory

% Step 0: Translate the circle points to the first point on the trajectory.
% Code is copy paste from last section.
% Initialize copy of circle points to translate.
% This is done to avoid having to run multiple sections while testing.
circleCopy = circlePoints;
pointDiff = datapoints(1,:);
% Loop trough all circle points and add the current point difference
for j = 1:size(circlePoints,1)
    circleCopy(j,:) = circleCopy(j,:) + pointDiff;
end

% Step 1.
% Define coloums of circle points. Get the first circle translation by
% selecting the first 'numberOfDiscPoints' points.
discXVals = circleCopy(:,1);
discYVals = circleCopy(:,2);
discZVals = circleCopy(:,3);
% Define the matrix for which to solve for the plane parameters
SLE = [discXVals discYVals ones(size(discZVals))]; % System of Linear Equations
% Operator '\' means solve the system of linear equations for A\B (Ax=B)
coeffs = SLE\discZVals;
% Find the min and max of the disc points
discXRange = [min(discXVals) max(discXVals)];
discYRange = [min(discYVals) max(discYVals)];
% Now, solve for the z-values of the highest and lowest X and Y points
% using the equation for a plane (Z = Ax + By + C)
discZRange = [discXRange', discYRange', ones(2,1)] * coeffs;
% Finding the center point of the plane
centerX = mean(discXRange);
centerY = mean(discYRange);
centerZ = mean(discZRange);
% Using the cross-product of two vectors in the plane to find 
% the perpendicular vector. They are both pointing 
% to the middle of the x and y edges of the restricted plane.
centerXZVal = coeffs(1) * max(discXRange) + coeffs(2) * mean(discYRange) + coeffs(3);
centerYZVal = coeffs(1) * mean(discXRange) + coeffs(2) * max(discYRange) + coeffs(3);
zHeightDiff = abs(centerXZVal - centerYZVal);
xCenterVec = [max(discXRange) mean(discYRange) 0];
yCenterVec = [mean(discXRange) max(discYRange) zHeightDiff];
perpVec = cross(xCenterVec,yCenterVec);
normVec = perpVec/norm(perpVec);

% Select all points from the point cloud which satisfy the radius constraint
contactPoints = [];
for i = 1:size(cloud.Location,1)
    curPoint = cloud.Location(i,:); % Get the ith point cloud point
    % Compute the perpendicular distance to the plane vector
    vecToLine = curPoint-perpVec;
    dist = norm(cross(vecToLine,perpVec))/norm(perpVec);
    if abs(dist) <= R % Point is on circle perimiter or within 
        contactPoints(end+1,:) = curPoint;
    end
end

% First plot, point cloud and line. Add a little to z of line to better
% visualize
figure(1)
plot3(cloud.Location(:,1),cloud.Location(:,2),cloud.Location(:,3),'or')
xlim([-20 120]);
ylim([-20 120]);
zlim([-0.5 1.5]);
xlabel('X')
ylabel('Y')
zlabel('Z')
view(-45,2); % Change the view of the 3D figure. This is around the x-axis
hold on
plot3(datapoints(:,1),datapoints(:,2),datapoints(:,3),'k')
hold off

% Second plot, circle points, plane
figure(2)
hold on
xlim([xRand-10 xRand+10]);
ylim([datapoints(1,2)-10 datapoints(1,2)+10]);
zlim([H-2*depth H-depth+depth]);
xlabel('X')
ylabel('Y')
zlabel('Z')
view(-45,45); % Change the view of the 3D figure. This is around the x-axis
plot3(circleCopy(:,1),circleCopy(:,2),circleCopy(:,3),'ob')
patch([min(discXRange) min(discXRange) max(discXRange) max(discXRange)], ...
      [min(discYRange) max(discYRange) max(discYRange) min(discYRange)], ...
      [min(discZRange) min(discZRange) max(discZRange) max(discZRange)], ...
      'r', 'FaceAlpha', 0.5)
hold off

% Third plot, plane, vectors and their points
% Needs to be fixed
figure(3)
hold on
xlim([xRand-10 xRand+10]);
ylim([datapoints(1,2)-10 datapoints(1,2)+10]);
zlim([H-2*depth H-depth+depth]);
view(-45,45); % Change the view of the 3D figure. This is around the x-axis
plot3(centerX, centerY, centerZ, 'oy')
plot3(xCenterVec(1),xCenterVec(2),centerXZVal,'oy')
plot3(yCenterVec(1),yCenterVec(2),centerYZVal,'oy')
quiver3(centerX,centerY,centerZ,normVec(1),normVec(2),normVec(3),'g')
quiver3(centerX,centerY,centerZ,R,0,0,'g')
quiver3(centerX,centerY,centerZ,0,R,0,'g')
patch([min(discXRange) min(discXRange) max(discXRange) max(discXRange)], ...
      [min(discYRange) max(discYRange) max(discYRange) min(discYRange)], ...
      [min(discZRange) min(discZRange) max(discZRange) max(discZRange)], ...
      'r', 'FaceAlpha', 0.5)
hold off

% Fourth plot plane with found points
figure(4)
hold on
xlim([-20 120]);
ylim([-20 120]);
zlim([-0.5 1.5]);
view(-45,45); % Change the view of the 3D figure. This is around the x-axis
plot3(contactPoints(:,1),contactPoints(:,2),contactPoints(:,3),'or')
patch([min(discXRange) min(discXRange) max(discXRange) max(discXRange)], ...
      [min(discYRange) max(discYRange) max(discYRange) min(discYRange)], ...
      [min(discZRange) min(discZRange) max(discZRange) max(discZRange)], ...
      'r', 'FaceAlpha', 0.5)
hold off

