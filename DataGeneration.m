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
%    disc radius from the normal vector to the plane. 
% 4. Filter out points which are on the other side of the object. This is
%    done via finding the mean point and filtering out those points that
%    are further away than the absolute distance from the mean point to the
%    plane
% 5. Calculate the unit area of the points, which is how much they each
%    contribute to the area of the polishing disc.
% 6. Find out how many points are in contact with the disc. (how?)
% 7. Multiply the unit area with the amount of points in contact to get the
%    estimated area

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
SLE = [discXVals discYVals ones(size(discZVals),1)]; % System of Linear Equations
% Operator '\' means solve the system of linear equations for A\B (Ax=B)
coeffs = SLE\discZVals;
% Find the min and max of the disc points
discXRange = [min(discXVals) max(discXVals)];
discYRange = [min(discYVals) max(discYVals)];
% Now, solve for the z-values of the highest and lowest X and Y points
% using the equation for a plane (Z = Ax + By + C)
discZRange = [discXRange', discYRange', ones(2,1)] * coeffs;

% Step 2.
% Finding the center point of the plane
centerX = mean(discXRange);
centerY = mean(discYRange);
centerZ = mean(discZRange);
% Using the cross-product of two vectors in the plane to find 
% the perpendicular vector. They are both pointing 
% to the middle of the x and y edges of the restricted plane.
centerXZVal = coeffs(1) * max(discXRange) + coeffs(2) * mean(discYRange) + coeffs(3);
centerYZVal = coeffs(1) * mean(discXRange) + coeffs(2) * max(discYRange) + coeffs(3);
% Vector from the center of the plane to the median points must be the
% vector from origo to the median points, minus the vector from origo to
% the center of the plane.
origoPlaneCenterVec = [centerX, centerY, centerZ];
origoPlaneXVec = [centerX,max(discYRange),centerXZVal];
origoPlaneYVec = [max(discXRange),centerY,centerYZVal];
planeXVec = origoPlaneXVec - origoPlaneCenterVec;
planeYVec = origoPlaneYVec - origoPlaneCenterVec;
perpVec = cross(planeYVec,planeXVec);
% Normalizing the vector
normVec = perpVec/norm(perpVec);

% Step 3.
% Select all points from the point cloud which satisfy the radius constraint
inRadiusPoints = [];
for i = 1:size(cloud.Location,1)
    curPoint = cloud.Location(i,:); % Get the ith point cloud point
    % Compute the vector from the current point to the center of the plane
    vecToLine = curPoint-origoPlaneCenterVec;
    % Compute the perpendicular distance from the current point to the
    % plane normal vector
    dist = norm(cross(vecToLine,perpVec))/norm(perpVec);
    if abs(dist) <= R % Point is on circle perimiter or within 
        inRadiusPoints(end+1,:) = curPoint;
    end
end

% Step 4
% Calculate mean point from found points within the radius constraint
meanInRadiusPoint = [mean(inRadiusPoints(:,1)),mean(inRadiusPoints(:,2)),mean(inRadiusPoints(:,3))];
% Define the vector from the mean point to the center of the plane
meanInRadiusCenterVec = origoPlaneCenterVec - meanInRadiusPoint;
% Find absolute value of the distance between the point and the plane,
% which is used as the threshold to filter out the contact points
% See https://mathinsight.org/distance_point_plane
meanPointDist = abs(dot(meanInRadiusCenterVec,normVec));
inContactPoints = [];
% Iterate through all points to filter them
for i = 1:size(inRadiusPoints,1)
    curPoint = inRadiusPoints(i,:); % Get the ith point
    % Define the vector from the current point to the center of the plane
    curPointVec = origoPlaneCenterVec - curPoint;
    % Find absolute value of the distance between the point and the plane
    curPointDist = abs(dot(curPointVec,normVec));
    if curPointDist <= meanPointDist
        inContactPoints(end+1,:) = curPoint;
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
plot3(origoPlaneXVec(1),origoPlaneXVec(2),origoPlaneXVec(3),'oy')
plot3(origoPlaneYVec(1),origoPlaneYVec(2),origoPlaneYVec(3),'oy')
quiver3(centerX,centerY,centerZ,normVec(1),normVec(2),normVec(3),'g')
quiver3(centerX,centerY,centerZ,planeXVec(1),planeXVec(2),planeXVec(3),'g')
quiver3(centerX,centerY,centerZ,planeYVec(1),planeYVec(2),planeYVec(3),'g')
patch([min(discXRange) min(discXRange) max(discXRange) max(discXRange)], ...
      [min(discYRange) max(discYRange) max(discYRange) min(discYRange)], ...
      [min(discZRange) min(discZRange) max(discZRange) max(discZRange)], ...
      'r', 'FaceAlpha', 0.5)
hold off

% Fourth plot plane with found points that satisfy the radius constraint
figure(4)
hold on
xlim([xRand-10 xRand+10]);
ylim([datapoints(1,2)-10 datapoints(1,2)+10]);
zlim([-0.5 1.5]);
view(-45,35); % Change the view of the 3D figure. This is around the x-axis
plot3(inRadiusPoints(:,1),inRadiusPoints(:,2),inRadiusPoints(:,3),'or')
patch([min(discXRange) min(discXRange) max(discXRange) max(discXRange)], ...
      [min(discYRange) max(discYRange) max(discYRange) min(discYRange)], ...
      [min(discZRange) min(discZRange) max(discZRange) max(discZRange)], ...
      'r', 'FaceAlpha', 0.5)
hold off

% Fifth plot plane with filtered radius constraint points
figure(5)
hold on
xlim([xRand-10 xRand+10]);
ylim([datapoints(1,2)-10 datapoints(1,2)+10]);
zlim([-0.5 1.5]);
view(-45,35); % Change the view of the 3D figure. This is around the x-axis
plot3(inContactPoints(:,1),inContactPoints(:,2),inContactPoints(:,3),'or')
plot3(meanInRadiusPoint(1),meanInRadiusPoint(2),meanInRadiusPoint(3),'ob')
patch([min(discXRange) min(discXRange) max(discXRange) max(discXRange)], ...
      [min(discYRange) max(discYRange) max(discYRange) min(discYRange)], ...
      [min(discZRange) min(discZRange) max(discZRange) max(discZRange)], ...
      'r', 'FaceAlpha', 0.5)
% Plot the normal vector from the mean point to the plane
quiver3(meanInRadiusPoint(1),meanInRadiusPoint(2),meanInRadiusPoint(3),meaninRadiusNormPlaneVec(1),meaninRadiusNormPlaneVec(2),meaninRadiusNormPlaneVec(3),'g')
% Plot the normal vector from the last contact point to the plane
quiver3(inContactPoints(end,1),inContactPoints(end,1),inContactPoints(end,1),curPointNormPlaneVec(1),curPointNormPlaneVec(2),curPointNormPlaneVec(3),'g')
hold off
%% NOTES
% The contact area estimation is given a trajectory from the (GMM/GMR or
% directly from the recording, probably the former) and estimates a contact
% force in each point from a specified contact pressure and the estimated
% contact area in each point. This is what the hybrid force-position
% controller uses to control the robot

% I need to specify a fitting constant pressure in order to get the forces.
% I can plot the estimated contact forces in order to see max/min values,
% such that the robot doesn't press too hard.

% I have to figure out how to correctly correclate the points of the
% trajectory, which are given in the robots base frame, to me to the model 
% here in MATLAB. One way is to define the the transformation between the 
% base frame of the robot, and the frame of the object,
% which can be measured as an X-Y displacement 
