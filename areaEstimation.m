clear
close all
clc

%% (TEST) LOADING AND PREPROCSEEING POINT CLOUD   
% Loaded file is a modified form of the original CAD-file, such that a smooth
% surface is obtained.
% Load .ply point cloud. Points are in mm.
orgCloud = pcread('FinalAssemblyPointCloudSimplified.ply');
% Do preprocessing to the point cloud points, so that points only have
% positive XYZ-values, with the height of the object being specified in the
% z-direction
points = orgCloud.Location;
% Swap X and Z values so that the bottom of the object lies on the XY-plane
% instead of the YZ-plane.
px = points(:,1);
pz = points(:,3);
points(:,1) = pz;
points(:,3) = px;
% Flip X- and Y-values so that they are positive
points = points * [-1 0 0;
                   0 -1 0;
                   0 0 1];
% Rotate pi around Y-axis to get flat side of the object up
YRotAng = -pi; % [rad]
YRotMat = [cos(YRotAng)     0       sin(YRotAng);
           0                1       0;
           -sin(YRotAng)    0       cos(YRotAng)];
points = points * YRotMat;
% Add the absolute value of the minimum of all points in the XYZ-axis to 
% their  respective axis to get all positive values
orgXMin = min(points(:,1));
orgYMin = min(points(:,2));
orgZMin = min(points(:,3));
points = [points(:,1)+abs(orgXMin) points(:,2)+abs(orgYMin) points(:,3)+abs(orgZMin)];
% Remove the "bottom" of the object, i.e. the surface of the object in
% contact with the table. This is done to make contact area estimation
% easier
pointPreprocessingZThreshold = 1; % [mm]
points = points(points(:,3) > pointPreprocessingZThreshold,:);

% Create and display point cloud
cloud = pointCloud(points);
pcshow(cloud);

%% (TEST) DEFINE AND DISCRETIZE POLISHING TOOL 

% Constants - Disc parameters
R = 25; % [mm] Radius of the tool
A = pi * R^2; % [mm^2] Area of the tool

polishingDepth = 1; % [mm] Depth at which the tool is below the surface of the object
toolRadiusStep = R/5; % [mm] Step-size of the radius of the tool when discretizing it
toolCircleStep = 36; % [degree] Step-size of the angle of the tool when discretizing it
toolRadiusRes = R / toolRadiusStep; % [cm] Resolution of the discretizied radius
toolCircleRes = 360 / toolCircleStep; % [degree] Resoultion of the discretizized circle

% Generate circle points
circlePoints = [zeros(toolRadiusRes*toolCircleRes,2) zeros(toolRadiusRes*toolCircleRes,1)];
radiusIndex = 0;
for j = toolRadiusStep:toolRadiusStep:R % Iterate through the points on the radius of the circle
    circleIndex = 1;
    for k = 0:toolCircleStep:360 % Iterate through all points in the sub-circle
        xval = cos(deg2rad(k))*j;
        circlePoints(radiusIndex*toolCircleRes+circleIndex,1) = xval;
        yval = sin(deg2rad(k))*j;
        circlePoints(radiusIndex*toolCircleRes+circleIndex,2) = yval;
        circleIndex = circleIndex + 1;
    end
    radiusIndex = radiusIndex + 1;
end

% Plot the circle
plot3(circlePoints(:,1),circlePoints(:,2),circlePoints(:,3),'o');

%% (TEST) GENERATE A TRAJECTORY ACROSS THE SURFACE OF THE OBJECT

% IDEA
% Could also just do something like picking a point where x=0, then
% increase x-value of point by step and pick the closest point as the next
% point. This will yield a non-linear trajectory, but they are also not
% linear when recorded using LfD.

% Choose random X-value for start and end point of random trajectory.
% They are limited such that the start and end-points are always within the
% object.
startPoint = [(150 + 150 * rand()) 0];
endPoint = [(150 + 150 * rand()) max(cloud.Location(:,2))];
% Calculate the vector from start to finish
randTrajVec = endPoint - startPoint;
% Discretize distance into points
numberOfPoints = 20;
testTrajectory = [zeros(numberOfPoints,3)] + [startPoint 0];
xDiff = endPoint(1) - startPoint(1);
xStep = xDiff/(numberOfPoints-1);
yDiff = endPoint(2) - startPoint(2);
yStep = yDiff/(numberOfPoints-1);
% Iterate through all points, for each get all points in radius, and set z
% to the avg of the 10% of points with the highest z
trajR = 10; % [mm]
zValPercent = 0.1; % Percent of points that will be used to calculate z-value 
for i = 1:numberOfPoints
    pointIncrement = [(i-1)*xStep (i-1)*yStep 0];
    curPoint = testTrajectory(i,:) + pointIncrement;
    cond1 = cloud.Location(:,1) <= (curPoint(1)+trajR);
    cond2 = cloud.Location(:,1) >= (curPoint(1)-trajR);
    cond3 = cloud.Location(:,2) <= (curPoint(2)+trajR);
    cond4 = cloud.Location(:,2) >= (curPoint(2)-trajR);
    zValsInRange = cloud.Location(cond1 & cond2 & cond3 & cond4,3);
    avgZ = mean(maxk(zValsInRange,ceil(size(zValsInRange,1)*zValPercent)));
    curPoint(3) = avgZ;
    testTrajectory(i,:) = curPoint;
end

% Plot the trajectory and the point cloud
figure(1)
hold on
view(-60,65); % Change the view of the 3D figure. This is around the x-axis
plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
plot3(cloud.Location(:,1),cloud.Location(:,2),cloud.Location(:,3),'o')
hold off

%% (TEST) MOVING THE TOOL ALONG THE TRAJECTORY

close all

% There are still scaling and slight rotation issues, but the general idea
% is there

% I think this is the way.
% Fit a plane to the ten closest points to the current trajectory-point.
% (Instead, just use radius constrint because i think it is easier)
% Then, define two vectors from the center of the plane, and use the
% cross-product to get the third axis. Translate those to the coordinate
% frame in origo (the center of the plane is known) and consrtuct the
% rotation matrix from the slides. Then multiply that on the tool points
% and finally translate them as well. Repeat for the remaning trajectory
% points.

% Initialize storage matrix for tool points
toolLocations = zeros(size(circlePoints,1),size(circlePoints,2),size(testTrajectory,1));

% Define unit vectors
unitVecX = [1 0 0];
unitVecY = [0 1 0];
unitVecZ = [0 0 1];

% Define XYZ rotation functions
RX = @(ang) [1  0          0;
             0  cos(ang)   -sin(ang);
             0  sin(ang)   cos(ang)];
RY = @(ang) [cos(ang)  0   sin(ang);
             0         1   0;
             -sin(ang) 0   cos(ang)];
RZ = @(ang) [cos(ang)   -sin(ang) 0;
             sin(ang)   cos(ang)  0;
             0          0         1];

% Define function handle for angel between two vectors in degrees
vecAng = @(a,b) acosd(dot(a,b)/(norm(a)*norm(b)));

% Iterate through the points in the trajectory
for i = 1:size(testTrajectory,1)
    % Grab the current trajectory point
    curPoint = testTrajectory(i,:);
    % Find all the points in a radius from the current point
    trajR = 10; % [mm]
    condXU = cloud.Location(:,1) <= (curPoint(1)+trajR);
    condXL = cloud.Location(:,1) >= (curPoint(1)-trajR);
    condYU = cloud.Location(:,2) <= (curPoint(2)+trajR);
    condYL = cloud.Location(:,2) >= (curPoint(2)-trajR);
    condZU = cloud.Location(:,3) <= (curPoint(3)+trajR);
    condZL = cloud.Location(:,3) >= (curPoint(3)-trajR);
    pointsInRange = cloud.Location(condXU & condXL & condYU & condYL & condZU & condZL,:);
    % Fit a plane to those points
    planeXPoints = pointsInRange(:,1);
    planeYPoints = pointsInRange(:,2);
    planeZPoints = pointsInRange(:,3);
    planeCoeffs = [planeXPoints planeYPoints ones(size(pointsInRange,1),1)] \ planeZPoints; % Linear regression using linear least squares
    % Get the ranges of the plane points
    planeXRange = [min(planeXPoints) max(planeXPoints)];
    planeYRange = [min(planeYPoints) max(planeYPoints)];
    % Now, solve for the z-values of the highest and lowest X and Y points
    % using the equation for a plane (Z = Ax + By + D), remember C is set
    % to 1, since no exact solution exists.
    planeZRange = [planeXRange', planeYRange', ones(2,1)] * planeCoeffs;
    % Finding the center point of the plane
    planeCenterX = mean(planeXRange);
    planeCenterY = mean(planeYRange);
    planeCenterZ = mean(planeZRange);
    % Using the cross-product of two vectors in the plane to find 
    % the perpendicular vector. They are both pointing 
    % to the middle of the x and y edges of the restricted plane.
    planeCenterXZVal = planeCoeffs(1) * max(planeXRange) + planeCoeffs(2) * mean(planeYRange) + planeCoeffs(3);
    planeCenterYZVal = planeCoeffs(1) * mean(planeXRange) + planeCoeffs(2) * max(planeYRange) + planeCoeffs(3);
    % Vector from the center of the plane to the median points must be the
    % vector from origo to the median points, minus the vector from origo to
    % the center of the plane.
    origoPlaneCenterVec = [planeCenterX, planeCenterY, planeCenterZ];
    origoPlaneXVec = [planeCenterX,max(planeYRange),planeCenterXZVal];
    origoPlaneYVec = [max(planeXRange),planeCenterY,planeCenterYZVal];
    planeXVec = origoPlaneXVec - origoPlaneCenterVec;
    planeYVec = origoPlaneYVec - origoPlaneCenterVec;
    perpVec = cross(planeYVec,planeXVec);
    % Normalizing the vector
    normVec = perpVec/norm(perpVec);
    % planeXVec, planeYVec and normVec can now be used to construct
    % rotation matrix
    rotMatrix = [dot(planeXVec,unitVecX) dot(planeXVec,unitVecY) dot(planeXVec,unitVecZ);
                 dot(planeYVec,unitVecX) dot(planeYVec,unitVecY) dot(planeYVec,unitVecZ);
                 dot(perpVec,unitVecX) dot(perpVec,unitVecY) dot(perpVec,unitVecZ)];
    % Now, get a copy of the cricle points, which will be translated and rotated
    % to fit the regression plane.
    curToolLocation = circlePoints;
    % Rotate the points
    curToolLocation = (rotMatrix * curToolLocation')';
    % Translate tool by adding trajectory-point. Remember, tool points are
    % defined in origo
    curToolLocation = curToolLocation + curPoint;
    % Save the tool points to a 3D matrix for potential later use
    toolLocations(:,:,i) = curToolLocation;    
end

% Plot the trajectory and the first tool points
figure(1)
hold on
%view(-60,65); % Change the view of the 3D figure. This is around the x-axis
% quiver3(0,0,0,curVec(1),curVec(2),curVec(3),'b')
% quiver3(0,0,0,1,0,0,'g')
% quiver3(0,0,0,0,1,0,'g')
% quiver3(0,0,0,0,0,1,'g')
plot3(circlePoints(:,1),circlePoints(:,2),circlePoints(:,3),'o')
plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
%plot3(toolLocations(:,1,19),toolLocations(:,2,19),toolLocations(:,3,19),'o') % Temporary
for i = 1:5:size(toolLocations,3)
    plot3(toolLocations(:,1,i),toolLocations(:,2,i),toolLocations(:,3,i),'o')
end
hold off

% Do this when plane has been fitted so it can be seen in the video
% Save figure
% figure(1)
% hold on
% if mod(i,2) == 0
%     plot3(circleCopy(:,1),circleCopy(:,2),circleCopy(:,3),'or')
% else
%     plot3(circleCopy(:,1),circleCopy(:,2),circleCopy(:,3),'ob')
% end
% plot3(datapoints(:,1),datapoints(:,2),datapoints(:,3),'k')
% xlim([-20 120]);
% ylim([-20 120]);
% zlim([0.5 1.5]);
% view(45,45); % Change the view of the 3D figure. This is around the x-axis
% hold off
% F(i) = getframe(gcf);
% Make an illustration video
% videoWriterObj = VideoWriter('testTrajectory.avi');
% videoWriterObj.FrameRate = 10;
% open(videoWriterObj);
% for i=1:length(F)
%  frame = F(i);
%  writeVideo(videoWriterObj, frame);
% end
% close(videoWriterObj);


%% (TEST) ESTIMATING CONTACT AREA IN ONE POINT ON TRAJECTORY

%% (TEST) LOADING TRAJECTORY RECORDED ON UR10E

%% CONTACT AREA ESTIMATION ON TRAJECTORY

clear
close all
clc


