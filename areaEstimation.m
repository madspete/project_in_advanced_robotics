clear
close all
clc

% REQUIREMENTS:
% Computer Vision Toolbox

%% (TEST) LOADING AND PREPROCSEEING POINT CLOUD   
% Loaded file is a modified form of the original CAD-file, such that a smooth
% surface is obtained.
% Load .ply point cloud. Distamces are in mm.
orgCloud = pcread('FinalAssemblyPointCloudSimplified.ply');
% Do preprocessing to the point cloud, so that points only have
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
% easier. Threshold is chosen experimentaly
pointPreprocessingZThreshold = 1; % [mm]
points = points(points(:,3) > pointPreprocessingZThreshold,:);

% Create and display point cloud
cloud = pointCloud(points);
%pcshow(cloud);

%% (TEST) DEFINE AND DISCRETIZE POLISHING TOOL 

% Constants - tool parameters
R = 25; % [mm] Radius of the tool
A = pi * R^2; % [mm^2] Area of the tool

toolRadiusStep = R/5; % [mm] Step-size of the radius of the tool when discretizing it
toolCircleStep = 36; % [degree] Step-size of the angle of the tool when discretizing it
toolRadiusRes = R / toolRadiusStep; % [cm] Resolution of the discretizied radius
toolCircleRes = 360 / toolCircleStep; % [degree] Resoultion of the discretizized circle

% Generate circle points
% Preallocate matrix
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
%plot3(circlePoints(:,1),circlePoints(:,2),circlePoints(:,3),'o');

%% (TEST) GENERATE A TRAJECTORY ACROSS THE SURFACE OF THE OBJECT

% IDEA
% Could also just do something like picking a point where x=0, then
% increase x-value of point by step and pick the closest point as the next
% point. This will yield a non-linear trajectory, but they are also not
% linear when recorded using LfD.

% Choose random X-value for start and end point of trajectory.
% They are limited such that the start and end-points are always within the
% object.
startPoint = [(150 + 150 * rand()) 0];
endPoint = [(150 + 150 * rand()) max(cloud.Location(:,2))];
% Calculate the vector from start to finish
randTrajVec = endPoint - startPoint;
% Discretize distance into points
numberOfPoints = 20;
% Initialize matrix containing trajectory points 
testTrajectory = [zeros(numberOfPoints,3)] + [startPoint 0];
% Calculate step-size in X and Y direction
xDiff = endPoint(1) - startPoint(1);
xStep = xDiff/(numberOfPoints-1);
yDiff = endPoint(2) - startPoint(2);
yStep = yDiff/(numberOfPoints-1);
% Iterate through all points and add the difference to each point. 
% Then select all points inside radius constraint, and set the z value of 
% the trajectory point to the avg of the 10% of points with the highest z
trajR = 10; % [mm]
zValPercent = 0.1; % Percent of points that will be used to calculate z-value 
for i = 1:numberOfPoints
    pointIncrement = [(i-1)*xStep (i-1)*yStep 0];
    curPoint = testTrajectory(i,:) + pointIncrement;
    % Creating selection conditions based on radius
    cond1 = cloud.Location(:,1) <= (curPoint(1) + trajR);
    cond2 = cloud.Location(:,1) >= (curPoint(1) - trajR);
    cond3 = cloud.Location(:,2) <= (curPoint(2) + trajR);
    cond4 = cloud.Location(:,2) >= (curPoint(2) - trajR);
    % Select points and calculate mean z of 10% of the largest
    zValsInRange = cloud.Location(cond1 & cond2 & cond3 & cond4,3);
    avgZ = mean(maxk(zValsInRange,ceil(size(zValsInRange,1)*zValPercent)));
    % Save point in trajectory
    curPoint(3) = avgZ;
    testTrajectory(i,:) = curPoint;
end

% Plot the trajectory and the point cloud
% figure(1)
% hold on
% view(-60,65); % Change the view of the 3D figure. This is around the x-axis
% plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
% plot3(cloud.Location(:,1),cloud.Location(:,2),cloud.Location(:,3),'o')
% hold off

%% (TEST) MOVING THE TOOL ALONG THE TRAJECTORY

% I think this is the way.
% Fit a plane to the ten closest points to the current trajectory-point.
% (Instead, just use radius constrint because i think it is easier)
% Then, define two vectors from the center of the plane, and use the
% cross-product to get the third axis. Translate those to the coordinate
% frame in origo (the center of the plane is known) and consrtuct the
% rotation matrix from the slides. Then multiply that on the tool points
% and finally translate them as well. Repeat for the remaning trajectory
% points.

% Depth at which the tool is below the surface of the object
polishingDepth = 1; % [mm]

% Initialize 3D storage matrix for tool points. This stores all tool points
% at each point on the trajectory
toolLocations = zeros(size(circlePoints,1),size(circlePoints,2),size(testTrajectory,1));

% Define the number of points the fitted plane is discretized with along
% each axis
planeDiscretizationStep = 50;

% Create 3D matrix object for storing discretized plane points
discretizedPlanePoints = zeros(planeDiscretizationStep^2, 3, size(testTrajectory,1));

% Iterate through the points in the trajectory
for i = 1:size(testTrajectory,1)
    % Grab the current trajectory point
    curPoint = testTrajectory(i,:);
    % Find all the points inside specified radius from the current point
    trajR = 10; % [mm]
    condXU = cloud.Location(:,1) <= (curPoint(1)+trajR);
    condXL = cloud.Location(:,1) >= (curPoint(1)-trajR);
    condYU = cloud.Location(:,2) <= (curPoint(2)+trajR);
    condYL = cloud.Location(:,2) >= (curPoint(2)-trajR);
    pointsInRange = cloud.Location(condXU & condXL & condYU & condYL, :);
    % Fit a plane to those points.
    % I want to solve Ax = B, where A is the pointInRange*3 matrix with the
    % first two columns being the x and y points found and the third being
    % ones, x is the pointsInRange*1 vector of coefficients of the plane
    % and B is a pointsInRange*1 vector constining all the z-values of the
    % pointsInRange points.
    % See the second anwser here
    % https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    % Operator '\' means solve the system of linear equations for A\B (Ax=B)
    planeXPoints = pointsInRange(:,1);
    planeYPoints = pointsInRange(:,2);
    planeZPoints = pointsInRange(:,3);
    AM = [planeXPoints planeYPoints ones(size(planeZPoints,1),1)];
    BM = [planeZPoints];
    planeCoeffs = AM\BM;
    % Get the ranges of the points used to fit the plane
    planeXRange = [min(planeXPoints) max(planeXPoints)];
    planeYRange = [min(planeYPoints) max(planeYPoints)];
    % Now, solve for the z-values of the highest and lowest X and Y points
    % using the equation for a plane (Z = Ax + By + D), remember C is set
    % to 1, since no exact solution exists.
    planeZRange = [planeXRange' planeYRange' ones(2,1)] * planeCoeffs;

    % Defining 50*50 equally spaced points on the plane for plotting.
    [plotX,plotY] = meshgrid(linspace(planeXRange(1),planeXRange(2),planeDiscretizationStep), linspace(planeYRange(1),planeYRange(2),planeDiscretizationStep));
    plotZ = planeCoeffs(1)*plotX + planeCoeffs(2)*plotY + planeCoeffs(3)*ones(size(plotX));
    discretizedPlanePoints(:,:,i) = [plotX(:) plotY(:) plotZ(:)];

    % Finding the center point of the plane and defining the vector from
    % origo to the center point of the plane
    planeMeanX = mean(planeXRange);
    planeMeanY = mean(planeYRange);
    planeMeanZ = mean(planeZRange);
    origoPlaneCenterVec = [planeMeanX, planeMeanY, planeMeanZ];
    % Using the cross-product of two vectors in the plane to find 
    % the plane normal. They are both pointing 
    % to the middle of the x and y edges of the fitted plane.
    % First, solving for the z-value in these points
    planeCenterXZVal = planeCoeffs(1) * max(planeXRange) + planeCoeffs(2) * planeMeanY + planeCoeffs(3);
    planeCenterYZVal = planeCoeffs(1) * planeMeanX + planeCoeffs(2) * max(planeYRange) + planeCoeffs(3);
    % Defining the vectors
    origoPlaneXVec = [max(planeXRange),planeMeanY,planeCenterXZVal];
    origoPlaneYVec = [planeMeanX,max(planeYRange),planeCenterYZVal];
    % Calculating the plane vectors relative to the center point of the plane
    planeXVec = origoPlaneXVec - origoPlaneCenterVec;
    planeYVec = origoPlaneYVec - origoPlaneCenterVec;
    % Normalizing the two plane vectors, calculating the plane normal
    % vector using the cross-product and normalizing that
    normPlaneXVec = planeXVec/norm(planeXVec);
    normPlaneYVec = planeYVec/norm(planeYVec);
    perpVec = cross(planeXVec,planeYVec);
    normVec = perpVec/norm(perpVec);
    % These three vector can now be used to construct the rotation matrix
    % between origo and the fitted plane
    % Theory is from prerequisites slides in 1. semester. 
    % Important that dot product components must be equal length! (definition)
    % Rotation and translation could have been combined, in which vectors
    % would have to have a 1 element added to them since the vector is 4*4
    rotMatrix = [normPlaneXVec', normPlaneYVec', normVec'];
    % Now, get a copy of the circle points, which will be translated and 
    % rotated to fit the fitted plane.
    curToolLocation = [circlePoints(:,1) circlePoints(:,2) circlePoints(:,3)-polishingDepth];
    % Rotate the points
    curToolLocation = (rotMatrix * curToolLocation')';
    % Translate tool by adding trajectory-point. Remember, tool points are
    % defined in origo
    curToolLocation = curToolLocation + curPoint;
    % Save the tool points to a 3D matrix for potential later use
    toolLocations(:,:,i) = curToolLocation;    
end

% Create and save figures for illustration video
% for i = 1:size(toolLocations,3)
%     figure('visible','off') % Dont display the figure when creating them
%     hold on
%     xlabel('X [mm]')
%     ylabel('Y [mm]')
%     zlabel('Z [mm]')
%     % Change the view of the 3D figure. This is around the x-axis
%     view(-80,20);
%     % Plot test trajectory
%     plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
%     % Plot plane
%     mesh(reshape(discretizedPlanePoints(:,1,i),[planeDiscretizationStep planeDiscretizationStep]), ...
%          reshape(discretizedPlanePoints(:,2,i),[planeDiscretizationStep planeDiscretizationStep]), ...
%          reshape(discretizedPlanePoints(:,3,i),[planeDiscretizationStep planeDiscretizationStep]))
%     % Plot tool points
%     plot3(toolLocations(:,1,i),toolLocations(:,2,i),toolLocations(:,3,i),'o')
%     hold off
%     figureFrames(i) = getframe(gcf);
%     clf(1) % Clear figure
% end
% 
% %Make an illustration video
% videoWriterObj = VideoWriter('finalTestTrajectory.avi');
% videoWriterObj.FrameRate = 2;
% open(videoWriterObj);
% for i=1:length(figureFrames)
%  curFrame = figureFrames(i);
%  writeVideo(videoWriterObj, curFrame);
% end
% close(videoWriterObj);

%% (TEST) LOADING TRAJECTORY RECORDED ON UR10E

% Loading should be relativly easy, depending on the format of the file.
% Use importdata maybe

% There exists some transformation between robot base frame and polishing
% object. This just has to be applied to the trajectory supplid in order to
% transform it to the object-space

% Display trajectory on object, like with the randomly generated one
%% (TEST) ESTIMATING CONTACT AREA IN ONE POINT ON TRAJECTORY

% Procedure:
% 1. Iterate through trajectory points
% 2. Fit plane to tool points in the current trajectory point
% 3. Calculate plane normal. This is the z-axis of the trajectory frame
% 4. Select all points from point cloud which satisfy the radius constraint 
%    of the tool, meaning the points that are closer or equal to the radius
%    distance from the plane normal
% 5. Calculate the unit area, which is the total area divided by the number
%    of points within the radius constraint
% 6. Calculate the amount of points in contact, by transforming them to the
%    frame of the current trajectory point an looking for pointZ >= 0
% 7. Compute the percentage of points in range that are in contact with the
%    tool, as well as the area that they cover. Multiply with a target
%    pressure to get a force stimate in that trajectory point
% 8. Plot contact percentage, contact area and force as a function of tool
%    trajectory point
% 9. Display figure with trajectory, plane, points in range and points in
%    contact

% Desired pressure that the tool has maintain in each point on the 
% trajectory. Following article suggests between 15 and 20 pounds of force
% per square foot.
% https://meguiarsonline.com/forum/information-station/how-to-articles/3079-how-to-use-the-g-100-to-remove-swirls?2965-How-to-use-the-G-100-to-remove-swirls=
% 15 pounds per square foot is used, which is around 718 Pa
% pascal
desiredToolPressure = 718; % [Pa]

% Matrix to store the points within the radius contraint
pointsInRange = [];

% Matrix to store the coordinates on the points in contact
pointsInContact = [];

% Define the number of points the fitted plane is discretized with along
% each axis
toolPlaneDiscretizationStep = 50;

% Create 3D matrix object for storing discretized plane points
discretizedToolPlanePoints = zeros(toolPlaneDiscretizationStep^2, 3, size(testTrajectory,1));

% Define matrix for storing percentage of points in contact (first column),
% estimated contact area (second column) and estimated force (third column)
% in each trajectory point
estimationMatrix = zeros(size(testTrajectory,1),3);

% Step 1:
% Iterate trough trajectory points

for i = 1:size(testTrajectory,1)

    % Step 2:
    % Fit plane to discretized tool points
    % Get the tool points from the i'th trajectory point
    toolXVals = toolLocations(:,1,i);
    toolYVals = toolLocations(:,2,i);
    toolZVals = toolLocations(:,3,i);
    % Define the system of linear equations to solve for the plane parameters
    SLE = [toolXVals toolYVals ones(size(toolZVals,1),1)]; % System of Linear Equations
    % Operator '\' means solve the system of linear equations for A\B (Ax=B)
    coeffs = SLE\toolZVals;
    
    % Step 3:
    % Calculate plane normal
    % Find the min and max of the tool points
    toolXRange = [min(toolXVals) max(toolXVals)];
    toolYRange = [min(toolYVals) max(toolYVals)];
    % Now, solve for the z-values of the highest and lowest X and Y points
    % using the equation for a plane (Z = Ax + By + C)
    toolZRange = [toolXRange', toolYRange', ones(2,1)] * coeffs;
    % Defining 50*50 equally spaced points on the plane for plotting.
    [plotToolPlaneX,plotToolPlaneY] = meshgrid(linspace(toolXRange(1),toolXRange(2),toolPlaneDiscretizationStep), ...
                                               linspace(toolYRange(1),toolYRange(2),toolPlaneDiscretizationStep));
    plotToolPlaneZ = coeffs(1)*plotToolPlaneX + coeffs(2)*plotToolPlaneY + coeffs(3)*ones(size(plotToolPlaneX));
    discretizedToolPlanePoints(:,:,i) = [plotToolPlaneX(:) plotToolPlaneY(:) plotToolPlaneZ(:)];
    % Find the center point of the plane
    centerX = mean(toolXRange);
    centerY = mean(toolYRange);
    centerZ = mean(toolZRange);
    % Using the cross-product of two vectors in the plane to find 
    % the plane normal. They are both pointing to the middle
    % of the x and y edges of the fitted plane.
    % First, solving for the z-value in these points
    centerXZVal = coeffs(1) * max(toolXRange) + coeffs(2) * centerY + coeffs(3);
    centerYZVal = coeffs(1) * centerX + coeffs(2) * max(toolYRange) + coeffs(3);
    % Vector from the center of the plane to the median points must be the
    % vector from origo to the median points, minus the vector from origo to
    % the center of the plane.
    origoToolPlaneCenterVec = [centerX, centerY, centerZ];
    origoToolPlaneXVec = [max(toolXRange),centerY,centerXZVal];
    origoToolPlaneYVec = [centerX,max(toolYRange),centerYZVal];
    toolPlaneXVec = origoToolPlaneXVec - origoToolPlaneCenterVec;
    toolPlaneYVec = origoToolPlaneYVec - origoToolPlaneCenterVec;
    normPlaneXVec = toolPlaneXVec / norm(toolPlaneXVec);
    normPlaneYVec = toolPlaneYVec / norm(toolPlaneYVec);
    toolPerpVec = cross(toolPlaneXVec,toolPlaneYVec);
    % Normalizing the normal vector to the plane
    toolNormVec = toolPerpVec/norm(toolPerpVec);
    
    % Step 4:
    % Select all points from the point cloud which satisfy the radius constraint
    inRadiusIndex = 1;
    for j = 1:size(cloud.Location,1)
        % Get the ith point cloud point
        curPoint = cloud.Location(j,:); 
        % Compute the vector from the current point to the center of the plane
        vecToLine = curPoint - origoToolPlaneCenterVec;
        % Compute the perpendicular distance from the current point to the
        % plane normal vector
        % See following link:
        % https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
        dist = norm(cross(vecToLine,perpVec))/norm(perpVec);
        if abs(dist) <= R % Point is on circle perimiter or within 
            pointsInRange(inRadiusIndex,:,i) = curPoint;
            inRadiusIndex = inRadiusIndex + 1;
        end
    end
    
    % Step 5:
    % Calculate the unit area each point, which is the amount each point
    % contributes to the area of the polishing disc.
    % Since pointsInRangeRange are stored in one big matrix, ignore zeros
    % to get the true number of points
    nonZeroCond = pointsInRange(:,1,i) > 0;
    numberOfInRangePoints = size(pointsInRange(nonZeroCond,:,1),1);
    unitArea = A / numberOfInRangePoints; % [mm^2]
    
    % Step 6:
    % Find out how many points are in contact with the disc.
    % Define rotation matrix (see previous section for desciption)
    rotMatrix = [normPlaneXVec; normPlaneYVec; toolNormVec;];
    numberOfContactPoints = 0;
    inContactIndex = 1;
    for j = 1:numberOfInRangePoints
        curInRangePoint = pointsInRange(j,:,i);
        % First, translate current point by subtracting the vector to the 
        % origo of the trajectory frame, which is the center of the plane
        curInRangePoint = curInRangePoint - origoToolPlaneCenterVec;
        % Then apply rotation of point to get it in the frame of the
        % current trajectory point.
        curInRangePoint = (rotMatrix * curInRangePoint')';
        if curInRangePoint(3) >= 0
            numberOfContactPoints = numberOfContactPoints + 1;
            pointsInContact(inContactIndex,:,i) = pointsInRange(j,:,i);
            inContactIndex = inContactIndex + 1;
        end
    end

    % Step 7:
    % Multiply the unit area with the amount of points in contact to get the
    % estimated area
    estimationMatrix(i,1) = numberOfContactPoints / numberOfInRangePoints * 100;
    estimationMatrix(i,2) = numberOfContactPoints * unitArea * 10^-6; %[m^2]
    estimationMatrix(i,3) = desiredToolPressure * estimationMatrix(i,2); % [N]
end

% Step 8:
% Plot percentage of constact area, size of contact area and estimated 
% force as a function of trajectory point
figure(1)
plot(1:size(testTrajectory,1), estimationMatrix(:,1))
xlabel('Test trajectory point')
ylabel('Percentage of points in contact with tool [%]')
figure(2)
plot(1:size(testTrajectory,1), estimationMatrix(:,2))
xlabel('Test trajectory point')
ylabel('Estimated contact area [m^2]')
figure(3)
plot(1:size(testTrajectory,1), estimationMatrix(:,3))
title(strcat('With desired contact pressure p = ', num2str(desiredToolPressure), ' [Pa]'))
xlabel('Test trajectory point')
ylabel('Estimated contact force [N]')

% Step 9:
% Display figure with trajectory, fitted tool plane, points within radius 
% constraint and points in contact
% Variable used to control which trajectory point contact area estiamation
% is displayed
trajectoryPointDisplay = 13;
figure(4)
hold on
plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
mesh(reshape(discretizedToolPlanePoints(:,1,trajectoryPointDisplay), ...
             [toolPlaneDiscretizationStep toolPlaneDiscretizationStep]), ...
     reshape(discretizedToolPlanePoints(:,2,trajectoryPointDisplay), ...
             [toolPlaneDiscretizationStep toolPlaneDiscretizationStep]), ...
     reshape(discretizedToolPlanePoints(:,3,trajectoryPointDisplay), ...
             [toolPlaneDiscretizationStep toolPlaneDiscretizationStep]))
% Since pointsInRange and pointsInContact are stored in one big matrix, 
% ignore zeros to get the actual points
nonZeroCondInRange = pointsInRange(:,1,trajectoryPointDisplay) > 0;
nonZeroCondInContact = pointsInContact(:,1,trajectoryPointDisplay) > 0;
plot3(pointsInRange(nonZeroCondInRange,1,trajectoryPointDisplay), ...
      pointsInRange(nonZeroCondInRange,2,trajectoryPointDisplay), ...
      pointsInRange(nonZeroCondInRange,3,trajectoryPointDisplay),'or')
plot3(pointsInContact(nonZeroCondInContact,1,trajectoryPointDisplay), ...
      pointsInContact(nonZeroCondInContact,2,trajectoryPointDisplay), ...
      pointsInContact(nonZeroCondInContact,3,trajectoryPointDisplay),'ob')
title('Trajectory, fitted plane in a point, in range points in red and in contact points in blue')
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
view(-80,20);
hold off
