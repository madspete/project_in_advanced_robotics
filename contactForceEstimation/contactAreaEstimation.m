clear
close all
clc

% Run the script while located in the directory where the script is located

% REQUIREMENTS:
% Computer Vision Toolbox
% Robotics System Toolbox

% --------- CONTROL OF TRAJECTORY TYPE ----------
% Control if a radomly generated trajectory is used, or if a recorded
% trajectory is loaded
useTestTrajectory = false;

% ---------- FIGURE DISPLAY CONTROL ----------
% Display point cloud
displayPointCloud = false;
% Display discretized tool points
showToolPoints = false;
% Display trajectory
showTrajectory = true;
% Create video of toolpath on trajectory 
% NOT RECOMMENDED ON ACTUAL PATH
makeTrajectoryVideo = false;
fileName = 'TrajectoryMovement.avi';
% Display estimated contact area and force graph
showEstimatedAreaAndContact = false;
% Display contact area and points in a specific trajectory point
trajectoryPointDisplay = 12;
displaySingleToolContactArea = true;

%% LOADING AND PREPROCSEEING OF POINT CLOUD   

% Loaded file is a modified form of the original CAD-file, such that a 
% smooth surface is obtained.
% Load .ply point cloud. Distamces are in mm.
orgCloud = pcread(strcat(pwd, filesep, 'pointClouds', filesep, ...
                         'objectPointCloudSimplified.ply'));

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
% easier. Threshold is chosen experimentally
pointPreprocessingZThreshold = 1; % [mm]
points = points(points(:,3) > pointPreprocessingZThreshold,:);

% Create and optionally display point cloud
cloud = pointCloud(points);
if displayPointCloud == true
    if ishandle(1) == true
        close(1)
    end
    figure(1)
    scatter3(points(:,1),points(:,2),points(:,3),5,points(:,3))
    title('Point cloud of the object to be polished')
    xlabel('X [mm]')
    xlim([0 max(points(:,1))])
    ylabel('Y [mm]')
    ylim([0 max(points(:,1))])
    zlabel('Z [mm]')
    zlim([0 max(points(:,1))/2])
end

%% DEFINE AND DISCRETIZE POLISHING TOOL 

% Radius of the tool
R = 125/2; % [mm]

% Step-size of the radius of the tool when discretizing it
toolRadiusStep = R/5; % [mm] 
% Step-size of the angle of the tool when discretizing it
toolCircleStep = 36; % [degree]
% Resolution of the discretizied radius
toolRadiusRes = R / toolRadiusStep; % [cm]
% Resoultion of the discretizized circle
toolCircleRes = 360 / toolCircleStep; % [degree]

% Generate tool points
% Preallocate matrix for storing the tool points
toolPoints = [zeros(toolRadiusRes*toolCircleRes,2) zeros(toolRadiusRes*toolCircleRes,1)];
radiusIndex = 0;
% Iterate through the points on the radius of the tool
for j = toolRadiusStep:toolRadiusStep:R
    toolIndex = 1;
    % Iterate through all points in the sub-circle
    for k = 0:toolCircleStep:360 
        xval = cos(deg2rad(k))*j;
        toolPoints(radiusIndex*toolCircleRes+toolIndex,1) = xval;
        yval = sin(deg2rad(k))*j;
        toolPoints(radiusIndex*toolCircleRes+toolIndex,2) = yval;
        toolIndex = toolIndex + 1;
    end
    radiusIndex = radiusIndex + 1;
end

% Plot the tool points
if showToolPoints == true
    if ishandle(2) == true
        close(2)
    end
    figure(2)
    plot3(toolPoints(:,1),toolPoints(:,2),toolPoints(:,3),'o');
    title('Discretized circle points')
    xlabel('X [mm]')
    ylabel('Y [mm]')
    zlabel('Z [mm]')
end

%% GENERATE A TRAJECTORY ACROSS THE SURFACE OF THE OBJECT

if useTestTrajectory == true
    % IDEA
    % Could also just do something like picking a point where x=0, then
    % increase x- and y-value of point by step and pick the closest point as 
    % the next point. This will yield a non-linear trajectory, but they are 
    % also not linear when recorded using LfD.
    
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
    if showTrajectory == true
        if ishandle(3) == true
            close(3)
        end
        figure(3)
        hold on
        % Change the view of the 3D figure. This is around the x-axis
        view(-70,25); 
        plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
        scatter3(cloud.Location(:,1),cloud.Location(:,2),cloud.Location(:,3),5)
        hold off
        title('Point cloud of the object and randomly generated test trajectory')
        xlabel('X [mm]')
        xlim([0 max(points(:,1))])
        ylabel('Y [mm]')
        ylim([0 max(points(:,1))])
        zlabel('Z [mm]')
        zlim([0 max(points(:,1))/2])
    end
end

%% LOAD TRAJECTORY AND APPLY TRANSFORMATIONS TO GET IN OBJECT BASE FRAME

% POTENTIAL IMPROVEMENTS:
% Plot the frame for every x'th trajectory point

% Procedure:
% Step 1: Load file with robot trajectory, convert translation into mm and
%         save it along with rotation in quaternions to matrix
% Step 2: Define transformation between TCP and Tool frame, and apply it
% Step 3: Define transformation between robot base frame and object base 
%         frame and apply it to the point cloud to get them transfromed
%         into the robot base frame
% Step 4: Display the TCP trajectory, tool trajectory and point cloud in 
%         the robot base frame, as well as poin

if useTestTrajectory == false

    % Constants
    % Tool height
    toolHeight = 45; % [mm]

    % Step 1:
    % Loading of the recorded trajectory. Transformations in the recorded
    % trajectory is between the robot base frame and the TCP frame of the
    % robot.
    trajectory = readtable(strcat(pwd, filesep, '..', filesep, ...
                           'trajectory_learning', filesep, ...
                           'demonstrations', filesep, 'tcptrial1.csv'));
    % Rename table names
    trajectory.Properties.VariableNames = ["ns","s","X","Y","Z","W","WX","WY","WZ"];
    % Extract translation and rotation into matrix
    TCPTrajectory = trajectory{:,["X","Y","Z","W","WX","WY","WZ"]};
    % Change units of translation into mm
    TCPTrajectory(:,1:3) = TCPTrajectory(:,1:3) * 1000; 
    % Save a copy for displaying
    TCPTrajectoryCopy = TCPTrajectory;
    % Initialize matrix for storing tool trajectory. first three elements
    % are translation, while element 4:6 is the first row vector of the
    % rotation matrix, 7:9 is the second, and 10:12 is the third
    toolTrajectory = zeros(size(TCPTrajectory,1),12);
    
    % Step 2:
    % Define the transformation between the robot TCP frame and the frame
    % of the tool. This is just an extension of the tool height along Z
    TTCPTool = [1 0 0 0;
                0 1 0 0;
                0 0 1 toolHeight;
                0 0 0 1];
    % Apply transformation from robot frame to tool frame by multiplying
    % all of the transformations.
    for i = 1:size(TCPTrajectory,1)
        orgTranslation = TCPTrajectory(i,1:3);
        orgRotation = quat2rotm(TCPTrajectory(i,4:7));
        orgTransformation = [orgRotation orgTranslation';
                             0 0 0 1];
        newTransform = orgTransformation * TTCPTool;
        toolTrajectory(i,1:3) = newTransform(1:3,4);
        toolTrajectory(i,4:12) = reshape(newTransform(1:3,1:3),[1 9]);
    end

    % Step 3:
    % Define the translation between the robot base frame and the object 
    % frame.
    robotObjectTranslation = [791 -14 0];
    % Define the rotation between the robot base frame and the object frame
    % It is rotated 90 degrees clockwise around the z robot axis
    ang = 22.26-90;
    robotObjectRotation = [cosd(ang) -sind(ang) 0;
                           sind(ang) cosd(ang)  0;
                           0     0      1];
    % Define transformation matrix from robot frame to object frame
    TObjectRobot = [robotObjectRotation robotObjectTranslation';
                    0 0 0 1];
    % Get the point cloud points
    points = cloud.Location;
    % Apply transformation to point cloud points. This is defined for 
    % column-vectors, hence the transpose of curToolLocation
    transformedPoints = TObjectRobot * [points ones(size(points,1),1)]';
    transformedCloud = pointCloud(transformedPoints(1:3,:)');

    % Step 4:
    % Display data
    if showTrajectory == true
        if ishandle(4) == true
            close(4)
        end
        figure(4)
        hold on
        % Change the view of the 3D figure. This is around the x-axis
        view(-70,25);
        % Plotting TCP trajectory
        plot3(TCPTrajectoryCopy(:,1),TCPTrajectoryCopy(:,2),TCPTrajectoryCopy(:,3),'og')
        % Plotting tool trajectory
        plot3(toolTrajectory(:,1),toolTrajectory(:,2),toolTrajectory(:,3),'ob')
        % Plot origo of the object frame in the robot frame
        plot3(robotObjectTranslation(1),robotObjectTranslation(2),robotObjectTranslation(3),'or')
        scatter3(transformedCloud.Location(:,1),transformedCloud.Location(:,2),transformedCloud.Location(:,3),5,transformedCloud.Location(:,3))
        hold off
        title('TCP and tool trajectory, as well as point cloud in robot base frame')
        xlabel('X [mm]')
        %xlim([0 max(points(:,1))])
        ylabel('Y [mm]')
        %ylim([0 max(points(:,1))])
        zlabel('Z [mm]')
        %zlim([0 max(points(:,1))/2])
    end
end

%% MOVING THE TOOL ALONG THE TRAJECTORY

% POTENTIAL IMPROVEMENTS:
% Plot the frame for every trajectory point
% Plot the plane when using the tool trajectory

% ---------- IF THE TEST TRAJECTORY IS CHOSEN, THE FOLLOWING STEPS ARE PERFORMED FIRST ----------
% Step 1: Iterate through all test trajectory points
% Step 2: For the current point, fit a plane to all points within an xy 
%         radius constraint to the current trajectory-point.
% Step 3: Define two vectors from the center of the plane, 
%         to the middle of one of the two xy edges of the plane, and use 
%         the cross-product to get the plane normal in positive z direction.
%         Save these vectors for later transformation

% ---------- TRANSFORMATION PROCEDURE FOR TOOL POINTS TO TRAJECTORY ----------
% Step 1: Iterate through all tool trajectory points
% Step 2: Transform test/tool points using the derived transformations

% Constants
% Depth at which the tool is below the surface of the object
polishingDepth = 1; % [mm]

% Define trajectory size, depending on if the test or real tool trajectory
% is used
if useTestTrajectory == true
    trajectorySize = size(testTrajectory,1);
else
    cloud = pointCloud(transformedCloud.Location);
    trajectorySize = size(toolTrajectory,1);
end

if useTestTrajectory == true
    % ---------- USING TEST TRAJECTORY ----------
    
    % Allocate matrix for storing the axis of each trajectory point
    testTrajectoryAxis = zeros(3,3,trajectorySize);

    % Define the number of points the fitted plane is discretized with along
    % each axis
    planeDiscretizationStep = 50;
    
    % Create 3D matrix object for storing discretized plane points
    discretizedPlanePoints = zeros(planeDiscretizationStep^2, 3, size(testTrajectory,1));
    
    % Step 1:
    % Iterate through the points in the trajectory
    for i = 1:size(testTrajectory,1)
        % Grab the current trajectory point
        curPoint = testTrajectory(i,:);
    
        % Step 2: 
        % Find all the points inside specified radius from the current point
        trajR = 10; % [mm]
        condXU = cloud.Location(:,1) <= (curPoint(1)+trajR);
        condXL = cloud.Location(:,1) >= (curPoint(1)-trajR);
        condYU = cloud.Location(:,2) <= (curPoint(2)+trajR);
        condYL = cloud.Location(:,2) >= (curPoint(2)-trajR);
        pointsInRange = cloud.Location(condXU & condXL & condYU & condYL, :);
        % Fit a plane to those points.
        % Solving Ax = B, where A is the pointInRange*3 matrix with the
        % first two columns being the x and y points found and the third being
        % ones, x is the pointsInRange*1 vector of coefficients of the plane
        % and B is a pointsInRange*1 vector constining all the z-values of the
        % pointsInRange points.
        % See the second anwser here for more detail
        % https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        % Operator '\' means solve the system of linear equations for A\B (Ax=B)
        planeXPoints = pointsInRange(:,1);
        planeYPoints = pointsInRange(:,2);
        planeZPoints = pointsInRange(:,3);
        AM = [planeXPoints planeYPoints ones(size(planeZPoints,1),1)];
        BM = planeZPoints;
        planeCoeffs = AM\BM;
    
        % Step 3:
        % Define trajectory point frame
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
        % Calculating the plane vectors relative to the center point of the 
        % plane
        planeXVec = origoPlaneXVec - origoPlaneCenterVec;
        planeYVec = origoPlaneYVec - origoPlaneCenterVec;
        % Normalizing the two plane vectors, calculating the plane normal
        % vector using the cross-product and normalizing that
        normPlaneXVec = planeXVec/norm(planeXVec);
        normPlaneYVec = planeYVec/norm(planeYVec);
        perpVec = cross(planeXVec,planeYVec);
        normVec = perpVec/norm(perpVec);
        % Save the vectors
        testTrajectoryAxis(:,:,i) = [normPlaneXVec;
                                     normPlaneYVec;
                                     normVec];
    end
end

% Initialize 3D storage matrix for tool points. This stores all tool points
% at each point on the trajectory
toolLocations = zeros(size(toolPoints,1),size(toolPoints,2),trajectorySize);

% Step 1:
% Iterate through all trajectory points
for i = 1:trajectorySize
    % Step 2:
    % Constructing transformation matrix. Depends on the chosen trajectory
    if useTestTrajectory == true
        currentTrajectoryPoint = testTrajectory(i,:);
        curTransform = [testTrajectoryAxis(1,:,i)', testTrajectoryAxis(2,:,i)', testTrajectoryAxis(3,:,i)', currentTrajectoryPoint';
                        0 0 0 1];
    else
        curTransform = [toolTrajectory(i,4:6) toolTrajectory(i,1);
                   toolTrajectory(i,7:9) toolTrajectory(i,2);
                   toolTrajectory(i,10:12) toolTrajectory(i,3);
                   0 0 0 1];
    end
    % Get a copy of the tool points to transform. Apply polishing depth
    % transformation at the same time
    curToolLocation = [toolPoints(:,1) toolPoints(:,2) toolPoints(:,3)-polishingDepth ones(size(toolPoints,1),1)];
    % Translate the tool points by adding trajectory-point. 
    curToolLocation = curTransform * curToolLocation';
    % Save the tool points to a 3D matrix for potential later use
    toolLocations(:,:,i) = curToolLocation(1:3,:)';
end

% Create and save figures for illustration video
if makeTrajectoryVideo == true
    close all
    for i = 1:size(toolLocations,3)
        figure('visible','off') % Dont display the figure when creating them
        hold on
        title('Discretized tool points on trajectory path')
        xlabel('X [mm]')
        %xlim([0 max(points(:,1))])
        ylabel('Y [mm]')
        %ylim([0 max(points(:,1))])
        zlabel('Z [mm]')
        %zlim([0 max(points(:,1))/2])
        % Change the view of the 3D figure. This is around the x-axis
        view(-80,20);
        % Plot point cloud
        scatter3(points(:,1),points(:,2),points(:,3),3,points(:,3))
        % Plot trajectory
        if useTestTrajectory == true
            plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
            % Plot plane
            mesh(reshape(discretizedPlanePoints(:,1,i),[planeDiscretizationStep planeDiscretizationStep]), ...
                 reshape(discretizedPlanePoints(:,2,i),[planeDiscretizationStep planeDiscretizationStep]), ...
                 reshape(discretizedPlanePoints(:,3,i),[planeDiscretizationStep planeDiscretizationStep]))
        else
            plot3(toolTrajectory(:,1),toolTrajectory(:,2),toolTrajectory(:,3))
        end
        % Plot tool points
        plot3(toolLocations(:,1,i),toolLocations(:,2,i),toolLocations(:,3,i),'or')
        hold off
        figureFrames(i) = getframe(gcf);
        clf(1) % Clear figure
    end
    
    %Make an illustration video
    videoWriterObj = VideoWriter(fileName);
    videoWriterObj.FrameRate = 2;
    open(videoWriterObj);
    for i=1:length(figureFrames)
     curFrame = figureFrames(i);
     writeVideo(videoWriterObj, curFrame);
    end
    close(videoWriterObj);
end

%% ESTIMATING CONTACT AREA IN TRAJECTORY

% I already know the transform, so i don't have to fit the plane
% I still want to plot the plane though, for visualization purposes

% Procedure:
% 1. Iterate through trajectory points
% 2. Fit plane to tool points in the current trajectory point
% 3. Select all points from point cloud which satisfy the radius constraint 
%    of the tool, meaning the points that are closer or equal to the radius
%    distance from the plane normal
% 4. Calculate the unit area, which is the total area divided by the number
%    of points within the radius constraint
% 5. Calculate the amount of points in contact, by transforming them to the
%    frame of the current trajectory point and looking for pointZ >= 0
% 6. Compute the percentage of points in range that are in contact with the
%    tool, as well as the area that they cover. Multiply with a target
%    pressure to get a force stimate in that trajectory point
% 7. Plot estimated contact percentage, area and force as a function of
%    trajectory point
% 8. Display figure with trajectory, plane, points in range and points in
%    contact

% Desired pressure that the tool has maintain in each point on the 
% trajectory. Following article suggests between 15 and 20 pounds of force
% per square foot.
% https://meguiarsonline.com/forum/information-station/how-to-articles/3079-how-to-use-the-g-100-to-remove-swirls?2965-How-to-use-the-G-100-to-remove-swirls=
% 15 pounds per square foot is used, which is around 718 Pa
desiredToolPressure = 718; % [Pa]

% Area of the tool
A = pi * R^2; % [mm^2] 

% Matricies to store the points within the radius constraint and in contact
pointsInRange = zeros(size(cloud.Location,1),3,trajectorySize);
pointsInContact = pointsInRange;

% Define the number of points the fitted plane is discretized with along
% each axis
toolPlaneDiscretizationStep = 50;

% Create 3D matrix object for storing discretized plane points
discretizedToolPlanePoints = zeros(toolPlaneDiscretizationStep^2, 3, trajectorySize);

% Define matrix for storing percentage of points in contact (first column),
% estimated contact area (second column) and estimated force (third column)
% in each trajectory point
estimationMatrix = zeros(trajectorySize,3);

% Step 1:
% Iterate trough trajectory points
for i = 1:trajectorySize

    % Step 2:
    % Fit plane to discretized tool points for plotting purposes
    % Get the tool points from the i'th trajectory point
    toolXVals = toolLocations(:,1,i);
    toolYVals = toolLocations(:,2,i);
    toolZVals = toolLocations(:,3,i);
    % Define the system of linear equations to solve for the plane 
    % parameters using a System of Linear Equations (SLE)
    SLE = [toolXVals toolYVals ones(size(toolZVals,1),1)];
    % Operator '\' means solve the system of linear equations for A\B (Ax=B)
    coeffs = SLE\toolZVals;
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
    % Find the center point of the plane, and define it as a vector
    centerX = mean(toolXRange);
    centerY = mean(toolYRange);
    centerZ = mean(toolZRange);
    planeCenter = [centerX centerY centerZ];

    % Step 3:
    % Select all points from the point cloud that satisfy the radius 
    % constraint
    % Define the current transform used depending on chosen trajectory
    if useTestTrajectory == true
        curTransform = [testTrajectoryAxis(1,:,i)', testTrajectoryAxis(2,:,i)', testTrajectoryAxis(3,:,i)', planeCenter';
                        0 0 0 1];
    else
        curTransform = [toolTrajectory(i,4:6) toolTrajectory(i,1);
                   toolTrajectory(i,7:9) toolTrajectory(i,2);
                   toolTrajectory(i,10:12) toolTrajectory(i,3);
                   0 0 0 1];
    end
    % Iterate over all point cloud points
    inRadiusIndex = 1;
    for j = 1:size(cloud.Location,1)
        % Get the ith point cloud point
        curPoint = cloud.Location(j,:); 
        % Compute the vector from the current point to the center of the plane
        vecCenterPoint = curPoint - planeCenter;

        % Construct a plane normal vector, it is just a transform in z
        zUnitTransform = [1 0 0 0;
                          0 1 0 0;
                          0 0 1 1;
                          0 0 0 1];
        planeZTransform = curTransform * zUnitTransform;
        planeZ = planeZTransform(1:3,4);
        planeNormal = planeZ' - planeCenter;

        % Compute the perpendicular distance from the current point to the
        % plane normal vector
        % See following link:
        % https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
        dist = norm(cross(vecCenterPoint,planeNormal))/norm(planeNormal);
        if abs(dist) <= R % Point is on circle perimiter or within 
            pointsInRange(inRadiusIndex,:,i) = curPoint;
            inRadiusIndex = inRadiusIndex + 1;
        end
    end
    
    % Step 4:
    % Calculate the unit area each point, which is the amount each point
    % contributes to the area of the polishing tool.
    % Since pointsInRangeRange are stored in one big 3D matrix, ignore 
    % zeros to get the true number of points
    nonZeroCond = pointsInRange(:,1,i) > 0;
    numberOfInRangePoints = size(pointsInRange(nonZeroCond,:,i),1);
    unitArea = A / numberOfInRangePoints; % [mm^2]
    
    % Step 5:
    % Find out how many points are in contact with the tool.
    curPointsInRange = [pointsInRange(1:numberOfInRangePoints,:,i) ones(numberOfInRangePoints,1)];
    curPointsInRangeTransformed = inv(curTransform) * curPointsInRange';
    % Remove unused dimension
    curPointsInRangeTransformed = curPointsInRangeTransformed(1:3,:)';
    % Filter points
    positiveZs = curPointsInRangeTransformed(:,3) >= 0;
    % Number of contact points is obtained by summing the logical structure
    numberOfContactPoints = sum(positiveZs);
    pointsInContact(1:numberOfContactPoints,:,i) = curPointsInRange(positiveZs,1:3);

    % Step 6:
    % Multiply the unit area with the amount of points in contact to get 
    % the estimated area
    estimationMatrix(i,1) = numberOfContactPoints / numberOfInRangePoints * 100;
    estimationMatrix(i,2) = numberOfContactPoints * unitArea * 10^-6; %[m^2]
    estimationMatrix(i,3) = desiredToolPressure * estimationMatrix(i,2); % [N]
end

% Step 7:
% Plot percentage of constact area, size of contact area and estimated 
% force as a function of trajectory point
if showEstimatedAreaAndContact == true
    if ishandle(5) == true
        close(5)
    end
    if ishandle(6) == true
        close(6)
    end
    figure(5)
    plot(1:trajectorySize, estimationMatrix(:,1))
    xlabel('Test trajectory point')
    ylabel('Percentage of points in contact with tool [%]')
    figure(6)
    plot(1:trajectorySize, estimationMatrix(:,3))
    title(strcat('With desired contact pressure p = ', num2str(desiredToolPressure), ' [Pa]'))
    xlabel('Test trajectory point')
    ylabel('Estimated contact force [N]')
end

% Step 8:
% Display figure with trajectory, fitted tool plane, points within radius 
% constraint and points in contact
% Variable used to control which trajectory point contact area estiamation
% is displayed
if displaySingleToolContactArea == true
    if ishandle(7) == true
        close(7)
    end
    f = figure(7);
    f.Position = [100 100 700 500];
    hold on
    if useTestTrajectory == true
        plot3(testTrajectory(:,1),testTrajectory(:,2),testTrajectory(:,3))
    else
        plot3(toolTrajectory(:,1),toolTrajectory(:,2),toolTrajectory(:,3))
    end
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
    scatter3(pointsInRange(nonZeroCondInRange,1,trajectoryPointDisplay), ...
          pointsInRange(nonZeroCondInRange,2,trajectoryPointDisplay), ...
          pointsInRange(nonZeroCondInRange,3,trajectoryPointDisplay),3,'r')
    scatter3(pointsInContact(nonZeroCondInContact,1,trajectoryPointDisplay), ...
          pointsInContact(nonZeroCondInContact,2,trajectoryPointDisplay), ...
          pointsInContact(nonZeroCondInContact,3,trajectoryPointDisplay),3,'b')
    title('Trajectory, fitted plane in a point, in range points in red and in contact points in blue')
    xlabel('X [mm]')
    ylabel('Y [mm]')
    zlabel('Z [mm]')
    view(-80,20);
    hold off
end
