clc
clear
close all

%% Initial experiment with STL-files

% Load object and extract points
object = stlread('C:\Users\Bruger\Desktop\SDU\Civilingeniør\2. Semester\Project in Advanced Robotics\platePoints.stl');
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

cloud = pcread('platePoints.ply');

% For visualization of the point cloud
%pcshow(cloud)

% Choose a random point along the x-axis as the starting point for the
% polishing path
xRand = randi([0,100]);
startpoint = [xRand 0];
endpoint = [xRand 100];

% Discretize the points on the line between the two points in order to simulate datapoints
% collected from LfD
numberOfPoints = 100; % The number of points to sample on the line
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