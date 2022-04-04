clc
clear
close all
%% Lfd with controller

% Load functions from https://www.epfl.ch/labs/lasa/older-source-codes/
 addpath('./lfd_functions/');

%% Setup data
time = [0:0.002:10];
y = time;
z = time;
x = time;
z([1:length(time)]) = 0.2;
x([1:3001]) = 0:0.0001:0.3;
x([3002:length(time)]) = 0.3;
y([1:2000]) = 0;
y([2001:length(time)]) = 0:0.0001:0.3;
motion2=[time;x;y;z;]';
desired_forcez = time;
desired_forcez([1:501]) = 0:0.04:20;
desired_forcez([502:length(time)]) = 20;


%% add noise to measurements and create three repitions
data = zeros(length(time)*3, 5);

old_i = 1;
constant = 50;
i = constant;
while i <= length(time)
    data(old_i:i,:) = [time(old_i:i)' (x(old_i:i) + 0.1*randn(1))' (y(old_i:i) + 0.1*randn(1))' (z(old_i:i) + 0.1*randn(1))' (desired_forcez(old_i:i) + 0.1*randn(1))'];
    data(length(time)+old_i:length(time)+i,:) = [time(old_i:i)' (x(old_i:i) + 0.1*randn(1))' (y(old_i:i) + 0.1*randn(1))' (z(old_i:i) + 0.1*randn(1))' (desired_forcez(old_i:i) + 0.1*randn(1))'];
    data(length(time)*2+old_i:length(time)*2+i,:) = [time(old_i:i)' (x(old_i:i) + 0.1*randn(1))' (y(old_i:i) + 0.1*randn(1))' (z(old_i:i) + 0.1*randn(1))' (desired_forcez(old_i:i) + 0.1*randn(1))'];
    old_i = i+1;
    i = i + constant;
end

figure("name", "trajectory position");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax);
plot(time, data(1:length(time), 2));
hold on;
plot(time, data(length(time)+1:length(time)*2, 2));
plot(time, data(length(time)*2+1:end, 2));
hold off;
ylabel('time [s]');
xlabel('position [mm]');
title("Trajectory for x");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax);
plot(time, data(1:length(time), 3));
hold on;
plot(time, data(length(time)+1:length(time)*2, 3));
plot(time, data(length(time)*2+1:end, 3));
hold off;
ylabel('time [s]');
xlabel('position [mm]');
title("Trajectory for y");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax);
plot(time, data(1:length(time), 4));
hold on;
plot(time, data(length(time)+1:length(time)*2, 4));
plot(time, data(length(time)*2+1:end, 4));
ylabel('time [s]');
xlabel('position [mm]');
title("Trajectory for z");

figure("name", "force");
plot(time, data(1:length(time), 5));
hold on;
plot(time, data(length(time)+1:length(time)*2, 5));
plot(time, data(length(time)*2+1:end, 5));
hold off;
ylabel('Contact force [N]');
xlabel('time [s]');
title("Contact force z direction");

%% Perform GMM
K=3;
gmm_model = fitgmdist(data,K,'Start','plus','Replicates',100);
gmm_model.mu

figure("name", "GMM models position");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(gmm_model.mu(:,[1,2])', gmm_model.Sigma([1,2], [1,2],:), [.8 0 0], 1);
hold off;
ylabel('x position [mm]');
xlabel('time [s]');
title("GMM for x position");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(gmm_model.mu(:,[1,3])', gmm_model.Sigma([1,3], [1,3],:), [.8 0 0], 1);
hold off;
ylabel('y position [mm]');
xlabel('time [s]');
title("GMM for y position");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax); hold on;
plotGMM(gmm_model.mu(:,[1,4])', gmm_model.Sigma([1,4], [1,4],:), [.8 0 0], 1);
ylabel('z position [mm]');
xlabel('time [s]');
title("GMM for z position");

figure("name", "force");
hold on;
plotGMM(gmm_model.mu(:,[1,5])', gmm_model.Sigma([1,5], [1,5],:), [.8 0 0], 1);
hold off;
ylabel('Contact force [N]');
xlabel('time [s]');
title("Contact force z direction");

%% GMR regression

expT(1,:) = linspace(min(data(:,1)),max(data(:,1)),length(time));
[expT(2:5,:), expSigma] = GMR(gmm_model.PComponents, gmm_model.mu', gmm_model.Sigma, time, [1], [2:5]);

figure("name", "GMR models position");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(expT([1,2],:), expSigma(1,1,:), [.8 0 0], 3);
hold off;
ylabel('x position [mm]');
xlabel('time [s]');
title("GMR for x position");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(expT([1,3],:), expSigma(2,2,:), [.8 0 0], 3);
hold off;
ylabel('y position [mm]');
xlabel('time [s]');
title("GMR for y position");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax); hold on;
plotGMM(expT([1,4],:), expSigma(3,3,:), [.8 0 0], 3);
hold off;
ylabel('z position [mm]');
xlabel('time [s]');
title("GMR for z position");

figure("Name", "GMR force estimation")
hold on;
plotGMM(expT([1,5],:), expSigma(4,4,:), [.8 0 0], 3);
hold off;
ylabel('force estimation [N]');
xlabel('time [s]');
title("GMR for force");

%% Create the hybrid force controller from the paper 
% The controller is located in simulink, here the constants are defined

constraint_matrix = diag([0 0 1 0 0 0]);
kpf = [0.1 0 0 0 0 0; 
       0 0.1 0 0 0 0; 
       0 0 0.1 0 0 0; 
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0];
kif = [0.001 0 0 0 0 0; 
       0 0.001 0 0 0 0; 
       0 0 0.001 0 0 0; 
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0];

% THis are the control gains for joint position control, this will be done
% internally on the robot on a real robot. 
kp = [10 0 0 0 0 0; 
      0 10 0 0 0 0;
      0 0 10 0 0 0;
      0 0 0 10 0 0;
      0 0 0 0 10 0;
      0 0 0 0 0 10];
kd = [5 0 0 0 0 0; 
      0 5 0 0 0 0;
      0 0 5 0 0 0;
      0 0 0 5 0 0;
      0 0 0 0 5 0;
      0 0 0 0 0 5];

% Load robot model
robot = loadrobot('universalUR5','DataFormat','column','Gravity',[0 0 -9.82]);
ik_weights = [1 1 1 1 1 1];

K = [0 0 0 0 0 0;
     0 0 0 0 0 0; 
     0 0 30 0 0 0; 
     0 0 0 0 0 0; 
     0 0 0 0 0 0; 
     0 0 0 0 0 0];
or = [0, 0, 0.1, 0 0 0]';

% Figure out how to correctly calculate the reproduction, this is the only
% thing missing. 
force = [time', zeros(2, length(time))', desired_forcez', zeros(3, length(time))'];
motion = [time', expT([2:4],:)',  zeros(3, length(time))'];


Md = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
invMd = inv(Md);
ik = inverseKinematics('RigidBodyTree',robot);
randConfig =[1 1 1 0 0 1.5708]';
initialguess = robot.homeConfiguration;
desired = robot.randomConfiguration;
tform = getTransform(robot,desired,'base','tool0')
sol = ik('tool0', tform, ik_weights, initialguess);
tform = getTransform(robot,sol,'base','tool0')