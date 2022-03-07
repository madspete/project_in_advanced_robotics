clc
clear
close all
%% Project lfd UR robot
% First 3 joints are used to generate a trajectory, but can be extended to
% include all joints, or it could be done for task space as well

% Functions are taken from here https://www.epfl.ch/labs/lasa/older-source-codes/
addpath('./lfd_functions/');
%% Generate trajectory
omega_1 = 1;
omega_2 = 2;
omega_3 = 3;
he = zeros(6, 1);
t = 0:0.002:5;
trajectory = zeros(length(t)*3, 7);
index = 1;
for i = 1:length(t)
    % We add gaussian noise to all the measurements
    trajectory(i,:) = [t(i) sin(omega_1*t(i)) + 0.1*randn(1) sin(omega_2*t(i))+ 0.1*randn(1) sin(omega_3*t(i))+ 0.1*randn(1) omega_1*cos(omega_1*t(i))+ 0.1*randn(1) omega_2*cos(omega_2*t(i))+ 0.1*randn(1) omega_3*cos(omega_3*t(i))+ 0.1*randn(1)];
    trajectory(length(t)+i,:) = [t(i) sin(omega_1*t(i)) + 0.1*randn(1) sin(omega_2*t(i))+ 0.1*randn(1) sin(omega_3*t(i)) + 0.1*randn(1) omega_1*cos(omega_1*t(i)) + 0.1*randn(1) omega_2*cos(omega_2*t(i)) + 0.1*randn(1) omega_3*cos(omega_3*t(i)) + 0.1*randn(1)];
    trajectory(length(t)*2+i,:) = [t(i) sin(omega_1*t(i)) + 0.1*randn(1) sin(omega_2*t(i))+ 0.1*randn(1) sin(omega_3*t(i)) + 0.1*randn(1) omega_1*cos(omega_1*t(i)) + 0.1*randn(1) omega_2*cos(omega_2*t(i)) + 0.1*randn(1) omega_3*cos(omega_3*t(i)) + 0.1*randn(1)];
end

figure("name", "trajectory position");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax);
plot(t, trajectory(1:length(t), 2));
hold on;
plot(t, trajectory(length(t)+1:length(t)*2, 2));
plot(t, trajectory(length(t)*2+1:end, 2));
hold off;
ylabel('time [s]');
xlabel('angle [rad]');
title("Trajectory for theta 1");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax);
plot(t, trajectory(1:length(t), 3));
hold on;
plot(t, trajectory(length(t)+1:length(t)*2, 3));
plot(t, trajectory(length(t)*2+1:end, 3));
hold off;
ylabel('time [s]');
xlabel('angle [rad]');
title("Trajectory for theta 2");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax);
plot(t, trajectory(1:length(t), 4));
hold on;
plot(t, trajectory(length(t)+1:length(t)*2, 4));
plot(t, trajectory(length(t)*2+1:end, 4));
ylabel('time [s]');
xlabel('angle [rad]');
title("Trajectory for theta 3");

figure("name", "trajectory velocity");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax);
plot(t, trajectory(1:length(t), 5));
hold on;
plot(t, trajectory(length(t)+1:length(t)*2, 5));
plot(t, trajectory(length(t)*2+1:end, 5));
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("velocity for theta 1");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax);
plot(t, trajectory(1:length(t), 6));
hold on;
plot(t, trajectory(length(t)+1:length(t)*2, 6));
plot(t, trajectory(length(t)*2+1:end, 6));
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("Velocity for theta 2");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax);
plot(t, trajectory(1:length(t), 7));
hold on;
plot(t, trajectory(length(t)+1:length(t)*2, 7));
plot(t, trajectory(length(t)*2+1:end, 7));
ylabel('time [s]');
xlabel('angle [rad/s]');
title("velocity for theta 3");

%% Create GMM model 
K=5;
obj2 = fitgmdist(trajectory,K,'Start','plus','Replicates',100);
obj2.mu

figure("name", "GMM models position");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(obj2.mu(:,[1,2])', obj2.Sigma([1,2], [1,2],:), [.8 0 0], 1);
hold off;
ylabel('time [s]');
xlabel('angle [rad]');
title("GMM for theta 1");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(obj2.mu(:,[1,3])', obj2.Sigma([1,3], [1,3],:), [.8 0 0], 1);
hold off;
ylabel('time [s]');
xlabel('angle [rad]');
title("GMM for theta 2");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax); hold on;
plotGMM(obj2.mu(:,[1,4])', obj2.Sigma([1,4], [1,4],:), [.8 0 0], 1);
ylabel('time [s]');
xlabel('angle [rad]');
title("GMM for theta 3");

figure("name", "GMM models velocity");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(obj2.mu(:,[1,5])', obj2.Sigma([1,5], [1,5],:), [.8 0 0], 1);
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("GMM velocity for theta 1");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(obj2.mu(:,[1,6])', obj2.Sigma([1,6], [1,6],:), [.8 0 0], 1);
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("GMM velocity for theta 2");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax); hold on;
plotGMM(obj2.mu(:,[1,7])', obj2.Sigma([1,7], [1,7],:), [.8 0 0], 1);
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("GMM velocity for theta 3");

%% Perform GMR regression 

expT(1,:) = linspace(min(trajectory(:,1)),max(trajectory(:,1)),length(t));
[expT(2:7,:), expSigma] = GMR(obj2.PComponents, obj2.mu', obj2.Sigma, t, [1], [2:7]);

figure("name", "GMR models position");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(expT([1,2],:), expSigma(1,1,:), [.8 0 0], 3);
hold off;
ylabel('time [s]');
xlabel('angle [rad]');
title("GMR for theta 1");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(expT([1,3],:), expSigma(2,2,:), [.8 0 0], 3);
hold off;
ylabel('time [s]');
xlabel('angle [rad]');
title("GMR for theta 2");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax); hold on;
plotGMM(expT([1,4],:), expSigma(3,3,:), [.8 0 0], 3);
hold off;
ylabel('time [s]');
xlabel('angle [rad]');
title("GMR for theta 3");

figure("name", "GMR models velocity");
ax = [0.05 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(expT([1,5],:), expSigma(4,4,:), [.8 0 0], 3);
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("GMR velocity for theta 1");
ax = [0.55 0.55 0.4 0.4];
subplot('Position',ax); hold on;
plotGMM(expT([1,6],:), expSigma(5,5,:), [.8 0 0], 3);
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("GMR velocity for theta 2");
ax = [0.3 0.02 0.4 0.4];
subplot('Position', ax); hold on;
plotGMM(expT([1,7],:), expSigma(6,6,:), [.8 0 0], 3);
hold off;
ylabel('time [s]');
xlabel('angle [rad/s]');
title("GMR velocity for theta 3");



