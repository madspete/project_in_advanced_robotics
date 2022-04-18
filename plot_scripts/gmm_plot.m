clc 
clear
close all
%% gmm model computed in c++
means = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/means.txt");
cov = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/cov.txt");
prior = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/priors.txt");

data_dimension = length(means(:,1));
n_gaus = length(means(1,:));
covariance = zeros(data_dimension, data_dimension, n_gaus);

start_row = 1;
end_row = data_dimension
for i = 1:n_gaus
    covariance(:,:,i) = cov(start_row:end_row, :);
    start_row = start_row + data_dimension;
    end_row = end_row + data_dimension;
end

figure("Name", "x position")
hold on;
plotGMM(means([8,1],:), covariance([8,1],[8,1],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("x position [m]")
hold off

figure("Name", "y position")
hold on;
plotGMM(means([8,2],:), covariance([8,2],[8,2],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("y position [m]")
hold off

figure("Name", "z position")
hold on;
plotGMM(means([8,3],:), covariance([8,3],[8,3],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("z position [m]")
hold off


%% Use gmm in matlab to compute 
trajectory1 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial1.csv");
trajectory2 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial2.csv");
if length(trajectory1(:,1)) > length(trajectory2(:,1))
    step = 10 / length(trajectory1(:,1));
    scaled_time = 0:step:10-step;
    trajectory1 = trajectory1(:,[3:9]);
    step2 = 10 / length(trajectory2(:,1));
    scaled_time2 = 0:step2:10-step2;
    trajectory2 = interp1(scaled_time2, trajectory2(:,[3:9]), scaled_time, 'linear','extrap');
else
    step = 10 / length(trajectory2(:,1));
    scaled_time = 0:step:10-step;
    trajectory2 = trajectory2(:,[3:9]);
    step2 = 10 / length(trajectory1(:,1));
    scaled_time2 = 0:step2:10-step2;
    trajectory1 = interp1(scaled_time2, trajectory1(:,[3:9]), scaled_time, 'linear','extrap');
end


time = [scaled_time scaled_time];
data = zeros(length(time), data_dimension);
data(:,[1:7]) = [trajectory1; trajectory2];
data(:,8) = time;
K=5;
gmm_model = fitgmdist(data,K,'Start','plus','Replicates',100);
means = gmm_model.mu';
covariance = gmm_model.Sigma;
figure("Name", "x position matlab gmm")
hold on
plotGMM(means([8,1],:), covariance([8,1],[8,1],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("x position [m]")
hold off

figure("Name", "y position matlab gmm")
hold on;
plotGMM(means([8,2],:), covariance([8,2],[8,2],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("y position [m]")
hold off

figure("Name", "z position matlab gmm")
hold on;
plotGMM(means([8,3],:), covariance([8,3],[8,3],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("z position [m]")
hold off

expT(1,:) = linspace(min(data(:,8)),max(data(:,8)),length(scaled_time));
[expT([2:8],:), expSigma] = GMR(gmm_model.PComponents, gmm_model.mu', gmm_model.Sigma, scaled_time, [8], [1:7]);
figure("Name", "GMR force estimation")
hold on;
plotGMM(expT([1,2],:), expSigma(1,1,:), [.8 0 0], 3);
hold off;
ylabel('force estimation [N]');
xlabel('time [s]');
title("GMR for force");

expT(1,:) = linspace(min(data(:,8)),max(data(:,8)),length(scaled_time));
[expT([2:8],:), expSigma] = GMR(prior, means, covariance, scaled_time, [8], [1:7]);
figure("Name", "GMR of c++ data")
hold on;
plotGMM(expT([1,2],:), expSigma(1,1,:), [.8 0 0], 3);
hold off;
ylabel('force estimation [N]');
xlabel('time [s]');
title("GMR for force");


%% Plot trajectories

figure("Name", "x position raw data")
plot(scaled_time, trajectory1(:,1))
hold on
plot(scaled_time, trajectory2(:,1))

figure("Name", "y position raw data")
plot(scaled_time, trajectory1(:,2))
hold on
plot(scaled_time, trajectory2(:,2))

figure("Name", "z position raw data")
plot(scaled_time, trajectory1(:,3))
hold on
plot(scaled_time, trajectory2(:,3))

%% Plot interpolated trajectories
traj1 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/trajectory1.csv")
traj2 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/trajectory2.csv")
figure("Name", "x position from c++")
plot(scaled_time, trajectory1(:,1))
plot(traj1(:,1), traj1(:,2))
hold on
plot(traj2(:,1), traj2(:,2))
plot(traj1(:,1), trajectory2(:,1))

figure("Name", "y position from c++")
plot(traj1(:,1), traj1(:,3))
hold on
plot(traj2(:,1), traj2(:,3))

figure("Name", "z position from c++")
plot(traj1(:,1), traj1(:,4))
hold on
plot(traj2(:,1), traj2(:,4))
