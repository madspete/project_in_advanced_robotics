clc
clear 
close all
%% GMR test
means = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/means.txt");
cov = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/cov.txt");
prior = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/priors.txt");
trajectory1 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial1.csv");
trajectory2 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial2.csv");

data_dimension = length(means(:,1));
n_gaus = length(means(1,:));
covariance = zeros(data_dimension, data_dimension, n_gaus);

start_row = 1;
end_row = data_dimension;
for i = 1:n_gaus
    covariance(:,:,i) = cov(start_row:end_row, :);
    start_row = start_row + data_dimension;
    end_row = end_row + data_dimension;
end
covariance

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
new_time = 0:0.01:10;

expT(1,:) = linspace(min(new_time),max(new_time),length(new_time));
[expT([2:8],:), expSigma] = GMR(prior, means, covariance, new_time, [8], [1:7]);
figure("Name", "GMR of c++ data")
hold on;
plotGMM(expT([1,2],:), expSigma(1,1,:), [.8 0 0], 3);
hold off;
ylabel('force estimation [N]');
xlabel('time [s]');
title("GMR for force");

figure("Name", "x position")
hold on;
plotGMM(means([8,1],:), covariance([8,1],[8,1],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("x position [m]")
hold off
