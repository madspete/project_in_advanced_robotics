clc 
clear
close all
%% gmm model computed in c++
addpath('/home/mads/Downloads/GMM-GMR-v2.0');
means = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/means.txt");
cov = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/cov.txt");
priors = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning//gmm_model/priors.txt")
prior = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/gmm_model/priors.txt");
data1 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial1.csv");
data2 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial2.csv");
data3 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/demonstrations/tcptrial3.csv");
data4 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/traj_tcp.csv");
scaled_time = importdata("/home/mads/git/project_in_advanced_robotics/input_time.csv");

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

time1 = 0:0.002:length(data1)*0.002-0.002;
time2 = 0:0.002:length(data2)*0.002-0.002;
time3 = 0:0.002:length(data3)*0.002-0.002;
time4 = 0:0.002:length(data4)*0.002-0.002;

figure("Name", "x position")
hold on;
plotGMM(means([7,3],:), covariance([7,3],[7,3],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("Base joint rad")
title("Clusters for the y complex part")
hold off

figure("Name", "y position")
hold on;
plotGMM(means([7,2],:), covariance([7,2],[7,2],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("s position [m]")
hold off

figure("Name", "z position")
hold on;
plotGMM(means([7,3],:), covariance([7,3],[7,3],:), [0 .8 0], 1);
xlabel("time [s]")
ylabel("z position [m]")
hold off

figure("Name", "recorded data")
hold on;
plot(time1, data1(:,4))
plot(time2, data2(:,4))
plot(time3, data3(:,4))
plot(time1, data1(:,5))
plot(time2, data2(:,5))
plot(time3, data3(:,5))
plot(time1, data1(:,6))
plot(time2, data2(:,6))
plot(time3, data3(:,6))
plot(time1, data1(:,7))
plot(time2, data2(:,7))
plot(time3, data3(:,7))
xlabel("time [s]")
title("Recorded trajectories for the orientation")
legend('w', 'x', 'y', 'z')
hold off;

figure("Name", "GMR output")
hold on;
plot(time4, data4(:,3))
xlabel("time [s]")
ylabel("base joint [rad]")
title("GMR output for the base joint")
hold off;

covariance
expT(1,:) = linspace(min(scaled_time),max(scaled_time),length(scaled_time));
[expT([2:7],:), expSigma] = GMR(priors, means, covariance, scaled_time', [7], [1:6]);

figure("Name", "GMR output of c++ data")
hold on;
plotGMM(expT([1,4],:), expSigma(4,4,:), [.8 0 0], 3);
hold off;
ylabel('Base joint [rad]');
xlabel('time [s]');
title("GMR output for the base joint");


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
covariance
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
