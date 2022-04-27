clc
clear
close all

%% Calculating BIC for joint trajectories
addpath('/home/mads/Downloads/GMM-GMR-v2.0');
data1 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/BIC/jointtrial1.csv");
data2 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/BIC/jointtrial2.csv");
data3 = importdata("/home/mads/git/project_in_advanced_robotics/trajectory_learning/BIC/jointtrial3.csv");
length(data1)
length(data2)
length(data3)

% Interpolate trajectories to have same length
step = 10 / length(data2(:,1));
scaled_time = 0:step:10-step;
step1 = 10 / length(data1(:,1));
scaled_time1 = 0:step1:10-step1;
data1= interp1(scaled_time1, data1, scaled_time, 'linear','extrap');
data1

step3 = 10 / length(data3(:,1));
scaled_time3 = 0:step3:10-step3;
data3 = interp1(scaled_time3, data3, scaled_time, 'linear','extrap');

time = [scaled_time scaled_time scaled_time];
data = zeros(length(time), 7);
data(:,[1:6]) = [data1; data2; data3];
data(:,7) = time;


BIC = zeros(1,19);
i = 1;
for k=[2:20]
 gmm_model = fitgmdist(data,k,'Start', 'randSample','Replicates',10);
 BIC(i) = gmm_model.BIC;
 i = i + 1;
end

%% Plot the BIC
figure("Name", "BIC for joint angles")
k = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20];
plot(k, BIC)
xlabel("Number of clusters")
ylabel("BIC score")
title("BIC scores versus number of clusters")
figure("Name", "BIC gradients")
plot(k,gradient(BIC))
xlabel("Number of clusters")
ylabel("Gradient of BIC score")
title("BIC scores versus number of clusters")
