clc
clear
close all

%% collect measurements 

% Load functions from https://www.epfl.ch/labs/lasa/older-source-codes/
addpath('./lfd_functions/');

wrench1 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\wrenchRobotTrial1.csv');
wrench2 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\wrenchRobotTrial2.csv');
wrench3 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\wrenchRobotTrial3.csv');
wrench4 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\wrenchRobotTrial4.csv');
wrench5 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\wrenchRobotTrial5.csv');

motion1 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\tcpRobotTrial1.csv');
motion2 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\tcpRobotTrial2.csv');
motion3 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\tcpRobotTrial3.csv');
motion4 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\tcpRobotTrial4.csv');
motion5 = importdata('C:\Users\peter\kandidat_lit\2.semester\git\project_in_advanced_robotics\lfd_matlab\data\tcpRobotTrial5.csv');

%% Interpolate all data to have the same length 
if length(motion5(:,1)) >= length(motion4(:,1)) && length(motion5(:,1)) >= length(motion3(:,1)) && length(motion5(:,1)) >= length(motion2(:,1)) && length(motion5(:,1)) >= length(motion1(:,1))
    step = 1 / length(wrench5(:,1));
    scaled_time_wrench = 0:step:1-step;
    step1 = 1 / length(wrench1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step2 = 1 / length(wrench2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step3 = 1 / length(wrench3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step4 = 1 / length(wrench4(:,1));
    scaled_time4 = 0:step4:1-step4;
    wrench1 = interp1(scaled_time1, wrench1(:,5), scaled_time_wrench, 'linear','extrap');
    wrench2 = interp1(scaled_time2, wrench2(:,5), scaled_time_wrench, 'linear','extrap');
    wrench3 = interp1(scaled_time3, wrench3(:,5), scaled_time_wrench, 'linear','extrap');
    wrench4 = interp1(scaled_time4, wrench4(:,5), scaled_time_wrench, 'linear','extrap');
    wrench5 =  wrench5(:,5);

    % Motion 
    step = 1 / length(motion5(:,1));
    scaled_time = 0:step:1-step;
    step1 = 1 / length(motion1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step2 = 1 / length(motion2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step3 = 1 / length(motion3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step4 = 1 / length(motion4(:,1));
    scaled_time4 = 0:step4:1-step4;
    motion1 = interp1(scaled_time1, motion1(:,[3:end]), scaled_time);
    motion2 = interp1(scaled_time2, motion2(:,[3:end]), scaled_time);
    motion3 = interp1(scaled_time3, motion3(:,[3:end]), scaled_time);
    motion4 = interp1(scaled_time4, motion4(:,[3:end]), scaled_time);
    motion5 =  motion5(:,[3:end]);
elseif length(motion4(:,1)) >= length(motion5(:,1)) && length(motion4(:,1)) >= length(motion3(:,1)) && length(motion4(:,1)) >= length(motion2(:,1)) && length(motion4(:,1)) >= length(motion1(:,1))
    step = 1 / length(wrench4(:,1));
    scaled_time_wrench = 0:step:1-step;
    step1 = 1 / length(wrench1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step2 = 1 / length(wrench2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step3 = 1 / length(wrench3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step5 = 1 / length(wrench5(:,1));
    scaled_time5 = 0:step5:1-step4;
    wrench1 = interp1(scaled_time1, wrench1(:,5), scaled_time_wrench, 'linear','extrap');
    wrench2 = interp1(scaled_time2, wrench2(:,5), scaled_time_wrench, 'linear','extrap');
    wrench3 = interp1(scaled_time3, wrench3(:,5), scaled_time_wrench, 'linear','extrap');
    wrench5 = interp1(scaled_time5, wrench5(:,5), scaled_time_wrench, 'linear','extrap');
    wrench4 =  wrench4(:,5);

    % Motion data    
    step = 1 / length(motion4(:,1));
    scaled_time = 0:step:1-step;
    step1 = 1 / length(motion1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step2 = 1 / length(motion2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step3 = 1 / length(motion3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step5 = 1 / length(motion5(:,1));
    scaled_time5 = 0:step5:1-step4;
    motion1 = interp1(scaled_time1, motion1(:,[3:end]), scaled_time);
    motion2 = interp1(scaled_time2, motion2(:,[3:end]), scaled_time);
    motion3 = interp1(scaled_time3, motion3(:,[3:end]), scaled_time);
    motion5 = interp1(scaled_time5, motion5(:,[3:end]), scaled_time);
    motion4 =  motion4(:,[3:end]);
elseif length(motion3(:,1)) >= length(motion5(:,1)) && length(motion3(:,1)) >= length(motion4(:,1)) && length(motion3(:,1)) >= length(motion2(:,1)) && length(motion3(:,1)) >= length(motion1(:,1))
    step = 1 / length(wrench3(:,1));
    scaled_time_wrench = 0:step:1-step;
    step1 = 1 / length(wrench1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step2 = 1 / length(wrench2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step5 = 1 / length(wrench5(:,1));
    scaled_time5 = 0:step5:1-step5;
    step4 = 1 / length(wrench4(:,1));
    scaled_time4 = 0:step4:1-step4;
    wrench1 = interp1(scaled_time1, wrench1(:,5), scaled_time_wrench, 'linear','extrap');
    wrench2 = interp1(scaled_time2, wrench2(:,5), scaled_time_wrench, 'linear','extrap');
    wrench5 = interp1(scaled_time5, wrench5(:,5), scaled_time_wrench, 'linear','extrap');
    wrench4 = interp1(scaled_time4, wrench4(:,5), scaled_time_wrench, 'linear','extrap');
    wrench3 =  wrench3(:,5);

    % Motion data
    step = 1 / length(motion3(:,1));
    scaled_time = 0:step:1-step;
    step1 = 1 / length(motion1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step2 = 1 / length(motion2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step5 = 1 / length(motion5(:,1));
    scaled_time5 = 0:step5:1-step5;
    step4 = 1 / length(motion4(:,1));
    scaled_time4 = 0:step4:1-step4;
    motion1 = interp1(scaled_time1, motion1(:,[3:end]), scaled_time);
    motion2 = interp1(scaled_time2, motion2(:,[3:end]), scaled_time);
    motion5 = interp1(scaled_time5, motion5(:,[3:end]), scaled_time);
    motion4 = interp1(scaled_time4, motion4(:,[3:end]), scaled_time);
    motion3 =  motion3(:,[3:end]);
elseif length(motion2(:,1)) >= length(motion5(:,1)) && length(motion2(:,1)) >= length(motion3(:,1)) && length(motion2(:,1)) >= length(motion3(:,1)) && length(motion2(:,1)) >= length(motion1(:,1))
    % Wrench data
    step = 1 / length(wrench2(:,1));
    scaled_time_wrench = 0:step:1-step;
    step1 = 1 / length(wrench1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step3 = 1 / length(wrench3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step5 = 1 / length(wrench5(:,1));
    scaled_time5 = 0:step5:1-step5;
    step4 = 1 / length(wrench4(:,1));
    scaled_time4 = 0:step4:1-step4;
    wrench1 = interp1(scaled_time1, wrench1(:,5), scaled_time_wrench, 'linear','extrap');
    wrench3 = interp1(scaled_time3, wrench3(:,5), scaled_time_wrench, 'linear','extrap');
    wrench5 = interp1(scaled_time5, wrench5(:,5), scaled_time_wrench, 'linear','extrap');
    wrench4 = interp1(scaled_time4, wrench4(:,5), scaled_time_wrench, 'linear','extrap');
    wrench2 =  wrench2(:,5);
    
    % Motion data
    step = 1 / length(motion2(:,1));
    scaled_time = 0:step:1-step;
    step1 = 1 / length(motion1(:,1));
    scaled_time1 = 0:step1:1-step1;
    step3 = 1 / length(motion3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step5 = 1 / length(motion5(:,1));
    scaled_time5 = 0:step5:1-step5;
    step4 = 1 / length(motion4(:,1));
    scaled_time4 = 0:step4:1-step4;
    motion1 = interp1(scaled_time1, motion1(:,[3:end]), scaled_time);
    motion3 = interp1(scaled_time3, motion3(:,[3:end]), scaled_time);
    motion5 = interp1(scaled_time5, motion5(:,[3:end]), scaled_time);
    motion4 = interp1(scaled_time4, motion4(:,[3:end]), scaled_time);
    motion2 =  motion2(:,[3:end]);
elseif length(motion1(:,1)) >= length(motion5(:,1)) && length(motion1(:,1)) >= length(motion3(:,1)) && length(motion1(:,1)) >= length(motion3(:,1)) && length(motion1(:,1)) >= length(motion2(:,1))
    step = 1 / length(wrench1(:,1));
    scaled_time_wrench = 0:step:1-step;
    step2 = 1 / length(wrench2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step3 = 1 / length(wrench3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step5 = 1 / length(wrench5(:,1));
    scaled_time5 = 0:step5:1-step5;
    step4 = 1 / length(wrench4(:,1));
    scaled_time4 = 0:step4:1-step4;
    wrench2 = interp1(scaled_time2, wrench2(:,5), scaled_time_wrench,'linear','extrap');
    wrench3 = interp1(scaled_time3, wrench3(:,5), scaled_time_wrench, 'linear','extrap');
    wrench5 = interp1(scaled_time5, wrench5(:,5), scaled_time_wrench, 'linear','extrap');
    wrench4 = interp1(scaled_time4, wrench4(:,5), scaled_time_wrench,'linear','extrap');
    wrench1 =  wrench1(:,5)';

    % Motion     
    step = 1 / length(motion1(:,1));
    scaled_time = 0:step:1-step;
    step2 = 1 / length(motion2(:,1));
    scaled_time2 = 0:step2:1-step2;
    step3 = 1 / length(motion3(:,1));
    scaled_time3 = 0:step3:1-step3;
    step5 = 1 / length(motion5(:,1));
    scaled_time5 = 0:step5:1-step5;
    step4 = 1 / length(motion4(:,1));
    scaled_time4 = 0:step4:1-step4;
    motion2 = interp1(scaled_time2, motion2(:,[3:end]), scaled_time);
    motion3 = interp1(scaled_time3, motion3(:,[3:end]), scaled_time);
    motion5 = interp1(scaled_time5, motion5(:,[3:end]), scaled_time);
    motion4 = interp1(scaled_time4, motion4(:,[3:end]), scaled_time);
    motion1 =  motion1(:,[3:end]);
else
    error("This shouldn't happen, have a look at the code and fix it")
end

figure("name", "Wrench data in z axis")
plot(scaled_time_wrench, wrench1)
hold on
plot(scaled_time_wrench, wrench2)
plot(scaled_time_wrench, wrench3)
plot(scaled_time_wrench, wrench4)
plot(scaled_time_wrench, wrench5)

figure("Name", "Motion x y")
plot(motion1(:,1), motion1(:,2))
hold on
plot(motion2(:,1), motion2(:,2))
plot(motion3(:,1), motion3(:,2))
plot(motion4(:,1), motion4(:,2))
plot(motion5(:,1), motion5(:,2))
% How to use DTW for multiple signals at once
%dtw(motion1(:,[1:6]), motion2(:,[1:6]), motion3(:,[1:6]), motion4(:,[1:6]), motion5(:,[1:6]))


%% Filter the force data
% This has been transferred from simulink by transforming rad/s to Hz.
fc = 4.1;
fs = 500;

[y, x] = butter(5, fc/(fs/2));
filtered_wrench1 = filter(y,x, wrench1);
filtered_wrench2 = filter(y,x, wrench2);
filtered_wrench3 = filter(y,x, wrench3);
filtered_wrench4 = filter(y,x, wrench4);
filtered_wrench5 = filter(y,x, wrench5);
figure("Name", "Filtered signal")
plot(scaled_time_wrench, filtered_wrench1)
hold on
plot(scaled_time_wrench, filtered_wrench2)
plot(scaled_time_wrench, filtered_wrench3)
plot(scaled_time_wrench, filtered_wrench4)
plot(scaled_time_wrench, filtered_wrench5)


%% Encode the force data using gmm and gmr
time = [scaled_time_wrench scaled_time_wrench scaled_time_wrench scaled_time_wrench scaled_time_wrench];
force_data = [filtered_wrench1 filtered_wrench2 filtered_wrench3 filtered_wrench4 filtered_wrench5];
data = [time', force_data'];

K=6;
gmm_model = fitgmdist(data,K,'Start','plus','Replicates',100);
gmm_model.mu
figure("name", "force");
hold on;
plotGMM(gmm_model.mu(:,[1,2])', gmm_model.Sigma([1,2], [1,2],:), [.8 0 0], 1);
hold off;
ylabel('Contact force [N]');
xlabel('time [s]');
title("Contact force z direction");

expT(1,:) = linspace(min(data(:,1)),max(data(:,1)),length(scaled_time_wrench));
[expT(2,:), expSigma] = GMR(gmm_model.PComponents, gmm_model.mu', gmm_model.Sigma, scaled_time_wrench, [1], [2]);
figure("Name", "GMR force estimation")
hold on;
plotGMM(expT([1,2],:), expSigma(1,1,:), [.8 0 0], 3);
hold off;
ylabel('force estimation [N]');
xlabel('time [s]');
title("GMR for force");

%% Create the hybrid force controller from the paper 

% The controller is located in simulink, here the constants are defined
constraint_matrix = diag([0 0 1 0 0 0]);
kpf = [0.1 0 0 0 0 0; 
       0 0.1 0 0 0 0; 
       0 0 1.2 0 0 0; 
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0];
kif = [0.01 0 0 0 0 0; 
       0 0.01 0 0 0 0; 
       0 0 0.01 0 0 0; 
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
     0 0 60 0 0 0; 
     0 0 0 0 0 0; 
     0 0 0 0 0 0; 
     0 0 0 0 0 0];
or = [0, 0, -0.2, 0 0 0]';

% Figure out how to get feedback from the environment when simulating the
% controller

% Figure out how to correctly calculate the reproduction, this is the only
% thing missing. 
step = 10 / length(scaled_time_wrench);
time = 0:step:10 - step;
desired_forcez = expT(2,:);

% Create force input instead of basing the controller on the measurements, 
% since it is hard to model the environment. 
desired_forcez = time;
desired_forcez([1:501]) = 0:0.04:20;
desired_forcez([502:length(time)]) = 20;
force = [time', zeros(2, length(time))', desired_forcez', zeros(3, length(time))'];

% Convert to euler angles
for i=1:length(motion1(:,1))
    motion1(i, [4:6]) = quat2eul(motion1(i,[4:7]), 'XYZ');
end

motion = [time', motion1(:,[1:6])];


Md = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
invMd = inv(Md);

%% Find the initial conditions
ik = inverseKinematics('RigidBodyTree',robot); % numerical solution

randConfig =[0.2 0.3 0.3 0 0 0]';
initialguess = robot.homeConfiguration;
t = zeros(4,4);
t(1:3,1:3) = eul2rotm(motion(1,[5:7]),'ZYX');
t(1:3,4) = motion(1,[2:4]);
t(4,4) = 1;
[config, sol] = ik('tool0', t, ik_weights, initialguess); 
config;



