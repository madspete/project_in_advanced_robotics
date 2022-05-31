clc
clear
close all
%% Project (Initial try at learning by demonstration)

% The demos in which this is taken from is based on this link
% https://www.epfl.ch/labs/lasa/older-source-codes/
 addpath('./lfd_functions/');

% Maybe this link could be useful aswell
% http://robohow.org/_media/meetings/second-integration-workshop/khansari_seds_presentation_robohow.pdf_

% Load DataT (in joint space) and DataX (in task space) (remember to update filepath)
load('/home/mads/git/project_in_advanced_robotics/lfd_matlab/lfd_functions/data/data1.mat');
nbVarX = size(DataX,1); 
nbVarT = size(DataT,1);
n_joints = 2;
gm_data = DataT';

figure("Name", "Theta trajectories")
% Theta 1
%subplot(2,1,1)
plot(gm_data(1:1000,1), gm_data(1:1000,2))
hold on 
plot(gm_data(1001:2000,1), gm_data(1001:2000,2))
plot(gm_data(2001:3000,1), gm_data(2001:3000,2))
hold off
xlabel("t")
ylabel("theta 1 [rad]")
title("Input trajectories")

% Theta 2
%subplot(2,1,2)
%plot(gm_data(1:1000,1), gm_data(1:1000,3))
%hold on 
%plot(gm_data(1001:2000,1), gm_data(1001:2000,3))
%plot(gm_data(2001:3000,1), gm_data(2001:3000,3))
%hold off
%xlabel("t")
%ylabel("theta 2 [rad]")


% Maybe it would be better to use their implementation, but not sure
K=6;
obj2 = fitgmdist(gm_data,K,'Start','plus','Replicates',100);
obj2.mu
figure("Name", "Gaussian Mixture Model")
%subplot(2,1,1); 
hold on;
% Specific joint angles would be 6 different plots in case of a UR robot
plotGMM(obj2.mu(:,[1,2])', obj2.Sigma([1,2], [1,2],:), [.8 0 0], 1);
hold off;
xlabel("t")
ylabel("theta 1 [rad]")
title("Gaussian Mixture Model")
%subplot(2,1,2); hold on;
%plotGMM(obj2.mu(:,[1,3])', obj2.Sigma([1,3],[1,3],:), [.8 0 0], 1);
%hold off;
%xlabel("t")
%ylabel("theta 2 [rad]")

input = [1:1:1000];
% This is needed in order to get a good plot of it
expT(1,:) = linspace(min(DataT(1,:)),max(DataT(1,:)),1000);
[expT(2:n_joints+1,:), expSigma] = GMR(obj2.PComponents, obj2.mu', obj2.Sigma, input, [1], [2:n_joints+1]);

figure("Name", "Gaussian Mixture Regression")
%subplot(2,1,1); 
hold on;
plotGMM(expT([1,2],:), expSigma(1,1,:), [.8 0 0], 3);
hold off;
xlabel("t")
ylabel("theta 1 [rad]")
title("Gaussian Mixture Regression")
%subplot(2,1,2); hold on;
%plotGMM(expT([1,3],:), expSigma(2,2,:), [.8 0 0], 3);
%hold off; 
%xlabel("t")
%ylabel("theta 2 [rad]")
