clc
clear
close all

%% Filter design 

%% Loaded from data exercise 7
data = importdata("data\forceData.mat");

figure("Name", "force data")
plot(data.Time, data.Data(:,1))
hold on
plot(data.Time, data.Data(:,2))
plot(data.Time, data.Data(:,3))
hold off

force_measurements = [data.Time, data.Data(:,1), data.Data(:,2), data.Data(:,3)];

%% Force estimated manually
desired_forcez = data.Time
desired_forcez([1:501]) = 0:0.04:20;
desired_forcez([502:length(data.Time)]) = 20;

% add noise to measurements
for i=1:length(desired_forcez)
    desired_forcez(i) = desired_forcez(i) + 2*randn(1);
end

force_artificial = [data.Time, desired_forcez];

figure("name", "Force artificial data")
plot(data.Time, desired_forcez)


