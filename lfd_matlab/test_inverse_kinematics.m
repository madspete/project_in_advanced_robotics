clc
clear
close all

%% Test inverse kinematics solver
robot = loadrobot('universalUR5','DataFormat','column','Gravity',[0 0 -9.82]);
showdetails(robot) % You can find the exact frame name here. 

ik_weights = [1 1 1 1 1 1]; % tolerence of erros: orientation and position 

ik = inverseKinematics('RigidBodyTree',robot); % numerical solution

randConfig =[0.2 0.3 0.3 0 0 0]';
initialguess = robot.homeConfiguration;
desired = robot.randomConfiguration;
t = zeros(4,4)
t(1:3,1:3) = eul2rotm(randConfig(4:6)','ZYX');
t(1:3,4) = randConfig(1:3);
t(4,4) = 1;
t
tform_input = getTransform(robot,randConfig,'tool0','base_link') % you were confused to define the frame, first one should be the child frame and second one should be the parent frame.
% Also you should start wiith base_link, not base. You can check showsdetails(robot).

[config, sol] = ik('tool0', t, ik_weights, initialguess); 
tform_output = getTransform(robot,config,'tool0','base_link')

% Compare two results in the figure
show(robot,randConfig);
hold on 
show(robot,config);

%tform_input == tform_output % This way is not recommended because the comparison should be '1' if the numbers of two frames are exactly same. 
error = tform_output-t % you can use this one to see what you want instead above. 

%The joint configuration might be different even if the tf information from base_link to tool0 are same each other. Because there are 8 solutions to reach the one tf.
