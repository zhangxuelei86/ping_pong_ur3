%% This script is used to test components of the system
% @Victor feel free to use this as an example of how to use my classes

%% Initialise the robot
clear all; close all;
clc;
ppr = PingPongRobot();
ppr.PlotAndColourRobot();

rosMasterURI = 'http://172.19.127.190:11311'; % default Ubuntu PC local IP
rosIP = '172.19.119.56'; % default Windows PC (MATLAB) local IP
rosRW = ROSRobotWrapper(ppr, rosMasterURI, rosIP);

%% Initialise the JointTrajectoryPublisher
% This takes ~ 10 seconds
steps = 150; % important
deltaT = 0.02;
rosTP = ROSTrajectoryPublisher();
rosTP.InitPublisher(steps);

%% Start updating origin pose and joint angles
rosRW.startRobotUpdate();

%% Calculate a test qMatrix and velMatrix
qBegin = ppr.model.getpos();
qEnd(2:7) = qBegin(2:7) + 0.4;
qMatrix = jtraj(qBegin,qEnd,steps);
velMatrix = zeros(steps,7);
for i = 1:steps-1
    velMatrix(i,:) = (qMatrix(i+1,:) - qMatrix(i,:))/deltaT;
end

%% Tests trajectory
for i = 1:steps
   ppr.model.animate(qMatrix(i,:));
   drawnow();
end

%% Sends trajectory to robot
% You can also get return value for 'success'
rosTP.SendTrajectory(qMatrix, velMatrix, deltaT);

%% Stop updating origin pose and joint angles
rosRW.stopRobotUpdate();

%% Shutdown ROS
rosshutdown;