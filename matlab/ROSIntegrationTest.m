%% This script is used to test components of the system
% @Victor feel free to use this as an example of how to use my classes

%% Step 1: Initialise the robot
clear all; close all;
clc;
ppr = PingPongRobot();
ppr.PlotAndColourRobot();

rosMasterURI = 'http://192.168.56.101:11311'; % default Ubuntu PC local IP
rosIP = '192.168.56.1'; % default Windows PC (MATLAB) local IP
rosRW = ROSRobotWrapper(ppr, rosMasterURI, rosIP);

%% Step 2: Initialise the JointTrajectoryPublisher
% This takes ~ 5 seconds
steps = 200; % important
deltaT = 0.02;
rosTP = ROSTrajectoryPublisher();
rosTP.InitPublisher(steps);

%% Step 3: Start updating origin pose and joint angles
rosRW.startRobotUpdate();

%% Step 4: Calculate a test qMatrix and velMatrix
qBegin = ppr.model.getpos();
qEnd = qBegin;
qEnd(2:7) = qBegin(2:7) - pi/3; % just the arm, without rail
qMatrix = jtraj(qBegin,qEnd,steps);
velMatrix = zeros(steps,7);
for i = 1:steps-1
    velMatrix(i,:) = (qMatrix(i+1,:) - qMatrix(i,:))/deltaT;
end

%% (Optional) Tests trajectory
rosRW.stopRobotUpdate(); pause(0.2);
for i = 1:steps
   ppr.model.animate(qMatrix(i,:));
   drawnow();
end

%% Step 5: Sends trajectory to robot
% You can also get return value for 'success'
rosRW.startRobotUpdate(); pause(0.2);
rosTP.SendTrajectory(qMatrix, velMatrix, deltaT);

%% (Optional) E-Stop
rosRW.eStopRobot(true);

%% (Optional) Turn off E-Stop
rosRW.eStopRobot(false);

%% (Optional) Comparing joint angles after
qEnd
qFinal = ppr.model.getpos()

%% Step 6: Stop updating origin pose and joint angles
rosRW.stopRobotUpdate();

%% Shutdown ROS
rosshutdown;