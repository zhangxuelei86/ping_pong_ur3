%% ROSBallReturn Test Script
% This script is used to test the path planner class with Unity simulation

%% Robot and Planner objects
close all; clc;
ppr = PingPongRobot;
ppr.PlotAndColourRobot();
axis equal;
hold on;
qHome = [-0.4000         0   -1.0396    1.7544   -2.2689   -1.5708         0]; 
planner = PathPlanner(ppr);

rosMasterURI = 'http://172.19.126.33:11311'; % default Ubuntu PC local IP
rosIP = '172.19.119.56'; % default Windows PC (MATLAB) local IP
rosRW = ROSRobotWrapper(ppr, rosMasterURI, rosIP);

%% Initialise the ROS Trajectory Publisher object
steps = 100; % important
pathTime = 2.0;
deltaT = pathTime/steps;
rosTP = ROSTrajectoryPublisher();
rosTP.InitPublisher(steps);

%% Updates joint angles
rosRW.updateRobot();

%% Order of PathPlanner Class
tr = ppr.model.fkine(ppr.model.getpos);
planner.SetTargetInfo(tr(1:3,4)'+[0.4,-0.075,0],[0.01 -0.02 0],pathTime); % ball position at intersection, velocity and total time

%% Compute the ball return action path 
path  = planner.BallReturnPath();
velMatrix = zeros(steps,7);
for i = 1:steps-1
    velMatrix(i,:) = (path(i+1,:) - path(i,:))/deltaT;
end

%% Sends trajectory to robot
rosTP.SendTrajectory(path, velMatrix, deltaT);

%%  Compute path to return robot home
path = planner.FinalJointStatePath(qHome);
velMatrix = zeros(steps,7);
for i = 1:steps-1
    velMatrix(i,:) = (path(i+1,:) - path(i,:))/deltaT;
end

%% Sends trajectory to robot
rosTP.SendTrajectory(path, velMatrix, deltaT);