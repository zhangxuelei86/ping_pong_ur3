%% Ball Prediction Test Script
% This script is used to test the ball trajectory prediction algorithm

%% Starts ROS Wrapper
clear all; close all; clc;
rosMasterURI = 'http://10.42.0.1:11311'; % default Ubuntu PC local IP
rosIP = '10.42.0.123'; % default Windows PC (MATLAB) local IP
ppr = PingPongRobot();
ppr.PlotAndColourRobot();
rosRW = ROSRobotWrapper(ppr, rosMasterURI, rosIP);
hold on;

%% Trajectory Publisher
steps = 100;
rosTP = ROSTrajectoryPublisher();
rosTP.InitPublisher(steps);
velMatrix = zeros(steps,7);

%% Path Planner
planner = PathPlanner(ppr);
planner.SetFixedTimeOffset(0.6);

%% Simulation Wrapper (Ball)
rosSW = ROSSimWrapper(ppr);

% try delete(pre_h); end
% pre_h = plot3(positions(:,1)',positions(:,2)',positions(:,3)','k.','LineWidth',1);

%%
hittingBall = false;
homed = false;
while(1)
    rosRW.updateRobot();
    rosSW.updateBall();
    if rosSW.ballState == 1
        if ~hittingBall
            [position, velocity, time, success] = rosSW.findInterceptPoint();
            if success
                tic
                deltaT = time/steps;
                planner.SetTargetInfo(position, velocity, time);
                path = planner.BallReturnPath();
                for i = 1:steps-1
                    velMatrix(i,:) = (path(i+1,:) - path(i,:))/deltaT;
                end
                toc
                rosTP.SendTrajectory(path, velMatrix, deltaT);
                hittingBall = true;
            end
        end
    else
        hittingBall = false;
    end
end