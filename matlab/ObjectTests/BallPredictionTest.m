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
homeDeltaT = 0.02;
rosTP = ROSTrajectoryPublisher();
rosTP.InitPublisher(steps);
velMatrix = zeros(steps,7);

%% Path Planner
planner = PathPlanner(ppr);
planner.SetFixedTimeOffset(0.7);

%% Simulation Wrapper (Ball)
rosSW = ROSSimWrapper(ppr);

%% ObstaclesProcessor
rosSW.updateObstacles('static');
staticObstacles = rosSW.getObstacles('static');
obsProc = ObstaclesProcessor(staticObstacles);

%% PathChecker
pathCheck = PathChecker(ppr, obsProc);

%%
hittingBall = false;
homed = true;
hitTime = now;
waitTime = 3.0;
while(1)
    rosRW.updateRobot();
    rosSW.updateBall();
    rosSW.updateObstacles('dynamic');
    
    dynamicObstacles = rosSW.getObstacles('dynamic');
    obsProc.UpdateDynamicObstacles(dynamicObstacles);
    
    if rosSW.ballState == 1
        if ~hittingBall
            [position, velocity, time, success] = rosSW.findInterceptPoint();
            if success
                deltaT = time/steps;
                planner.SetTargetInfo(position, velocity, time);
                path = planner.BallReturnPath();
                for i = 1:steps-1
                    velMatrix(i,:) = (path(i+1,:) - path(i,:))/deltaT;
                end
                rosTP.SendTrajectory(path, velMatrix, deltaT);
                hittingBall = true;
                homed = false;
                hitTime = now;
            end
        end
    else
        if rosSW.ballState == 3 || (now-hitTime)*100000 > waitTime
            if ~homed
                path = planner.FinalJointStatePath(ppr.qHome);
                for i = 1:steps-1
                    velMatrix(i,:) = (path(i+1,:) - path(i,:))/homeDeltaT;
                end
                rosTP.SendTrajectory(path, velMatrix, homeDeltaT);
                homed = true;
            end
        end
        hittingBall = false;
    end
end