%% This script is to test jogging of the robot with game controller
function GameControllerTest
    %% Step 1: Initialise the robot
    clear all; close all;
    clc;
    ppr = PingPongRobot();
    ppr.PlotAndColourRobot();
    hold on;

    rosMasterURI = 'http://192.168.56.101:11311'; % default Ubuntu PC local IP
    rosIP = '192.168.56.1'; % default Windows PC (MATLAB) local IP
    rosRW = ROSRobotWrapper(ppr, rosMasterURI, rosIP);
    ps4ControllerID = 1;
    ps4Contoller = GameController(ps4ControllerID);

    %% Start updating origin pose and joint angles
    rosRW.startRobotUpdate();

    %% E-Stop
    rosRW.eStopRobot(true);

    %% Turn off E-Stop
    rosRW.eStopRobot(false);

    %% TESTING JOGGING WITH GAME CONTROLLER
    while (1)
        rosRW.updateRobot();
        endEffVel = ps4Contoller.GetContollerCommands();
        qVel = LivePBVSWrapper.GetQDotFromEEVel(rosRW.robot,endEffVel);
        rosRW.jogRobot(qVel);
    end

    %% Stop updating origin pose and joint angles
    rosRW.stopRobotUpdate();

    %% Shutdown ROS
    rosshutdown;

end