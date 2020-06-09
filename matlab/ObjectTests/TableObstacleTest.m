function TableObstacleTest
%% Ping pong table as obstacle test
    close all;
    clear all;
    clc;

    %% Create Robot, PathPlanner, ObstaclesProcessor
    
    close all;
    clear
    clc
    robot = PingPongRobot;
    robot.model.base = transl(0.4,0,0) * robot.model.base * trotx(pi/2) * troty(pi/2);
    robot.model.base = transl(0, -1.65, 0.1) * robot.model.base;
    robot.PlotAndColourRobot();
    axis equal;
    hold on;
    qHome = [-0.4000         0   -1.0396    1.7544   -2.2689   -1.5708         0];
    robot.model.animate(qHome);
    
    %% Obstacle as table
%     PlaceObject("@EnvironmentComponent/environment_models/ping_pong_table.ply")
    
    filePath = "@EnvironmentComponent/environment_models/ping_pong_table.ply";
    table = EnvironmentComponent(filePath);
    table.UpdatePose(trotz(pi/2)*transl(0,0.175,0));
    axis equal;

    obsSize = [0.9441+0.9387, 1.3711+1.3689, 0.7770];
    position = [0 0 -obsSize(3)/2];
    
    obs = Obstacle(obsSize, position);
    obs.SetPlotTransparency(0.1);
    obs.PlotObstacle();

end