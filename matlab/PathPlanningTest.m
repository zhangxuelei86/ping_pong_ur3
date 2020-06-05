%% PathPlanning Test Script
% This script is used to test the if a path to hit the ball with an
% incoming velocity at a specific 

%% Robot and Planner objects
    close all;
    robot = PingPongRobot;
    robot.model.base = transl(0.4,0,0) * robot.model.base * trotx(pi/2) * troty(pi/2);
    robot.PlotAndColourRobot();
    axis equal;
    hold on;
    qHome = [-0.4000         0   -1.0396    1.7544   -2.2689   -1.5708         0];
    robot.model.animate(qHome);
    
    planner = PathPlanner(robot);
    
%% Order of PathPlanner Class

tr = robot.model.fkine(robot.model.getpos);
planner.SetTargetInfo(tr(1:3,4)'+[-0.4,-0.075,0],[-0.2 -2 0],2); % ball position at intersection, velocity and total time

%% Compute the ball return action path 

path  = planner.BallReturnPath();

%% Visualize robot follow the calculated ball return path
for i = 1:size(path,1)
    robot.model.animate(path(i,:));
    drawnow();
end

%%  Compute path to return robot home
 path = planner.FinalJointStatePath(qHome);

 %% Visualize robot follow the calculated ball return path
for i = 1:size(path,1)
    robot.model.animate(path(i,:));
    drawnow();
end