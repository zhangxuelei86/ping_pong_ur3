%% PathPlanning Test Script
% This script is used to test the if a path to hit the ball with an
% incoming velocity at a specific 

%% Robot and Planner objects
    close all;
    robot = PingPongRobot;
    robot.PlotAndColourRobot();
    hold on;
    robot.model.animate([-0.4 0 -pi/4 pi/2 -pi/4 0 0]);
    
    planner = PathPlanner(robot);
    
%% New Order of PathPlanner Class

tr = robot.model.fkine(robot.model.getpos);
planner.SetTargetInfo(transl(0,0,0)*tr,[-0.2 -1 0],2);
%%  
    steps = 25;
    tr = robot.model.fkine(robot.model.getpos);
    cartTraj = planner.GenerateCartersianTrajectory(tr(1:3,4)'+[0.5 -0.1 0.1], [-0.2 -1 0]);
    rot = zeros(3,steps);
    for i = 1:steps
        rot(:,i) = (tr2rpy(tr))';
    end
    cartTraj = [cartTraj; rot];
    
%%
    planner.currentJointState = robot.model.getpos;
    path = planner.RMRCTrajectory(2, cartTraj);
    
    planner.SetTargetPose(robot.model.fkine(path(1,:)),robot.model.getpos);
    Ikpath = planner.IKTrajectory();
    
    path = [Ikpath; path];
    
    %%
for i = 1:size(path(:,1))
    robot.model.animate(path(i,:));
%     robot.model.fkine(robot.model.getpos);
    drawnow();
end
