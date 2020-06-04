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
    robot.model.animate([-0.4000         0   -1.0396    1.7544   -2.2689   -1.5708         0]);
    
    planner = PathPlanner(robot);
    
%% New Order of PathPlanner Class

tr = robot.model.fkine(robot.model.getpos);
planner.SetTargetInfo(tr(1:3,4)'+[-0.4,-0.075,0],[-0.2 -2 0],2);

%%

path  = planner.BallReturnPath();
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
for i = 1:size(path,1)
    robot.model.animate(path(i,:));
%     robot.model.fkine(robot.model.getpos);
    drawnow();
end
