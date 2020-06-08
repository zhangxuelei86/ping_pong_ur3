%% PathChecker Test
function PathCheckerTest
    %% Create Robot, PathPlanner, ObstaclesProcessor
    
%     close all;
%     clear
%     clc
    app = PingPongRobotApp;
    ax = app.UIAxes;
    hold(ax, "on");
%%     
%     p1 = [0 0 0];
%     p2 = [1 1 1];
%     plot3(ax,[p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],"r--");
    
    robot = PingPongRobot;
    robot.model.base = transl(0.4,0,0) * robot.model.base * trotx(pi/2) * troty(pi/2);
    robot.PlotAndColourRobot();
    
    robotAx = gca;
    robotAxChil = robotAx.Children;
    copyobj(robotAxChil, app.UIAxes);
%     axis equal;
%     hold on;
    qHome = [-0.4000         0   -1.0396    1.7544   -2.2689   -1.5708         0];
    robot.model.animate(qHome);
    
    pathPlanner = PathPlanner(robot);

    axis(ax, "equal");

    %% Create Obstacles
    
    obsSize = [0.25 0.25 0.25];
    obstacles = cell(10,1);
    
    hold on;
    for i = 1:size(obstacles,1)
        position = rand(1,3) .* randi([-1 1],1,3);
        obstacles{i} = Obstacle(obsSize,position);
    end
    
    %% Seperate static and dynamic obstacles
    
    staticObstacles = obstacles(1);
    dynamicObstacles = obstacles(8:10);
%     staticObstacles = obstacles(1);
%     dynamicObstacles = obstacles(8);
%     
    %% Create ObstaclesProcessor
    
    obsProc = ObstaclesProcessor(staticObstacles);
    obsProc.UpdateDynamicObstacles(dynamicObstacles);
    
    %% Create PathChecker
    
    pathCheck = PathChecker(robot, obsProc);
    
    %% compute path with PathPlanner Class

    tr = robot.model.fkine(robot.model.getpos);
    pathPlanner.SetTargetInfo(tr(1:3,4)'+[-0.4,-0.075,0],[-0.2 -2 0],2); % ball position at intersection, velocity and total time

    %% Compute the ball return action path 

    path  = pathPlanner.BallReturnPath();
    
    %% Check path based in selected index in the path
    pathCheck.SetCurrentPath(path);

     %% Visualize robot follow the calculated ball return path
     % Dynamic obstacle is also updated in the scene
%     count = 0;
    randNum = round(rand(1)*100);
    for i = 1:size(path,1)
        robot.model.animate(path(i,:));
        pathCheck.SetPathIndex(i-1);
        
        tr = robot.model.fkine(path(i,:));
        if (i == randNum)
            obs = Obstacle(obsSize, tr(1:3,4)'-[obsSize(1) 0 0]);
            obsProc.UpdateDynamicObstacles({obs});
        end
        
        tic
        nextJSOk = pathCheck.CheckPath();
        toc
        if ~nextJSOk
            disp("This joint state " + num2str(i) + " is in COLLISION with obstacles");
%             count = count + 1;
            break;
        end
%         keyboard;
        
        drawnow();
    end
    
%     disp("Total number of collisions: " + num2str(count));
end