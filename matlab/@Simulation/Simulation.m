classdef Simulation < handle
    %SIMULATION The big class that runs everything and communicates with
    %the GUI for user input
    
    properties
        rosMasterURI;
        rosIP;
        
        ppr;
        rosRW;
        pbvsRW;
        rosTP;
        rosSW;
        planner;
        obsProc;
        pathChkr;
        
        lightCtn;
        isLightCtnTest;
        
        sheep;
        sheepDefaultPose;
        
        gameController;
        joggingVel;
        
        initialised;
        
        trajSteps;
        velMatrix;
        homingTime;

        playing;
        unityRobotQ;
        currentTraj;
        trajCalculated;
        normalTrajTime;
    end
    
    methods
        function self = Simulation()
            %SIMULATION Construct an instance of this class
            %   Initialise some objects that can be initialised
            self.initialised = false;
            self.trajSteps = 100;
            self.homingTime = 1.0;
            self.normalTrajTime = 2.0;
            self.velMatrix = zeros(self.trajSteps,7);
            self.currentTraj = [];
            self.trajCalculated = false;
            
            % sheep poses for Light curtain test
            self.sheepDefaultPose = transl(0,1.5,0) * trotz(-pi/2);
            self.isLightCtnTest = false;
            
            self.ppr = PingPongRobot();
            
            self.planner = PathPlanner(self.ppr);
            self.planner.SetFixedTimeOffset(0.4);
            disp("Created Simulation");
        end
        
        function init(self, rosMasterURI, rosIP)
            %INIT Initialise the rest of the objects with IP addresses
            %   You MUST call this function before running anything else.
            %   It might take a while (~ 15 seconds), so it might freeze
            %   the GUI a bit.
            self.initialised = false;
            self.playing = false;

            self.rosMasterURI = rosMasterURI;
            self.rosIP = rosIP;
            
            self.ppr.PlotAndColourRobot(); hold on;
            self.unityRobotQ = self.ppr.model.getpos();
            
            disp("... Initialising ROSRobotWrapper");
            self.rosRW = ROSRobotWrapper(self.ppr, self.rosMasterURI, self.rosIP);
            disp("... Initialising LivePBVSWrapper");
            self.pbvsRW = LivePBVSWrapper(self.rosRW);
            self.pbvsRW.init(0.03, [480 360]);
            self.pbvsRW.enableROSUpdate(true);
            disp("... Initialising ROSTrajectoryPublisher");
            self.rosTP = ROSTrajectoryPublisher();
            self.rosTP.InitPublisher(self.trajSteps);
            disp("... Initialising ROSSimWrapper");
            self.rosSW = ROSSimWrapper(self.ppr);
            disp("... Initialising ObstaclesProcessor - Static");
            self.rosSW.updateObstacles('static');
            staticObstacles = self.rosSW.getObstacles('static');
            self.obsProc = ObstaclesProcessor(staticObstacles);
            disp("... Initialising PathChecker");
            self.pathChkr = PathChecker(self.ppr, self.obsProc);
            
            self.initialised = true;
            self.rosRW.updateRobot();
            self.SetupLightCurtain();
            self.SpawnSheep();
            disp("Finished Simulation Initialisation");
        end
        
        function status = isInitialised(self)
            %ISINITIALISED Check if the simulation is properly initialised
            status = self.initialised; 
        end
        
        function status = isPlaying(self)
            %ISPLAYING Check if the robot is playing ping pong
            status = self.playing;
        end
        
        function eStop(self)
            %ESTOP E-STOP the simulation
            %   Note: You will need to close the figure and re-initialise
            self.rosRW.eStopRobot(true);
            self.stopPlaying();
            self.initialised = false;
            disp("ROBOT E-STOPPED - PLEASE RE-INITIALISE");
        end
        
        function turnOnLightCurtain(self)
            if ~self.isLightCtnTest
                self.isLightCtnTest = true;
                self.lightCtn.PlotSourceRay();
            end
        end
        
        function addController(self, id)
            try
                self.gameController = GameController(id);
            catch
                disp("Cannot connect to controller");
            end
        end
        
        function jogUnityRobotJoint(self, qdot)
            %JOGUNITYROBOTJOINT Jogs the robot by joint velocities
            self.rosRW.jogRobot(qdot);
            self.rosRW.updateRobot();
            self.unityRobotQ = self.ppr.model.getpos();
            drawnow();
        end
        
        function jogUnityRobotEE(self, endEffVel)
            %JOGUNITYROBOTEE Jogs the robot by end-effector velocity
            self.rosRW.updateRobot();
            qdot = LivePBVSWrapper.GetQDotFromEEVel(self.ppr, endEffVel);
            self.rosRW.jogRobot(qdot);
            self.rosRW.updateRobot();
            self.unityRobotQ = self.ppr.model.getpos();
            drawnow();
        end
        
        function jogUnityRobotController(self)
            %JOGUNITYROBOTCONTROLLER Jogs the robot by controller input
            try
                endEffVel = self.gameController.GetContollerCommands();
            catch
                disp("Game Controller not connected");
            end
            self.jogUnityRobotEE(endEffVel);
        end
        
        function eePose = setMATLABRobotQ(self, q)
            %SETMATLABROBOTQ Gets the ee pose from input joint config
            %   It will also:
            %   - Stops the robot from playing ping pong
            %   - Animate (snaps) the robot to the set joint config
            self.stopPlaying();
            self.trajCalculated = false;
            eePose = self.ppr.model.fkine(q);
            self.ppr.model.animate(q);
        end
        
        function q = setMATLABRobotEE(self, eePose)
            %SETMATLABROBOTEE Gets the joint config from input ee pose
            %   It will also:
            %   - Stops the robot from playing ping pong
            %   - Animate (snaps) the robot to the calculated joint config
            self.stopPlaying();
            self.trajCalculated = false;
            q = self.ppr.model.ikcon(eePose, self.unityRobotQ);
            self.ppr.model.animate(q);
        end
        
        function calculateAndPreview(self, qGoal, isQuintic)
            %CALCULATEANDPREVIEW Calculates the joint trajectory, then
            %plays a MATLAB-only animation.
            %   You need to call this function before moving the Unity
            %   Robot in Teach mode
            self.ppr.model.animate(self.unityRobotQ);
            self.currentTraj = self.planner.FinalJointStatePath(qGoal, isQuintic);
            self.trajCalculated = true;
            deltaT = self.normalTrajTime/self.trajSteps;
            for i = 1:self.trajSteps
                self.ppr.model.animate(self.currentTraj(i,:));
                pause(deltaT);
                drawnow();
            end
        end
        
        function success = moveUnityRobot(self)
            %MOVEUNITYROBOT Moves the Unity Robot after calculating and
            %previewing the trajectory
            %   This function will block for ~2.0 seconds while the Unity
            %   Robot is moving
            if ~self.trajCalculated
                success = false;
                disp("You need to calculate and preview motion first!");
                return
            end
            self.sendRobotPath(self.currentTraj, self.normalTrajTime);
            self.rosRW.updateRobot();
            while self.rosRW.getCurrentTrajectoryIndex() < 100
                self.rosRW.updateRobot();
                drawnow();
            end
            self.unityRobotQ = self.ppr.model.getpos();
            success = true;
        end
        
        function q = getUnityRobotQ(self)
            self.rosRW.updateRobot();
            self.unityRobotQ = self.ppr.model.getpos();
            q = self.unityRobotQ;
        end
        
        function homeRobot(self)
            %HOMEROBOT Homes the robot
            %   This function is NOT blocking, and it is used in playing
            self.currentTraj = self.planner.FinalJointStatePath(self.ppr.qHome);
            self.sendRobotPath(self.currentTraj, self.homingTime);
        end

        function sendRobotPath(self, path, totalTime)
            %SENDROBOTPATH Sends a path to the Unity Robot
            %   This function is used by many other functions
            if ~self.initialised
                return
            end
            if ~self.checkCollisions(1) % if a collision can happen
                return % do not send the traj to robot
            end
            self.rosRW.eStopRobot(false);
            deltaT = totalTime / self.trajSteps;
            for i = 1:self.trajSteps-1
                self.velMatrix(i,:) = (path(i+1,:) - path(i,:))/deltaT;
            end
            self.rosTP.SendTrajectory(path, self.velMatrix, deltaT);
        end
        
        function stopPlaying(self)
            %STOPPLAYING Pauses the playing mode
            %   This clears a flag that will stop the while loop that runs
            %   in startPlaying()
            if self.playing
                self.rosRW.eStopRobot(true);
                self.playing = false;
                pause(0.2);
            end
        end
        
        function result = checkCollisions(self, index)
            % Collisions checking
            self.pathChkr.SetCurrentPath(self.currentTraj);
            self.pathChkr.SetPathIndex(index);
            result = self.pathChkr.CheckPath(3);
            if ~result
                self.rosRW.eStopRobot(true);
                self.currentTraj = [];
                disp("Potential collision detected at index #" + num2str(index));
            end
        end

        function startPlaying(self)
            %STARTPLAYING Main loop that plays ping pong
            %   Also updates obstacles, ball, and anything necessary
            if ~self.initialised
                return
            end
            self.playing = true;
            
            % flags used for stages of ping pong playing
            hittingBall = false;
            homed = true;
            hitTime = now;
            
            % in case the robot could not hit the ball, wait for this time
            % before homing
            waitTime = 2.0 / 100000; % 3 seconds
            
            self.rosRW.eStopRobot(false);
            self.currentTraj = [];
            while(self.playing)
                self.rosRW.updateRobot();
                self.unityRobotQ = self.ppr.model.getpos();
                
                % If QR code is in camera view, and robot is under PBVS
                % control, ignore ping pong playing and continue;
                self.pbvsRW.robotPBVSControl();
                if self.pbvsRW.isQRInView()
                   drawnow();
                   continue;
                end
                
                % Update the sheep model based on Light curtain test mode
                lightCtnBreach = false;
                if self.isLightCtnTest
                    lightCtnBreach = self.lightCtn.CheckBreach(self.sheep.pose(1:3,4)');
                end
                if lightCtnBreach
                    self.rosRW.eStopRobot(true);
                    continue;
                end
                
                % Updates the ball and dynamic obstacles
                self.rosSW.updateBall();
                self.rosSW.updateObstacles('dynamic');
                dynamicObstacles = self.rosSW.getObstacles('dynamic');
                self.obsProc.UpdateDynamicObstacles(dynamicObstacles);
                
                index = self.rosRW.getCurrentTrajectoryIndex();
                if index == 100 % if finished current trajectory
                    self.currentTraj = [];
                end
                % if trajectory is finished, no need for checking
                if ~isempty(self.currentTraj)
                    % Collisions checking
                    self.checkCollisions(index);
                end
                
                drawnow();
                
                if self.rosSW.getBallState() == 1 % ball left player paddle
                    % tries to return the ball
                    if ~hittingBall
                        [position, velocity, time, success] = self.rosSW.findInterceptPoint();
                        if success % if not, ball is out of reach
                            self.planner.SetTargetInfo(position, velocity, time);
                            [self.currentTraj, time] = self.planner.BallReturnPath();
                            self.sendRobotPath(self.currentTraj, time);
                            hittingBall = true;
                            homed = false;
                            hitTime = now;
                        end
                    end
                else
                    % state 3: ball has left robot paddle
                    if self.rosSW.getBallState() == 3 || (now-hitTime) > waitTime
                        if ~homed
                            self.homeRobot();
                            homed = true;
                        end
                    end
                    hittingBall = false;
                end
            end
        end
        
        function SpawnSheep(self)
            sheepPlyPath = "@EnvironmentComponent/environment_models/sheep.ply";
            self.sheep = EnvironmentComponent(sheepPlyPath,self.sheepDefaultPose);
        end
        
        function UpdateSheep(self, pose)
            self.sheep.UpdatePose(pose);
        end
        
        function SetupLightCurtain(self)
            base = self.ppr.model.base();
            sources(1) = struct('origin',[base(1,4)+0.35,base(2,4),0.3],'angDiff',5,'yaw',0);
            sources(2) = struct('origin',[base(1,4)-1.35,base(2,4),0.3],'angDiff',5,'yaw',180);
            sources(3) = struct('origin',[base(1,4)-0.4,base(2,4)+0.3,0.3],'angDiff',5,'yaw',90);
            sources(4) = struct('origin',[base(1,4)-0.4,base(2,4)-0.3,0.3],'angDiff',5,'yaw',270);
            self.lightCtn = LightCurtain(sources);
        end
    end
end

