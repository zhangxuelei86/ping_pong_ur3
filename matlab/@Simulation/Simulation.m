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
        
        initialised;
        
        trajSteps;
        velMatrix;
        homingTime;

        playing;
        unityRobotQ;
        predictedTraj;
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
            self.predictedTraj = zeros(self.trajSteps,7);
            self.trajCalculated = false;
            
            self.ppr = PingPongRobot();
            self.unityRobotQ = self.ppr.model.getpos();
            
            planner = PathPlanner(ppr);
            planner.SetFixedTimeOffset(0.4);
            disp("Created Simulation");
        end
        
        function init(self, rosMasterURI, rosIP)
            %INIT Initialise the rest of the objects with IP addresses
            %   You MUST call this function before running anything else.
            %   It might take a while (~ 10 seconds), so it might freeze
            %   the GUI a bit.
            self.initialised = false;
            self.playing = false;

            self.rosMasterURI = rosMasterURI;
            self.rosIP = rosIP;
            
            self.ppr.PlotAndColourRobot();
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
        
        function eePose = setMATLABRobotQ(self, q)
            %SETMATLABROBOTQ Gets the ee pose from input joint config
            %   It will also:
            %   - Stops the robot from playing ping pong
            %   - Animate (snaps) the robot to the set joint config
            self.stopPlaying(); pause(0.2);
            self.trajCalculated = false;
            eePose = self.ppr.model.fkine(q);
            self.ppr.model.animate(q);
        end
        
        function q = setMATLABRobotEE(self, eePose)
            %SETMATLABROBOTEE Gets the joint config from input ee pose
            %   It will also:
            %   - Stops the robot from playing ping pong
            %   - Animate (snaps) the robot to the calculated joint config
            self.stopPlaying(); pause(0.2);
            self.trajCalculated = false;
            q = self.ppr.model.ikcon(eePose, self.unityRobotQ);
            self.ppr.model.animate(q);
        end
        
        function calculateAndPreview(self, qGoal, isQuintic)
            %CALCULATEANDPREVIEW Calculates the joint trajectory, then
            %plays a MATLAB-only animation.
            %   You need to call this function before moving the Unity
            %   Robot in Teach mode
            self.predictedTraj = self.planner.FinalJointStatePath(qGoal, isQuintic);
            self.trajCalculated = true;
            for i = 1:self.trajSteps
                self.ppr.model.animate(self.predictedTraj(i,:));
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
            self.sendRobotPath(self.predictedTraj, self.normalTrajTime);
            self.rosRW.updateRobot();
            while self.rosRW.getCurrentTrajectoryIndex() < 100
                self.rosRW.updateRobot();
                drawnow();
            end
            self.unityRobotQ = self.ppr.model.getpos();
        end
        
        function homeRobot(self)
            %HOMEROBOT Homes the robot
            %   This function is NOT blocking, and it is used in playing
            path = self.planner.FinalJointStatePath(self.ppr.qHome);
            self.sendRobotPath(path, self.homingTime);
        end

        function sendRobotPath(self, path, totalTime)
            %SENDROBOTPATH Sends a path to the Unity Robot
            %   This function is used by many other functions
            if ~self.initialised
                return
            end
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
            self.playing = false;
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
            waitTime = 3.0 / 100000; % 3 seconds
            
            self.rosRW.eStopRobot(false);
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
                
                % Updates the ball and dynamic obstacles
                self.rosSW.updateBall();
                self.rosSW.updateObstacles('dynamic');
                dynamicObstacles = self.rosSW.getObstacles('dynamic');
                self.obsProc.UpdateDynamicObstacles(dynamicObstacles);
                
                drawnow();
                
                if self.rosSW.getBallState() == 1 % ball left player paddle
                    % tries to return the ball
                    if ~hittingBall
                        [position, velocity, time, success] = self.rosSW.findInterceptPoint();
                        if success % if not, ball is out of reach
                            self.planner.SetTargetInfo(position, velocity, time);
                            [path, time] = self.planner.BallReturnPath();
                            self.sendRobotPath(path, time);
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
        
        
        
    end
end

