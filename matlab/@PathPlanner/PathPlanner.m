%% PathPlanner
% This class plans the path for the robot to follow with collision
% avoidance by calculating inverse kinematics or RMRC
classdef PathPlanner < handle
    properties
        path;
                
        RMRCCartTraj_h;
    end
    
    properties (SetAccess = private)
        robot;
        
        targetPose;
        targetVel;
        totalTime;
        
        fixedTimeOffset = 0;
        
        stepsTotal = 100;
        deltaT;
    end
    
    methods
        function self = PathPlanner(robot)
            
            self.robot = robot;
            
        end
        
        function SetTotalSteps(self, steps)
            
            self.stepsTotal = steps;
            
        end

        function SetFixedTimeOffset (self, timeOffset)
            
            self.fixedTimeOffset = timeOffset;
            
        end
        
        function SetTargetInfo (self, targetPose, targetVel, timeToTarget)
            
            self.targetPose = targetPose;
            self.targetVel = targetVel;
            self.totalTime = (timeToTarget - self.fixedTimeOffset)/0.88;
            self.deltaT = self.totalTime / self.stepsTotal;
            
        end
        
        function trajectory = IKTrajectory(self, trGoal, isQuintic)
            if nargin < 3
                isQuintic = false;
            end
            
            currentJointState = self.robot.model.getpos();
            qGoal = self.robot.model.ikcon(trGoal, currentJointState);
            steps = 0.76 * self.stepsTotal;
            
            if isQuintic
                trajectory = jtraj(currentJointState, qGoal, steps);
            else
                trajectory = self.TrapezoidalVelocityProfile(currentJointState, qGoal, steps);
            end
            
        end
        
        %% RMRCTrajectory
        % Function computes a joint state trajectory for the PPR based on
        % time required to hit the ping pong ball
        function trajectory = RMRCTrajectory(self, cartesianTrajectory)
                        
            steps = 0.24*self.stepsTotal;
            epsilon = 0.1;
            W = diag([1 1 1 0.1 0.1 0.1]);
            
            manipltyMatrix = zeros(steps,1); % Manipulability
            qMatrix = zeros(steps,7); % qMatrix of trajectory
            qdot = zeros(steps,7); % joint velocities vector
            x = zeros(3,steps);
            theta = zeros(3,steps);
            
            % On the condition the steps in the catesian trajectory is not equal to the steps of the RMRC
            if size(cartesianTrajectory(1,:)) ~= steps
                x1 = cartesianTrajectory(1,1:3);
                x2 = cartesianTrajectory(end,1:3);

                s = lspb(0,1,steps);                                 % Interpolation scalar
                for i = 1:steps
                    x(:,i) = x1*(1-s(i)) + s(i)*x2;
                    theta(:,i) = cartesianTrajectory(end, 4:6);
                end
            else
                x = cartesianTrajectory(1:3,:);
                theta = cartesianTrajectory(4:6,:);
            end
                        
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Traform of cartesian trajectory
            q0 = self.robot.model.getpos();                                                 
            qMatrix(1,:) = self.robot.model.ikcon(T,q0);                                

            for i = 1:steps-1
                T = self.robot.model.fkine(qMatrix(i,:));                 % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);            % Get position error from next waypoint
                Rd = rpy2r(theta(:,i+1)');      % Get next RPY angles, convert to rotation matrix
                Ra = tr2rt(T);                % Current end-effector rotation matrix

                Rdot = (1/self.deltaT)*(Rd - Ra);       % Calculate rotation matrix error (see RMRC lectures)
                S = Rdot * Ra';                      % Skew symmetric! S(\omega)
                linearVelocity = (1/self.deltaT)*deltaX;
                angularVelocity = [S(3,2);S(1,3);S(2,1)];

                xdot = W*[linearVelocity;angularVelocity];
                J = self.robot.model.jacob0(qMatrix(i,:));

                if self.robot.model.maniplty(qMatrix(i,:)) < epsilon
                    lambda = 0.01;
                    invJ = pinv(J'*J + lambda*eye(7))*J';
                else
                    invJ = pinv(J);
                end
                qdot(i,:) = invJ * xdot;
                
                for link = 1:length(self.robot.model.links)
                    q = qMatrix(i,link) + self.deltaT * qdot(i,:)';
                    if q(link) < self.robot.model.qlim(link,1)
                        qdot(i,link) = 0;
                    elseif q(link) > self.robot.model.qlim(link,2)
                        qdot(i,link) = 0;
                    end
                end
                
                qMatrix(i+1,:) = qMatrix(i,:) + self.deltaT * qdot(i,:);
                manipltyMatrix(i) = self.robot.model.maniplty(qMatrix(i,:));  % Populate manipulability matrix
            end
            
            trajectory = qMatrix;

        end
        
        %% GenerateCartesianTrajectory
        % This input of this function is the ball pose, ball vellocity,
        % approach peak velocity scalar for the hit motion, and number of
        % steps to be computed
        % Based on whether the ball position is reachable the function then outputs the generated hit trajectory for the
        % robot and the flag what states if the ball is reachable or not
        function [cartTraj, isReachable] = GenerateCartersianTrajectory(self)
            
            % The cartesian trjectory generator has the same number of
            % steps as the RMRC trajectory
            steps = 0.24 * self.stepsTotal;
            
            ballPose = self.targetPose(1:3,4)';
            ballVelocity = self.targetVel;
            ballPoseInRobotBaseTr = homtrans(self.robot.model.base, ballPose');
                        
            % Check if the norm of the velocity vector is greater than the
            % set threshold
            % else set the normal to the cieling value of the threshold
            if norm(ballVelocity) > 10
                normBallVel = 10;
            else
                normBallVel = norm(ballVelocity);
            end
            
            % Create memory allocation for the generated cartesian
            % trajectory
            x = zeros(3,steps);
            theta = zeros(3,steps);
            
            % Check if the ball position lies within the workspace of the
            % robot
            % It the ball position lies inside the workspace of the robot,
            % the start and end position of the hit motion is calculated
            % based on the incoming velocity of the ball (speed and direction)
            % A cartesian trajectory of the hit motion is then computed using the trapezoidal
            % velocity profile
            if self.robot.CheckPointsInRobotWorkspace(ballPose) && ballPoseInRobotBaseTr(2)>=0
            
                startPt = ballPose + ballVelocity/normBallVel*(1-normBallVel/10)*0.1;
                endPt = ballPose - ballVelocity/normBallVel*(1-normBallVel/10)*0.1;

                s = lspb(0,1,steps);                                 % Create interpolation scalar
                for i = 1:steps
                    x(1:2,i) = startPt(1:2)*(1-s(i)) + s(i)*endPt(1:2);
                    x(3,i) = ballPose(3);
                end
                
                tr = self.robot.model.fkine(self.robot.model.getpos);
                for i = 1:steps
                    theta(:,i) = (tr2rpy(tr))';
                end
                
                % The cartesian trajectory computed is plotted in the 3D
                % workspace for visalisation
                try delete(self.RMRCCartTraj_h); end
                
                self.RMRCCartTraj_h = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1);

                isReachable = true;
                cartTraj = [x ; theta];
                
            else
                isReachable = false;
            end
            
        end
        
        %% BallReturnPath
        %
        function path = BallReturnPath(self)
            
            [cartTraj, isReachable] = self.GenerateCartersianTrajectory();
            
            if isReachable
                
                RMRCTraj = self.RMRCTrajectory(cartTraj);
                IKTraj = self.IKTrajectory(self.robot.model.fkine(RMRCTraj(1,:)), true);
                
                path = [IKTraj; RMRCTraj];
                self.path = path;
                
            else
                disp("Ball pose is UNREACHABLE");
            end
        end
        
        %% FinalJointStatePath
        %
        function path = FinalJointStatePath(self, qGoal, isQuintic)
            if nargin < 3
                isQuintic = false;
            end
            
            if isQuintic
                path = jtraj(currentJointState, qGoal, steps);
            else
                path = self.TrapezoidalVelocityProfile(currentJointState, qGoal, steps);
            end
            
            self.path = path;
            
        end
        
        %% GetPath
        %
        function path = GetPath(self)
            
            path = self.path;
            
        end
        
    end
    
    methods (Static)
        
        %% TrapezoidalVelocityProfile
        % Function takes in the intial joints state of the 7DOF robot (PPR)
        % final joint state and number of steps, then computes a trapezoidal velocity profile
        function qMatrix = TrapezoidalVelocityProfile(q1, q2, steps)
            s = lspb(0,1,steps);
            qMatrix = nan(steps,7);
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
            end
        end
        
    end
    
end