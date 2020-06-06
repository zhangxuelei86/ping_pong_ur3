%% PathPlanner
% This class plans the path for the robot to follow with collision
% avoidance by calculating inverse kinematics or RMRC
classdef PathPlanner < handle
    properties
        % path generated by the class
        path;
                
        % RMRC cartesian trajectory
        RMRCCartTraj_h;
    end
    
    properties (SetAccess = private)
        % robot (PPR) for which a trajectory is generated
        robot;
        
        % target information (i.e. pingpong ball) including position at
        % intersection, velocity, and time taken to get to intesectioin
        % point
        targetPosition;
        targetVel;
        totalTime;
        
        % fixedTimeOffset due to time taken by trajectory prediction and
        % control command
        fixedTimeOffset = 0;
        
        % total steps for each path of joint trajectory
        stepsTotal = 100;
        deltaT;
    end
    
    methods
        %% Constructor
        % A robot (PPR) object must be used to create an object of this
        % class
        function self = PathPlanner(robot)
            
            self.robot = robot;
            
        end
        
        %% SetTotalSteps
        % This function changes the total number of steps required for the
        % motion to the target location
        function SetTotalSteps(self, steps)
            
            self.stepsTotal = steps;
            
        end

        %% SetFixedTimeOffset
        % Function to set the fixed time offset for the return motion of
        % the pingpong ball due to the time taken by trajectory prediction
        % and control command sent to the robot
        function SetFixedTimeOffset (self, timeOffset)
            
            self.fixedTimeOffset = timeOffset;
            
        end
        
        %% SetTargetInfo
        % Function to set the information on the pingpong ball at the
        % intersection point in the robot workspace
        % This information includes the robot position, velocity and total
        % time from the paddle of player till the intersection point 
        function SetTargetInfo (self, targetPosition, targetVel, timeToTarget)
            
            self.targetPosition = targetPosition;
            self.targetVel = targetVel;
            self.totalTime = (timeToTarget - self.fixedTimeOffset)/0.88;
            self.deltaT = self.totalTime / self.stepsTotal;
            
        end
        
        %% IkTrajectory
        % Using Inverse Kinematics, a joint trajectory is calculated to
        % reach a pose in the cartesian frame
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
                        
            steps = length(cartesianTrajectory( 1,: ));
            epsilon = 0.1;
            W = diag([1 1 1 0.1 0.1 0.1]);
            
            manipltyMatrix = zeros(steps,1); % Manipulability
            qMatrix = zeros(steps,7); % qMatrix of trajectory
            qdot = zeros(steps,7); % joint velocities vector

            x = cartesianTrajectory(1:3,:);
            theta = cartesianTrajectory(4:6,:);
                        
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Traform of cartesian trajectory
            q0 = self.robot.model.getpos();                                                 
            qMatrix(1,:) = self.robot.model.ikcon(T,q0);                                

            for i = 1:steps-1
                T = self.robot.model.fkine(qMatrix(i,:));                 % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);            % Get position error from next cartesian waypoint
                Rd = rpy2r(theta(:,i+1)');      % Get the next RPY angles and convert to rotation matrix
                Ra = tr2rt(T);                % Current end-effector rotation matrix

                Rdot = (1/self.deltaT)*(Rd - Ra);       % Calculate rotation matrix error
                S = Rdot * Ra';                      % Skew symmetric matrix for angular velocity
                linearVelocity = (1/self.deltaT)*deltaX;
                angularVelocity = [S(3,2);S(1,3);S(2,1)];

                xdot = W*[linearVelocity;angularVelocity];
                J = self.robot.model.jacob0(qMatrix(i,:));

                if self.robot.model.maniplty(qMatrix(i,:)) < epsilon
                    lambda = 0.01;
                    invJ = pinv(J'*J + lambda*eye(7))*J';       % compute the inverse Jacobian
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
            
            ballPosition = self.targetPosition;
            ballVelocity = self.targetVel;
            ballPositionInRobotBaseTr = homtrans(self.robot.model.base, ballPosition');
%             ballPositionInRobotBaseTr = ballPoseInRobotBaseTr(1:3,4)
                        
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
            if self.robot.CheckPointsInRobotWorkspace(ballPosition) && ballPositionInRobotBaseTr(3)>=0
            
                startPt = ballPosition + ballVelocity/normBallVel*(1-normBallVel/10)*0.1;
                endPt = ballPosition - ballVelocity/normBallVel*(1-normBallVel/10)*0.1;

                s = lspb(0,1,steps);                                 % Create interpolation scalar
                for i = 1:steps
                    x(1:2,i) = startPt(1:2)*(1-s(i)) + s(i)*endPt(1:2);
                    x(3,i) = ballPosition(3);
                end
                
                % Test for changing orientation of endEff to match ball
                % orientation
                tr = self.robot.model.fkine(self.robot.model.getpos);
                diff = endPt-startPt;
                rpy = tr2rpy(tr);
                roll = rpy(1)+pi/18;
                if ballPosition(1)-tr(1,4) > 0
                    yaw = atan2(diff(3),diff(1))+pi/2;
                else
                    yaw = atan2(diff(3),diff(1))-pi/2;
                end
                    
                pitch = rpy(2);
                
                for i = 1:steps
                    theta(:,i) = [roll pitch yaw]';
                end
                                
                % The cartesian trajectory computed is plotted in the 3D
                % workspace for visalisation
                try delete(self.RMRCCartTraj_h); end
                
                self.RMRCCartTraj_h = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1);

                isReachable = true;
                cartTraj = [x ; theta];
                
            else
                isReachable = false;
                disp("Ball pose is UNREACHABLE");
            end
            
        end
        
        %% BallReturnPath
        % The function return the total computed path using Inverse
        % Kinematics and RMRC to the path to return the incoming ping pong
        % ball
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
        % This class takes in a specific joint state and uses either
        % Quintic polynomials or Trapezoidal Velocity profile to drive to
        % that joint state
        function path = FinalJointStatePath(self, qGoal, isQuintic)
            if nargin < 3
                isQuintic = false;
            end
            
            currentJointState = self.robot.model.getpos();
            steps = self.stepsTotal;
            
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