classdef ROSSimWrapper < handle
    %ROSSIMWRAPPER SimWrapper mostly subscribes to obstacles and ball info
    %from the Unity Simulation through ROS
    %   Details needed
    
    properties
        ppr;
        
        ballPosition;
        ballVelocity;
        ballTwistTopic;
        ballTwistSub;
        
        ballState;
        ballStateTopic;
        ballStateSub;
        
        ballPlot_h;
        prediction_h;
        intercept_h;
    end
    
    methods
        function self = ROSSimWrapper(ppr)
            %ROSSIMWRAPPER Construct an instance of this class
            %   Detailed needed
            self.ppr = ppr;
            
            self.ballPosition = [0, 0, 0];
            self.ballVelocity = [0, 0, 0];
            
            self.ballPlot_h = plot3(self.ballPosition(1),self.ballPosition(2),self.ballPosition(3), ...
                                    'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            
            self.ballTwistTopic = '/ball/twist';
            self.ballTwistSub = rossubscriber(self.ballTwistTopic, 'geometry_msgs/Twist');
            
            self.ballState = 0;
            self.ballStateTopic = '/ball/state';
            self.ballStateSub = rossubscriber(self.ballStateTopic, 'std_msgs/UInt8');
        end
        
        function updateBall(self)
            %UPDATEROBOT Updates the ball position, velocity, and state
            ballTwistMsg = self.ballTwistSub.LatestMessage;
            ballStateMsg = self.ballStateSub.LatestMessage;
            if ~isempty(ballTwistMsg) && ~isempty(ballStateMsg)
                
                self.ballPosition(1) = ballTwistMsg.Angular.X;
                self.ballPosition(2) = ballTwistMsg.Angular.Y;
                self.ballPosition(3) = ballTwistMsg.Angular.Z;
                
                self.ballVelocity(1) = ballTwistMsg.Linear.X;
                self.ballVelocity(2) = ballTwistMsg.Linear.Y;
                self.ballVelocity(3) = ballTwistMsg.Linear.Z;
                
                self.ballState = ballStateMsg.Data;
                
                try delete(self.ballPlot_h); end
                self.ballPlot_h = plot3(self.ballPosition(1),self.ballPosition(2),self.ballPosition(3), ...
                                    'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
                drawnow();
            end
        end
        
        function [position, velocity, time, success] = findInterceptPoint(self)
            deltaT = 0.02;
            surfaceHeight = 1.3;
            
            position = zeros(3);
            velocity = zeros(3);
            time = 0;
            
            [positions, velocities] = ROSSimWrapper.PredictBall(self.ballPosition, self.ballVelocity, surfaceHeight, deltaT);
            
            try delete(self.prediction_h); end
            self.prediction_h = plot3(positions(:,1)',positions(:,2)',positions(:,3)','k.','LineWidth',1);
            
            flags = self.ppr.CheckPointsInRobotWorkspace(positions);
            pointsInRange = find(flags);
            if isempty(pointsInRange)
                success = false;
                return;
            end
            
            interceptIndex = pointsInRange(ceil(end/3));
            position = positions(interceptIndex,:);
            velocity = velocities(interceptIndex,:);
            time = deltaT*(interceptIndex-1);
            success = true;
            
            try delete(self.intercept_h); end
            self.intercept_h = plot3(position(1),position(2),position(3), ...
                                'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        end
    end
    
    methods(Static)
        % estimate future ball positions and velocities
        function [positions, velocities] = PredictBall(currentPos, currentVel, surfaceHeight, deltaT)
            gravity = -9.81;
            bounciness = 0.8;
            
            positions = currentPos;
            velocities = currentVel;
            
            bounced = false;
            finished = false;
            while ~finished
                velocities = [velocities; velocities(end,1:2) 0];
                velocities(end, 3) = velocities(end-1, 3) + gravity*deltaT;
                
                positions = [positions; (positions(end,1:2)+velocities(end,1:2)*deltaT) 0];
                positions(end, 3) = positions(end-1,3) + velocities(end-1,3)*deltaT;
                
                if positions(end, 3) <= surfaceHeight
                    positions(end, 3) = surfaceHeight;
                    velocities(end, 3) = velocities(end, 3)*(-1)*bounciness;
                    velocities(end, 1:2) = velocities(end, 1:2)*bounciness;
                    if bounced
                        finished = true;
                        break;
                    else
                        bounced = true;
                    end
                end
            end
        end
    end
end

