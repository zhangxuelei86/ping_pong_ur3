classdef ROSRobotWrapper < handle
    %ROSROBOTWRAPPER Wraps around PingPongRobot, communicates with Unity
    %via ROS to subscribe to joint angles, origin (linear rail) transform
    
    properties
        rosMasterURI;
        rosIP;
        
        robot; % Ping Pong Robot
        
        jointStateTopic;
        jointStateSub;
        
        originTransformTopic;
        originTransformSub;
        
        robotUpdateTimer;
    end
    
    methods
        function self = ROSRobotWrapper(robot)
            %ROSROBOTWRAPPER Construct an instance of this class
            %   Initialises default properties
            self.robot = robot;
            
            self.rosMasterURI = 'http://192.168.1.118:11311'; % default Ubuntu PC local IP
            self.rosIP = '192.168.1.116'; % default Windows PC (MATLAB) local IP
            
            self.jointStateTopic = '/ppr/joint_states';
            self.jointStateSub = rossubscriber(self.jointStateTopic, 'sensor_msgs/JointState');

            self.originTransformTopic = '/ppr/origin_pose';
            self.originTransformSub = rossubscriber(self.originTransformTopic, 'geometry_msgs/PoseStamped');
        
            self.robotUpdateTimer = timer('StartDelay', 0, 'Period', 0.05, 'TasksToExecute', Inf, 'ExecutionMode', 'fixedDelay');
            self.robotUpdateTimer.TimerFcn = @(obj, event)updateRobot(self);
        end
        
        function updateRobot(self)
            %UPDATEROBOT Updates the robot base transform and joint angles
            
            originTransformMsg = self.originTransformSub.LatestMessage;
            jointStateMsg = self.jointStateSub.LatestMessage;
            
            if ~istempty(originTransformMsg) && ~isempty(jointStateMsg)
                self.robot.model.base = PoseStampedToTransform(originTransformMsg);
                self.robot.model.animate(jointStateMsg.Position');
                drawnow();
            end
        end
        
        function startRobotUpdate(self)
            start(self.robotUpdateTimer);
        end
        
        function stopRobotUpdate(self)
            stop(self.robotUpdateTimer);
        end
    end
    
    methods(Static)
        function transform = PoseStampedToTransform(poseStamped)
            transform = transl(poseStamped.Pose.Position.X ...
                              ,poseStamped.Pose.Position.Y ...
                              ,poseStamped.Pose.Position.Z);
            quat = [poseStamped.Pose.Orientation.W ...
                   ,poseStamped.Pose.Orientation.X ...
                   ,poseStamped.Pose.Orientation.Y ...
                   ,poseStamped.Pose.Orientation.Z];
            transform(1:3,1:3) = quat2rotm(quat);
            return;
        end
    end
end

