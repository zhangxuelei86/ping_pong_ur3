classdef ROSRobotWrapper < handle
    %ROSROBOTWRAPPER Wraps around PingPongRobot, communicates with Unity
    %via ROS to subscribe to joint angles, origin (linear rail) transform
    
    properties        
        robot; % Ping Pong Robot
        
        jointStateTopic;
        jointStateSub;
        
        originTransformTopic;
        originTransformSub;
        
        eStopTopic;
        eStopPub;
        eStopMsg;
        
        robotUpdateTimer;
    end
    
    methods
        function self = ROSRobotWrapper(robot, rosMasterURI, rosIP)
            %ROSROBOTWRAPPER Construct an instance of this class
            %   Initialises default properties
            self.robot = robot;
            
            ROSRobotWrapper.InitROS(rosMasterURI, rosIP);
            
            self.jointStateTopic = '/ppr/joint_states';
            self.jointStateSub = rossubscriber(self.jointStateTopic, 'sensor_msgs/JointState');

            self.originTransformTopic = '/ppr/origin_pose';
            self.originTransformSub = rossubscriber(self.originTransformTopic, 'geometry_msgs/PoseStamped');
        
            self.eStopTopic = '/ppr/e_stop';
            self.eStopMsg = rosmessage('std_msgs/Bool');
            
            self.robotUpdateTimer = timer('StartDelay', 0, 'Period', 0.05, 'TasksToExecute', Inf, 'ExecutionMode', 'fixedDelay');
            self.robotUpdateTimer.TimerFcn = @(obj, event)updateRobot(self);
        end
        
        function updateRobot(self)
            %UPDATEROBOT Updates the robot base transform and joint angles
            
            originTransformMsg = self.originTransformSub.LatestMessage;
            jointStateMsg = self.jointStateSub.LatestMessage;
            
            if ~isempty(originTransformMsg) && ~isempty(jointStateMsg)
                self.robot.model.base = ROSRobotWrapper.PoseStampedToTransform(originTransformMsg);
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
        
        function eStopRobot(self, value)
            %ESTOPROBOT Stops, or restarts the robot
            % value = true: stop (e-stop), value = false: resume
            
            self.eStopPub = rospublisher(self.eStopTopic,'std_msgs/Bool');
            self.eStopMsg.Data = value;
            send(self.eStopPub, self.eStopMsg); pause(0.1);
            disp("Sent E-Stop message");
            try delete(self.eStopPub); end
        end
    end
    
    methods(Static)
        function transform = PoseStampedToTransform(poseStamped)
            %POSESTAMPEDTOTRANSFORM Turn a PoseStamped ROS message into a
            %Homogenneous transform (PoseStamped orientation is in Quaternion)
            
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
        
        function InitROS(masterURI, ip)
            %INITROS Initialises the ROS Network if needed
            rosshutdown
            setenv('ROS_MASTER_URI',masterURI);
            setenv('ROS_IP',ip);
            rosinit
        end
    end
end

