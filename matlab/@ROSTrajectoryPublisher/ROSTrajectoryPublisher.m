classdef ROSTrajectoryPublisher < handle
    %ROSTRAJECTORYPUBLISHER Packages trajectory into JointTrajectory format
    %and sends to to the robot via ROS
    
    properties
        
        dataType;
        jointTrajTopic;
        jointTrajPub;
        jointTrajMsg;
        
        jointNames;
        
        steps;
    end
    
    methods
        function self = ROSTrajectoryPublisher()
            %ROSTRAJECTORYPUBLISHER Construct an instance of this class
            %   Initialises default properties

            self.dataType = 'trajectory_msgs/JointTrajectory';
            self.jointTrajTopic = '/ppr/follow_joint_trajectory';
            self.jointTrajMsg = rosmessage(self.dataType);
            
            self.jointNames = ["base_link",     ...
                               "shoulder_link", ...
                               "upper_arm_link",...
                               "forearm_link",  ...
                               "wrist_1_link",  ...
                               "wrist_2_link",  ...
                               "wrist_3_link"];
            self.jointTrajMsg.JointNames = self.jointNames;

            self.steps = 0;
        end
        
        function success = SendTrajectory(self, qMatrix, velMatrix, deltaT)
            %SENDTRAJECTORY Sends a joint trajectory to the robot
            % Returns false if the number of steps is not similar
            
            [qMatrixSize, ~] = size(qMatrix);
            [velMatrixSize, ~] = size(velMatrix);
            if qMatrixSize ~= self.steps || velMatrixSize ~= self.steps
                disp("Number of steps not matched. Use InitPublisher() again.");
                success = false;
                return;
            end
            
            deltaT_msec = deltaT*(1000);
            for i = 1:self.steps
               self.jointTrajMsg.Points(i,1).Positions = qMatrix(i,:);
               self.jointTrajMsg.Points(i,1).Velocities = velMatrix(i,:);
               self.jointTrajMsg.Points(i,1).TimeFromStart.Nsec = deltaT_msec*(i-1);
            end
            self.jointTrajPub = rospublisher(self.jointTrajTopic,self.dataType);
            send(self.jointTrajPub, self.jointTrajMsg); pause(0.2);
            disp("Joint Trajectory SENT to robot");
            try delete(self.jointTrajPub); end
            success = true; return;
        end
        
        function InitPublisher(self, interpolationSteps)
            %INITPUBLISHER Initialises the publishers and messages
            % The number of interpolation steps need to be defined first
            % because creating the JointTrajectory message takes a lot of
            % time (can be done at the start).
            
            self.steps = interpolationSteps;
            for i = 1:self.steps
                jointTrajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                self.jointTrajMsg.Points = [self.jointTrajMsg.Points; jointTrajPoint];
            end
        end
    end
end

