classdef RobotController < handle
    
    properties
        isCollision = false;
    end
    
    properties (SetAccess = private)
        robot; % PingPongRobot
        
        isKillRobot = false; % flag for whether robot is in kill state

        trajectory; % current tranjectory the robot is to follow
    end
    
    properties
        currentJointState;
    end
    
    methods
        function self = RobotController (robot)
            
            self.robot = robot; %% Ping Pong Robot
        end
        
        %% StopRobot
        % This function is sets the flag of isKillRobot to 0
        % This function is called on the event of any collision and unwanted
        % or unexpected results
        % The flag isKillRobot determines if the robot will progress in
        % it's motion or not.
        function StopRobot ( self )
            
            if self.isKillRobot == true
                self.isKillRobot = false;
            end
            
            self.trajectory = self.currentJointState;
        end
        
        %% SetRobotTrajectory
        % The function is called to set the current tranjectory the robot
        % should follow
        
        function SetRobotJointTrajectory(self, trajectory)
            
            self.trajectory = trajectory;
        end
        
        %% SetRobotJointState
        
        function SetRobotJointState (self, jointState)
            
            self.currentJointState = jointState;
            
            self.robot.model.animate(self.currentJointState);
            drawnow();
        end
        
        %% CheckCollision
        % This is tailored from Lab 5 exercises
        % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
        % and triangle obstacles in the environment (faces,vertex,faceNormals)
        % It checks for collision of any of the links and the plane as a line plane
        % intersection
        function isCollision = CheckCollision ( self,qMatrix,pointOnPlane,planeNormal,returnOnceFound )
            if nargin < 5
                returnOnceFound = true;
            end

            for qIndex = 1:size(qMatrix,1)

                tr = self.robot.GetRobotLinksTransforms;

                % Go through each link
                for i = 1 : size(tr,3)-1    
                    [~,check] = self.LinePlaneIntersection(planeNormal,pointOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');

                    if check == 1
                        display('Intersection');
                        isCollision = true;
                        
                        if returnOnceFound
                            return
                        end
                        
                    end
                end
            end
        end
        
    end
    
    methods (Static)
        %% LinePlaneIntersection
        % This function is provided from the Lecture materials provided during the
        % 41013 Robotics subject
        % Given a plane (normal and point) and two points that make up another line, get the intersection
        % Check == 0 if there is no intersection
        % Check == 1 if there is a line plane intersection between the two points
        % Check == 2 if the segment lies in the plane (always intersecting)
        % Check == 3 if there is intersection point which lies outside line segment
        function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
    end
end