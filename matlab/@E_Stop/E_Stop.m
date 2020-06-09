%% E_Stop Class
% This class represents the animation and state of the estop button of the
% scene
% The pose of the EStop can be changed and the state can be obtained from
% the object
classdef E_Stop < handle
    % Properties of the EStop
    % base: The housing of the button
    % button: push and twist button
    % state: if the EStop is active or not
    properties (SetAccess = private)
        base;
        button;
        state = false;
    end
    
    properties
        pose = eye(4);
    end
    
    methods
        %% E_Stop
        % Class Constructor
        % The object of this class can be created with the required pose
        function self = E_Stop(pose)
            if nargin > 0
                self.pose = pose;
            end
            
            self.base = EnvironmentComponent("base.ply",self.pose);
            self.button = EnvironmentComponent("button.ply", self.pose*transl(0,-0.0105,0.0475));
        end
        
        %% PushButton
        % Animation of EStop button pushed
        % The state of EStop is updated to true when the button is pushed
        function PushButton(self)
            self.state = true;
            
            for i = 1:5
                self.button.UpdatePose(self.button.pose * transl(0,0,-0.00125));
                drawnow();
            end
        end
        
        %% ReleaseButton
        % Animation of twist and release of EStop
        % The state of EStop button is updated to false when button is
        % released
        function ReleaseButton (self)
            self.state = false;
            
            for i = 1:20
                if i <= 10
                    self.button.UpdatePose(self.button.pose * transl(0,0,0.0002875) * trotz(pi/72));
                else
                    self.button.UpdatePose(self.button.pose * transl(0,0,0.0002875) * trotz(-pi/72));
                end
                drawnow();
            end
        end
        
        function state = GetEStopState(self)
            state = self.state;
        end
        
    end
end