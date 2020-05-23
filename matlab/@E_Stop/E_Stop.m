classdef E_Stop < handle
    
    properties (SetAccess = private)
        base;
        button;
    end
    
    properties
        pose = eye(4);
    end
    
    methods
        function self = E_Stop(pose)
            if nargin > 0
                self.pose = pose;
            end
            
            self.base = EnvironmentComponent("base.ply",self.pose);
            self.button = EnvironmentComponent("button.ply", self.pose*transl(0,-0.0105,0.0475));
        end
        
        function pressButton(self)
            for i = 1:5
                self.button.UpdatePose(self.button.pose * transl(0,0,-0.00125));
                drawnow();
            end
        end
        
        function releaseButton (self)
            for i = 1:20
                if i <= 10
                    self.button.UpdatePose(self.button.pose * transl(0,0,0.0002875) * trotz(pi/72));
                else
                    self.button.UpdatePose(self.button.pose * transl(0,0,0.0002875) * trotz(-pi/72));
                end
                drawnow();
            end
        end
    end
end