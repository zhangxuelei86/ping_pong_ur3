classdef RobotController < handle
    
    properties (SetAccess = private)
        robot;
        
        isKillRobot = 0;
    end
    
    methods
        function self = RobotController (robot)
            
            self.robot = robot;
        end
        
        function stopRobot ( self )
            if self.isKillRobot == 1
                self.isKillRobot = 0;
            end
        end
        
    end
end