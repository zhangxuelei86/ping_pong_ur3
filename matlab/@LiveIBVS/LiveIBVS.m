classdef LiveIBVS < handle
    properties
        robot;
        
        cam;
        fps;
        lambda;
        depth;
        camPos;
    end
    
    methods
        function self = LiveIBVS(robot)
            
            self.robot = robot;
            
        end
        
        function setCameraInfo(self, focalLength, pixelSize, resolution, centre, name)
            if nargin < 6
                name = "robotCamera";
                if nargin < 5
                    centre = resolution/2;
                end
            end
            
            self.cam = CentralCamera('focal', focalLength, 'pixel', pixelSize, ...
                'resolution', resolution, 'centre', centre,'name', name);
        end
        
        function setCameraFPS(self, fps)
            self.fps = fps;
        end
        
        function setIBVSDepth(self, depth)
            self.depth = depth;
        end
        
        
    end
end