classdef LightCurtain < handle
    
    properties
         sources = [];
         
         maxDistance = 0.5;
    end
    
    methods
        
        function self = LightCurtain(sources)
            for i = 1:length(sources)
                self.sources(i).origin = sources(i).origin;
                self.sources(i).angDiff = sources(i).angDiff;
                self.sources(i).yaw = sources(i).yaw;
            end
        end
        
        function AddNewSource(self, source)
            self.sources = [self.sources;source];
        end
        
        function SetMaxDistance(self, distance)
            self.maxDistance = distance;
        end
        
        function PlotSourceRay(self, source)
            
            if ~ishold
                hold on;
            end
            
            for i = source.yaw-90:source.angDiff:source.yaw+90
                x = source.origin(1) + self.maxDistance * cos(deg2rad(i));
                y = source.origin(2) + self.maxDistance * sin(deg2rad(i));
                plot3(gca,[source.origin(1),x],[source.origin(2),y],[source.origin(3),source.origin(3)],"r--");
            end
            
        end
        
        function isBreached = CheckBreach(self, position)
            
            for i = 1:length(self.sources)
                startAngle = self.sources(i).yaw - 90;
                endAngle = self.sources(i).yaw + 90;
                
                polarRadius = sqrt(position(1)^2+position(2)^2);
                angle = atan(position(2)/position(1));
                
                if angle>=startAngle && angle<= endAngle && ...
                        polarRadius<self.maxDistance
                    isBreached = true;
                    return;
                end
            end
            
            isBreached = false;
        end
        
    end
end