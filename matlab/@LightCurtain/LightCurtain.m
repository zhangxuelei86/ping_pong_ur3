%% Class for horizontal light curtains
classdef LightCurtain < handle
    
    properties
         sources = []; % total number of sources of the light curtains for one object
         
         maxDistance = 0.5; % maximum allowable distance from the light curtain
    end
    
    methods
        %% Constructor
        % Given the sources (an array of structs with parameters; origin, 
        % angDiff(angular difference),yaw (degree)) and the flag t plot on
        % creation
        % The class creates an object of all the light curtains
        function self = LightCurtain(sources, plot)
            if nargin < 2
                plot = false;
            end
            
            for i = 1:length(sources)
                self.sources(i).origin = sources(i).origin;
                self.sources(i).angDiff = sources(i).angDiff;
                self.sources(i).yaw = sources(i).yaw;
                
                if plot
                    self.PlotSourceRay(self.sources(i));
                end
                
            end
        end
        
        %% AddNewSource
        %
        function AddNewSource(self, source)
            self.sources = [self.sources;source];
        end
        
        %% SetMaxDistance
        % Given a new distance update the minimum distance to be detected
        % as breached
        function SetMaxDistance(self, distance)
            self.maxDistance = distance;
        end
        
        %% PlotSourceRay
        % This plots the rays of the light curtain based on the angular
        % difference
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
        
        %% ChecKBreach
        % Thsi function checks whether a point is located in the
        % environment of the light curtain
        % The function returns whether or not the point has breached the
        % light curtain
        function isBreached = CheckBreach(self, position)
            
            for i = 1:length(self.sources)
                
                % overflow is used to represent the case when the yaw lies
                % between 270&360 cause the end angle to overflow into 360+
                % degrees
                overflow = false;
                startAngle = self.sources(i).yaw - 90;
                endAngle = self.sources(i).yaw + 90;
                if endAngle > 360
                    endAngle = endAngle - 360;
                    overflow = true;
                end
                
                xd = position(1)-self.sources(i).origin(1);
                yd = position(2)-self.sources(i).origin(2);
                
                polarRadius = sqrt(xd^2+yd^2);
                angle = atan2(yd,xd);
                if angle < 0
                    angle = rad2deg(angle) + 360;
                else
                    angle = rad2deg(angle);
                end
                
                % Based on the overflow, the point is checked to see
                % whether it lies in the 180 degree segment or not
                if ~overflow
                    if angle>=startAngle && angle<=endAngle && ...
                            polarRadius<self.maxDistance
                        isBreached = true;
                        return;
                    end
                else
                    if ((angle>=0 && angle <=endAngle) || ...
                            (angle>=startAngle && angle <= 360))...
                            && polarRadius<self.maxDistance
                        isBreached = true;
                        return;
                    end                    
                end
            end
            
            isBreached = false;
        end
        
    end        

end