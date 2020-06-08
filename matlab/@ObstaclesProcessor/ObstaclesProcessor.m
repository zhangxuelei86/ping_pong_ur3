%% ObstacleProcessor class
% This class manages the existence and update of both dynamic and static
% obstacles in the scene
classdef ObstaclesProcessor < handle
    
    properties
        staticObstacles = {};
        dynamicObstacles = {};
    end
    
    properties (SetAccess = private)
        staticObstaclesSet = false; %this flag ensures that static obstacles are set only one for this object
    end
    
    methods
        
        %% Constructor
        % An object of this class must be created with a minimum of the
        % static obstcles in the scene
        function self = ObstaclesProcessor(staticObstacles)
            
            self.CreateStaticObstacles(staticObstacles);
            
        end

        %% CreateStaticObstacles
        % Creates and adds static obstacles to the scene
        function CreateStaticObstacles(self, staticObstacles)
            if self.staticObstaclesSet
                return;
            end
            hold on;
            
            for i = 1 : size(staticObstacles,1)
                self.staticObstacles{i} = Obstacle(staticObstacles{i}.size, staticObstacles{i}.position);
                self.staticObstacles{i}.SetPlotTransparency(0.1);
                self.staticObstacles{i}.PlotObstacle();
            end
            
            self.staticObstaclesSet = true;
        end
        
        %% UpdateDynamicObstacles
        % This function creates and updates the dynamic obstacles in the
        % scene as they move or pop up in the scene
        function UpdateDynamicObstacles(self, dynamicObstacles)
            
            for i = 1 : size(self.dynamicObstacles,2)
                self.dynamicObstacles{i}.DeleteObstaclePlot();
            end
                
            self.dynamicObstacles = {};
            
            for i = 1 : size(dynamicObstacles,1)
                self.dynamicObstacles{i} = Obstacle(dynamicObstacles{i}.size, dynamicObstacles{i}.position);
                self.dynamicObstacles{i}.PlotObstacle();
            end
            
        end
        
    end
end