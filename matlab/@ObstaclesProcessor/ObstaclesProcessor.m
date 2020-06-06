classdef ObstaclesProcessor < handle
    properties
        staticObstacles = {};
        dynamicObstacles = {};
    end
    properties (SetAccess = private)
        staticObstaclesSet = false;
    end
    methods
        
        function self = ObstaclesProcessor(staticObstacles)
            
            self.CreateStaticObstacles(staticObstacles);
            
        end

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
        
        function UpdateDynamicObstacles(self, dynamicObstacles)
            
            if ~isempty(self.dynamicObstacles)
                self.dynamicObstacles = {};
            end
            
            for i = 1 : size(dynamicObstacles,1)
                self.dynamicObstacles{i} = Obstacle(dynamicObstacles{i}.size, dynamicObstacles{i}.position);
                self.dynamicObstacles{i}.PlotObstacle();
            end
            
        end
        
    end
end