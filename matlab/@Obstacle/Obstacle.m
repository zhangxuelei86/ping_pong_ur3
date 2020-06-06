classdef Obstacle < handle
    properties
        
        position;
        obstacle;
        
    end
    
    properties (SetAccess = private)

        % size is a row vector of the [length width height] of the
        % rectangular prism obstacle
        size;
        sizeInflationFactor = 0.1;
        
        obstaclePlot;
        plotTransparency = 0.7;

    end
    methods
        %% Constructor
        % An object of the Obstacle class must be created with it's position and
        % size
        function self = Obstacle(size, position)
            
            self.size = size + self.sizeInflationFactor*ones(1,3);
            self.position = position;
            
            self.obstacle = self.RectangularPrism();
            
        end
        
        %% RectangularPrism
        % Modified from the RectangularPrism function provided in the
        % materials from the Robotics course
        % The function creates the rectangular prism of the obstacle from
        % the size and the centre location of the obstacle in world
        % coordinates
        function obstacle = RectangularPrism(self)
            
            lower = self.position - self.size/2;
            upper = self.position + self.size/2;
            
            vertices(1,:)=lower;
            vertices(2,:)=[upper(1),lower(2:3)];
            vertices(3,:)=[upper(1:2),lower(3)];
            vertices(4,:)=[upper(1),lower(2),upper(3)];
            vertices(5,:)=[lower(1),upper(2:3)];
            vertices(6,:)=[lower(1:2),upper(3)];
            vertices(7,:)=[lower(1),upper(2),lower(3)];
            vertices(8,:)=upper;

            faces=[1,2,3;1,3,7;
                 1,6,5;1,7,5;
                 1,6,4;1,4,2;
                 6,4,8;6,5,8;
                 2,4,8;2,3,8;
                 3,7,5;3,8,5;
                 6,5,8;6,4,8];

            faceNormals = zeros(size(faces,1),3);
            for faceIndex = 1:size(faces,1)
                v1 = vertices(faces(faceIndex,1)',:);
                v2 = vertices(faces(faceIndex,2)',:);
                v3 = vertices(faces(faceIndex,3)',:);
                faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
            obstacle.vertices = vertices;
            obstacle.faces = faces;
            obstacle.faceNormals = faceNormals;

        end
        
        %% PlotObstacle
        % This function both adds the rectangular prism obstacle into the
        % scene and can also be used to update the location of the obstacle
        function PlotObstacle(self, position)
            
            if 1<nargin
                self.position = position;
                self.obstacle = self.RectangularPrism();
            end

            tcolor = [.2 .2 .8];
            self.DeleteObstaclePlot();
            self.obstaclePlot =  patch('Faces',self.obstacle.faces,'Vertices' ...
                ,self.obstacle.vertices,'FaceVertexCData',tcolor,'FaceColor', ...
                'flat','lineStyle','none','FaceAlpha',self.plotTransparency);
            
        end
        
        %% DeleteObstaclePlot
        % This function deletes the current plot of the obstacle from the
        % scene
        function DeleteObstaclePlot(self)
            
            try delete(self.obstaclePlot); end
            
        end
        
        %% GetObstacle
        % Returns the current obstacle and its charecteristics in the object
        function obstacle = GetObstacle(self)
            
            obstacle = self.obstacle;
            
        end
        
        %% SetSizeInflation
        % In the case where the obstacle inflation factor would like to be
        % chnaged, this function is used to change it and the obstacle
        % rectagular prism of the obstacle is recomputed
        function SetSizeInflation(self, factor)
            
            self.sizeInflationFactor = factor;
            self.size = size + self.sizeInflationFactor*ones(1,3);
            self.obstacle = self.RectangularPrism();
            
        end
        
        %% SetTransparency
        % This function sets the transparency of the obstacle plot due to
        % it's characteristics such as static obstacle or dynamic obstacles
        function SetPlotTransparency(self, transparency)
            if transparency < 0 || transparency > 1
                disp('The value of transparency must be between 0 and 1');
                return
            end
            
            self.plotTransparency = transparency;
        end
        
    end
            
end