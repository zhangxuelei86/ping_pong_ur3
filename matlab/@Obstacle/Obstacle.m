classdef Obstacle < handle
    properties
        
        position;
        
    end
    
    properties (SetAccess = private)

        % size is a row vector of the [length width height] of the
        % rectangular prism obstacle
        size;
        sizeInflationFactor = 0.1;
        
        obstacle;
        obstaclePlot;
        plotTransparency = 0.7;

    end
    methods
        function self = Obstacle(size, position)
            
            self.size = size + self.sizeInflationFactor*ones(1,3);
            self.position = position;
            
            self.obstacle = self.RectangularPrism();
            
        end
        
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
        
        function PlotObstacle(self, position)
            
            if 1<nargin
                self.position = position;
                self.obstacle = self.RectangularPrism();
            end

            tcolor = [.2 .2 .8];
            try delete(self.obstaclePlot); end
            self.obstaclePlot =  patch('Faces',self.obstacle.faces,'Vertices' ...
                ,self.obstacle.vertices,'FaceVertexCData',tcolor,'FaceColor', ...
                'flat','lineStyle','none','FaceAlpha',self.plotTransparency);
            
        end
        
        function obstacle = GetObstacle(self)
            
            obstacle = self.obstacle;
            
        end
        
        function SetSizeInflation(self, factor)
            
            self.sizeInflationFactor = factor;
            
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