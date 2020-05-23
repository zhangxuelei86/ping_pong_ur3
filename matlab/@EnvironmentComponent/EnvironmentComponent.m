classdef EnvironmentComponent < handle

    properties (Access = private)
        meshFilePath;
    end

    properties        
        mesh;
        meshVerts;
        meshVertexCount;
        
        pose = eye(4);
    end

    methods
        
        function obj = EnvironmentComponent (meshFilePath, pose)
            if nargin > 1
                obj.meshFilePath = meshFilePath;
                obj.pose = pose;
            elseif nargin < 2
                obj.meshFilePath = meshFilePath;
            end
            
            obj.LoadMesh();
            obj.UpdatePose(obj.pose);
        end
        
        function LoadMesh(obj)
            [f,v,data] = plyread(obj.meshFilePath,'tri');

            % Get vertex count
            obj.meshVertexCount = size(v,1);

            % Move center point to origin
            midPoint = sum(v)/obj.meshVertexCount;
            obj.meshVerts = v - repmat(midPoint,obj.meshVertexCount,1);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Then plot the surfaces of the vertices trisurf
            obj.mesh = trisurf(f,obj.meshVerts(:,1), obj.meshVerts(:,2), obj.meshVerts(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            
        end
        
        function UpdatePose(obj, pose)
            obj.pose = pose;
            updatedPoints = [obj.pose * [obj.meshVerts,ones(obj.meshVertexCount,1)]']';  

            % Now update the Vertices
            obj.mesh.Vertices = updatedPoints(:,1:3);
            drawnow();
        end
        
    end

end