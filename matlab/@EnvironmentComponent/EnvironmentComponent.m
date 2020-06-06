%% EnvironmentComponent
% This class creates an instance of objects to be placed in the environment
% of the simulation
% The pose of the object can be changed and assessed as requested on use
classdef EnvironmentComponent < handle

    properties (Access = private)
        % path to .ply file of model
        meshFilePath;
    end

    properties        
        % object properties of pose and mesh information
        mesh;
        meshVerts;
        meshVertexCount;
        
        pose = eye(4);
    end

    methods
        
        %% Constructor
        % The object of thos class is created with a the path of the .ply
        % file of object and the pose of the object can be included
        % optionally at the instantiation of the object
        % The object is automatically loaded into the workspace at the set
        % or default pose when the object is created
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
        
        %% LoadMesh
        % The object mesh is loaded into the workspace when this fucntion
        % is called
        function LoadMesh(obj)
            [f,v,data] = plyread(obj.meshFilePath,'tri');

            % Get vertex count of ply model
            obj.meshVertexCount = size(v,1);

            % Move center point to origin of mesh
            midPoint = sum(v)/obj.meshVertexCount;
            obj.meshVerts = v - repmat(midPoint,obj.meshVertexCount,1);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Then plot the surfaces of the vertices trisurf
            obj.mesh = trisurf(f,obj.meshVerts(:,1), obj.meshVerts(:,2), obj.meshVerts(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            if ~ishold
                hold on;
            end
                        
        end
        
        %% UpdatePose
        % This object takes in a desired pose of the object mesh to be set
        % in the environment
        % The entire mesh is tranformed to the new pose the instance this
        % function is called with the new desired pose
        function UpdatePose(obj, pose)
            obj.pose = pose;
            updatedPoints = [obj.pose * [obj.meshVerts,ones(obj.meshVertexCount,1)]']';  

            % Now update the Vertices
            obj.mesh.Vertices = updatedPoints(:,1:3);
            drawnow();
        end
        
    end

end