classdef PathChecker < handle
    
    properties
        
        robot; % PPR robot
        obstaclesProcessor; % Obstacle processor of the simulation
        
        currentPath;
        
        pathIndex;
        
    end
    
    methods
        %% Constructor
        % An object of this class must be created with a robot object and
        % an ObstacleProcessor object
        function self = PathChecker(robot, obstaclesProcessor)
            
            self.robot = robot;
            self.obstaclesProcessor = obstaclesProcessor;
            
        end
        
        %% SetCurrentMesh
        % The function is used to set the current path being traveled by
        % the robot (PPR)
        function SetCurrentPath(self, path)
            self.currentPath = path;
        end
        
        %% SetPathIndex
        % This function is used to set the index of the current joint state
        % in the path that the robot is in
        function SetPathIndex(self, pathIndex)
            self.pathIndex = pathIndex;
        end
        
        %% CheckPath
        function isNextJSOk = CheckPath(self, offset)
            if self.pathIndex+offset >= length(self.currentPath)
                offset = length(self.currentPath) - self.pathIndex;
            end
            
            for offsetIndex = 0:offset
                q = self.currentPath(self.pathIndex+offsetIndex,:);
          
                tr = self.robot.GetRobotLinksTransforms(q);
                staticObstacles = self.obstaclesProcessor.staticObstacles;
                dynamicObstacles = self.obstaclesProcessor.dynamicObstacles;
                obstacles = [staticObstacles(:)',dynamicObstacles(:)'];

                % Go through each link and check for collision
                for i = 2 : size(tr,3)-1
                    % Go through each obstacle in the simulation
                    for j = 1:size(obstacles,2)
                        % Go through each face of the rectangular prism
                        % obstacle
                        for faceIndex = 1:size(obstacles{j}.obstacle.faces,1)
                            vertOnPlane = obstacles{j}.obstacle.vertices(obstacles{j}.obstacle.faces(faceIndex,1)',:);
                            [intersectionPoint,check] = self.LinePlaneIntersection(obstacles{j}.obstacle.faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                            % Check if the line intersects with the plane on
                            % the size and if it lies inside the triangle on
                            % that rectangular prism side
                            if (check == 1 && self.IsIntersectionPointInsideTriangle(intersectionPoint,obstacles{j}.obstacle.vertices(obstacles{j}.obstacle.faces(faceIndex,:)',:)))
                                disp('Intersection');
                                isNextJSOk = false;
                                return
                            end
                        end
                    end
                end
            end  
            isNextJSOk = true;
            
        end
    end
    
    methods (Static)
        
        %% LinePlaneIntersection
        % This function is provided from the Lecture materials provided during the
        % 41013 Robotics subject
        % Given a plane (normal and point) and two points that make up another line, get the intersection
        % Check == 0 if there is no intersection
        % Check == 1 if there is a line plane intersection between the two points
        % Check == 2 if the segment lies in the plane (always intersecting)
        % Check == 3 if there is intersection point which lies outside line segment
        function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
        
        %% LineLineIntersection
        % This function is implemented for checking self collision of the
        % robot i.e. collisions between the links of the robot
        function [intersectionPoint, check] = LineLineIntersection(startP,endP)
            
            intersectionPoint = [0 0 0];
            lineVec = endP - startP;

            syms t s;
            eqn1 = startP(1,:) + t*lineVec(1,:);
            eqn2= startP(2,:) + s*lineVec(2,:);
            eqns = [eqn1(1,1) == eqn2(1,1), eqn1(1,2) == eqn2(1,2)];

            S = solve(eqns);
            if isempty(S)
                
                check = false;
                return;
                
            else
                
                Pt1 = subs(eqn1,t,S.t);
                Pt2 = subs(eqn2,s,S.s);
                
                if (Pt1 == Pt2)
                    intersectionPoint = Pt1;
                    check = true;
                else
                    check = false;
                end
                
            end
        end
        
        %% IsIntersectionPointInsideTriangle
        % This function has been adopted from the materials provied in the
        % Lab05 of the Robotics course
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is 
        % inside (result == 1) or 
        % outside a triangle (result ==0 )
        function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

        u = triangleVerts(2,:) - triangleVerts(1,:);
        v = triangleVerts(3,:) - triangleVerts(1,:);

        uu = dot(u,u);
        uv = dot(u,v);
        vv = dot(v,v);

        w = intersectP - triangleVerts(1,:);
        wu = dot(w,u);
        wv = dot(w,v);

        D = uv * uv - uu * vv;

        % Get and test parametric coords (s and t)
        s = (uv * wv - vv * wu) / D;
        if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
            result = 0;
            return;
        end

        t = (uv * wu - uu * wv) / D;
        if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
            result = 0;
            return;
        end

        result = 1;                      % intersectP is in Triangle
        end

        
    end
end