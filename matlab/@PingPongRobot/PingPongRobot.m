%% PingPongRobot Class
% This class has was adapted from the Peter Corke's Robotics Toolbox Linear
%UR5
% The definition of the class has been modifies to suit the purpose of our
% spedific applictaion
% Calling this class creates an instance of an object of this class 
classdef PingPongRobot < handle
    properties
        %< Robot model
        model;
        
        %< Robot workspace
        workspace = [-1 1 -2 2 0 3];

    end
    
    properties (Access = private)
        
        %< Robot name
        name;

    end
    
    methods
        %% PingPongRobot
        % Given a name (optional), initialize object of PingPong Robot class
        function self = PingPongRobot( name, workspace )
            if nargin < 1
                % Create a unique name (ms timestamp after 1ms pause)
                pause(0.001);
                name = ['PingPongRobot'];
            end
            
            if nargin > 1
                % Set Robot workspace to user specified workspace
                self.workspace = workspace;
            end
            
            self.name = name;
            
            % Get the PingPongRobot model
            self.GetPingPongRobot();
        end
        
        %% GetPingPongRobot
        % Create and return a UR3 robot model
        function GetPingPongRobot ( self )

            % Create the UR3 model mounted on a linear rail
            RobotLinks(1) = Link([pi  0        0        pi/2  1]); % PRISMATIC Link
            RobotLinks(2) = Link([0  0.152+0.03   0        pi/2   0]);
            RobotLinks(3) = Link([0  0.12         -0.244   0      0]);
            RobotLinks(4) = Link([0  -0.093       -0.213   0      0]);
            RobotLinks(5) = Link([0  0.083        0        pi/2   0]);
            RobotLinks(6) = Link([0  0.083        0        -pi/2	0]);
            RobotLinks(7) = Link([0  0.082        0        0       0]);

            % Incorporate joint limits
            RobotLinks(1).qlim = [-0.8 0];
            RobotLinks(2).qlim = deg2rad([-360 360]);
            RobotLinks(3).qlim = deg2rad([-360 360]);
            RobotLinks(4).qlim = deg2rad([-360 360]);
            RobotLinks(5).qlim = deg2rad([-360 360]);
            RobotLinks(6).qlim = deg2rad([-360 360]);
            RobotLinks(7).qlim = deg2rad([-360 360]);

            self.model = SerialLink(RobotLinks,'name',self.name);
            
            % Rotate PingPong robot to the correct orientation
%             self.model.base = transl(0.4,0,0) * self.model.base * trotx(pi/2) * troty(pi/2);

        end
        
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot ( self )
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR3Link',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display UR3 robot as polygon mesh
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are available in the ply files)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        %% GetRobotLinksTranform
        % Given the joint states of the UR3 robot
        % The transform of the base and all the links of the robot is calculated
        function robotLinksTransforms = GetRobotLinksTransforms(self, q)
            if nargin < 2
                q = self.model.getpos();
            end

            tr = zeros(4,4,self.model.n+1);
            tr(:,:,1) = self.model.base;
            for i = 1 : self.model.n
                tr(:,:,i+1) = tr(:,:,i) * self.model.links(i).A(q(i));
            end

            robotLinksTransforms = tr;

        end
        
        %% CheckPointsInRobotWorkspace
        % Given a set of cartesian points row vectors, the function
        % returns a vector flag of whether the points lie inside the
        % ellipsoid of the robot workspace
        function flag = CheckPointsInRobotWorkspace (self, points)
            
            robotBase = homtrans(self.model.base,transl(0,0,-0.4));
            robotBaseCentre = robotBase(1:3,4)';
            ellipsoidRadii = [0.8,0.45,0.5];
            
            dist = self.GetAlgebraicDist(points,robotBaseCentre,ellipsoidRadii);
            flag = zeros(1,length(dist));
            flag(dist<1) = 1;
            
        end
    end
    
    methods (Static)
        
        %% GetAlgebraicDist
        % This function is from the Lab Exercise 6 in the Robotics subject
        % materials provided
        % determine the algebraic distance given a set of points and the center
        % point and radii of an elipsoid
        % *Inputs:* 
        %
        % _points_ (many*(2||3||6) double) x,y,z cartesian point
        %
        % _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
        %
        % _radii_ (1 * 3 double) a,b,c of an ellipsoid
        %
        % *Returns:* 
        %
        % _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid

        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

        algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                      + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                      + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
        
    end
end