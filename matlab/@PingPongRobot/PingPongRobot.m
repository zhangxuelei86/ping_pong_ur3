classdef PingPongRobot < handle
    properties
        %< Robot model
        model;
        
        %< Robot workspace
        workspace = [-1 1 -1 1 -0.3 1];

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
                name = ['PingPongRobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
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
            L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            L(2) = Link([0      0.152+0.03  0       pi/2   0]);
            L(3) = Link([0      0.12  -0.244   0     0]);
            L(4) = Link([0      -0.093  -0.213 0      0]);
            L(5) = Link([0      0.083   0       pi/2   0]);
            L(6) = Link([0      0.083   0       -pi/2	0]);
            L(7) = Link([0      0.082      0       0       0]);

            % Incorporate joint limits
            L(1).qlim = [-0.4 0.4];
            L(2).qlim = deg2rad([-360 360]);
            L(3).qlim = deg2rad([-360 360]);
            L(4).qlim = deg2rad([-360 360]);
            L(5).qlim = deg2rad([-360 360]);
            L(6).qlim = deg2rad([-360 360]);
            L(7).qlim = deg2rad([-360 360]);

            self.model = SerialLink(L,'name',self.name);
            
            % Rotate PingPong robot to the correct orientation
            self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);

        end
        
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot ( self )
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
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
    end
end