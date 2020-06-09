classdef GameController < handle
    
    properties
        id;
        joystick;
    end
    
    properties (SetAccess = private)
        Kv; % linear velocity gain
        Kw; % angular velocity gain
    end
    
    methods
        function self = GameController(id)
            
            self.id = id;
            self.joystick = vrjoystick(id);
            
            self.Kv = 0.2;
            self.Kw = 1.0;
            
            %% For debug purpose, joystick info is printed
            
            joystickInfo = caps(self.joystick);
            fprintf('This joystick has:\n');
            fprintf(' - %i buttons\n',joystickInfo.Buttons);
            fprintf(' - %i axes\n', joystickInfo.Axes);
            
        end
        
        function endEffVel = GetContollerCommands(self)
            
            vx = 0;
            vy = 0;
            vz = 0;
            wx = 0;
            wy = 0;
            wz = 0;
            [axes, ~, ~] = read(self.joystick);
            
            if round(axes(4)) == 1
                vx = self.Kv * axes(1);
                vy = self.Kv * axes(2);
                vz = self.Kv * axes(6);
            
            elseif round(axes(5)) == 1
                
                wx = self.Kw * axes(1);
                wy = self.Kw * axes(2);
                wz = self.Kw * axes(6);
                
            end
            
            endEffVel = [vx;vy;vz;wx;wy;wz];
            
        end
        
        function SetLinVelGain(self, gain)
            
            self.Kv = gain;
            
        end
        
        function SetAngVelGain(self, gain)
            
            self.Kw = gain;
            
        end
    end
end